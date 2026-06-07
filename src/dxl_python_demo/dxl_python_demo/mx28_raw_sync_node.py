import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from dynamixel_sdk import GroupSyncWrite, PortHandler, PacketHandler, COMM_SUCCESS


class MX28RawNode(Node):
    def __init__(self):
        super().__init__("mx28_raw_node")

        # ==============================
        # 基本参数
        # ==============================
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 57600)

        # 现在你有 4 个电机，可以传 [1, 2, 4, 6]
        # 以后 6 根杆子可以传 [1, 2, 3, 4, 5, 6]
        self.declare_parameter("dxl_ids", [1, 2, 3, 4, 5, 6])

        # 是否配置 Multi-turn Mode
        # 第一次配置可以 true，之后建议 false
        self.declare_parameter("configure_multiturn", False)

        # Resolution Divider:
        # 1: 4096 units/rev，大约 ±7 圈
        # 2: 2048 units/rev，大约 ±14 圈
        # 4: 1024 units/rev，大约 ±28 圈
        self.declare_parameter("resolution_divider", 1)

        # raw 安全范围
        # Multi-turn Mode 下 MX-28 常用范围大约是 -28672 ~ 28672
        self.declare_parameter("min_raw", -28672)
        self.declare_parameter("max_raw", 28672)

        # Moving Speed，地址 32
        self.declare_parameter("moving_speed", 80)

        # 读取反馈周期
        self.declare_parameter("publish_period", 0.5)

        # Protocol 1.0 的 Status Return Level:
        # 1 = 只对 read/ping 返回，write 不返回，避免 TxOnly 后残留状态包干扰总线。
        self.declare_parameter("status_return_level", 1)

        self.port_name = self.get_parameter("port").value
        self.baud_rate = int(self.get_parameter("baud_rate").value)
        self.dxl_ids = list(self.get_parameter("dxl_ids").value)

        self.configure_multiturn = bool(self.get_parameter("configure_multiturn").value)
        self.resolution_divider = int(self.get_parameter("resolution_divider").value)

        self.min_raw = int(self.get_parameter("min_raw").value)
        self.max_raw = int(self.get_parameter("max_raw").value)

        self.moving_speed = int(self.get_parameter("moving_speed").value)
        self.publish_period = float(self.get_parameter("publish_period").value)
        self.status_return_level = int(self.get_parameter("status_return_level").value)

        # ==============================
        # MX-28 Protocol 1.0 控制表地址
        # ==============================
        self.PROTOCOL_VERSION = 1.0

        # EEPROM
        self.ADDR_CW_ANGLE_LIMIT = 6
        self.ADDR_CCW_ANGLE_LIMIT = 8
        self.ADDR_STATUS_RETURN_LEVEL = 16
        self.ADDR_RESOLUTION_DIVIDER = 22

        # RAM
        self.ADDR_TORQUE_ENABLE = 24
        self.ADDR_GOAL_POSITION = 30
        self.ADDR_MOVING_SPEED = 32
        self.ADDR_PRESENT_POSITION = 36

        self.TORQUE_DISABLE = 0
        self.TORQUE_ENABLE = 1

        # ==============================
        # 打开串口
        # ==============================
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            raise RuntimeError(f"无法打开串口: {self.port_name}")

        self.get_logger().info(f"已打开串口: {self.port_name}")

        if not self.port_handler.setBaudRate(self.baud_rate):
            raise RuntimeError(f"无法设置波特率: {self.baud_rate}")

        self.get_logger().info(f"已设置波特率: {self.baud_rate}")

        # ==============================
        # Ping 检查
        # ==============================
        for dxl_id in self.dxl_ids:
            model_number, comm_result, dxl_error = self.packet_handler.ping(
                self.port_handler,
                dxl_id
            )
            self.check_result(comm_result, dxl_error, f"ping ID={dxl_id}")

            self.get_logger().info(
                f"找到 MX-28: ID={dxl_id}, model_number={model_number}"
            )

        # ==============================
        # 初始化电机
        # ==============================
        for dxl_id in self.dxl_ids:
            self.write1(
                dxl_id,
                self.ADDR_TORQUE_ENABLE,
                self.TORQUE_DISABLE,
                "torque off"
            )

            self.write1(
                dxl_id,
                self.ADDR_STATUS_RETURN_LEVEL,
                self.status_return_level,
                "set status return level"
            )

            if self.configure_multiturn:
                # Multi-turn Mode:
                # CW Angle Limit = 4095
                # CCW Angle Limit = 4095
                self.write2(
                    dxl_id,
                    self.ADDR_CW_ANGLE_LIMIT,
                    4095,
                    "set CW angle limit"
                )
                self.write2(
                    dxl_id,
                    self.ADDR_CCW_ANGLE_LIMIT,
                    4095,
                    "set CCW angle limit"
                )

                self.write1(
                    dxl_id,
                    self.ADDR_RESOLUTION_DIVIDER,
                    self.resolution_divider,
                    "set resolution divider"
                )

            self.write2(
                dxl_id,
                self.ADDR_MOVING_SPEED,
                self.moving_speed,
                "set moving speed"
            )

            self.write1(
                dxl_id,
                self.ADDR_TORQUE_ENABLE,
                self.TORQUE_ENABLE,
                "torque on"
            )

        self.get_logger().info("所有 MX-28 初始化完成。")
        self.get_logger().info(
            f"raw 安全范围: [{self.min_raw}, {self.max_raw}], "
            f"resolution_divider={self.resolution_divider}"
        )

        # ==============================
        # ROS 2 topic
        # ==============================

        # 绝对 raw 位置控制
        # 例如 [4096, 0, 0, 0] 表示第一个电机目标 raw=4096
        self.goal_raw_sub = self.create_subscription(
            Int32MultiArray,
            "/dxl_goal_raw",
            self.goal_raw_callback,
            10
        )

        # 增量 raw 控制
        # 例如 [100, 0, 0, 0] 表示第一个电机在当前 raw 基础上 +100
        self.delta_raw_sub = self.create_subscription(
            Int32MultiArray,
            "/dxl_delta_raw",
            self.delta_raw_callback,
            10
        )

        # 当前 raw 位置反馈
        self.present_raw_pub = self.create_publisher(
            Int32MultiArray,
            "/dxl_present_raw",
            10
        )

        self.timer = self.create_timer(self.publish_period, self.timer_callback)

        self.get_logger().info("订阅 /dxl_goal_raw")
        self.get_logger().info("订阅 /dxl_delta_raw")
        self.get_logger().info("发布 /dxl_present_raw")

    def check_result(self, comm_result, dxl_error, action_name):
        if comm_result != COMM_SUCCESS:
            msg = self.packet_handler.getTxRxResult(comm_result)
            raise RuntimeError(f"{action_name} 通信失败: {msg}")

        if dxl_error != 0:
            msg = self.packet_handler.getRxPacketError(dxl_error)
            raise RuntimeError(f"{action_name} 电机返回错误: {msg}")

    def write1(self, dxl_id, address, value, action_name):
        """
        写 1 字节，使用 TxOnly，避免 Protocol 1.0 多电机时返回包干扰。
        """
        comm_result = self.packet_handler.write1ByteTxOnly(
            self.port_handler,
            dxl_id,
            address,
            int(value)
        )

        time.sleep(0.01)
        self.clear_port()

        if comm_result != COMM_SUCCESS:
            msg = self.packet_handler.getTxRxResult(comm_result)
            raise RuntimeError(f"{action_name}, ID={dxl_id} 通信失败: {msg}")

    def write2(self, dxl_id, address, value, action_name):
        """
        写 2 字节。

        Multi-turn raw 可能是负数。
        Dynamixel Protocol 1.0 实际发送的是 16-bit。
        所以这里用 & 0xFFFF 转成补码形式。
        """
        value_u16 = int(value) & 0xFFFF

        comm_result = self.packet_handler.write2ByteTxOnly(
            self.port_handler,
            dxl_id,
            address,
            value_u16
        )

        time.sleep(0.01)
        self.clear_port()

        if comm_result != COMM_SUCCESS:
            msg = self.packet_handler.getTxRxResult(comm_result)
            raise RuntimeError(f"{action_name}, ID={dxl_id} 通信失败: {msg}")

    def clear_port(self):
        try:
            self.port_handler.clearPort()
        except Exception:
            pass

    def read2(self, dxl_id, address, action_name):
        value_u16, comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
            self.port_handler,
            dxl_id,
            address
        )
        self.check_result(comm_result, dxl_error, f"{action_name}, ID={dxl_id}")
        return value_u16

    @staticmethod
    def u16_to_i16(value):
        """
        unsigned 16-bit -> signed 16-bit。

        Multi-turn 模式下：
        65535 应该理解为 -1，
        61440 应该理解为 -4096。
        """
        value = int(value) & 0xFFFF
        if value >= 32768:
            value -= 65536
        return value

    def read_present_raw_signed(self, dxl_id):
        raw_u16 = self.read2(
            dxl_id,
            self.ADDR_PRESENT_POSITION,
            "read present position"
        )
        return self.u16_to_i16(raw_u16)

    def clip_raw(self, raw):
        raw = int(raw)

        clipped = max(self.min_raw, min(self.max_raw, raw))

        if clipped != raw:
            self.get_logger().warn(
                f"目标 raw={raw} 超出范围，已限制为 {clipped}"
            )

        return clipped

    @staticmethod
    def i16_to_bytes(value):
        value_u16 = int(value) & 0xFFFF
        return [
            value_u16 & 0xFF,
            (value_u16 >> 8) & 0xFF,
        ]

    def sync_write_goal_raws(self, target_raws):
        if len(target_raws) != len(self.dxl_ids):
            self.get_logger().warn(
                f"/dxl_goal_raw 收到 {len(target_raws)} 个值，"
                f"但 dxl_ids 有 {len(self.dxl_ids)} 个：{self.dxl_ids}"
            )
            return

        group = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            self.ADDR_GOAL_POSITION,
            2
        )

        clipped_targets = []

        for dxl_id, raw in zip(self.dxl_ids, target_raws):
            target_raw = self.clip_raw(raw)
            clipped_targets.append(target_raw)

            if not group.addParam(dxl_id, self.i16_to_bytes(target_raw)):
                self.get_logger().error(
                    f"GroupSyncWrite addParam 失败: ID={dxl_id}, raw={target_raw}"
                )
                group.clearParam()
                return

        comm_result = group.txPacket()

        if comm_result != COMM_SUCCESS:
            msg = self.packet_handler.getTxRxResult(comm_result)
            self.get_logger().error(f"GroupSyncWrite 发送失败: {msg}")
        else:
            self.get_logger().info(
                f"已同步写入目标 raw: {dict(zip(self.dxl_ids, clipped_targets))}"
            )

        group.clearParam()

    def goal_raw_callback(self, msg):
        goals = [int(x) for x in msg.data]
        self.sync_write_goal_raws(goals)

    def delta_raw_callback(self, msg):
        deltas = list(msg.data)

        if len(deltas) != len(self.dxl_ids):
            self.get_logger().warn(
                f"/dxl_delta_raw 收到 {len(deltas)} 个值，"
                f"但 dxl_ids 有 {len(self.dxl_ids)} 个：{self.dxl_ids}"
            )
            return

        try:
            current_raws = [
                self.read_present_raw_signed(dxl_id)
                for dxl_id in self.dxl_ids
            ]
        except RuntimeError as e:
            self.get_logger().warn(f"读取当前位置失败，无法做增量控制：{e}")
            return

        goals = [
            current_raws[i] + int(deltas[i])
            for i in range(len(self.dxl_ids))
        ]

        msg_goal = Int32MultiArray()
        msg_goal.data = goals
        self.goal_raw_callback(msg_goal)

    def timer_callback(self):
        present_raws = []

        for dxl_id in self.dxl_ids:
            try:
                raw = self.read_present_raw_signed(dxl_id)
                present_raws.append(int(raw))

            except RuntimeError as e:
                self.get_logger().warn(str(e))
                # 用一个很特殊的值表示这次读取失败
                present_raws.append(2147483647)

        msg = Int32MultiArray()
        msg.data = present_raws
        self.present_raw_pub.publish(msg)

    def shutdown_dynamixel(self):
        for dxl_id in self.dxl_ids:
            try:
                self.write1(
                    dxl_id,
                    self.ADDR_TORQUE_ENABLE,
                    self.TORQUE_DISABLE,
                    "torque off"
                )
                self.get_logger().info(f"ID={dxl_id} 已关闭 torque")
            except Exception as e:
                self.get_logger().warn(f"ID={dxl_id} 关闭 torque 失败: {e}")

        try:
            self.port_handler.closePort()
            self.get_logger().info("已关闭串口")
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = MX28RawNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f"节点异常: {e}")

    finally:
        if node is not None:
            node.shutdown_dynamixel()
            node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":
    main()
