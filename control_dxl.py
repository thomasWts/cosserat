import argparse
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray


class DXLController(Node):
    def __init__(self, dxl_ids: List[int]):
        super().__init__("dxl_controller_client")

        self.dxl_ids = dxl_ids
        self.present_raw: Optional[List[int]] = None

        # 发布绝对 raw 目标
        self.goal_pub = self.create_publisher(
            Int32MultiArray,
            "/dxl_goal_raw",
            10
        )

        # 发布增量 raw 目标
        self.delta_pub = self.create_publisher(
            Int32MultiArray,
            "/dxl_delta_raw",
            10
        )

        # 订阅当前 raw 反馈
        self.present_sub = self.create_subscription(
            Int32MultiArray,
            "/dxl_present_raw",
            self.present_callback,
            10
        )

    def present_callback(self, msg: Int32MultiArray):
        self.present_raw = list(msg.data)

    def wait_for_low_level_node(self, timeout: float = 5.0) -> bool:
        """
        等待底层 mx28_raw_sync_node 连接。
        """
        start = time.time()

        while rclpy.ok():
            goal_sub_count = self.goal_pub.get_subscription_count()
            delta_sub_count = self.delta_pub.get_subscription_count()

            if goal_sub_count > 0 and delta_sub_count > 0:
                print("已经连接到底层 Dynamixel 控制节点")
                return True

            if time.time() - start > timeout:
                print("等待底层控制节点超时，请确认 mx28_raw_sync_node 是否正在运行")
                return False

            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_present_raw(self, timeout: float = 5.0) -> Optional[List[int]]:
        """
        等待收到一次当前位置反馈。
        """
        start = time.time()

        while rclpy.ok():
            if self.present_raw is not None:
                return self.present_raw

            if time.time() - start > timeout:
                print("等待 /dxl_present_raw 超时")
                return None

            rclpy.spin_once(self, timeout_sec=0.1)

    def spin_once(self, timeout: float = 0.1):
        """
        手动更新一次反馈。
        """
        rclpy.spin_once(self, timeout_sec=timeout)

    def get_present_raw(self) -> Optional[List[int]]:
        """
        获取最近一次 raw 反馈。
        """
        self.spin_once(0.1)
        return self.present_raw

    def _check_length(self, values: List[int]):
        if len(values) != len(self.dxl_ids):
            raise ValueError(
                f"输入长度为 {len(values)}，但电机数量为 {len(self.dxl_ids)}，"
                f"dxl_ids={self.dxl_ids}"
            )

    def _publish(self, publisher, values: List[int]):
        msg = Int32MultiArray()
        msg.data = [int(v) for v in values]

        # 为了避免刚创建 publisher 后第一帧丢失，这里连续发两次
        publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.05)
        publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.05)

    def set_goal_raw(self, goal_raws: List[int]):
        """
        绝对 raw 控制。

        如果 dxl_ids = [1, 2, 4, 6]
        set_goal_raw([4, 2702, 2696, 800])

        表示：
        ID=1 -> 4
        ID=2 -> 2702
        ID=4 -> 2696
        ID=6 -> 800
        """
        self._check_length(goal_raws)
        self._publish(self.goal_pub, goal_raws)

        print("发送绝对 raw 目标：")
        for dxl_id, raw in zip(self.dxl_ids, goal_raws):
            print(f"  ID={dxl_id}: {raw}")

    def move_delta_raw(self, delta_raws: List[int]):
        """
        增量 raw 控制。

        如果 dxl_ids = [1, 2, 4, 6]
        move_delta_raw([100, 0, 0, -100])

        表示：
        ID=1 当前基础上 +100
        ID=2 不动
        ID=4 不动
        ID=6 当前基础上 -100
        """
        self._check_length(delta_raws)
        self._publish(self.delta_pub, delta_raws)

        print("发送增量 raw：")
        for dxl_id, delta in zip(self.dxl_ids, delta_raws):
            print(f"  ID={dxl_id}: {delta:+d}")

    def move_one_delta_raw(self, dxl_id: int, delta_raw: int):
        """
        只让一个电机做增量运动。
        """
        if dxl_id not in self.dxl_ids:
            raise ValueError(f"dxl_id={dxl_id} 不在 {self.dxl_ids} 中")

        delta_raws = [0 for _ in self.dxl_ids]
        index = self.dxl_ids.index(dxl_id)
        delta_raws[index] = int(delta_raw)

        self.move_delta_raw(delta_raws)

    def set_one_goal_raw(self, dxl_id: int, goal_raw: int):
        """
        只设置一个电机的绝对 raw 位置。
        其他电机保持当前反馈位置不变。
        """
        if dxl_id not in self.dxl_ids:
            raise ValueError(f"dxl_id={dxl_id} 不在 {self.dxl_ids} 中")

        current = self.wait_for_present_raw(timeout=3.0)

        if current is None:
            raise RuntimeError("没有收到当前位置反馈，无法保持其他电机不动")

        goal_raws = list(current)
        index = self.dxl_ids.index(dxl_id)
        goal_raws[index] = int(goal_raw)

        self.set_goal_raw(goal_raws)


def parse_int_list(text: str) -> List[int]:
    return [int(x.strip()) for x in text.split(",") if x.strip()]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ids",
        default="1,2,3,4,5,6",
        help="Comma-separated Dynamixel IDs, order must match mx28_raw_sync_node.",
    )
    parser.add_argument(
        "--goal",
        help="Absolute raw targets, comma-separated. Example: 0,0,0,0,0,0",
    )
    parser.add_argument(
        "--delta",
        help="Relative raw deltas, comma-separated. Example: 100,0,0,0,0,0",
    )
    parser.add_argument("--one-id", type=int, help="Move only one Dynamixel ID.")
    parser.add_argument("--one-goal", type=int, help="Absolute raw target for --one-id.")
    parser.add_argument("--one-delta", type=int, help="Relative raw delta for --one-id.")
    args = parser.parse_args()

    rclpy.init()

    # 这里要和你启动 mx28_raw_sync_node 时的 dxl_ids 顺序一致
    controller = DXLController(dxl_ids=parse_int_list(args.ids))

    if not controller.wait_for_low_level_node():
        controller.destroy_node()
        rclpy.shutdown()
        return

    current = controller.wait_for_present_raw()
    print("当前 raw:", current)

    command_count = sum(
        value is not None
        for value in [args.goal, args.delta, args.one_goal, args.one_delta]
    )

    if command_count > 1:
        raise ValueError("一次只传一种控制命令：--goal/--delta/--one-goal/--one-delta")

    if args.goal is not None:
        controller.set_goal_raw(parse_int_list(args.goal))
    elif args.delta is not None:
        controller.move_delta_raw(parse_int_list(args.delta))
    elif args.one_goal is not None:
        if args.one_id is None:
            raise ValueError("--one-goal 需要同时传 --one-id")
        controller.set_one_goal_raw(args.one_id, args.one_goal)
    elif args.one_delta is not None:
        if args.one_id is None:
            raise ValueError("--one-delta 需要同时传 --one-id")
        controller.move_one_delta_raw(args.one_id, args.one_delta)
    else:
        print("未发送运动命令。使用 --delta 或 --goal 才会控制电机。")

    time.sleep(2.0)
    controller.spin_once()
    print("运动后 raw:", controller.get_present_raw())

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
