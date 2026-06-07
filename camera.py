import cv2
import numpy as np


class ArucoPoseDetector:
    def __init__(self):
        # ==========================
        # 相机内参矩阵
        # 你给的 camera_matrix
        # ==========================
        self.camera_matrix = np.array([
            [385.2354, 0, 316.67],
            [0, 385.0368, 239.43],
            [0, 0, 1]
        ], dtype=np.float64)

        # ==========================
        # 相机畸变参数
        # 如果你没有标定出来的畸变参数，先用 0 代替
        # 后续如果有 dist_coeffs，可以替换这里
        # ==========================
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # ==========================
        # ArUco 参数
        # ==========================
        self.target_ids = set(range(12))  # 只识别 ID 0~11

        # 这里要和你生成/打印 ArUco 的字典一致
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_5X5_50
        )

        self.parameters = cv2.aruco.DetectorParameters()
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict,
            self.parameters
        )

        # ==========================
        # ArUco 实际边长，单位：米
        # 例如你打印的码边长是 2 cm，就写 0.02
        # 这个值会直接影响 tvec 的尺度
        # ==========================
        self.marker_length = 0.02

        half = self.marker_length / 2.0

        # ArUco 四个角点在自身坐标系下的 3D 坐标
        # 顺序要和 OpenCV 检测出来的角点顺序一致：
        # 左上、右上、右下、左下
        self.object_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

    @staticmethod
    def rotation_matrix_to_euler_angles(rotation_matrix):
        sy = np.sqrt(
            rotation_matrix[0, 0] * rotation_matrix[0, 0]
            + rotation_matrix[1, 0] * rotation_matrix[1, 0]
        )
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])
            pitch = np.arctan2(-rotation_matrix[2, 0], sy)
            yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
        else:
            roll = np.arctan2(-rotation_matrix[1, 2], rotation_matrix[1, 1])
            pitch = np.arctan2(-rotation_matrix[2, 0], sy)
            yaw = 0.0

        return np.degrees([roll, pitch, yaw])

    def detect(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self.detector.detectMarkers(gray)

        results = []

        if ids is None:
            return frame, results

        ids_flatten = ids.flatten()

        valid_corners = []
        valid_ids = []

        for marker_corners, marker_id in zip(corners, ids_flatten):
            if marker_id in self.target_ids:
                valid_corners.append(marker_corners)
                valid_ids.append(marker_id)

        if len(valid_ids) == 0:
            return frame, results

        valid_ids_np = np.array(valid_ids, dtype=np.int32).reshape(-1, 1)

        # 画出检测到的 ArUco 边框
        cv2.aruco.drawDetectedMarkers(frame, valid_corners, valid_ids_np)

        for marker_corners, marker_id in zip(valid_corners, valid_ids):
            image_points = marker_corners.reshape(4, 2).astype(np.float32)

            # 用 solvePnP 求 ArUco 相对于相机的姿态
            success, rvec, tvec = cv2.solvePnP(
                self.object_points,
                image_points,
                self.camera_matrix,
                self.dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE
            )

            if not success:
                continue

            # tvec 是 ArUco 坐标系原点相对于相机坐标系的位置
            x, y, z = tvec.flatten()

            # rvec 描述 ArUco 坐标系相对于相机坐标系的旋转
            rotation_matrix, _ = cv2.Rodrigues(rvec)
            roll, pitch, yaw = self.rotation_matrix_to_euler_angles(
                rotation_matrix
            )

            # 画坐标轴
            # 坐标轴长度设置成 marker 边长的一半
            cv2.drawFrameAxes(
                frame,
                self.camera_matrix,
                self.dist_coeffs,
                rvec,
                tvec,
                self.marker_length * 0.5
            )

            # 计算图像中心点
            center_x = int(np.mean(image_points[:, 0]))
            center_y = int(np.mean(image_points[:, 1]))

            cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

            text1 = f"ID={marker_id}"
            text2 = f"x={x:.3f}, y={y:.3f}, z={z:.3f} m"
            text3 = f"r={roll:.1f}, p={pitch:.1f}, y={yaw:.1f} deg"

            cv2.putText(
                frame,
                text1,
                (center_x - 40, center_y - 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )

            cv2.putText(
                frame,
                text2,
                (center_x - 80, center_y + 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                2
            )

            cv2.putText(
                frame,
                text3,
                (center_x - 80, center_y + 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 150, 255),
                2
            )

            results.append({
                "id": int(marker_id),
                "rvec": rvec,
                "tvec": tvec,
                "rotation_matrix": rotation_matrix,
                "position_m": {
                    "x": float(x),
                    "y": float(y),
                    "z": float(z)
                },
                "euler_deg": {
                    "roll": float(roll),
                    "pitch": float(pitch),
                    "yaw": float(yaw)
                }
            })

        return frame, results


def main():
    cap = cv2.VideoCapture(2)

    if not cap.isOpened():
        print("无法打开摄像头，请尝试把 VideoCapture(0) 改成 VideoCapture(1)")
        return

    # 你的相机内参看起来是 640x480 分辨率下的
    # 所以这里固定分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    detector = ArucoPoseDetector()

    print("开始识别 ArUco ID 0~11，按 q 退出")
    print("注意：tvec 的单位取决于 marker_length，这里默认 marker_length=0.02 m")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("无法读取摄像头画面")
            break

        frame, results = detector.detect(frame)

        for item in results:
            marker_id = item["id"]
            pos = item["position_m"]
            euler = item["euler_deg"]

            print(
                f"ID={marker_id}, "
                f"x={pos['x']:.3f} m, "
                f"y={pos['y']:.3f} m, "
                f"z={pos['z']:.3f} m, "
                f"roll={euler['roll']:.1f} deg, "
                f"pitch={euler['pitch']:.1f} deg, "
                f"yaw={euler['yaw']:.1f} deg"
            )

        cv2.imshow("Aruco 0-11 Pose Detection", frame)

        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
