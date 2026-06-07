import cv2
import numpy as np


# ==========================
# 需要识别的 ArUco ID 范围
# ==========================
TARGET_IDS = set(range(12))  # 识别 0,1,2,...,11

# ==========================
# ArUco 字典
# 如果你的码不是 4X4_50，要改这里
# ==========================
ARUCO_DICT_TYPE = cv2.aruco.DICT_4X4_50


def main():
    # 0 是默认摄像头，如果打不开可以改成 1 或 2
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("无法打开摄像头，请尝试把 VideoCapture(0) 改成 VideoCapture(1)")
        return

    # 可选：设置摄像头分辨率
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # 创建 ArUco 字典和检测器
    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_TYPE)
    parameters = cv2.aruco.DetectorParameters()

    # 提高角点检测精度
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    print("开始识别 ArUco ID 0~11，按 q 退出")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("无法读取摄像头画面")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 检测 ArUco
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            ids_flatten = ids.flatten()

            valid_corners = []
            valid_ids = []

            for marker_corners, marker_id in zip(corners, ids_flatten):
                if marker_id in TARGET_IDS:
                    valid_corners.append(marker_corners)
                    valid_ids.append(marker_id)

            if len(valid_ids) > 0:
                valid_ids_np = np.array(valid_ids, dtype=np.int32).reshape(-1, 1)

                # 画出检测到的 0~11 号 ArUco
                cv2.aruco.drawDetectedMarkers(frame, valid_corners, valid_ids_np)

                for marker_corners, marker_id in zip(valid_corners, valid_ids):
                    pts = marker_corners.reshape(4, 2)

                    # 四个角点
                    top_left = pts[0]
                    top_right = pts[1]
                    bottom_right = pts[2]
                    bottom_left = pts[3]

                    # 中心点
                    center_x = int(np.mean(pts[:, 0]))
                    center_y = int(np.mean(pts[:, 1]))

                    # 画中心点
                    cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

                    # 显示 ID 和中心坐标
                    text = f"ID={marker_id}, center=({center_x},{center_y})"
                    cv2.putText(
                        frame,
                        text,
                        (center_x - 40, center_y - 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 0, 255),
                        2
                    )

                    print(f"检测到 ArUco ID={marker_id}, 中心坐标=({center_x}, {center_y})")

        cv2.imshow("Detect ArUco 0-11", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()