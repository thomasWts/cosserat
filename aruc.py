import cv2
import numpy as np


def main():
    # 0 通常表示默认摄像头
    # 如果你有多个摄像头，可以改成 1、2 试试
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("无法打开摄像头，请检查摄像头是否连接，或者尝试把 VideoCapture(0) 改成 1")
        return

    # 选择 ArUco 字典
    # 注意：这里必须和你打印/生成的 ArUco 标记字典一致
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)

    # 创建检测参数
    parameters = cv2.aruco.DetectorParameters()

    # 新版 OpenCV 推荐使用 ArucoDetector
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    print("开始识别 ArUco，按 q 退出")

    while True:
        ret, frame = cap.read()

        if not ret:
            print("无法读取摄像头画面")
            break

        # 转灰度图，检测更稳定
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 检测 ArUco
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            # 在图像上画出检测到的 ArUco 边框和 ID
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for marker_corners, marker_id in zip(corners, ids.flatten()):
                # marker_corners 形状大概是 (1, 4, 2)
                pts = marker_corners.reshape((4, 2))

                # 四个角点
                top_left, top_right, bottom_right, bottom_left = pts

                # 计算中心点
                center_x = int(np.mean(pts[:, 0]))
                center_y = int(np.mean(pts[:, 1]))

                # 在中心点画一个圆
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

                # 显示 ID 和中心坐标
                text = f"ID:{marker_id} ({center_x},{center_y})"
                cv2.putText(
                    frame,
                    text,
                    (center_x + 10, center_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 0, 255),
                    2
                )

                print(f"检测到 ArUco ID: {marker_id}, 中心坐标: ({center_x}, {center_y})")

        cv2.imshow("ArUco Detection", frame)

        # 按 q 退出
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()