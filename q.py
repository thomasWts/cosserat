import cv2
import numpy as np

ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,

    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,

    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,

    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,

    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def main():
    cap = cv2.VideoCapture(1)

    if not cap.isOpened():
        print("无法打开摄像头")
        return

    parameters = cv2.aruco.DetectorParameters()

    print("开始检测，按 q 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法读取摄像头画面")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detected_any = False

        for dict_name, dict_id in ARUCO_DICTS.items():
            aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
            detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

            corners, ids, rejected = detector.detectMarkers(gray)

            if ids is not None:
                detected_any = True

                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

                for marker_id in ids.flatten():
                    print(f"检测到：字典 = {dict_name}, ID = {marker_id}")

                cv2.putText(
                    frame,
                    f"{dict_name}, ID={ids.flatten().tolist()}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )

                # 如果已经检测到了，就不继续扫其他字典
                break

        if not detected_any:
            cv2.putText(
                frame,
                "No ArUco/AprilTag detected",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 0, 255),
                2,
            )

        cv2.imshow("Auto ArUco Dictionary Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
