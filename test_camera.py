import argparse
import time
from pathlib import Path

import cv2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-index", type=int, default=0)
    parser.add_argument("--output-dir", default="camera_test")
    parser.add_argument("--warmup-frames", type=int, default=10)
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--no-preview", action="store_true")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(args.camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开摄像头 index={args.camera_index}")

    if args.width is not None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height is not None:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    for _ in range(max(args.warmup_frames, 0)):
        cap.read()

    print(f"摄像头 index={args.camera_index} 已打开")

    shot_count = 0
    preview_available = not args.no_preview

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("摄像头读取失败")

            if not preview_available:
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                image_path = output_dir / f"camera_{timestamp}.jpg"
                cv2.imwrite(str(image_path), frame)
                print(f"已保存: {image_path}")
                break

            try:
                cv2.imshow("camera test", frame)
            except cv2.error as e:
                print("当前 OpenCV 不支持预览窗口，改为直接保存一张图片。")
                print(f"OpenCV error: {e}")
                preview_available = False
                continue

            print("按空格保存图片，按 q 或 Esc 退出", end="\r")
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break

            if key == ord(" "):
                shot_count += 1
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                image_path = output_dir / f"camera_{timestamp}_{shot_count:03d}.jpg"

                if not cv2.imwrite(str(image_path), frame):
                    raise RuntimeError(f"图片保存失败: {image_path}")

                print(f"已保存: {image_path}")

    finally:
        cap.release()
        if preview_available:
            try:
                cv2.destroyAllWindows()
            except cv2.error:
                pass


if __name__ == "__main__":
    main()
