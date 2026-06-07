import argparse
import csv
import time
from pathlib import Path
from typing import List, Optional


def read_targets(path: Path, expected_rods: int) -> List[List[int]]:
    rows: List[List[int]] = []

    with path.open("r", newline="") as f:
        reader = csv.reader(f)

        for line_number, row in enumerate(reader, start=1):
            if not row:
                continue

            try:
                values = [int(cell.strip()) for cell in row]
            except ValueError:
                if line_number == 1:
                    continue
                raise ValueError(f"{path}:{line_number} contains non-integer values")

            if len(values) != expected_rods:
                raise ValueError(
                    f"{path}:{line_number} has {len(values)} values, "
                    f"expected {expected_rods}"
                )

            rows.append(values)

    return rows


def max_abs_error(current: List[int], target: List[int]) -> int:
    return max(abs(c - t) for c, t in zip(current, target))


def parse_int_list(text: str) -> List[int]:
    return [int(x.strip()) for x in text.split(",") if x.strip()]


def add_zero_offset(target: List[int], zero_offset: List[int]) -> List[int]:
    if len(target) != len(zero_offset):
        raise ValueError(
            f"target has {len(target)} values, zero offset has {len(zero_offset)}"
        )

    return [value + offset for value, offset in zip(target, zero_offset)]


class CameraCapture:
    def __init__(self, camera_index: int, image_dir: Path, warmup_frames: int):
        import cv2

        self.cv2 = cv2
        self.image_dir = image_dir
        self.image_dir.mkdir(parents=True, exist_ok=True)

        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头 index={camera_index}")

        for _ in range(max(warmup_frames, 0)):
            self.cap.read()

    def save(self, row_number: int) -> Path:
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("摄像头读取失败")

        image_path = self.image_dir / f"row_{row_number:04d}.jpg"
        if not self.cv2.imwrite(str(image_path), frame):
            raise RuntimeError(f"图片保存失败: {image_path}")

        return image_path

    def release(self):
        self.cap.release()


def wait_until_reached(
    controller,
    target: List[int],
    tolerance: int,
    stable_delta: int,
    stable_samples: int,
    timeout: float,
) -> Optional[List[int]]:
    import rclpy

    start = time.time()
    last_current: Optional[List[int]] = None
    stable_count = 0

    while rclpy.ok():
        current = controller.get_present_raw()

        if current is not None:
            in_target_range = (
                len(current) == len(target)
                and max_abs_error(current, target) <= tolerance
            )
            is_still = (
                last_current is not None
                and len(current) == len(last_current)
                and max_abs_error(current, last_current) <= stable_delta
            )

            if in_target_range and is_still:
                stable_count += 1
            else:
                stable_count = 0

            last_current = current

            if stable_count >= stable_samples:
                return current

        if time.time() - start > timeout:
            return last_current

        time.sleep(0.05)

    return last_current


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file", help="CSV file with one raw target row per step.")
    parser.add_argument(
        "--ids",
        default="1,2,3,4,5,6",
        help="Comma-separated Dynamixel IDs. Order must match the CSV columns.",
    )
    parser.add_argument(
        "--zero-offset",
        default="2400,4300,0,-3000,2200,1500",
        help="Raw zero position. CSV values are relative to this offset.",
    )
    parser.add_argument("--dwell", type=float, default=2.0, help="Seconds to stop after arrival.")
    parser.add_argument("--tolerance", type=int, default=30, help="Raw arrival tolerance.")
    parser.add_argument(
        "--stable-delta",
        type=int,
        default=5,
        help="Max raw change between feedback samples before capture.",
    )
    parser.add_argument(
        "--stable-samples",
        type=int,
        default=5,
        help="Number of consecutive stable feedback samples required before capture.",
    )
    parser.add_argument("--timeout", type=float, default=30.0, help="Max seconds to wait per row.")
    parser.add_argument("--start-row", type=int, default=1, help="1-based target row index.")
    parser.add_argument("--max-rows", type=int, help="Only play this many rows.")
    parser.add_argument("--camera-index", type=int, default=2, help="OpenCV camera index.")
    parser.add_argument("--image-dir", default="captures", help="Directory for captured images.")
    parser.add_argument("--warmup-frames", type=int, default=10, help="Camera frames to discard before use.")
    parser.add_argument("--no-camera", action="store_true", help="Play CSV without taking pictures.")
    parser.add_argument("--dry-run", action="store_true", help="Print targets without moving.")
    args = parser.parse_args()

    dxl_ids = parse_int_list(args.ids)
    zero_offset = parse_int_list(args.zero_offset)
    if len(zero_offset) != len(dxl_ids):
        raise ValueError(
            f"--zero-offset has {len(zero_offset)} values, "
            f"but --ids has {len(dxl_ids)} values"
        )

    targets = read_targets(Path(args.csv_file), expected_rods=len(dxl_ids))
    absolute_targets = [
        add_zero_offset(target, zero_offset)
        for target in targets
    ]

    start_index = max(args.start_row - 1, 0)
    relative_targets = targets[start_index:]
    absolute_targets = absolute_targets[start_index:]
    if args.max_rows is not None:
        relative_targets = relative_targets[:args.max_rows]
        absolute_targets = absolute_targets[:args.max_rows]

    if not absolute_targets:
        raise ValueError("No target rows to play")

    if args.dry_run:
        for index, (relative, absolute) in enumerate(
            zip(relative_targets, absolute_targets),
            start=args.start_row,
        ):
            print(f"row {index}: relative={dict(zip(dxl_ids, relative))}")
            print(f"row {index}: absolute={dict(zip(dxl_ids, absolute))}")
        return

    import rclpy

    from control_dxl import DXLController

    rclpy.init()
    controller = DXLController(dxl_ids=dxl_ids)
    camera = None

    try:
        if not args.no_camera:
            camera = CameraCapture(
                camera_index=args.camera_index,
                image_dir=Path(args.image_dir),
                warmup_frames=args.warmup_frames,
            )
            print(f"摄像头已打开，图片保存到: {args.image_dir}")

        if not controller.wait_for_low_level_node():
            return

        current = controller.wait_for_present_raw()
        print("当前 raw:", current)

        for offset, (relative, target) in enumerate(
            zip(relative_targets, absolute_targets)
        ):
            row_number = args.start_row + offset
            print(f"\nrow {row_number}: CSV 相对值 {dict(zip(dxl_ids, relative))}")
            print(f"row {row_number}: 实际发送 raw {dict(zip(dxl_ids, target))}")
            controller.set_goal_raw(target)

            reached = wait_until_reached(
                controller=controller,
                target=target,
                tolerance=args.tolerance,
                stable_delta=args.stable_delta,
                stable_samples=args.stable_samples,
                timeout=args.timeout,
            )

            if reached is None:
                print(f"row {row_number}: 没有收到反馈，停止播放")
                break

            error = max_abs_error(reached, target)
            if error > args.tolerance:
                print(
                    f"row {row_number}: 等待超时，当前 raw={reached}, "
                    f"max_error={error}，停止播放"
                )
                break

            print(f"row {row_number}: 已到达，当前 raw={reached}")

            if camera is not None:
                image_path = camera.save(row_number)
                print(f"row {row_number}: 已拍照 {image_path}")

            print(f"row {row_number}: 停 {args.dwell}s")
            time.sleep(args.dwell)

    finally:
        if camera is not None:
            camera.release()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
