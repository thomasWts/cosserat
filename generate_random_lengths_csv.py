import argparse
import csv
import random
from pathlib import Path


def clipped_step(value: int, min_raw: int, max_raw: int, max_delta: int) -> int:
    low = max(min_raw, value - max_delta)
    high = min(max_raw, value + max_delta)
    return random.randint(low, high)


def generate_rows(
    num_rows: int,
    num_rods: int,
    min_raw: int,
    max_raw: int,
    max_delta: int,
) -> list[list[int]]:
    if num_rows <= 0:
        return []

    current = [random.randint(min_raw, max_raw) for _ in range(num_rods)]
    rows = [current.copy()]

    for _ in range(num_rows - 1):
        current = [
            clipped_step(value, min_raw, max_raw, max_delta)
            for value in current
        ]
        rows.append(current.copy())

    return rows


def validate_rows(rows: list[list[int]], min_raw: int, max_raw: int, max_delta: int):
    for row_index, row in enumerate(rows):
        for rod_index, value in enumerate(row):
            if not min_raw <= value <= max_raw:
                raise ValueError(
                    f"row {row_index}, rod {rod_index + 1}: {value} out of range"
                )

    for row_index in range(1, len(rows)):
        previous = rows[row_index - 1]
        current = rows[row_index]

        for rod_index, (prev_value, value) in enumerate(zip(previous, current)):
            delta = abs(value - prev_value)
            if delta > max_delta:
                raise ValueError(
                    f"row {row_index}, rod {rod_index + 1}: delta {delta} > {max_delta}"
                )


def write_csv(path: Path, rows: list[list[int]], num_rods: int):
    path.parent.mkdir(parents=True, exist_ok=True)

    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([f"rod_{i + 1}" for i in range(num_rods)])
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="random_lengths.csv")
    parser.add_argument("--rows", type=int, default=100)
    parser.add_argument("--rods", type=int, default=6)
    parser.add_argument("--min-raw", type=int, default=-4096)
    parser.add_argument("--max-raw", type=int, default=4096)
    parser.add_argument("--max-delta", type=int, default=500)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    if args.seed is not None:
        random.seed(args.seed)

    rows = generate_rows(
        num_rows=args.rows,
        num_rods=args.rods,
        min_raw=args.min_raw,
        max_raw=args.max_raw,
        max_delta=args.max_delta,
    )
    validate_rows(rows, args.min_raw, args.max_raw, args.max_delta)
    write_csv(Path(args.output), rows, args.rods)

    print(f"generated {len(rows)} rows -> {args.output}")


if __name__ == "__main__":
    main()
