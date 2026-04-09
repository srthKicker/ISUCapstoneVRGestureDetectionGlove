"""
parse_gesture_csv.py
--------------------
Converts raw WebUI-exported CSVs into clean, per-gesture CSV files
ready for 1D-CNN training.

WebUI CSV format (one row per recording):
    Gesture,"w,x,y,z,w,x,y,z,...  w,x,y,z,w,x,y,z,..."

Each serial line contains 6 quaternions (one per IMU) as:
    w0,x0,y0,z0,w1,x1,y1,z1,...,w5,x5,y5,z5,
Printed every ~10ms. A 3-second recording = ~300 rows.

Output (one CSV per recording, saved to output_dir/):
    timestamp_idx, imu0_w, imu0_x, imu0_y, imu0_z,
                   imu1_w, imu1_x, imu1_y, imu1_z,
                   ...
                   imu5_w, imu5_x, imu5_y, imu5_z

Usage:
    python parse_gesture_csv.py --input session.csv --output_dir data/
    python parse_gesture_csv.py --input *.csv --output_dir data/  (glob supported)
"""

import argparse
import csv
import os
import glob
import numpy as np
from pathlib import Path

# ── Constants ────────────────────────────────────────────────────────────────
NUM_IMUS = 6
VALS_PER_IMU = 4          # w, x, y, z
VALS_PER_LINE = NUM_IMUS * VALS_PER_IMU   # 24 floats per serial line
TIMESTEPS = 300           # 3 seconds @ 100Hz (10ms loop). Crop/pad to this.
# ─────────────────────────────────────────────────────────────────────────────

COLUMN_NAMES = ["timestep"] + [
    f"imu{i}_{c}" for i in range(NUM_IMUS) for c in ["w", "x", "y", "z"]
]


def parse_serial_line(line: str):
    """
    Parse one serial line of 24 comma-separated floats into a numpy row.
    The ESP32 prints a trailing comma, so we strip empties.
    Returns shape (24,) float32 array, or None if line is malformed.
    """
    parts = [p.strip() for p in line.split(",") if p.strip()]
    if len(parts) < VALS_PER_LINE:
        return None
    try:
        return np.array(parts[:VALS_PER_LINE], dtype=np.float32)
    except ValueError:
        return None


def parse_webui_csv(filepath: str):
    """
    Read a WebUI-exported CSV. Returns list of (gesture_label, np.ndarray)
    where ndarray has shape (T, 24) — T raw timesteps before cropping.
    """
    recordings = []

    with open(filepath, newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader, None)  # skip "Gesture,Data" header

        for row in reader:
            if len(row) < 2:
                continue

            gesture_label = row[0].strip().strip('"')
            raw_data_str  = row[1].strip().strip('"')

            # The WebUI joins serial lines with spaces; split them back out.
            # Each "serial line" is a group of 24 comma-separated numbers.
            # Strategy: split the whole blob by commas, chunk into groups of 24.
            all_values = [v.strip() for v in raw_data_str.replace("\n", " ").split(",") if v.strip()]

            rows = []
            for i in range(0, len(all_values) - VALS_PER_LINE + 1, VALS_PER_LINE):
                chunk = all_values[i : i + VALS_PER_LINE]
                try:
                    row_vals = np.array(chunk, dtype=np.float32)
                    rows.append(row_vals)
                except ValueError:
                    continue  # skip malformed chunks

            if len(rows) == 0:
                print(f"  [WARN] No valid rows in recording: gesture={gesture_label}")
                continue

            data = np.stack(rows)   # shape (T, 24)
            recordings.append((gesture_label, data))

    return recordings


def crop_or_pad(data: np.ndarray, timesteps: int = TIMESTEPS) -> np.ndarray:
    """
    Crop to first `timesteps` rows, or zero-pad if shorter.
    Shape in:  (T, 24)
    Shape out: (timesteps, 24)
    """
    T = data.shape[0]
    if T >= timesteps:
        return data[:timesteps]
    else:
        pad = np.zeros((timesteps - T, data.shape[1]), dtype=np.float32)
        return np.vstack([data, pad])


def save_recording(gesture_label: str, data: np.ndarray, output_dir: str, index: int):
    """Save one recording as its own CSV file."""
    os.makedirs(output_dir, exist_ok=True)
    filename = f"{gesture_label}_{index:04d}.csv"
    filepath = os.path.join(output_dir, filename)

    with open(filepath, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(COLUMN_NAMES)
        for t, row in enumerate(data):
            writer.writerow([t] + row.tolist())

    return filepath


def process_files(input_pattern: str, output_dir: str, timesteps: int = TIMESTEPS):
    """Main entry point: process all matching input CSVs."""
    files = glob.glob(input_pattern)
    if not files:
        print(f"No files matched: {input_pattern}")
        return

    global_index = 0
    label_counts = {}

    for filepath in files:
        print(f"\nProcessing: {filepath}")
        recordings = parse_webui_csv(filepath)

        for gesture_label, data in recordings:
            T_raw = data.shape[0]
            data_fixed = crop_or_pad(data, timesteps)

            out_path = save_recording(gesture_label, data_fixed, output_dir, global_index)
            global_index += 1
            label_counts[gesture_label] = label_counts.get(gesture_label, 0) + 1

            print(f"  [{gesture_label}] raw={T_raw} timesteps → saved to {out_path}")

    print(f"\n✓ Done. {global_index} recordings saved to '{output_dir}/'")
    print("  Label distribution:")
    for label, count in sorted(label_counts.items()):
        print(f"    {label}: {count}")


# ── CLI ───────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Parse WebUI gesture CSVs for 1D-CNN training.")
    parser.add_argument("--input",       default="*.csv",  help="Input CSV file or glob pattern (default: *.csv)")
    parser.add_argument("--output_dir",  default="data",   help="Output directory for per-gesture CSVs (default: data/)")
    parser.add_argument("--timesteps",   type=int, default=TIMESTEPS, help=f"Fixed timestep length (default: {TIMESTEPS})")
    args = parser.parse_args()

    process_files(args.input, args.output_dir, args.timesteps)
