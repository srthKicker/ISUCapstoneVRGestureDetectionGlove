"""
parse_gesture_csv.py
--------------------
Converts WebUI session CSVs into individual per-recording CSV files
ready for dataLoader.py and the 1D-CNN.

Your session CSV already has this clean format (one row per timestep):
    gesture, recording_id, timestep, imu0_w, imu0_x, ... imu5_z

This script:
  1. Reads one or more session CSVs
  2. Splits them into one file per recording
  3. Crops/pads every recording to a fixed TIMESTEPS length
  4. Saves them to data/ as GestureLabel_NNNN.csv

Usage:
    python parse_gesture_csv.py
        (reads all gesture_data_*.csv in the current folder)

    python parse_gesture_csv.py --input my_session.csv
    python parse_gesture_csv.py --input "gesture_data_*.csv" --output_dir data/
"""

import argparse
import glob
import os
import pandas as pd
import numpy as np

# ── Config ────────────────────────────────────────────────────────────────────
TIMESTEPS = 230      # rows per recording -- matches your actual ~230 row captures
NUM_IMUS  = 6
# -----------------------------------------------------------------------------

FEATURE_COLS = [
    f"imu{i}_{c}" for i in range(NUM_IMUS) for c in ["w", "x", "y", "z"]
]


def crop_or_pad(df, timesteps):
    """
    Ensure every recording is exactly `timesteps` rows.
    - If longer  -> keep only the first `timesteps` rows
    - If shorter -> pad with zeros to reach `timesteps` rows
    """
    if len(df) >= timesteps:
        return df.iloc[:timesteps].reset_index(drop=True)
    else:
        pad_rows = timesteps - len(df)
        pad_df = pd.DataFrame(
            np.zeros((pad_rows, len(df.columns))),
            columns=df.columns
        )
        return pd.concat([df, pad_df], ignore_index=True)


def process(input_pattern, output_dir, timesteps):
    files = glob.glob(input_pattern)
    if not files:
        print(f"No files found matching: {input_pattern}")
        return

    os.makedirs(output_dir, exist_ok=True)

    saved        = 0
    skipped      = 0
    label_counts = {}

    for filepath in sorted(files):
        print(f"\nReading: {filepath}")
        try:
            session = pd.read_csv(filepath)
        except Exception as e:
            print(f"  Could not read file: {e}")
            skipped += 1
            continue

        # Validate expected columns exist
        missing = [c for c in FEATURE_COLS if c not in session.columns]
        if missing:
            print(f"  Missing columns {missing} -- skipping")
            skipped += 1
            continue

        if 'gesture' not in session.columns or 'recording_id' not in session.columns:
            print(f"  Missing gesture or recording_id column -- skipping")
            skipped += 1
            continue

        # Split into individual recordings
        for (gesture, rec_id), group in session.groupby(['gesture', 'recording_id']):
            raw_rows = len(group)
            data = group[FEATURE_COLS].reset_index(drop=True)
            data = crop_or_pad(data, timesteps)

            # Add timestep column
            data.insert(0, 'timestep', range(timesteps))

            # Save as GestureLabel_NNNN.csv
            filename = f"{gesture}_{saved:04d}.csv"
            out_path = os.path.join(output_dir, filename)
            data.to_csv(out_path, index=False)

            label_counts[gesture] = label_counts.get(gesture, 0) + 1
            print(f"  [{gesture}] rec_id={rec_id}  raw={raw_rows} rows  ->  {out_path}")
            saved += 1

    print(f"\n{'='*50}")
    print(f"Done. {saved} recordings saved to '{output_dir}/'")
    if skipped:
        print(f"  ({skipped} files skipped)")
    print("\nLabel distribution:")
    for label, count in sorted(label_counts.items()):
        print(f"  {label}: {count} recordings")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        default="gesture_data_*.csv",
        help="Session CSV file or glob pattern (default: gesture_data_*.csv)"
    )
    parser.add_argument(
        "--output_dir",
        default="data",
        help="Output folder for per-recording CSVs (default: data/)"
    )
    parser.add_argument(
        "--timesteps",
        type=int,
        default=TIMESTEPS,
        help=f"Fixed length per recording (default: {TIMESTEPS})"
    )
    args = parser.parse_args()
    process(args.input, args.output_dir, args.timesteps)