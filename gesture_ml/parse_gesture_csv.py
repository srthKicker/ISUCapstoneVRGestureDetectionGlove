"""
parse_gesture_csv.py
--------------------
Converts raw session CSVs (output of the WebUI index.html) into the
per-recording CSVs expected by dataLoader.py.

Expected input structure:
    dataRaw/
        Left Hand/
            50Hz/
                gesture_data_*.csv
            90Hz/
                gesture_data_*.csv
        RightHand/
            50Hz/
                gesture_data_*.csv
            90Hz/
                gesture_data_*.csv

Output structure:
    data/
        left_hand/
            Base_0.csv
            Base_1.csv
            Gesture1_0.csv
            ...
        right_hand/
            Base_0.csv
            ...

Input CSV format (one row per timestep, all recordings concatenated):
    gesture, recording_id, timestep, imu0_w, imu0_x, ..., imu5_z

Output CSV format (one file per recording, 300 rows resampled):
    timestep, imu0_w, imu0_x, ..., imu5_z

Gesture labels found in data:
    Base, Gesture1, Gesture2, Gesture3, Gesture4

Usage:
    python parse_gesture_csv.py
"""

import os
import glob
import pandas as pd
import numpy as np
from scipy.interpolate import interp1d

# ── CONFIG ────────────────────────────────────────────────────────────────────

# Root of the raw data — change this if your repo is in a different location
DATA_RAW_DIR = "/media/srth/GamerDisk/VSCodeProjects/ISUCapstoneVRGestureDetectionGlove/gesture_ml/dataRaw"

# Input folder name → output subfolder name
DATASETS = {
    "Left Hand": "left_hand",
    "RightHand": "right_hand",
}

# Base output directory (sits next to this script inside gesture_ml/)
OUTPUT_BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

# Target timesteps per recording. dataLoader.py has TIMESTEPS = 300.
# Raw data has ~230 rows regardless of 50Hz or 90Hz subfolder,
# so everything gets resampled up to this value.
TARGET_TIMESTEPS = 300
RESAMPLE         = True

# Recordings with fewer than this many rows are discarded (partial captures)
MIN_ROWS = 50

# ── COLUMNS ───────────────────────────────────────────────────────────────────

IMU_COLS     = [f"imu{i}_{c}" for i in range(6) for c in ["w", "x", "y", "z"]]
OUTPUT_COLS  = ["timestep"] + IMU_COLS

# ── HELPERS ───────────────────────────────────────────────────────────────────

def resample_recording(df: pd.DataFrame, target: int) -> pd.DataFrame:
    """
    Linearly resample a recording from however many rows it has to `target` rows.
    Works for both upsampling and downsampling.
    """
    n = len(df)
    if n == target:
        out = df[IMU_COLS].reset_index(drop=True).copy()
        out.insert(0, "timestep", range(target))
        return out

    x_old = np.linspace(0, 1, n)
    x_new = np.linspace(0, 1, target)

    resampled = {}
    for col in IMU_COLS:
        f = interp1d(x_old, df[col].values, kind="linear")
        resampled[col] = f(x_new)

    out = pd.DataFrame(resampled)
    out.insert(0, "timestep", range(target))
    return out


def process_session_file(filepath: str) -> list[tuple[str, pd.DataFrame]]:
    """
    Read one session CSV and return a list of (gesture_label, recording_df) pairs,
    one per unique (gesture, recording_id) combination found in the file.
    """
    try:
        df = pd.read_csv(filepath)
    except Exception as e:
        print(f"    [SKIP] Cannot read {os.path.basename(filepath)}: {e}")
        return []

    # Validate required columns
    required = {"gesture", "recording_id", "timestep"} | set(IMU_COLS)
    missing  = required - set(df.columns)
    if missing:
        print(f"    [SKIP] {os.path.basename(filepath)} missing columns: {missing}")
        return []

    results = []

    for (gesture, rec_id), group in df.groupby(["gesture", "recording_id"], sort=False):
        group = group.sort_values("timestep").reset_index(drop=True)

        if len(group) < MIN_ROWS:
            print(f"    [SKIP] {gesture} rec {rec_id}: only {len(group)} rows (< {MIN_ROWS})")
            continue

        if group[IMU_COLS].isnull().any().any():
            print(f"    [SKIP] {gesture} rec {rec_id}: contains NaN values")
            continue

        # Reject unscaled int16 recordings — valid quaternion components must
        # be in [-1, 1]. Threshold of 1.1 gives buffer for floating point noise
        # while catching raw int16 data which can be in the tens of thousands.
        imu_max = group[IMU_COLS].abs().values.max()
        if imu_max > 1.1:
            print(f"    [SKIP] {gesture} rec {rec_id}: values out of range "
                  f"(max abs={imu_max:.2f}) — likely unscaled int16 data")
            continue

        recording_df = resample_recording(group, TARGET_TIMESTEPS) if RESAMPLE else group[OUTPUT_COLS].reset_index(drop=True)
        results.append((str(gesture), recording_df))

    return results


def process_dataset(input_dir: str, output_dir: str, label: str):
    """
    Walk all subfolders (50Hz, 90Hz, etc.) under input_dir,
    process every gesture_data_*.csv found, and write one output
    CSV per recording into output_dir.
    """
    os.makedirs(output_dir, exist_ok=True)

    # Recursive search catches both 50Hz/ and 90Hz/ subfolders
    all_files = sorted(glob.glob(os.path.join(input_dir, "**", "gesture_data_*.csv"), recursive=True))

    if not all_files:
        # Fallback in case files aren't named gesture_data_*
        all_files = sorted(glob.glob(os.path.join(input_dir, "**", "*.csv"), recursive=True))

    if not all_files:
        print(f"  [WARN] No CSV files found under: {input_dir}")
        return

    print(f"\n{'='*58}")
    print(f"  Dataset : {label}")
    print(f"  Input   : {input_dir}")
    print(f"  Output  : {output_dir}")
    print(f"  Found   : {len(all_files)} session file(s)")
    print(f"{'='*58}")

    global_counts: dict[str, int] = {}
    total_written = 0

    # Group files by subfolder so output is easier to follow in the log
    subfolders = sorted(set(os.path.dirname(f) for f in all_files))
    for subfolder in subfolders:
        rel_sub   = os.path.relpath(subfolder, input_dir)
        sub_files = [f for f in all_files if os.path.dirname(f) == subfolder]
        print(f"\n  [{rel_sub}]  ({len(sub_files)} file(s))")

        for filepath in sub_files:
            print(f"    {os.path.basename(filepath)}")
            recordings = process_session_file(filepath)

            for gesture, rec_df in recordings:
                idx      = global_counts.get(gesture, 0)
                out_path = os.path.join(output_dir, f"{gesture}_{idx}.csv")
                rec_df.to_csv(out_path, index=False)
                global_counts[gesture] = idx + 1
                total_written += 1

            print(f"      → {len(recordings)} recording(s) written")

    # Summary
    print(f"\n  ── {label} summary ──")
    print(f"  Total recordings written : {total_written}")
    print(f"  Per-gesture breakdown:")
    for gesture, count in sorted(global_counts.items()):
        print(f"    {gesture:12s}: {count}")

    # Sanity check on one output file
    if total_written > 0:
        first_gesture = sorted(global_counts)[0]
        sample_path   = os.path.join(output_dir, f"{first_gesture}_0.csv")
        check = pd.read_csv(sample_path)
        print(f"\n  Sample check — {os.path.basename(sample_path)}:")
        print(f"    Shape  : {check.shape}  (expected {TARGET_TIMESTEPS} × {len(OUTPUT_COLS)})")
        print(f"    Columns: {list(check.columns)}")
        print(f"    Range  : [{check[IMU_COLS].values.min():.4f}, {check[IMU_COLS].values.max():.4f}]"
              f"  (should be ~[-1, 1])")


# ── MAIN ──────────────────────────────────────────────────────────────────────

def main():
    print("Gesture CSV Parser")
    print(f"  Raw data  : {DATA_RAW_DIR}")
    print(f"  Output    : {OUTPUT_BASE}")
    print(f"  Resample  : {RESAMPLE}  →  {TARGET_TIMESTEPS} timesteps per recording")

    for input_folder, output_folder in DATASETS.items():
        input_dir  = os.path.join(DATA_RAW_DIR, input_folder)
        output_dir = os.path.join(OUTPUT_BASE, output_folder)

        if not os.path.isdir(input_dir):
            print(f"\n[WARN] Folder not found, skipping: {input_dir}")
            continue

        process_dataset(input_dir, output_dir, label=input_folder)

    print(f"\n{'='*58}")
    print("Done. Output folders:")
    for output_folder in DATASETS.values():
        path = os.path.join(OUTPUT_BASE, output_folder)
        n    = len(glob.glob(os.path.join(path, "*.csv")))
        print(f"  {path}  ({n} files)")


if __name__ == "__main__":
    main()
