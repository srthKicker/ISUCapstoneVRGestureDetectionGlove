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

Output CSV format (one file per window slice, WINDOW_SIZE rows):
    timestep, imu0_w, imu0_x, ..., imu5_z

Gesture labels found in data:
    Base, Gesture1, Gesture2, Gesture3, Gesture4

Strategy:
    Instead of resampling each recording to a fixed length, we slice each
    recording into non-overlapping windows of WINDOW_SIZE frames, discarding
    any remainder at the end. This is appropriate for STATIC / POSITIONAL
    gestures where the person holds a pose — every window of the hold is a
    valid example of that pose.

    At 50 Hz inference, WINDOW_SIZE=10 means the model needs just 0.2 seconds
    of data to make a prediction.

    Frame-count reality check (raw data, collected at ~256 Hz):
        3-second hold @ 256 Hz  →  ~768 frames  →  76 windows of 10
        3-second hold @  90 Hz  →  ~270 frames  →  27 windows of 10
        3-second hold @  50 Hz  →  ~150 frames  →  15 windows of 10
        Slow firmware  @  30 Hz →   ~90 frames  →   9 windows of 10

Usage:
    python parse_gesture_csv.py
"""

import os
import glob
import pandas as pd
import numpy as np

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

# ── SLICING CONFIG ────────────────────────────────────────────────────────────

# Number of raw frames per output window.
# At 50 Hz inference: 10 frames = 0.2 s, 20 frames = 0.4 s, 5 frames = 0.1 s
# Smaller = more samples generated + faster inference window.
# Larger  = more temporal averaging, potentially more robust to noise.
# Recommended: 10–20 for positional/static gestures.
WINDOW_SIZE = 10

# Step between window starts (frames). WINDOW_SIZE == STRIDE → no overlap.
# Set STRIDE < WINDOW_SIZE for overlapping windows (more samples, correlated).
# Non-overlapping (STRIDE == WINDOW_SIZE) is conservative and avoids
# leaking near-identical frames into both train and val splits.
STRIDE = WINDOW_SIZE   # non-overlapping by default

# Skip the first N frames of each recording to avoid the transition period
# where the person was still moving into the pose.
SKIP_LEAD_FRAMES = 10

# Recordings with fewer than this many usable frames (after lead skip) are
# discarded. Must be >= WINDOW_SIZE to produce at least one window.
MIN_ROWS = WINDOW_SIZE

# ── COLUMNS ───────────────────────────────────────────────────────────────────

IMU_COLS    = [f"imu{i}_{c}" for i in range(6) for c in ["w", "x", "y", "z"]]
OUTPUT_COLS = ["timestep"] + IMU_COLS

# ── HELPERS ───────────────────────────────────────────────────────────────────

def slice_recording(df: pd.DataFrame) -> list[pd.DataFrame]:
    """
    Slice a single recording DataFrame into non-overlapping windows.

    Parameters
    ----------
    df : DataFrame with IMU_COLS columns, one row per raw frame.

    Returns
    -------
    List of DataFrames, each with WINDOW_SIZE rows and OUTPUT_COLS columns.
    Returns an empty list if the recording is too short to produce any window.
    """
    # Drop the transition period at the start of each hold
    df = df.iloc[SKIP_LEAD_FRAMES:].reset_index(drop=True)

    n_frames = len(df)
    if n_frames < WINDOW_SIZE:
        return []

    windows = []
    start = 0
    while start + WINDOW_SIZE <= n_frames:
        chunk = df[IMU_COLS].iloc[start : start + WINDOW_SIZE].reset_index(drop=True)
        chunk.insert(0, "timestep", range(WINDOW_SIZE))
        windows.append(chunk)
        start += STRIDE

    return windows


def process_session_file(filepath: str) -> list[tuple[str, pd.DataFrame]]:
    """
    Read one session CSV and return a list of (gesture_label, window_df) pairs.
    Each recording in the session may produce multiple windows.
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

        # Basic quality checks
        if len(group) < MIN_ROWS + SKIP_LEAD_FRAMES:
            print(f"    [SKIP] {gesture} rec {rec_id}: only {len(group)} rows after lead-skip margin")
            continue

        if group[IMU_COLS].isnull().any().any():
            print(f"    [SKIP] {gesture} rec {rec_id}: contains NaN values")
            continue

        # Reject unscaled int16 data — quaternion components must be in [-1, 1]
        imu_max = group[IMU_COLS].abs().values.max()
        if imu_max > 1.1:
            print(f"    [SKIP] {gesture} rec {rec_id}: values out of range "
                  f"(max abs={imu_max:.2f}) — likely unscaled int16 data")
            continue

        windows = slice_recording(group)

        if not windows:
            print(f"    [SKIP] {gesture} rec {rec_id}: too short to produce any window "
                  f"(need {WINDOW_SIZE + SKIP_LEAD_FRAMES} frames, got {len(group)})")
            continue

        for window_df in windows:
            results.append((str(gesture), window_df))

    return results


def process_dataset(input_dir: str, output_dir: str, label: str):
    """
    Walk all subfolders under input_dir, process every gesture_data_*.csv,
    and write one output CSV per window slice into output_dir.
    """
    os.makedirs(output_dir, exist_ok=True)

    all_files = sorted(glob.glob(os.path.join(input_dir, "**", "gesture_data_*.csv"), recursive=True))

    if not all_files:
        all_files = sorted(glob.glob(os.path.join(input_dir, "**", "*.csv"), recursive=True))

    if not all_files:
        print(f"  [WARN] No CSV files found under: {input_dir}")
        return

    print(f"\n{'='*58}")
    print(f"  Dataset     : {label}")
    print(f"  Input       : {input_dir}")
    print(f"  Output      : {output_dir}")
    print(f"  Window size : {WINDOW_SIZE} frames")
    print(f"  Stride      : {STRIDE} frames  ({'non-overlapping' if STRIDE == WINDOW_SIZE else 'overlapping'})")
    print(f"  Lead skip   : {SKIP_LEAD_FRAMES} frames")
    print(f"  Found       : {len(all_files)} session file(s)")
    print(f"{'='*58}")

    global_counts: dict[str, int] = {}
    total_written = 0

    subfolders = sorted(set(os.path.dirname(f) for f in all_files))
    for subfolder in subfolders:
        rel_sub   = os.path.relpath(subfolder, input_dir)
        sub_files = [f for f in all_files if os.path.dirname(f) == subfolder]
        print(f"\n  [{rel_sub}]  ({len(sub_files)} file(s))")

        for filepath in sub_files:
            print(f"    {os.path.basename(filepath)}")
            windows = process_session_file(filepath)

            for gesture, window_df in windows:
                idx      = global_counts.get(gesture, 0)
                out_path = os.path.join(output_dir, f"{gesture}_{idx}.csv")
                window_df.to_csv(out_path, index=False)
                global_counts[gesture] = idx + 1
                total_written += 1

            print(f"      → {len(windows)} window(s) written")

    # Summary
    print(f"\n  ── {label} summary ──")
    print(f"  Total windows written : {total_written}")
    print(f"  Per-gesture breakdown:")
    for gesture, count in sorted(global_counts.items()):
        print(f"    {gesture:12s}: {count}")

    # Sanity check on one output file
    if total_written > 0:
        first_gesture = sorted(global_counts)[0]
        sample_path   = os.path.join(output_dir, f"{first_gesture}_0.csv")
        check = pd.read_csv(sample_path)
        print(f"\n  Sample check — {os.path.basename(sample_path)}:")
        print(f"    Shape  : {check.shape}  (expected {WINDOW_SIZE} × {len(OUTPUT_COLS)})")
        print(f"    Columns: {list(check.columns)}")
        print(f"    Range  : [{check[IMU_COLS].values.min():.4f}, {check[IMU_COLS].values.max():.4f}]"
              f"  (should be ~[-1, 1])")


# ── MAIN ──────────────────────────────────────────────────────────────────────

def main():
    print("Gesture CSV Parser  —  STATIC POSE / SLICING MODE")
    print(f"  Raw data    : {DATA_RAW_DIR}")
    print(f"  Output      : {OUTPUT_BASE}")
    print(f"  Window size : {WINDOW_SIZE} frames (no resampling)")
    print(f"  Stride      : {STRIDE} frames")
    print(f"  Lead skip   : {SKIP_LEAD_FRAMES} frames per recording")

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
