"""
dataLoader.py
-------------
Loads the per-gesture CSVs produced by parse_gesture_csv.py and returns
arrays ready for 1D-CNN training.

Supports loading left hand, right hand, or both combined.

File naming convention (set by parse_gesture_csv.py):
    data/left_hand/<GestureLabel>_<index>.csv
    data/right_hand/<GestureLabel>_<index>.csv

Each CSV has columns:
    timestep, imu0_w, imu0_x, imu0_y, imu0_z,
              imu1_w, ..., imu5_w, imu5_x, imu5_y, imu5_z

Output shapes:
    X : (N, TIMESTEPS, 24)   float32   — 6 IMUs × 4 quaternion components
    y : (N,)                 int        — encoded class labels
    encoder : LabelEncoder               — maps int ↔ gesture name
"""

import numpy as np
import pandas as pd
import glob
import os
from sklearn.preprocessing import LabelEncoder

# ── Config ────────────────────────────────────────────────────────────────────

TIMESTEPS    = 300      # rows per recording — must match parse_gesture_csv.py TARGET_TIMESTEPS
NUM_IMUS     = 6
VALS_PER_IMU = 4        # w, x, y, z
FEATURES     = NUM_IMUS * VALS_PER_IMU   # 24

# Base data directory — sits next to this script inside gesture_ml/
_BASE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")

# Paths for each hand dataset
DATA_PATHS = {
    "left":  os.path.join(_BASE_DIR, "left_hand"),
    "right": os.path.join(_BASE_DIR, "right_hand"),
}

# ── Column names ──────────────────────────────────────────────────────────────

FEATURE_COLS = [
    f"imu{i}_{c}" for i in range(NUM_IMUS) for c in ["w", "x", "y", "z"]
]

# ── Core loader ───────────────────────────────────────────────────────────────

def load_dataset(
    hand: str = "left",
    timesteps: int = TIMESTEPS,
):
    """
    Load all per-gesture CSVs for the specified hand.

    Parameters
    ----------
    hand      : "left", "right", or "both"
                "both" combines left and right into a single dataset —
                only use this if training a universal model.
    timesteps : expected number of timestep rows per file.
                Must match TARGET_TIMESTEPS in parse_gesture_csv.py (default 300).

    Returns
    -------
    X       : np.ndarray, shape (N, timesteps, 24), dtype float32
    y       : np.ndarray, shape (N,),               dtype int
    encoder : fitted sklearn LabelEncoder
    """
    if hand not in ("left", "right", "both"):
        raise ValueError(f"hand must be 'left', 'right', or 'both' — got '{hand}'")

    # Determine which folders to pull from
    if hand == "both":
        folders = list(DATA_PATHS.values())
    else:
        folders = [DATA_PATHS[hand]]

    # Verify folders exist
    for folder in folders:
        if not os.path.isdir(folder):
            raise FileNotFoundError(
                f"Data folder not found: {folder}\n"
                f"Run parse_gesture_csv.py first to generate the data."
            )

    X, y    = [], []
    skipped = 0

    for folder in folders:
        files = sorted(glob.glob(os.path.join(folder, "*.csv")))
        if not files:
            print(f"  [WARN] No CSV files found in: {folder}")
            continue

        print(f"Loading from: {folder}  ({len(files)} files)")

        for filepath in files:
            # Label is the part of the filename before the first underscore
            basename = os.path.basename(filepath)
            label    = basename.split("_")[0]

            try:
                df = pd.read_csv(filepath)
            except Exception as e:
                print(f"  [WARN] Could not read {basename}: {e}")
                skipped += 1
                continue

            # Verify all feature columns are present
            missing = [c for c in FEATURE_COLS if c not in df.columns]
            if missing:
                print(f"  [WARN] {basename} missing columns {missing}, skipping.")
                skipped += 1
                continue

            data = df[FEATURE_COLS].values.astype(np.float32)   # (T, 24)

            if data.shape[0] != timesteps:
                print(f"  [WARN] {basename} has {data.shape[0]} rows, expected {timesteps}. Skipping.")
                skipped += 1
                continue

            X.append(data)
            y.append(label)

    if not X:
        raise ValueError(
            "No valid samples loaded. "
            "Check your data directory and that parse_gesture_csv.py has been run."
        )

    X = np.stack(X, axis=0)   # (N, timesteps, 24)

    encoder = LabelEncoder()
    y_enc   = encoder.fit_transform(y)

    print(f"\nLoaded {len(X)} samples")
    print(f"  Shape   : {X.shape}")
    print(f"  Classes : {list(encoder.classes_)}")
    print(f"  Hand    : {hand}")
    if skipped:
        print(f"  Skipped : {skipped} files")

    return X, y_enc, encoder


# ── Convenience wrappers ──────────────────────────────────────────────────────

def load_left_hand(**kwargs):
    """Load only the left hand dataset."""
    return load_dataset(hand="left", **kwargs)


def load_right_hand(**kwargs):
    """Load only the right hand dataset."""
    return load_dataset(hand="right", **kwargs)


def load_both_hands(**kwargs):
    """
    Load left and right hand data combined into one dataset.
    Only use this if training a single universal model.
    """
    return load_dataset(hand="both", **kwargs)


# ── Sanity check ──────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys

    hand = sys.argv[1] if len(sys.argv) > 1 else "left"
    print(f"Running sanity check for hand='{hand}'\n")

    X, y, enc = load_dataset(hand=hand)

    print(f"\nX shape    : {X.shape}")
    print(f"y shape    : {y.shape}")
    print(f"Label map  : { {i: c for i, c in enumerate(enc.classes_)} }")
    print(f"NaN check  : {np.isnan(X).any()}")
    print(f"Value range: [{X.min():.4f}, {X.max():.4f}]")

    # Class balance
    print(f"\nClass balance:")
    for i, cls in enumerate(enc.classes_):
        count = (y == i).sum()
        print(f"  {cls:12s}: {count} samples")