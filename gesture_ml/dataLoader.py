"""
dataLoader.py
-------------
Loads the per-gesture CSVs produced by parse_gesture_csv.py and returns
arrays ready for 1D-CNN training.

File naming convention (set by parse_gesture_csv.py):
    data/<GestureLabel>_<index>.csv

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

# ── Config (must match parse_gesture_csv.py) ─────────────────────────────────
TIMESTEPS  = 300    # rows per recording (3s @ 100Hz)
NUM_IMUS   = 6
VALS_PER_IMU = 4    # w, x, y, z
FEATURES   = NUM_IMUS * VALS_PER_IMU   # 24
# ─────────────────────────────────────────────────────────────────────────────

# Column names for the 24 feature columns (everything except "timestep")
FEATURE_COLS = [
    f"imu{i}_{c}" for i in range(NUM_IMUS) for c in ["w", "x", "y", "z"]
]


def load_dataset(path: str = "data/*.csv", timesteps: int = TIMESTEPS):
    """
    Load all per-gesture CSVs from `path` glob.

    Parameters
    ----------
    path      : glob pattern pointing to output of parse_gesture_csv.py
    timesteps : expected number of timestep rows per file

    Returns
    -------
    X       : np.ndarray, shape (N, timesteps, 24), dtype float32
    y       : np.ndarray, shape (N,),               dtype int
    encoder : fitted sklearn LabelEncoder
    """
    X, y = [], []
    skipped = 0

    files = sorted(glob.glob(path))
    if not files:
        raise FileNotFoundError(f"No CSV files found matching: {path}")

    for filepath in files:
        # Label is the part of the filename before the first underscore
        basename = os.path.basename(filepath)
        label = basename.split("_")[0]

        try:
            df = pd.read_csv(filepath)
        except Exception as e:
            print(f"  [WARN] Could not read {filepath}: {e}")
            skipped += 1
            continue

        # Make sure all expected columns exist
        missing = [c for c in FEATURE_COLS if c not in df.columns]
        if missing:
            print(f"  [WARN] {filepath} missing columns {missing}, skipping.")
            skipped += 1
            continue

        data = df[FEATURE_COLS].values.astype(np.float32)   # (T, 24)

        if data.shape[0] != timesteps:
            print(f"  [WARN] {filepath} has {data.shape[0]} rows, expected {timesteps}. Skipping.")
            skipped += 1
            continue

        X.append(data)
        y.append(label)

    if not X:
        raise ValueError("No valid samples loaded. Check your data directory and CSV format.")

    X = np.stack(X, axis=0)   # (N, timesteps, 24)

    encoder = LabelEncoder()
    y_enc = encoder.fit_transform(y)

    print(f"Loaded {len(X)} samples | shape={X.shape} | classes={list(encoder.classes_)}")
    if skipped:
        print(f"  ({skipped} files skipped due to errors)")

    return X, y_enc, encoder


# ── Quick sanity check ────────────────────────────────────────────────────────
if __name__ == "__main__":
    X, y, enc = load_dataset()
    print(f"X: {X.shape}  y: {y.shape}")
    print(f"Label mapping: { {i: c for i, c in enumerate(enc.classes_)} }")
    print(f"NaN check: {np.isnan(X).any()}")
    print(f"Value range: [{X.min():.4f}, {X.max():.4f}]")