# predict.py
import numpy as np
import tensorflow as tf
import pickle
import serial
import time
from sklearn.preprocessing import StandardScaler
from dataLoader import load_dataset

# ── Config ───────────────────────────────────────────────
SERIAL_PORT  = "COM5"       # change to your port
BAUD_RATE    = 500000
WINDOW_SIZE  = 30           # frames to buffer before predicting
NUM_IMUS     = 6
FEATURES     = NUM_IMUS * 4 # 24
HAND         = "right"      # change to "left" for left hand
# ─────────────────────────────────────────────────────────

def mirror_left_hand(X):
    """Flip Y axis of each quaternion to convert left→right hand frame."""
    X_mirror = X.copy()
    for imu in range(NUM_IMUS):
        X_mirror[..., imu*4 + 2] *= -1
    return X_mirror

def preprocess(buffer, scaler, hand):
    """buffer: list of WINDOW_SIZE frames, each a list of 24 floats"""
    X = np.array(buffer, dtype=np.float32)          # (30, 24)
    X = X.reshape(-1, FEATURES)
    X = scaler.transform(X)
    X = X.reshape(1, WINDOW_SIZE, FEATURES)         # (1, 30, 24)
    if hand == "left":
        X = mirror_left_hand(X)
    return X

def parse_line(line):
    """Parse a CSV serial line into 24 floats. Returns None if invalid."""
    try:
        parts = [p.strip() for p in line.split(",") if p.strip()]
        if len(parts) < 24:
            return None
        values = list(map(float, parts[:24]))
        if any(np.isnan(v) for v in values):
            return None
        return values
    except:
        return None

# ── Fit scaler on training data (same transform used during training) ──
print("Loading training data to fit scaler...")
X_train, y_train, encoder = load_dataset()
N, T, F = X_train.shape
scaler = StandardScaler()
scaler.fit(X_train.reshape(-1, F))
print(f"Classes: {list(encoder.classes_)}")

# ── Load model ────────────────────────────────────────────
print("Loading model...")
model = tf.keras.models.load_model("gesture_model.h5")

# ── Run inference ─────────────────────────────────────────
print(f"Opening {SERIAL_PORT} @ {BAUD_RATE} baud ({HAND} hand mode)...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)  # let serial settle

buffer = []
print("Ready — hold a gesture...\n")

try:
    while True:
        raw = ser.readline().decode("utf-8", errors="ignore").strip()
        if not raw:
            continue

        frame = parse_line(raw)
        if frame is None:
            continue

        buffer.append(frame)

        # Keep buffer at fixed window size
        if len(buffer) > WINDOW_SIZE:
            buffer.pop(0)

        # Predict once buffer is full
        if len(buffer) == WINDOW_SIZE:
            X = preprocess(buffer, scaler, HAND)
            probs = model.predict(X, verbose=0)[0]
            pred_idx = np.argmax(probs)
            pred_label = encoder.classes_[pred_idx]
            confidence = probs[pred_idx] * 100
            print(f"Gesture: {pred_label:12s}  Confidence: {confidence:.1f}%")

except KeyboardInterrupt:
    print("\nStopped.")
    ser.close()

