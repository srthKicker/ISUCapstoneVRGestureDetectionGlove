# predict.py
import collections
import numpy as np
import tensorflow as tf
import pickle
import serial
import time
import os
from sklearn.preprocessing import StandardScaler
from dataLoader import load_dataset

# ── Config ───────────────────────────────────────────────
SERIAL_PORT    = "COM5"     # change to your port
BAUD_RATE      = 500000
HAND           = "right"    # "left" or "right"

WINDOW_SIZE    = 300        # MUST match training TIMESTEPS (was 30 — that caused the stuck bug)
PREDICT_EVERY  = 15         # run inference once every N new samples (~150 ms at 100 Hz)
                            # was predicting every frame → ~100 Keras calls/sec → frozen output
VOTE_WINDOW    = 5          # majority-vote over this many recent predictions to reduce jitter
MIN_CONFIDENCE = 0.55       # below this → print "uncertain" instead of a class

NUM_IMUS       = 6
FEATURES       = NUM_IMUS * 4   # 24
# ─────────────────────────────────────────────────────────

def mirror_left_hand(X):
    """Flip quaternion Y axis to convert left→right hand frame."""
    X_mirror = X.copy()
    for imu in range(NUM_IMUS):
        X_mirror[..., imu * 4 + 2] *= -1
    return X_mirror

def preprocess(window_deque, scaler, hand):
    """
    window_deque: deque of WINDOW_SIZE frames, each a list of 24 floats.
    Returns numpy array shaped (1, WINDOW_SIZE, FEATURES).
    """
    X = np.array(window_deque, dtype=np.float32)   # (WINDOW_SIZE, 24)
    X = scaler.transform(X)                         # same transform as training
    X = X.reshape(1, WINDOW_SIZE, FEATURES)         # (1, 300, 24)
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
    except Exception:
        return None

def majority_vote(vote_buf):
    """Return the most common label in vote_buf."""
    return collections.Counter(vote_buf).most_common(1)[0][0]

# ── Load the hand-specific label encoder ─────────────────
def load_encoder(hand):
    """
    Look for gesture_model_{hand}_encoder.pkl, then label_encoder.pkl,
    then fall back to the encoder returned by load_dataset().
    """
    for path in [f"gesture_model_{hand}_encoder.pkl", "label_encoder.pkl"]:
        if os.path.exists(path):
            with open(path, "rb") as f:
                enc = pickle.load(f)
            print(f"Loaded label encoder from {path}")
            return enc
    print("No encoder .pkl found — deriving from training data...")
    _, _, enc = load_dataset()
    return enc

# ── Fit scaler on training data ───────────────────────────
print("Loading training data to fit scaler...")
X_train, y_train, _enc = load_dataset()
N, T, F = X_train.shape

if T != WINDOW_SIZE:
    print(
        f"\nWARNING: Training TIMESTEPS={T} but WINDOW_SIZE={WINDOW_SIZE}. "
        f"Setting WINDOW_SIZE={T} to match.\n"
    )
    WINDOW_SIZE = T

scaler = StandardScaler()
scaler.fit(X_train.reshape(-1, F))

encoder = load_encoder(HAND)
print(f"Classes: {list(encoder.classes_)}")

# ── Load the correct hand model ───────────────────────────
# Priority: gesture_model_{hand}.keras  →  gesture_model.keras
hand_model_path    = f"gesture_model_{HAND}.keras"
generic_model_path = "gesture_model.keras"

if os.path.exists(hand_model_path):
    model_path = hand_model_path
    print(f"Loading {HAND}-hand model: {hand_model_path}")
elif os.path.exists(generic_model_path):
    model_path = generic_model_path
    print(
        f"No hand-specific model found ({hand_model_path!r}). "
        f"Falling back to {generic_model_path!r}."
    )
else:
    raise FileNotFoundError(
        f"No model file found. Expected 'gesture_model_{HAND}.keras' or 'gesture_model.keras'."
    )

model = tf.keras.models.load_model(model_path)

# ── Run inference ─────────────────────────────────────────
print(f"\nOpening {SERIAL_PORT} @ {BAUD_RATE} baud ({HAND} hand mode)...")
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
time.sleep(2)   # let serial settle

# Use a deque so appending a new frame automatically evicts the oldest —
# no manual pop(0), and the window is always rolling/fresh.
window               = collections.deque(maxlen=WINDOW_SIZE)
vote_buf             = collections.deque(maxlen=VOTE_WINDOW)
frames_since_predict = 0
last_display         = ""

print("Ready — hold a gesture...\n")

try:
    while True:
        raw = ser.readline().decode("utf-8", errors="ignore").strip()
        if not raw:
            continue

        frame = parse_line(raw)
        if frame is None:
            continue

        # Rolling window: deque evicts the oldest frame automatically
        window.append(frame)
        frames_since_predict += 1

        # Wait until window is full before predicting
        if len(window) < WINDOW_SIZE:
            if len(window) % 100 == 0:
                print(f"  Filling window... {len(window)}/{WINDOW_SIZE}")
            continue

        # Throttle: only run inference every PREDICT_EVERY new frames
        if frames_since_predict < PREDICT_EVERY:
            continue
        frames_since_predict = 0

        # ── Inference ────────────────────────────────────
        X          = preprocess(window, scaler, HAND)
        probs      = model.predict(X, verbose=0)[0]
        idx        = int(np.argmax(probs))
        confidence = float(probs[idx])
        label      = encoder.classes_[idx]

        # ── Vote smoothing ────────────────────────────────
        vote_buf.append(label)
        smoothed_label = majority_vote(vote_buf)

        # ── Display (only reprint when output changes) ────
        if confidence < MIN_CONFIDENCE:
            display = f"{'uncertain':12s}  ({label} @ {confidence*100:.1f}%)"
        else:
            display = f"{smoothed_label:12s}  Confidence: {confidence*100:.1f}%"

        if display != last_display:
            print(f"Gesture: {display}")
            last_display = display

except KeyboardInterrupt:
    print("\nStopped.")
    ser.close()