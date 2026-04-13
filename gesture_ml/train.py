"""
train.py
--------
Trains a 1D-CNN gesture classifier for a specified hand.

Usage:
    python train.py left      # train left hand model
    python train.py right     # train right hand model
    python train.py both      # train a combined universal model

Outputs (saved next to this script):
    gesture_model_left.keras      — trained left hand model
    gesture_model_right.keras     — trained right hand model
    gesture_model_both.keras      — trained universal model (if both)
    label_encoder_left.pkl        — label encoder for left hand
    label_encoder_right.pkl       — label encoder for right hand
    label_encoder_both.pkl        — label encoder for universal model
"""

import sys
import os
import numpy as np
import tensorflow as tf

from dataLoader import load_dataset
from model import build_model
from utils import save_label_encoder

# ── Config ────────────────────────────────────────────────────────────────────

EPOCHS         = 75
BATCH_SIZE     = 16
VALIDATION_SPLIT = 0.2

# Early stopping — stops training if val_loss doesn't improve for this many epochs
EARLY_STOPPING_PATIENCE = 7

# ── Argument parsing ──────────────────────────────────────────────────────────

def get_hand_arg() -> str:
    """Read hand from command line, defaulting to 'left'."""
    valid = ("left", "right", "both")
    if len(sys.argv) < 2:
        print("No hand specified — defaulting to 'left'.")
        print("Usage: python train.py [left|right|both]\n")
        return "left"
    hand = sys.argv[1].lower()
    if hand not in valid:
        print(f"Invalid hand '{hand}'. Choose from: {valid}")
        sys.exit(1)
    return hand

# ── Training ──────────────────────────────────────────────────────────────────

def train(hand: str):
    print(f"\n{'='*55}")
    print(f"  Training gesture model — hand: {hand.upper()}")
    print(f"{'='*55}\n")

    # ── Load data ─────────────────────────────────────────────────────────────
    X, y, encoder = load_dataset(hand=hand)

    # Class balance report — important to catch under-represented gestures
    print("\nClass balance:")
    for i, cls in enumerate(encoder.classes_):
        count = (y == i).sum()
        pct   = count / len(y) * 100
        print(f"  {cls:12s}: {count:4d} samples  ({pct:.1f}%)")

    num_classes = len(encoder.classes_)

    # ── Build model ───────────────────────────────────────────────────────────
    model = build_model(
        timesteps=X.shape[1],       # 300
        features=X.shape[2],        # 24  (6 IMUs x 4 quaternion components)
        num_classes=num_classes,
    )
    model.summary()

    # ── Callbacks ─────────────────────────────────────────────────────────────
    callbacks = [
        # Stop early if validation loss plateaus
        tf.keras.callbacks.EarlyStopping(
            monitor="val_loss",
            patience=EARLY_STOPPING_PATIENCE,
            restore_best_weights=True,
            verbose=1,
        ),
        # Save the best checkpoint during training
        tf.keras.callbacks.ModelCheckpoint(
            filepath=f"gesture_model_{hand}_best.keras",
            monitor="val_loss",
            save_best_only=True,
            verbose=1,
        ),
    ]

    # ── Shuffle + split ───────────────────────────────────────────────────────
    # IMPORTANT: do NOT use Keras validation_split without shuffling first.
    # Files are loaded alphabetically (Base_0...Gesture4_N), so the last 20%
    # would be almost entirely one class, making validation meaningless.
    from sklearn.model_selection import train_test_split

    X_train, X_val, y_train, y_val = train_test_split(
        X, y,
        test_size=VALIDATION_SPLIT,
        random_state=42,
        stratify=y,     # ensures each class is proportionally represented in both splits
    )

    print(f"\nTraining on {len(X_train)} samples, validating on {len(X_val)} samples")
    print("Val class distribution:")
    for i, cls in enumerate(encoder.classes_):
        count = (y_val == i).sum()
        print(f"  {cls:12s}: {count}")

    history = model.fit(
        X_train, y_train,
        epochs=EPOCHS,
        batch_size=BATCH_SIZE,
        validation_data=(X_val, y_val),
        callbacks=callbacks,
        verbose=1,
    )

    # ── Save final model + encoder ────────────────────────────────────────────
    model_path   = f"gesture_model_{hand}.keras"
    encoder_path = f"label_encoder_{hand}.pkl"

    model.save(model_path)
    save_label_encoder(encoder, encoder_path)

    # ── Results ───────────────────────────────────────────────────────────────
    best_val_acc  = max(history.history["val_accuracy"])
    best_val_loss = min(history.history["val_loss"])
    epochs_run    = len(history.history["val_loss"])

    print(f"\n{'='*55}")
    print(f"  Training complete — {hand.upper()} hand")
    print(f"  Epochs run       : {epochs_run}/{EPOCHS}")
    print(f"  Best val accuracy: {best_val_acc:.4f}  ({best_val_acc*100:.1f}%)")
    print(f"  Best val loss    : {best_val_loss:.4f}")
    print(f"  Model saved to   : {model_path}")
    print(f"  Encoder saved to : {encoder_path}")
    print(f"{'='*55}\n")

    return model, encoder, history


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    hand = get_hand_arg()
    train(hand)