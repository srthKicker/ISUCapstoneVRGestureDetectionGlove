"""
model.py
--------
1D-CNN for gesture classification using 6-IMU quaternion data.

Input shape : (batch, TIMESTEPS, 24)   — 6 IMUs × 4 quaternion components
Output shape: (batch, num_classes)      — softmax probabilities
"""

import tensorflow as tf
from tensorflow.keras import layers, regularizers


def build_model(timesteps: int, features: int, num_classes: int) -> tf.keras.Model:
    """
    Builds and compiles a 1D-CNN for quaternion gesture recognition.

    Architecture
    ------------
    Two stacked Conv1D blocks (16 → 32 filters) with MaxPooling, followed
    by a Dense head. Dropout is included to reduce overfitting on small
    gesture datasets.

    Parameters
    ----------
    timesteps   : sequence length (e.g. 300 for 3s @ 100Hz)
    features    : number of features per timestep (24 for 6 IMUs × 4 quat components)
    num_classes : number of gesture classes

    Returns
    -------
    Compiled tf.keras.Model
    """
    model = tf.keras.Sequential([
        layers.Input(shape=(timesteps, features)),

        # Block 1 — broad temporal patterns
        layers.Conv1D(16, kernel_size=9, padding="same", activation="relu"),
        layers.BatchNormalization(),
        layers.MaxPooling1D(pool_size=2),
        layers.Dropout(0.2),

        # Block 2 — finer temporal patterns
        layers.Conv1D(32, kernel_size=5, padding="same", activation="relu"),
        layers.BatchNormalization(),
        layers.MaxPooling1D(pool_size=2),
        layers.Dropout(0.2),

        # Block 3 — high-level features
        layers.Conv1D(64, kernel_size=3, padding="same", activation="relu"),
        layers.BatchNormalization(),
        layers.GlobalAveragePooling1D(),   # more parameter-efficient than Flatten

        # Classification head
        layers.Dense(64, activation="relu", kernel_regularizer=regularizers.l2(1e-4)),
        layers.Dropout(0.3),
        layers.Dense(num_classes, activation="softmax"),
    ])

    model.compile(
        optimizer=tf.keras.optimizers.Adam(learning_rate=1e-3),
        loss="sparse_categorical_crossentropy",
        metrics=["accuracy"],
    )

    return model


if __name__ == "__main__":
    m = build_model(timesteps=300, features=24, num_classes=5)
    m.summary()