import numpy as np
import tensorflow as tf
from dataLoader import load_dataset
from sklearn.preprocessing import StandardScaler
from sklearn.model_selection import train_test_split

X, y, encoder = load_dataset()

# Normalize across the feature dimension
# Reshape to (N*T, 24), scale, reshape back
N, T, F = X.shape
X_flat = X.reshape(-1, F)
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X_flat).reshape(N, T, F)

X_train, X_val, y_train, y_val = train_test_split(
    X_scaled, y, test_size=0.15, stratify=y, random_state=42
)

model = tf.keras.Sequential([
    tf.keras.layers.Input(shape=(T, F)),

    # Can learn temporal patterns when dynamic data is added later
    tf.keras.layers.Conv1D(16, kernel_size=5, padding="same", activation="relu"),
    tf.keras.layers.GlobalAveragePooling1D(),  # averages time — works for static poses now

    tf.keras.layers.Dense(32, activation="relu"),
    tf.keras.layers.Dropout(0.3),
    tf.keras.layers.Dense(len(encoder.classes_), activation="softmax"),
])

model.compile(
    optimizer=tf.keras.optimizers.Adam(1e-3),
    loss="sparse_categorical_crossentropy",
    metrics=["accuracy"],
)

model.summary()

history = model.fit(
    X_train, y_train,
    epochs=50,
    batch_size=8,
    validation_data=(X_val, y_val),
    callbacks=[
        tf.keras.callbacks.EarlyStopping(patience=10, restore_best_weights=True)
    ]
)

model.save("gesture_model.keras")
print(f"\nFinal val accuracy: {max(history.history['val_accuracy'])*100:.1f}%")