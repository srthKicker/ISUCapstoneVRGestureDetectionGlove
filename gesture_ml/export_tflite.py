# export_tflite.py

import tensorflow as tf
import numpy as np
from dataloader import load_dataset

MODEL_PATH = "gesture_model.h5"
OUTPUT_TFLITE = "gesture_model_int8.tflite"

# -----------------------------
# Load trained model
# -----------------------------
model = tf.keras.models.load_model(MODEL_PATH)

# -----------------------------
# Load data for quantization
# -----------------------------
X, _, _ = load_dataset()

# -----------------------------
# TFLite converter
# -----------------------------
converter = tf.lite.TFLiteConverter.from_keras_model(model)

# Enable optimization
converter.optimizations = [tf.lite.Optimize.DEFAULT]

# Representative dataset (VERY IMPORTANT)
def representative_dataset():
    for i in range(200):
        yield [X[i:i+1].astype(np.float32)]

converter.representative_dataset = representative_dataset

# Force INT8 ops
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS_INT8]
converter.inference_input_type = tf.int8
converter.inference_output_type = tf.int8

# -----------------------------
# Convert
# -----------------------------
tflite_model = converter.convert()

# -----------------------------
# Save model
# -----------------------------
with open(OUTPUT_TFLITE, "wb") as f:
    f.write(tflite_model)

print(f"Saved INT8 TFLite model → {OUTPUT_TFLITE}")
print(f"Model size: {len(tflite_model) / 1024:.1f} KB")
