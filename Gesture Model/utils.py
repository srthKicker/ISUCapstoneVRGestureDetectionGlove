# utils.py
# This file was created by ChatGPT to include any possible functions that may be useful across the project,
#  such as quaternion normalization, dataset validation, label encoder persistence, 
# prediction decoding, and evaluation helpers like confusion matrix plotting.


import numpy as np
import pickle
from sklearn.metrics import confusion_matrix
import matplotlib.pyplot as plt

# -----------------------------
# Quaternion normalization
# -----------------------------
def normalize_quaternion(q):
    """
    q: (..., 4) numpy array
    """
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    return q / (norm + 1e-8)


# -----------------------------
# Dataset validation
# -----------------------------
def validate_dataset(X, y):
    assert len(X) == len(y), "X and y size mismatch"
    assert not np.isnan(X).any(), "NaNs in input data"
    print(f"Dataset OK: {X.shape}, labels: {len(set(y))}")


# -----------------------------
# Label encoder persistence
# -----------------------------
def save_label_encoder(encoder, path="label_encoder.pkl"):
    with open(path, "wb") as f:
        pickle.dump(encoder, f)

def load_label_encoder(path="label_encoder.pkl"):
    with open(path, "rb") as f:
        return pickle.load(f)


# -----------------------------
# Prediction helpers
# -----------------------------
def decode_prediction(pred, encoder):
    """
    pred: model output (softmax)
    """
    idx = np.argmax(pred)
    return encoder.inverse_transform([idx])[0]


# -----------------------------
# Evaluation helpers
# -----------------------------
def plot_confusion(y_true, y_pred, labels):
    cm = confusion_matrix(y_true, y_pred)

    plt.imshow(cm, cmap='Blues')
    plt.xticks(range(len(labels)), labels, rotation=45)
    plt.yticks(range(len(labels)), labels)
    plt.xlabel("Predicted")
    plt.ylabel("True")
    plt.title("Gesture Confusion Matrix")

    for i in range(len(labels)):
        for j in range(len(labels)):
            plt.text(j, i, cm[i, j], ha='center', va='center')

    plt.tight_layout()
    plt.show()
