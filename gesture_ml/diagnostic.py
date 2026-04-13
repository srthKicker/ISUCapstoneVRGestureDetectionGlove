# diagnostic.py - save and run this in gesture_ml/
import numpy as np
from dataLoader import load_dataset
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt

X, y, encoder = load_dataset()

# Flatten each sample to a vector and run PCA
X_flat = X.reshape(len(X), -1)
pca = PCA(n_components=2)
X_2d = pca.fit_transform(X_flat)

# Plot — if gestures are separable, you'll see distinct clusters
plt.figure(figsize=(8, 6))
for label in np.unique(y):
    mask = y == label
    plt.scatter(X_2d[mask, 0], X_2d[mask, 1], label=encoder.classes_[label], alpha=0.7)

plt.legend()
plt.title("PCA of gesture data — do you see clusters?")
plt.savefig("pca_check.png")
plt.show()
print("Saved pca_check.png")