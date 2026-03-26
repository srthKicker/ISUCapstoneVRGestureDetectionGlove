from dataLoader import load_dataset
from model import build_model
from sklearn.utils.class_weight import compute_class_weight
import numpy as np

X, y, encoder = load_dataset()

# Compute class weights to handle any remaining imbalance
class_weights = compute_class_weight('balanced', classes=np.unique(y), y=y)
class_weight_dict = dict(enumerate(class_weights))
print(f"Class weights: {class_weight_dict}")

model = build_model(
    timesteps=X.shape[1],
    features=X.shape[2],
    num_classes=len(np.unique(y))
)

model.summary()

history = model.fit(
    X, y,
    epochs=50,
    batch_size=16,
    validation_split=0.2,
    class_weight=class_weight_dict
)

model.save("gesture_model.h5")
print("Model saved to gesture_model.h5")

# Print final accuracy
final_val_acc = history.history['val_accuracy'][-1]
print(f"\nFinal validation accuracy: {final_val_acc:.1%}")