# train.py


from dataloader import load_dataset
from model import build_model

X, y, encoder = load_dataset()

model = build_model(
    timesteps=X.shape[1],
    features=X.shape[2],
    num_classes=len(set(y))
)

model.summary()

model.fit(
    X, y,
    epochs=30,
    batch_size=16,
    validation_split=0.2
)

model.save("gesture_model.h5")
