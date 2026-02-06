# model.py


import tensorflow as tf
from tensorflow.keras import layers

def build_model(timesteps, features, num_classes):
    model = tf.keras.Sequential([
        layers.Input(shape=(timesteps, features)),

        layers.Conv1D(16, 5, activation='relu'),
        layers.MaxPooling1D(2),

        layers.Conv1D(32, 3, activation='relu'),
        layers.MaxPooling1D(2),

        layers.Flatten(),
        layers.Dense(32, activation='relu'),
        layers.Dense(num_classes, activation='softmax')
    ])

    model.compile(
        optimizer='adam',
        loss='sparse_categorical_crossentropy',
        metrics=['accuracy']
    )

    return model
