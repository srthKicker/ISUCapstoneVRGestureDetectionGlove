# This code loads the data from CSV files, processes it, and prepares it for training a machine learning model.
# It reads the quaternion data from the CSV files, extracts the relevant features, and encodes the labels for classification tasks. 
# The resulting arrays X and y can then be used for training a model.

import numpy as np
import pandas as pd
import glob
from sklearn.preprocessing import LabelEncoder

TIMESTEPS = 100
FEATURES = 6 * 4

def load_dataset(path="data/*.csv"):
    X, y = [], []

    for file in glob.glob(path):
        label = file.split("/")[-1].split("_")[0]
        df = pd.read_csv(file)

        data = df[['q0','q1','q2','q3']].values

        if len(data) >= TIMESTEPS:
            X.append(data[:TIMESTEPS])
            y.append(label)

    X = np.array(X)

    encoder = LabelEncoder()
    y = encoder.fit_transform(y)

    return X, y, encoder
