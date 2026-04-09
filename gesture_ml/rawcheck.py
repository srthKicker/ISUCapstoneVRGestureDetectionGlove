# rawcheck.py
import pandas as pd
import matplotlib.pyplot as plt
import glob

files = sorted(glob.glob("data/*.csv"))

# Plot the first recording of each gesture
gestures_seen = {}
fig, axes = plt.subplots(5, 1, figsize=(12, 15))

for f in files:
    label = f.split("\\")[-1].split("_")[0]
    if label not in gestures_seen and len(gestures_seen) < 5:
        gestures_seen[label] = f
        idx = list(gestures_seen.keys()).index(label)
        df = pd.read_csv(f)
        # Plot just the finger 1 w component over time
        axes[idx].plot(df["imu1_w"].values, label="imu1_w")
        axes[idx].plot(df["imu1_x"].values, label="imu1_x")
        axes[idx].plot(df["imu2_w"].values, label="imu2_w")
        axes[idx].set_title(f"{label}: {f.split(chr(92))[-1]}")
        axes[idx].legend()
        axes[idx].set_ylim(-1.1, 1.1)

plt.tight_layout()
plt.savefig("rawcheck.png")
plt.show()
