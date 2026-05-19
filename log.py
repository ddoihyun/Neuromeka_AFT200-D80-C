import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# =========================
# 설정
# =========================

DATA_DIR = Path(".")   # 현재 폴더
CSV_FILES = sorted(DATA_DIR.glob("aft200_*.csv"))

USE_MOVING_AVG = False
WINDOW_SIZE = 20

CHANNELS = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]

# =========================
# 함수
# =========================

def moving_average(x, w):
    return np.convolve(x, np.ones(w)/w, mode='same')

# =========================
# 전체 파일 처리
# =========================

for csv_file in CSV_FILES:

    print(f"\nLoading: {csv_file.name}")

    # CSV 읽기
    df = pd.read_csv(csv_file)

    # timestamp -> 상대시간(sec)
    t = df["timestamp"].values
    t = (t - t[0]) * 1e-9 if t[0] > 1e12 else (t - t[0])

    # Figure 생성
    fig, axes = plt.subplots(6, 1, figsize=(14, 12), sharex=True)
    fig.suptitle(csv_file.name, fontsize=14)

    # 채널별 plotting
    for i, ch in enumerate(CHANNELS):

        y = df[ch].values

        # moving average
        if USE_MOVING_AVG:
            y_plot = moving_average(y, WINDOW_SIZE)
        else:
            y_plot = y

        # plot
        axes[i].plot(t, y_plot, linewidth=1)

        # noise statistics
        mean_val = np.mean(y)
        std_val = np.std(y)
        rms_val = np.sqrt(np.mean(y**2))

        axes[i].set_ylabel(ch)
        axes[i].grid(True)

        axes[i].set_title(
            f"{ch} | mean={mean_val:.4f}, std={std_val:.4f}, rms={rms_val:.4f}",
            fontsize=10
        )

        print(
            f"{ch:>2} | mean={mean_val:10.4f} | std={std_val:10.4f} | rms={rms_val:10.4f}"
        )

    axes[-1].set_xlabel("Time [sec]")

    plt.tight_layout()
    plt.show()