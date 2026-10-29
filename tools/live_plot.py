# -*- coding: utf-8 -*-
"""实时刷新训练曲线：左侧 loss，右侧 mAP，每 5 秒更新一次"""
import csv
import time
from pathlib import Path

import matplotlib.pyplot as plt

CSV = Path(r"C:\yolo_data\train\results.csv")

plt.ion()
fig, axes = plt.subplots(1, 2, figsize=(13, 5))
fig.canvas.manager.set_window_title("YOLO8 Training - Live")

while True:
    if CSV.exists():
        try:
            rows = list(csv.DictReader(open(CSV, encoding="utf-8")))
            if rows:
                epoch = [float(r["epoch"]) for r in rows]
                for ax in axes:
                    ax.clear()
                axes[0].plot(epoch, [float(r["train/box_loss"]) for r in rows], label="box_loss")
                axes[0].plot(epoch, [float(r["train/cls_loss"]) for r in rows], label="cls_loss")
                axes[0].plot(epoch, [float(r["train/dfl_loss"]) for r in rows], label="dfl_loss")
                axes[0].set_xlabel("epoch")
                axes[0].set_ylabel("loss")
                axes[0].set_title("Train Loss")
                axes[0].legend()
                axes[0].grid(True)

                axes[1].plot(epoch, [float(r["metrics/mAP50(B)"]) for r in rows], label="mAP50", marker="o", ms=3)
                axes[1].plot(epoch, [float(r["metrics/mAP50-95(B)"]) for r in rows], label="mAP50-95", marker="o", ms=3)
                axes[1].set_xlabel("epoch")
                axes[1].set_ylabel("mAP")
                axes[1].set_title(f"Val mAP  (epoch {int(epoch[-1])})")
                axes[1].legend()
                axes[1].grid(True)

                fig.canvas.draw()
                fig.canvas.flush_events()
        except Exception:
            pass
    time.sleep(5)
