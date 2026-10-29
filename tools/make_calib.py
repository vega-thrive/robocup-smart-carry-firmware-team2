# -*- coding: utf-8 -*-
"""从训练集抽取 95 张按类别均衡的 INT8 标定图并打包 zip"""
import random, zipfile
from pathlib import Path
from collections import defaultdict

TRAIN_IMG = Path(r"C:\yolo_data\images\train")
TRAIN_LBL = Path(r"C:\yolo_data\labels\train")
ANN_DIR = Path(r"C:\Users\HP\Downloads\搬运 (1)\annotations")
DATA_YAML = Path(r"C:\yolo_data\data.yaml")
OUT_ZIP = Path(r"C:\Users\HP\Desktop\calib_95.zip")
N = 95
PER_CLASS = N // 7

names = {}
for line in DATA_YAML.read_text(encoding="utf-8").splitlines():
    if ":" in line and line.strip()[:1].isdigit():
        i, nm = line.split(":", 1)
        names[int(i)] = nm.strip()

xml_stems = {x.stem for x in ANN_DIR.glob("*.xml")}
buckets = defaultdict(list)
for p in sorted(TRAIN_IMG.iterdir()):
    if p.suffix.lower() not in {".jpg", ".jpeg", ".png", ".bmp", ".webp"} or p.stem not in xml_stems:
        continue
    lbl = TRAIN_LBL / (p.stem + ".txt")
    if not lbl.is_file():
        continue
    ids = {int(l.split()[0]) for l in lbl.read_text(encoding="utf-8").splitlines() if l.strip()}
    for c in ids:
        buckets[c].append(p)

rng = random.Random(42)
picked, per_class_pick = set(), {}
for c in sorted(buckets):
    b = buckets[c]
    rng.shuffle(b)
    k = min(PER_CLASS, len(b))
    per_class_pick[c] = k
    picked.update(b[:k])

pool, seen = [], set()
for p in buckets.values():
    for q in p:
        if q not in seen:
            seen.add(q)
            pool.append(q)
rng.shuffle(pool)
for p in pool:
    if len(picked) >= N:
        break
    picked.add(p)

if len(picked) < N:
    print(f"[警告] 可用标定图仅 {len(picked)} 张，少于 {N}")

with zipfile.ZipFile(OUT_ZIP, "w", zipfile.ZIP_DEFLATED) as z:
    for p in sorted(picked, key=lambda x: x.name):
        z.write(p, p.name)

print(f"完成: {len(picked)} 张 -> {OUT_ZIP}  ({OUT_ZIP.stat().st_size / 1024:.0f} KB)")
for c in sorted(per_class_pick):
    print(f"  {names.get(c, c)}: {per_class_pick[c]} 张")
