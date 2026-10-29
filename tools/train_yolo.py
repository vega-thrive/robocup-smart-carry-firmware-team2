# -*- coding: utf-8 -*-
"""VOC XML -> YOLO txt -> 8:2 划分 -> YOLO11n 训练 -> best.onnx(224x320)"""
import sys, shutil, random
from pathlib import Path
from xml.etree import ElementTree as ET

SRC = Path(next((a for a in sys.argv[1:] if not a.startswith("--")), r"C:\Users\HP\Downloads\搬运 (1)"))
DST = Path(r"C:\yolo_data")
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
CLEAN = "--clean" in sys.argv

try:
    from PIL import Image
except ImportError:
    Image = None


def parse_voc(xml_path):
    root = ET.parse(xml_path).getroot()
    size = root.find("size")
    w = float(size.findtext("width")) if size is not None else 0
    h = float(size.findtext("height")) if size is not None else 0
    objs = []
    for obj in root.findall("object"):
        name = obj.findtext("name")
        box = obj.find("bndbox")
        if not name or box is None:
            continue
        objs.append((name, float(box.findtext("xmin")), float(box.findtext("ymin")),
                     float(box.findtext("xmax")), float(box.findtext("ymax"))))
    return objs, w, h


def find_image(img_dir, stem):
    for e in IMG_EXTS:
        p = img_dir / (stem + e)
        if p.is_file():
            return p
    return None


def main():
    global DST
    img_dir, ann_dir = SRC / "images", SRC / "annotations"
    if not img_dir.is_dir() or not ann_dir.is_dir():
        sys.exit(f"[错误] 目录结构不对: {SRC} 下需要 images/ 和 annotations/")

    if CLEAN:
        shutil.rmtree(DST, ignore_errors=True)
        print(f"[清理] 已删除 {DST}")
    try:
        DST.mkdir(parents=True, exist_ok=True)
    except OSError:
        alt = Path.cwd() / "yolo_data"
        alt.mkdir(parents=True, exist_ok=True)
        print(f"[警告] 无法创建 {DST}，改用 {alt}")
        DST = alt
    if (DST / "images").exists() and any((DST / "images").iterdir()):
        sys.exit(f"[错误] {DST} 里已有旧数据，继续会混入训练。确认清除请加 --clean 重跑")

    classes, cls_idx, records, skipped = [], {}, [], 0
    for xml_path in sorted(ann_dir.glob("*.xml")):
        objs, w, h = parse_voc(xml_path)
        img_path = find_image(img_dir, xml_path.stem)
        if not objs or img_path is None:
            skipped += 1
            continue
        if Image is not None:
            try:
                with Image.open(img_path) as im:
                    w, h = im.size
            except OSError:
                pass
        if not (w and h):
            skipped += 1
            continue
        lines = []
        for name, xmin, ymin, xmax, ymax in objs:
            if name not in cls_idx:
                cls_idx[name] = len(classes)
                classes.append(name)
            c = cls_idx[name]
            lines.append(f"{c} {min(max((xmin + xmax) / 2 / w, 0), 1):.6f} "
                         f"{min(max((ymin + ymax) / 2 / h, 0), 1):.6f} "
                         f"{min(max((xmax - xmin) / w, 0), 1):.6f} "
                         f"{min(max((ymax - ymin) / h, 0), 1):.6f}")
        records.append((img_path, "\n".join(lines)))

    if not classes:
        sys.exit("[错误] 没有解析到任何标注")
    print(f"类别({len(classes)}): {classes}   有效样本: {len(records)}   跳过: {skipped}")

    rng = random.Random(42)
    rng.shuffle(records)
    n_val = max(1, round(len(records) * 0.2))
    val_records, train_records = records[:n_val], records[n_val:]
    print(f"train: {len(train_records)}   val: {len(val_records)}")

    for split, recs in (("train", train_records), ("val", val_records)):
        img_out, lbl_out = DST / "images" / split, DST / "labels" / split
        img_out.mkdir(parents=True, exist_ok=True)
        lbl_out.mkdir(parents=True, exist_ok=True)
        for img_path, label in recs:
            shutil.copy2(img_path, img_out / img_path.name)
            (lbl_out / (img_path.stem + ".txt")).write_text(label, encoding="utf-8")

    yaml_path = DST / "data.yaml"
    yaml_path.write_text(
        f"path: {DST.as_posix()}\ntrain: images/train\nval: images/val\n"
        f"nc: {len(classes)}\nnames:\n"
        + "".join(f"  {i}: {n}\n" for i, n in enumerate(classes)), encoding="utf-8")
    print(f"[1/3] 数据已写入 {DST}")

    from ultralytics import YOLO
    import torch

    device = 0 if torch.cuda.is_available() else "cpu"
    print(f"[2/3] 开始训练  GPU: {torch.cuda.get_device_name(0) if device == 0 else 'CPU!'}")
    model = YOLO("yolo11n.pt")  # 首次运行自动下载权重，需联网
    model.train(data=str(yaml_path), epochs=150, patience=20, imgsz=(224, 320),
                batch=-1, device=device, project=str(DST / "runs"), name="train", exist_ok=True)

    best = DST / "runs" / "train" / "weights" / "best.pt"
    if not best.is_file():
        sys.exit(f"[错误] 未找到 {best}")
    print("[3/3] 导出 ONNX ...")
    YOLO(str(best)).export(format="onnx", imgsz=(224, 320), device=device)
    print(f"[完成] ONNX: {DST / 'runs' / 'train' / 'weights' / 'best.onnx'}")


if __name__ == "__main__":
    main()
