"""Train one YOLO11n on the SnowPole Detection dataset's "Combination 4" input (combined_color) and save
its validation/test metrics plus per-image test predictions (for 20_compare_yolo_init.py).

  python scripts/17_train_yolo11n_comb4.py --init coco    --seed 0   # -> models/comb4_coco_s0/
  python scripts/17_train_yolo11n_comb4.py --init scratch --seed 0   # -> models/comb4_scratch_s0/

Combination 4 = the three Ouster image modalities stacked as one 16-bit, 3-channel PNG (1024 x 128):
OpenCV/BGR channel order (B, G, R) = (near-IR, signal, reflectivity). Every dataset image is a raw frame of
the SnowPole bag (byte-identical to /ouster/{nearir,signal,reflec}_image, see 18_dataset_frame_map.py),
so 03_cache_detections.py --input comb4 rebuilds exactly this input from the bag.

What the detector sees: Ultralytics reads each 16-bit PNG with cv2.IMREAD_COLOR, i.e. uint8 = raw16 >> 8,
BGR order, then flips to RGB and divides by 255 internally. Inference must hand it the same uint8 BGR array.

Training choices (Ultralytics defaults except where noted):
  - init: COCO-pretrained yolo11n.pt, or yolo11n.yaml (random weights, "from scratch")
  - up to 500 epochs, early stop after 50 epochs without a better validation fitness
    (0.1 mAP50 + 0.9 mAP50-95); best.pt = the best-fitness epoch; batch 16, deterministic
  - imgsz 1024: the images are 1024 px wide, so nothing is down-scaled (640 would shrink poles that are
    only a few pixels wide to ~60 %)
  - hsv_h 0: the channels are LiDAR modalities, not colours, so a hue rotation would mix them; the
    saturation/value jitter (per-pixel contrast/brightness) is kept
The test split is used only after training (never for early stopping or checkpoint choice).
"""
import argparse
import json
import shutil
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
SRC = ROOT.parent / ("SnowPole Detection A Comprehensive Dataset for Detection and Localization Using LiDAR "
                     "Imaging in Nordic Winter Conditions") / "SnowPole_Detection_Dataset"
SPLITS = {"train": "train", "val": "valid", "test": "test"}
DATA_DIR = ROOT / "data" / "snowpole_comb4"


def build_dataset(dst):
    """combined_color/<split> + labels/<split> -> Ultralytics layout images/<split>, labels/<split>."""
    for s, src in SPLITS.items():
        for kind, sub, ext in (("images", "combined_color", ".png"), ("labels", "labels", ".txt")):
            d = dst / kind / s
            d.mkdir(parents=True, exist_ok=True)
            for f in sorted((SRC / sub / src).glob("*" + ext)):
                if not (d / f.name).exists():
                    shutil.copy2(f, d / f.name)
    yaml = dst / "data.yaml"
    yaml.write_text(f"path: {dst.as_posix()}\ntrain: images/train\nval: images/val\ntest: images/test\n"
                    "nc: 1\nnames: ['snow_pole']\n")
    return yaml


def load_bgr(f):
    """The array Ultralytics' own loader produces for a 16-bit PNG: uint8 BGR = raw16 >> 8."""
    return (cv2.imread(str(f), cv2.IMREAD_UNCHANGED) >> 8).astype(np.uint8)


def ground_truth(f, w, h):
    gt = []
    for line in (DATA_DIR / "labels" / "test" / (f.stem + ".txt")).read_text().split("\n"):
        if line.strip():
            _, cx, cy, bw, bh = map(float, line.split())
            gt.append([(cx - bw / 2) * w, (cy - bh / 2) * h, (cx + bw / 2) * w, (cy + bh / 2) * h])
    return np.array(gt, dtype=float).reshape(-1, 4)


VAL_PAD = 16  # rows of grey (114) above and below, as Ultralytics' val letterbox: 1024x128 -> 1024x160


def predict(model, bgr, conf, imgsz=1024):
    """(n, 5) [x1, y1, x2, y2, conf] in image pixels. predict() would letterbox 1024x128 with no padding at all
    (auto stride padding); validation (where mAP and early stopping are measured) pads to 1024x160, and the
    model needs that context: without it test mAP50-95 fell 0.377 -> 0.302 and recall at conf 0.6 0.57 -> 0.43
    (28-epoch check model). So inference reproduces the val input exactly. Same in 03_cache_detections.py."""
    img = cv2.copyMakeBorder(bgr, VAL_PAD, VAL_PAD, 0, 0, cv2.BORDER_CONSTANT, value=(114, 114, 114))
    b = model.predict(img, imgsz=imgsz, conf=conf, iou=0.7, max_det=300, verbose=False)[0].boxes
    p = np.hstack([b.xyxy.cpu().numpy(), b.conf.cpu().numpy()[:, None]])
    p[:, [1, 3]] = np.clip(p[:, [1, 3]] - VAL_PAD, 0, bgr.shape[0])
    return p


def dump_test_predictions(model, imgsz, out):
    """Per test image: ground-truth boxes and all predictions down to conf 0.001 (as Ultralytics' mAP)."""
    rec = {}
    for f in sorted((DATA_DIR / "images" / "test").glob("*.png")):
        bgr = load_bgr(f)
        rec[f.stem + "_gt"] = ground_truth(f, bgr.shape[1], bgr.shape[0])
        rec[f.stem + "_pred"] = predict(model, bgr, 0.001, imgsz)
    np.savez_compressed(out, **rec)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--init", choices=["coco", "scratch"], required=True)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--epochs", type=int, default=500)
    ap.add_argument("--patience", type=int, default=50)
    ap.add_argument("--imgsz", type=int, default=1024)
    ap.add_argument("--batch", type=int, default=16)
    args = ap.parse_args()
    from ultralytics import YOLO

    name = f"comb4_{args.init}_s{args.seed}"
    run = ROOT / "models" / name
    yaml = build_dataset(DATA_DIR)
    last = run / "weights" / "last.pt"
    if (run / "train_done").exists():
        pass
    elif last.exists():
        YOLO(str(last)).train(resume=True)  # interrupted run (e.g. reboot) continues where it stopped
    else:
        model = YOLO("yolo11n.pt" if args.init == "coco" else "yolo11n.yaml")
        model.train(data=str(yaml), epochs=args.epochs, patience=args.patience, imgsz=args.imgsz,
                    batch=args.batch, seed=args.seed, deterministic=True, hsv_h=0.0, workers=4,
                    pretrained=args.init == "coco", project=str(ROOT / "models"), name=name, exist_ok=True)
    (run / "train_done").touch()

    best = run / "weights" / "best.pt"
    model = YOLO(str(best))
    rows = [r.split(",") for r in (run / "results.csv").read_text().strip().split("\n")]
    out = {"run": name, "init": args.init, "seed": args.seed, "weights": str(best.relative_to(ROOT)),
           "epochs_run": len(rows) - 1, "imgsz": args.imgsz,
           "input": "combined_color (B,G,R = near-IR, signal, reflec), uint8 = raw16 >> 8"}
    for split in ("val", "test"):
        m = model.val(data=str(yaml), split=split, imgsz=args.imgsz, batch=args.batch, plots=False,
                      project=str(run), name=f"eval_{split}", exist_ok=True)
        p, r = float(m.box.mp), float(m.box.mr)
        out[split] = {"precision": round(p, 4), "recall": round(r, 4), "f1": round(2 * p * r / max(p + r, 1e-12), 4),
                      "mAP50": round(float(m.box.map50), 4), "mAP50-95": round(float(m.box.map), 4)}
        out[split]["fitness"] = round(0.1 * out[split]["mAP50"] + 0.9 * out[split]["mAP50-95"], 4)
    dump_test_predictions(model, args.imgsz, run / "test_predictions.npz")
    (run / "eval.json").write_text(json.dumps(out, indent=2))
    print(json.dumps(out))


if __name__ == "__main__":
    main()
