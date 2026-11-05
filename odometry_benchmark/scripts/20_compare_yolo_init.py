"""COCO-pretrained vs from-scratch YOLO11n on Combination 4: which is better, statistically; then pick the
detector used for localization.

  python scripts/20_compare_yolo_init.py               # needs models/comb4_{coco,scratch}_s{0,1,2}/eval.json
  python scripts/20_compare_yolo_init.py --seeds 0     # seed 0 only (the seed experiment was stopped there)

All runs are scored by the same code on the same 197 test images (per-image predictions saved by
17_train_yolo11n_comb4.py; matching and AP as Ultralytics: IoU 0.50:0.95, ap_per_class).
  - primary test: two-level bootstrap (10 000 draws). Each draw resamples the seeds of each init with
    replacement AND the test images with replacement (the same images for both inits), and takes
    mean-over-seeds mAP50-95(COCO) - mean-over-seeds mAP50-95(scratch). This covers both training-seed
    and test-set variation. 95 % percentile CI; two-sided p = 2 * min(P(diff <= 0), P(diff >= 0)).
  - secondary: Welch t-test over the per-seed test mAP50-95 (n = 3 per init, low power; needs >= 2 seeds).
  With a single seed the bootstrap reduces to a paired bootstrap over test images (test-set variation only).
Decision rule (fixed before looking at results): an init "wins" if the bootstrap 95 % CI of the difference
excludes 0; otherwise the difference is reported as not significant and the higher mean is used.
Final detector = the winning init's seed with the best VALIDATION fitness (test is not used to pick it).
-> models/yolo11n_comb4_final.pt, results/detector_yolo11n_comb4.{json,md}
"""
import argparse
import json
import shutil
from pathlib import Path

import numpy as np
from scipy import stats
from ultralytics.utils.metrics import ap_per_class

ROOT = Path(__file__).resolve().parent.parent
INITS = ("coco", "scratch")
IOUV = np.linspace(0.5, 0.95, 10)
PAPER = {"precision": 0.884, "recall": 0.848, "f1": 0.866, "mAP50": 0.911, "mAP50-95": 0.458, "mIoU": 0.394}


def box_iou(a, b):
    tl, br = np.maximum(a[:, None, :2], b[None, :, :2]), np.minimum(a[:, None, 2:], b[None, :, 2:])
    inter = np.clip(br - tl, 0, None).prod(2)
    area = lambda x: (x[:, 2] - x[:, 0]) * (x[:, 3] - x[:, 1])
    return inter / (area(a)[:, None] + area(b)[None, :] - inter + 1e-12)


def match(gt, pred):
    """Ultralytics DetectionValidator.match_predictions (single class): (n_pred, 10) true-positive matrix."""
    tp = np.zeros((len(pred), len(IOUV)), bool)
    if len(gt) == 0 or len(pred) == 0:
        return tp
    iou = box_iou(gt, pred[:, :4])
    for k, t in enumerate(IOUV):
        m = np.array(np.nonzero(iou >= t)).T
        if m.shape[0]:
            if m.shape[0] > 1:
                m = m[iou[m[:, 0], m[:, 1]].argsort()[::-1]]
                m = m[np.unique(m[:, 1], return_index=True)[1]]
                m = m[np.unique(m[:, 0], return_index=True)[1]]
            tp[m[:, 1], k] = True
    return tp


def load(run):
    z = np.load(ROOT / "models" / run / "test_predictions.npz")
    names = sorted({k.rsplit("_", 1)[0] for k in z.files})
    per = []
    for n in names:
        gt, pred = z[n + "_gt"], z[n + "_pred"]
        per.append((match(gt, pred), pred[:, 4], len(gt), gt, pred))
    return names, per


def metrics(per, idx):
    tp = np.concatenate([per[i][0] for i in idx])
    conf = np.concatenate([per[i][1] for i in idx])
    ngt = sum(per[i][2] for i in idx)
    if len(tp) == 0:
        return 0.0, 0.0
    ap = ap_per_class(tp, conf, np.zeros(len(tp)), np.zeros(ngt))[5]
    return float(ap.mean()), float(ap[:, 0].mean())


def operating_point(per, conf_thr=0.6):
    """P/R/F1 (greedy one-to-one, IoU >= 0.5) and mIoU (mean over GT of best predicted IoU, 0 if missed)."""
    tp = fp = fn = 0
    ious = []
    for _, _, _, gt, pred in per:
        p = pred[pred[:, 4] >= conf_thr]
        iou = box_iou(gt, p[:, :4]) if len(gt) and len(p) else np.zeros((len(gt), len(p)))
        used = set()
        for g in range(len(gt)):
            ious.append(iou[g].max() if len(p) else 0.0)
            free = [(iou[g, k], k) for k in range(len(p)) if k not in used and iou[g, k] >= 0.5]
            if free:
                used.add(max(free)[1]); tp += 1
            else:
                fn += 1
        fp += len(p) - len(used)
    P, R = tp / max(tp + fp, 1), tp / max(tp + fn, 1)
    return {"conf": conf_thr, "tp": tp, "fp": fp, "fn": fn, "precision": round(P, 4), "recall": round(R, 4),
            "f1": round(2 * P * R / max(P + R, 1e-12), 4), "mIoU": round(float(np.mean(ious)), 4)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1, 2])
    SEEDS = tuple(ap.parse_args().seeds)
    runs = {(i, s): f"comb4_{i}_s{s}" for i in INITS for s in SEEDS}
    ev = {k: json.loads((ROOT / "models" / r / "eval.json").read_text()) for k, r in runs.items()}
    data = {k: load(r)[1] for k, r in runs.items()}
    n_img = len(next(iter(data.values())))
    full = np.arange(n_img)
    test = {k: metrics(data[k], full) for k in runs}

    rng = np.random.default_rng(0)
    diffs = []
    B = 10000
    for b in range(B):
        idx = rng.integers(0, n_img, n_img)
        means = {}
        for i in INITS:
            ss = rng.choice(SEEDS, len(SEEDS), replace=True)
            means[i] = np.mean([metrics(data[(i, s)], idx)[0] for s in ss])
        diffs.append(means["coco"] - means["scratch"])
        if b % 1000 == 0:
            print(f"bootstrap {b}/{B}", flush=True)
    diffs = np.array(diffs)
    lo, hi = np.percentile(diffs, [2.5, 97.5])
    p_boot = min(1.0, 2 * min((diffs <= 0).mean(), (diffs >= 0).mean()))
    per_seed = {i: [test[(i, s)][0] for s in SEEDS] for i in INITS}
    welch = stats.ttest_ind(per_seed["coco"], per_seed["scratch"], equal_var=False) if len(SEEDS) > 1 else None
    sd = lambda v: round(float(np.std(v, ddof=1)), 4) if len(v) > 1 else None
    mean = {i: float(np.mean(per_seed[i])) for i in INITS}
    significant = lo > 0 or hi < 0
    winner = max(INITS, key=lambda i: mean[i])
    final_seed = max(SEEDS, key=lambda s: ev[(winner, s)]["val"]["fitness"])
    final = runs[(winner, final_seed)]
    shutil.copy2(ROOT / "models" / final / "weights" / "best.pt", ROOT / "models" / "yolo11n_comb4_final.pt")
    op = [operating_point(data[(winner, final_seed)], c) for c in (0.25, 0.6)]

    out = {"runs": {r: {**ev[k], "test_recomputed": {"mAP50-95": round(test[k][0], 4), "mAP50": round(test[k][1], 4)}}
                    for k, r in runs.items()},
           "comparison": {"metric": "test mAP50-95", "mean_coco": round(mean["coco"], 4),
                          "mean_scratch": round(mean["scratch"], 4),
                          "sd_coco": sd(per_seed["coco"]),
                          "sd_scratch": sd(per_seed["scratch"]),
                          "diff_coco_minus_scratch": round(mean["coco"] - mean["scratch"], 4),
                          "bootstrap_ci95": [round(float(lo), 4), round(float(hi), 4)],
                          "bootstrap_p": round(float(p_boot), 4), "bootstrap_draws": B,
                          "welch_t": round(float(welch.statistic), 3) if welch else None,
                          "welch_p": round(float(welch.pvalue), 4) if welch else None, "seeds": list(SEEDS),
                          "significant": bool(significant), "winner": winner},
           "final": {"run": final, "weights": "models/yolo11n_comb4_final.pt", "test": ev[(winner, final_seed)]["test"],
                     "test_at_conf": op}}
    (ROOT / "results" / "detector_yolo11n_comb4.json").write_text(json.dumps(out, indent=2))

    c = out["comparison"]
    md = ["# YOLO11n on Combination 4 (combined_color): COCO-pretrained vs from scratch", "",
          "SnowPole Detection dataset, input uint8 BGR = (near-IR, signal, reflec) >> 8, imgsz 1024, up to 500 "
          f"epochs with early stop (patience 50 on validation fitness), hsv_h 0. Seeds {', '.join(map(str, SEEDS))} "
          "per init.", "",
          "| Run | Epochs run | Val mAP50-95 | Val fitness | Test P | Test R | Test F1 | Test mAP50 | Test mAP50-95 |",
          "|---|---|---|---|---|---|---|---|---|"]
    for k, r in runs.items():
        e = ev[k]
        t = e["test"]
        md.append(f"| {r} | {e['epochs_run']} | {e['val']['mAP50-95']} | {e['val']['fitness']} | {t['precision']} | "
                  f"{t['recall']} | {t['f1']} | {t['mAP50']} | {t['mAP50-95']} |")
    md.append("| paper draft, Combination 4 (test) |  |  |  | " + " | ".join(
        str(PAPER[x]) for x in ("precision", "recall", "f1", "mAP50", "mAP50-95")) + " |")
    md += ["", "## Statistical comparison (test mAP50-95)", "",
           f"- COCO: {c['mean_coco']} +/- {c['sd_coco']} (mean +/- sd over seeds); "
           f"scratch: {c['mean_scratch']} +/- {c['sd_scratch']}",
           f"- difference COCO - scratch: {c['diff_coco_minus_scratch']:+.4f}; two-level bootstrap (seeds x test images, "
           f"{B} draws) 95 % CI [{c['bootstrap_ci95'][0]:+.4f}, {c['bootstrap_ci95'][1]:+.4f}], p = {c['bootstrap_p']}",
           (f"- Welch t-test over seeds: t = {c['welch_t']}, p = {c['welch_p']} (n = {len(SEEDS)} per init)" if welch else
            "- one seed per init: no seed-level test; the bootstrap covers test-image variation only "
            "(seed experiment stopped after seed 0, the two inits being close)"),
           f"- **Decision: {winner.upper()} " + ("is significantly better**" if significant else
                                                  "has the higher mean, but the difference is NOT significant**"),
           "", f"## Detector used for localization: `{final}` (best validation fitness among the {winner} seeds)", "",
           "Copied to `models/yolo11n_comb4_final.pt`. At fixed confidence on the test split (greedy match, "
           "IoU >= 0.5; mIoU = mean over ground-truth boxes of the best predicted-box IoU, 0 if missed):", "",
           "| conf | TP | FP | FN | Precision | Recall | F1 | mIoU |", "|---|---|---|---|---|---|---|---|"]
    for o in op:
        md.append(f"| {o['conf']} | {o['tp']} | {o['fp']} | {o['fn']} | {o['precision']} | {o['recall']} | "
                  f"{o['f1']} | {o['mIoU']} |")
    md.append("\nThe localization pipeline uses conf 0.6 (same as the signal YOLOv5 model).")
    (ROOT / "results" / "detector_yolo11n_comb4.md").write_text("\n".join(md) + "\n")
    print("\n".join(md))


if __name__ == "__main__":
    main()
