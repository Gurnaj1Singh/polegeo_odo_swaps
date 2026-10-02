#!/usr/bin/env python3
"""(Faster-LIO integration) Paper figures from the timing/accuracy metrics.

Reads `fasterlio_integration/output/timing_comparison.csv` (written by perf_timer via
the instrumented localization scripts) and produces two PNGs in the same folder:

  1. sweep_error_vs_gnss.png   — pole-corrected error vs GNSS availability, per backend.
  2. pipeline_time_breakdown.png — where the pipeline wall-clock goes, per backend
     (model-load / bag-load / setup·kriging / YOLO detection / loop-other).

Headless (Agg). Run:  python fasterlio_integration/scripts/30_plot_timing_and_sweep.py
"""
import os
import csv
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.normpath(os.path.join(HERE, '..', 'output'))
CSV = os.path.join(OUT, 'timing_comparison.csv')

# consistent colour per backend across both figures
BACKEND_COLOR = {'FastReg': '#e8853a', 'Faster-LIO': '#3a7ce8'}
BACKEND_MARKER = {'FastReg': 'o', 'Faster-LIO': 's'}


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return None


def load_rows():
    with open(CSV) as f:
        return list(csv.DictReader(f))


# --------------------------------------------------------------------------- #
# Figure 1 — pole-corrected error vs GNSS availability (the sweep)
# --------------------------------------------------------------------------- #
def plot_sweep(rows):
    sweep = [r for r in rows if r['label'] == 'gnss_percentage']
    by = {}
    for r in sweep:
        by.setdefault(r['odometry'], []).append((_f(r['gnss_percentage']),
                                                  _f(r['err_pred_median'])))
    if not by:
        print('[plot] no gnss_percentage rows — skipping sweep figure')
        return
    fig, ax = plt.subplots(figsize=(7.2, 4.8))
    for backend in sorted(by):
        pts = sorted(p for p in by[backend] if p[1] is not None)
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, marker=BACKEND_MARKER.get(backend, 'o'), markersize=8,
                linewidth=2.2, color=BACKEND_COLOR.get(backend), label=backend)
        for x, y in zip(xs, ys):
            ax.annotate(f'{y:.2f}', (x, y), textcoords='offset points',
                        xytext=(6, 7), fontsize=9, color=BACKEND_COLOR.get(backend))
    ax.set_yscale('log')
    ax.set_xlabel('GNSS availability during the drive (%)', fontsize=12)
    ax.set_ylabel('Pole-corrected error — median (m, log scale)', fontsize=12)
    ax.set_title('Localization error vs GNSS availability\n(georeferenced snow-pole correction, seeded sweep)',
                 fontsize=12.5)
    ax.grid(True, which='both', ls=':', alpha=0.5)
    ax.legend(title='LiDAR odometry backend', fontsize=11, title_fontsize=11)
    xs_all = sorted({p[0] for v in by.values() for p in v})
    ax.set_xticks(xs_all)
    fig.tight_layout()
    path = os.path.join(OUT, 'sweep_error_vs_gnss.png')
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print('[plot] wrote', path)


# --------------------------------------------------------------------------- #
# Figure 2 — pipeline wall-clock breakdown (0% GNSS head-to-head)
# --------------------------------------------------------------------------- #
def plot_breakdown(rows):
    runs = [r for r in rows if r['label'] == 'pipeline']
    if not runs:
        print('[plot] no pipeline rows — skipping breakdown figure')
        return
    # segment order (bottom→top of each stacked bar) + colours
    seg_names = ['model load', 'bag load', 'setup (kriging, transforms)',
                 'YOLO detection', 'loop (geo-loc, decode)']
    seg_colors = ['#9467bd', '#8c9e3a', '#e8853a', '#d62728', '#3a7ce8']

    backends, segs, totals = [], [], []
    for r in sorted(runs, key=lambda r: r['odometry']):
        model = _f(r['stage_model_load_s']) or 0.0
        bag = _f(r['stage_bag_load_s']) or 0.0
        loop = _f(r['stage_loop_s']) or 0.0
        wall = _f(r['wall_clock_s']) or 0.0
        det = (_f(r['detection_frames']) or 0) * (_f(r['detection_ms_mean']) or 0) / 1000.0
        loop_other = max(loop - det, 0.0)
        setup = max(wall - model - bag - loop, 0.0)   # kriging + GNSS transforms etc.
        backends.append(r['odometry'])
        segs.append([model, bag, setup, det, loop_other])
        totals.append(wall)

    fig, ax = plt.subplots(figsize=(8.4, 3.4 + 0.5 * len(backends)))
    y = range(len(backends))
    for si, (name, col) in enumerate(zip(seg_names, seg_colors)):
        left = [sum(segs[bi][:si]) for bi in range(len(backends))]
        vals = [segs[bi][si] for bi in range(len(backends))]
        ax.barh(list(y), vals, left=left, color=col, label=name, edgecolor='white')
        for bi in range(len(backends)):
            if vals[bi] > 3.5:   # label only wide-enough segments
                ax.text(left[bi] + vals[bi] / 2, bi, f'{vals[bi]:.0f}s',
                        ha='center', va='center', fontsize=9, color='white')
    for bi in range(len(backends)):
        ax.text(totals[bi] + 1.5, bi, f'{totals[bi]:.0f}s total',
                va='center', fontsize=10, fontweight='bold')
    ax.set_yticks(list(y))
    ax.set_yticklabels(backends, fontsize=11)
    ax.set_xlabel('Wall-clock time (s)', fontsize=12)
    ax.set_title('Pipeline running-time breakdown (0% GNSS, 5,420 frames)\n'
                 'detection + geo-localization is odometry-agnostic', fontsize=12.5)
    ax.set_xlim(0, max(totals) * 1.16)
    ax.legend(ncol=3, fontsize=9, loc='lower right', framealpha=0.9)
    ax.grid(True, axis='x', ls=':', alpha=0.5)
    fig.tight_layout()
    path = os.path.join(OUT, 'pipeline_time_breakdown.png')
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print('[plot] wrote', path)


if __name__ == '__main__':
    rows = load_rows()
    plot_sweep(rows)
    plot_breakdown(rows)
