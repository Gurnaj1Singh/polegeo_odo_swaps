"""Lightweight run-time instrumentation for the snow-pole localization pipeline.

(Faster-LIO integration) Measures wall-clock, per-stage and per-frame timing so
FastReg vs Faster-LIO runs can be compared on **speed** as well as accuracy.
Pure standard-library, side-effect-free on import (no third-party deps, no
matplotlib), so it is safe to import from the heavy localization scripts.

Each finished run writes:
  * fasterlio_integration/output/timing_<odom>_gnss<pct>.json  (this run)
  * fasterlio_integration/output/timing_comparison.csv         (one row appended)

Typical use inside a script::

    from perf_timer import RunTimer
    RT = RunTimer(label='gnss_percentage')      # starts the wall-clock now
    RT.start('model_load'); model = load(...)   ; RT.stop('model_load')
    RT.odom = 'Faster-LIO'                       # once the CSV name is known
    RT.start('bag_load');  data = read_bag(...) ; RT.stop('bag_load')
    RT.start('loop')
    for frame in frames:
        t = RT.tic(); results = model(img); RT.add_detection(RT.toc(t))
    RT.stop('loop')
    RT.finish(n_frames=N, n_pole_events=M, extra={...accuracy...})
"""

import os
import csv
import json
import time
import platform
import datetime
import statistics

# fasterlio_integration/output lives next to this file (project root).
_OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        'fasterlio_integration', 'output')

# Canonical, fixed column order for timing_comparison.csv so rows from *any* run
# type (pipeline, gnss_percentage, ...) always align. The per-run JSON still holds
# the complete summary; anything not listed here is dropped from the CSV only.
_CSV_FIELDS = [
    'odometry', 'gnss_percentage', 'label', 'timestamp', 'host', 'cpu_count',
    'wall_clock_s', 'frames_total', 'pole_detection_events', 'detection_frames',
    'detection_ms_mean', 'detection_ms_median', 'detection_ms_p95', 'detection_fps',
    'stage_model_load_s', 'stage_bag_load_s', 'stage_loop_s', 'pipeline_fps',
    'err_pred_median', 'err_pred_mean', 'err_pred_max',
    'err_odom_median', 'err_odom_mean', 'err_odom_max',
    'gnss_used_count', 'predictive_count',
]


class RunTimer:
    """Accumulates timing for one pipeline run and serialises a summary."""

    def __init__(self, odom_name='unknown', gnss_percentage=0.0,
                 label='pipeline', out_dir=None):
        self.odom = odom_name
        self.gnss_pct = float(gnss_percentage)
        self.label = label
        self.out_dir = out_dir or _OUT_DIR
        self.t0 = time.perf_counter()          # full wall-clock start
        self.stages = {}                        # name -> seconds
        self._marks = {}                        # name -> perf_counter at start()
        self.detections = []                    # per-frame YOLO seconds

    # -- one-shot stage timing -------------------------------------------------
    def start(self, name):
        self._marks[name] = time.perf_counter()

    def stop(self, name):
        if name in self._marks:
            self.stages[name] = time.perf_counter() - self._marks.pop(name)
        return self.stages.get(name)

    # -- per-frame detection timing -------------------------------------------
    @staticmethod
    def tic():
        return time.perf_counter()

    @staticmethod
    def toc(t):
        return time.perf_counter() - t

    def add_detection(self, seconds):
        self.detections.append(float(seconds))

    # -- finalise --------------------------------------------------------------
    def summary(self, n_frames=None, n_pole_events=None, extra=None):
        total = time.perf_counter() - self.t0
        det_ms = sorted(d * 1000.0 for d in self.detections)
        n_det = len(det_ms)
        s = {
            'odometry': self.odom,
            'gnss_percentage': self.gnss_pct,
            'label': self.label,
            'timestamp': datetime.datetime.now().isoformat(timespec='seconds'),
            'host': platform.node(),
            'cpu_count': os.cpu_count(),
            'wall_clock_s': round(total, 2),
            'frames_total': n_frames,
            'pole_detection_events': n_pole_events,
            'detection_frames': n_det,
            'detection_ms_mean': round(statistics.mean(det_ms), 2) if det_ms else None,
            'detection_ms_median': round(statistics.median(det_ms), 2) if det_ms else None,
            'detection_ms_p95': round(det_ms[max(0, int(0.95 * n_det) - 1)], 2) if n_det >= 20 else None,
            'detection_fps': round(n_det / sum(self.detections), 2) if self.detections else None,
        }
        for k, v in self.stages.items():
            s['stage_%s_s' % k] = round(v, 2)
        loop_s = self.stages.get('loop')
        if n_frames and loop_s:
            s['pipeline_fps'] = round(n_frames / loop_s, 2)
        if extra:
            s.update(extra)
        return s

    def finish(self, n_frames=None, n_pole_events=None, extra=None, quiet=False):
        s = self.summary(n_frames=n_frames, n_pole_events=n_pole_events, extra=extra)
        try:
            os.makedirs(self.out_dir, exist_ok=True)
            # include the run label so e.g. a 'pipeline' and a 'gnss_percentage' run of
            # the same backend/GNSS% do not overwrite each other's JSON.
            tag = '%s_%s_gnss%g' % (str(self.odom).replace(' ', ''), self.label, self.gnss_pct)
            json_path = os.path.join(self.out_dir, 'timing_%s.json' % tag)
            with open(json_path, 'w') as f:
                json.dump(s, f, indent=2)
            csv_path = os.path.join(self.out_dir, 'timing_comparison.csv')
            new = not os.path.exists(csv_path)
            with open(csv_path, 'a', newline='') as f:
                w = csv.DictWriter(f, fieldnames=_CSV_FIELDS, extrasaction='ignore')
                if new:
                    w.writeheader()
                w.writerow(s)
        except Exception as e:                                  # never crash a run over metrics
            print('[perf_timer] WARNING: could not write metrics: %s' % e)
            json_path = None
        if not quiet:
            self._print(s, json_path)
        return s

    @staticmethod
    def _print(s, json_path):
        line = '=' * 60
        print('\n' + line)
        print(' RUN-TIME SUMMARY  (%s, GNSS %.0f%%)' % (s['odometry'], s['gnss_percentage']))
        print(line)
        print('  wall-clock total      : %8.1f s  (%.1f min)'
              % (s['wall_clock_s'], s['wall_clock_s'] / 60.0))
        for k in ('stage_model_load_s', 'stage_bag_load_s', 'stage_loop_s'):
            if k in s:
                print('  %-22s: %8.1f s' % (k.replace('stage_', '').replace('_s', ''), s[k]))
        if s.get('pipeline_fps'):
            print('  pipeline throughput   : %8.1f frames/s (%d frames)'
                  % (s['pipeline_fps'], s['frames_total']))
        if s.get('detection_ms_mean') is not None:
            print('  YOLO detection/frame  : %8.1f ms mean / %.1f ms median  (%d frames, %.1f fps)'
                  % (s['detection_ms_mean'], s['detection_ms_median'],
                     s['detection_frames'], s['detection_fps']))
        for key, lbl in (('err_pred_median', 'pole-corrected median err'),
                         ('err_odom_median', 'odometry-only median err')):
            if key in s and s[key] is not None:
                print('  %-22s: %8.2f m' % (lbl, s[key]))
        if json_path:
            print('  metrics ->', json_path)
        print(line + '\n')
