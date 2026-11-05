"""Phase 3, CPU-only parts of the modality study (no Docker):

  ../venv/Scripts/python paper_ipas/analysis/phase3_imu_only.py      (from odometry_benchmark/)

  IMU-only        strapdown dead reckoning from the built-in IMU alone: gyro-integrated attitude, gravity removed,
                  accelerometer integrated to velocity, with the vehicle constraint that velocity points along the
                  vehicle's forward axis (no sideways / vertical motion).
  Gyro+<odometry> heading from the gyro (as in IMU-only) combined with the travelled distance per scan of the
                  FastReg / FAST-LIO2 / Faster-LIO trajectory: tests whether the IMU's benefit is its heading.
  FastReg-nolevel FastReg without the one-off gravity levelling: what that single IMU use contributes.

Calibration uses only the stationary start (paper_ipas/data/imu_native_noise.json): gyro bias = mean rate, initial
attitude levelled from the mean specific force. Vehicle forward = IMU +x (accelerometer x rises when the car pulls
away; paper_ipas/data/imu.npz). Trajectories -> outputs/imu_only/*.txt (TUM, Ouster sensor clock); bridged and scored
exactly like the paper's methods (scripts/02, scripts/04 with cache_comb4) -> outputs/scores_imu/.
"""
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
PY = sys.executable
OUT = ROOT / "outputs" / "imu_only"
G = 9.80665
FWD = np.array([1.0, 0.0, 0.0])


def attitude(t, gyro, acc, still):
    """World-from-IMU rotations at every sample: levelled at the start, then gyro-integrated (bias removed)."""
    up = np.mean(acc[still], axis=0)
    up /= np.linalg.norm(up)
    R0 = Rotation.align_vectors([[0, 0, 1]], [up])[0]  # maps the measured up vector to world z
    bias = gyro[still].mean(0)
    dt = np.diff(t, prepend=t[0])
    steps = Rotation.from_rotvec((gyro - bias) * dt[:, None])
    R = [R0]
    for k in range(1, len(t)):
        R.append(R[-1] * steps[k])
    return Rotation.concatenate(R), bias


def heading(R):
    f = R.apply(FWD)
    return np.arctan2(f[:, 1], f[:, 0])


def write_tum(path, t, xyz):
    with open(path, "w") as f:
        f.write("#timestamp x y z q_x q_y q_z q_w\n")
        for ti, p in zip(t, xyz):
            f.write(f"{ti:.6f} {p[0]:.6f} {p[1]:.6f} {p[2]:.6f} 0 0 0 1\n")


def imu_only(t, acc, R, still):
    """Strapdown with the non-holonomic constraint (velocity kept along the forward axis, horizontal)."""
    f_w = R.apply(acc)
    g_w = f_w[still].mean(0)  # gravity as measured at rest (absorbs the accelerometer bias at the start)
    h = heading(R)
    v, p = 0.0, np.zeros(2)
    xy = np.zeros((len(t), 2))
    for k in range(1, len(t)):
        dt = t[k] - t[k - 1]
        a = f_w[k] - g_w
        d = np.array([np.cos(h[k]), np.sin(h[k])])
        v = 0.0 if still[k] else v + float(a[:2] @ d) * dt  # forward speed only
        p = p + v * d * dt
        xy[k] = p
    return xy


def gyro_distance(t, R, tum):
    """Heading from the gyro, travelled distance per scan from a LiDAR(-inertial) trajectory."""
    tr = np.loadtxt(tum)
    ts, step = tr[:, 0], np.r_[0, np.linalg.norm(np.diff(tr[:, 1:4], axis=0), axis=1)]
    h = np.interp(ts, t, np.unwrap(heading(R)))
    xy = np.cumsum(np.c_[step * np.cos(h), step * np.sin(h)], axis=0)
    return ts, xy


def score(name, tum, key, level=False):
    csv = ROOT / "outputs" / "csv" / f"incremental_navigation_results_{key}.csv"
    br = subprocess.run([PY, "scripts/02_bridge_to_csv.py", "--tum", str(tum), "--out", str(csv)]
                        + (["--level-gravity"] if level else []), cwd=ROOT, capture_output=True, text=True)
    print(name, [l for l in br.stdout.split("\n") if "align" in l])
    for s, pcts in ((0, ["0", "2", "5", "10", "25", "50"]), (1, ["10", "25", "50"]), (2, ["10", "25", "50"])):
        r = subprocess.run([PY, "scripts/04_gnss_sweep.py", "--csv", str(csv), "--name", name, "--seed", str(s),
                            "--pcts", *pcts, "--cache-dir", "cache_comb4", "--out-dir", "outputs/scores_imu"],
                           cwd=ROOT, capture_output=True, text=True)
        if s == 0:
            print("  ", [json.loads(l)["err_pred_median"] for l in r.stdout.split("\n") if l.startswith("{")])


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    z = np.load(ROOT / "paper_ipas" / "data" / "imu.npz")
    t, gyro, acc = z["t"], z["gyro"], z["acc"]
    still_s = json.loads((ROOT / "paper_ipas" / "data" / "imu_native_noise.json").read_text())["stationary_s"]
    still = t <= t[0] + still_s
    R, bias = attitude(t, gyro, acc, still)
    print("gyro bias (rad/s)", bias.round(5))

    xy = imu_only(t, acc, R, still)
    sub = slice(0, None, 10)  # 10 Hz is enough for the bridge
    write_tum(OUT / "imu_only_tum.txt", t[sub], np.c_[xy[sub], np.zeros(len(t[sub]))])
    hybrids = [("Gyro+FastReg", "gyro_fastreg", ROOT / "outputs" / "fastreg" / "fastreg_tum.txt"),
               ("Gyro+FAST-LIO2", "gyro_fastlio2", ROOT / "outputs" / "fastlio2" / "fastlio2_tum.txt"),
               ("Gyro+Faster-LIO", "gyro_fasterlio", ROOT / "outputs" / "fasterlio" / "fasterlio_tum.txt")]
    for _, key, tum in hybrids:
        ts, xy2 = gyro_distance(t, R, tum)
        write_tum(OUT / f"{key}_tum.txt", ts, np.c_[xy2, np.zeros(len(ts))])

    score("IMU-only", OUT / "imu_only_tum.txt", "imu_only")
    for name, key, _ in hybrids:
        score(name, OUT / f"{key}_tum.txt", key)
    score("FastReg-nolevel", ROOT / "outputs" / "fastreg" / "fastreg_tum.txt", "fastreg_nolevel", level=False)


if __name__ == "__main__":
    main()
