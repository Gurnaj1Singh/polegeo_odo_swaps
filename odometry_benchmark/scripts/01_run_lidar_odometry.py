"""Run GenZ-ICP or MAD-ICP over the scans extracted by 00_extract_scans.py.

GenZ-ICP and MAD-ICP run inside the odom-bench Docker image (Linux-only);
KISS-ICP 1.2.3 runs natively in the Windows venv:

  docker run --rm -v D:/Odometry/odometry_benchmark:/work odom-bench:latest \
      python scripts/01_run_lidar_odometry.py --method genz_icp --cache cache --out outputs/genz_icp

Writes <out>/<method>_tum.txt  (t x y z qx qy qz qw, t = Ouster sensor clock, s)
and <out>/timing.json (per-scan registration time).
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import pandas as pd


def rot_to_quat(R):
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
    y = np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0
    z = np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    return x, y, z, w


def make_genz(args):
    from genz_icp.config import load_config
    from genz_icp.genz_icp import GenZICP

    # GenZ-ICP's pretuned "outdoor" set, with the shared 2 m / 150 m range and deskew on.
    cfg = load_config(None)
    cfg.data.deskew = True
    cfg.data.max_range = args.max_range
    cfg.data.min_range = args.min_range
    cfg.mapping.voxel_size = 0.6
    cfg.mapping.desired_num_voxelized_points = 3000
    cfg.mapping.max_points_per_voxel = 3
    cfg.mapping.planarity_threshold = 0.2
    odom = GenZICP(cfg)

    def step(stamp, pts):
        t = pts[:, 3].astype(np.float64)
        t = t / t.max() if t.max() > 0 else t  # KISS/GenZ deskew expects [0, 1]
        odom.register_frame(pts[:, :3].astype(np.float64), t)
        return odom.last_pose

    return step


def make_kiss(args):
    from kiss_icp.config import KISSConfig
    from kiss_icp.kiss_icp import KissICP

    # KISS-ICP defaults (voxel = max_range / 100) with the shared 2 m / 150 m range.
    cfg = KISSConfig()
    cfg.data.deskew = True
    cfg.data.max_range = args.max_range
    cfg.data.min_range = args.min_range
    cfg.mapping.voxel_size = args.max_range / 100.0
    odom = KissICP(cfg)

    def step(stamp, pts):
        t = pts[:, 3].astype(np.float64)
        t = t / t.max() if t.max() > 0 else t
        odom.register_frame(pts[:, :3].astype(np.float64), t)
        return odom.last_pose

    return step


def make_rko(args, cache):
    """RKO-LIO through its Python pipeline (same C++ core as the `rko_lio` CLI used on the
    SnowPole bag), for datasets converted to the common cache: needs imu.csv and
    extrinsics.yaml (T_lidar_imu). Base frame = LiDAR, as on SnowPole."""
    import yaml
    from rko_lio.config import PipelineConfig
    from rko_lio.lio_pipeline import LIOPipeline
    from scipy.spatial.transform import Rotation

    cfg = yaml.safe_load(open(Path(__file__).resolve().parent.parent / "configs" / "rko_lio_os2_128.yaml"))
    T = np.array(yaml.safe_load(open(cache / "extrinsics.yaml"))["T_lidar_imu"])
    cfg["extrinsic_imu2base_quat_xyzw_xyz"] = [*Rotation.from_matrix(T[:3, :3]).as_quat(), *T[:3, 3]]
    cfg["extrinsic_lidar2base_quat_xyzw_xyz"] = [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    cfg["lio"]["min_range"], cfg["lio"]["max_range"] = args.min_range, args.max_range
    pipe = LIOPipeline(PipelineConfig.from_dict(cfg))
    imu = pd.read_csv(cache / "imu.csv").to_numpy()
    imu_ns = np.round(imu[:, 0] * 1e9).astype(np.int64)
    state = {"next": 0}

    def step(stamp, pts):
        start_ns = int(round(stamp * 1e9))
        t_ns = start_ns + np.round(pts[:, 3].astype(np.float64) * 1e9).astype(np.int64)
        end_ns = int(t_ns.max())
        while state["next"] < len(imu) and imu_ns[state["next"]] <= end_ns:
            i = state["next"]
            pipe.add_imu(int(imu_ns[i]), imu[i, 1:4], imu[i, 4:7])
            state["next"] += 1
        pipe.register_scan(start_ns, end_ns, pts[:, :3].astype(np.float64), t_ns)
        return pipe.lio.pose()

    step.pipeline = pipe
    return step


def make_mad(args, sensor_hz=10.0):
    from mad_icp.configurations.mad_params import MADConfiguration_lut
    from mad_icp.src.pybind.pypeline import Pipeline, VectorEigen3d

    p = MADConfiguration_lut["default"]
    # sensor rate from the data (10.0 on SnowPole), deskew on (as MAD-ICP's own Ouster configs),
    # 4 keyframes / 4 threads.
    pipe = Pipeline(sensor_hz, True, p["b_max"], p["rho_ker"], p["p_th"], p["b_min"],
                    p["b_ratio"], 4, args.threads, False)

    def step(stamp, pts):
        pipe.compute(stamp, VectorEigen3d(pts[:, :3].astype(np.float64)))
        return np.asarray(pipe.currentPose())

    return step


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--method", choices=["genz_icp", "mad_icp", "kiss_icp", "rko_lio"], required=True)
    ap.add_argument("--cache", default="cache")
    ap.add_argument("--out", required=True)
    ap.add_argument("--min-range", type=float, default=2.0)
    ap.add_argument("--max-range", type=float, default=150.0)
    ap.add_argument("--threads", type=int, default=4)
    ap.add_argument("--cpu", type=int,
                    help="pin this process to one CPU before the method loads; GenZ-ICP and KISS-ICP "
                         "reduce with tbb::parallel_reduce and are only bit-reproducible single-threaded "
                         "(oneTBB sizes its pool from the affinity mask)")
    args = ap.parse_args()
    if args.cpu is not None:
        import psutil
        psutil.Process().cpu_affinity([args.cpu])

    cache, out = Path(args.cache), Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    stamps = pd.read_csv(cache / "scan_stamps.csv")
    sensor_hz = round(1.0 / float(np.median(np.diff(stamps.sensor_stamp))), 1)
    print(f"[{args.method}] {len(stamps)} scans at {sensor_hz} Hz from {cache}", flush=True)
    if args.method == "mad_icp":
        step = make_mad(args, sensor_hz)
    elif args.method == "rko_lio":
        step = make_rko(args, cache)
    else:
        step = {"genz_icp": make_genz, "kiss_icp": make_kiss}[args.method](args)

    rows, dts = [], []
    for k, r in stamps.iterrows():
        pts = np.load(cache / "scans" / f"{int(r.idx):06d}.npy")
        t0 = time.perf_counter()
        T = step(float(r.sensor_stamp), pts)
        dts.append(time.perf_counter() - t0)
        qx, qy, qz, qw = rot_to_quat(T[:3, :3])
        rows.append((r.sensor_stamp, *T[:3, 3], qx, qy, qz, qw))
        if k % 250 == 0:
            print(f"[{args.method}] scan {k}/{len(stamps)}  pos {T[:3, 3].round(1)}  "
                  f"{1000 * np.mean(dts[-250:]):.1f} ms/scan", flush=True)

    if args.method == "rko_lio":
        # RKO-LIO stamps each pose itself (scan end); write its own trajectory as the CLI does
        ts, poses = step.pipeline.lio.poses_with_timestamps()
        rows = [(t * 1e-9, *p) for t, p in zip(ts, poses)]
    np.savetxt(out / f"{args.method}_tum.txt", np.array(rows), fmt="%.9f")
    dts = np.array(dts)
    timing = {"method": args.method, "n_scans": len(dts),
              "ms_per_scan_mean": round(1000 * dts.mean(), 2),
              "ms_per_scan_median": round(1000 * np.median(dts), 2),
              "fps": round(1.0 / dts.mean(), 1), "threads": args.threads}
    (out / "timing.json").write_text(json.dumps(timing, indent=2))
    print(timing)


if __name__ == "__main__":
    main()
