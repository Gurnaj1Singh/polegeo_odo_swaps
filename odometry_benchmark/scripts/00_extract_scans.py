"""Extract /ouster/points from the full SnowPole bag into compact per-scan files.

Each scan is saved as scans/NNNNNN.npy with float32 columns [x, y, z, t] where
t is the per-point time in seconds relative to the scan start (Ouster 't' field,
ns). Points outside [min_range, max_range] are dropped (same 2 m blind / 150 m
range as the Faster-LIO and Super-LIO configs). scan_stamps.csv records the
header stamp (Ouster sensor clock) and bag record time of every scan, which the
bridge uses to place the trajectory on the GNSS timeline.

These files are the shared input for the GenZ-ICP and MAD-ICP runners (Docker).
"""
import argparse
from pathlib import Path

import numpy as np
from rosbags.highlevel import AnyReader


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--min-range", type=float, default=2.0)
    ap.add_argument("--max-range", type=float, default=150.0)
    ap.add_argument("--intensity-only", action="store_true",
                    help="only write scans_intensity/NNNNNN.npy: Ouster calibrated reflectivity (uint8, 0-255), "
                         "aligned point-for-point with scans/, for the learned methods that take a remission channel")
    args = ap.parse_args()

    out = Path(args.out)
    (out / ("scans_intensity" if args.intensity_only else "scans")).mkdir(parents=True, exist_ok=True)
    rows = []
    with AnyReader([Path(args.bag)]) as r:
        cons = [c for c in r.connections if c.topic == "/ouster/points"]
        for i, (c, t_rec, raw) in enumerate(r.messages(connections=cons)):
            m = r.deserialize(raw, c.msgtype)
            buf = np.frombuffer(np.ascontiguousarray(m.data).tobytes(), dtype=np.uint8)
            buf = buf.reshape(-1, m.point_step)
            xyz = buf[:, 0:12].copy().view(np.float32).reshape(-1, 3)
            t = buf[:, 20:24].copy().view(np.uint32).reshape(-1).astype(np.float64) * 1e-9
            rng = np.linalg.norm(xyz, axis=1)
            keep = (rng >= args.min_range) & (rng <= args.max_range) & np.isfinite(rng)
            arr = np.empty((int(keep.sum()), 4), np.float32)
            arr[:, :3] = xyz[keep]
            arr[:, 3] = t[keep]
            if args.intensity_only:
                refl = buf[:, 24:26].copy().view(np.uint16).reshape(-1)
                np.save(out / "scans_intensity" / f"{i:06d}.npy", np.clip(refl[keep], 0, 255).astype(np.uint8))
            else:
                np.save(out / "scans" / f"{i:06d}.npy", arr)
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            rows.append((i, f"{stamp:.9f}", f"{t_rec / 1e9:.9f}", arr.shape[0]))
            if i % 500 == 0:
                print(f"scan {i}: {arr.shape[0]} pts, stamp {stamp:.3f}", flush=True)
    if args.intensity_only:
        print(f"wrote reflectivity for {len(rows)} scans -> {out / 'scans_intensity'}")
        return
    with open(out / "scan_stamps.csv", "w") as f:
        f.write("idx,sensor_stamp,record_time,n_points\n")
        for row in rows:
            f.write(",".join(map(str, row)) + "\n")
    print(f"wrote {len(rows)} scans -> {out}")


if __name__ == "__main__":
    main()
