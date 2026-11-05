"""Run the snow-pole YOLO detector once per frame and cache everything the GNSS
sweep needs that does NOT depend on the odometry backend.

In the reference pipeline (snowpole_based_vehicle_localization_GNSS_percentage.py)
each bounding box yields (center_xyz, new_distance, azimuth_gnss) purely from the
range/signal images; only the subsequent geo-projection uses the odometry. So the
per-frame detections are computed here once and replayed by 04_gnss_sweep.py for
every backend and GNSS percentage. Frames are streamed (the reference loads the
whole 5.7 GB bag of images into RAM).

--input comb4 --weights <yolo11 .pt> --out cache_comb4/detections.pkl runs the YOLO11n trained on the
SnowPole Detection dataset's Combination 4 (17_train_yolo11n_comb4.py) instead: its input is rebuilt from
the bag's /ouster/{nearir,signal,reflec}_image exactly as the dataset PNGs were (byte-identical, see
18_dataset_frame_map.py) and as Ultralytics read them in training: uint8 BGR = (near-IR, signal, reflec) >> 8.

Outputs cache/detections.pkl:
  frames[i] = list of (center_xyz[3], new_distance, azimuth_gnss) in bbox order,
              including boxes later skipped by the 5 m gate (the sweep applies it)
  timestamps_range, timestamps_left_gnss, left_gnss, right_gnss (as the reference)
"""
import argparse
import os
import pickle
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE / "polegeo"))
os.environ.setdefault("MPL_BACKEND", "Agg")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", default=str(HERE.parent / "snow_pole_geo_localization_data" /
                                         "2024-02-28-12-59-51_no_unwanted_topics.bag"))
    ap.add_argument("--out", default=str(HERE / "cache" / "detections.pkl"))
    ap.add_argument("--input", choices=["signal", "comb4"], default="signal")
    ap.add_argument("--weights", default=str(HERE / "polegeo" / "model" / "pole_best_signal.pt"))
    ap.add_argument("--conf", type=float, default=0.6)
    ap.add_argument("--imgsz", type=int, default=1024, help="comb4 only (the training size)")
    args = ap.parse_args()

    import cv2
    from ouster.sdk import core as client  # ouster-sdk 1.0 (reference used 0.10 `ouster.client`)
    from rosbags.highlevel import AnyReader
    import geoloc_utils as gu

    if args.input == "signal":
        model = gu.load_custom_model(args.weights, confidence_threshold=args.conf, backend="Agg")
    else:
        from ultralytics import YOLO
        model = YOLO(args.weights)
    _, xyzlut = gu.load_lidar_configuration(str(HERE / "polegeo" / "Trip068.json"), client)
    gnss_offset = np.array([-0.32, 0.0, 1.24])
    lidar_offset = np.array([0.7, 0.0, 1.8])
    gnss_to_lidar_offset = gnss_offset - lidar_offset

    def img16(m):
        return np.frombuffer(np.ascontiguousarray(m.data).tobytes(),
                             dtype=np.uint16).reshape((m.height, m.width))

    signal, rng, reflec, nearir = {}, {}, {}, {}
    frames = {}
    ts_range, ts_left, ts_right, left, right = [], [], [], [], []
    n_sig = n_rng = n_ref = n_nir = 0
    t0 = time.time()

    def process(i):
        range_image, sig = rng.pop(i), signal.pop(i)
        if args.input == "comb4":
            nir, ref = nearir.pop(i), reflec.pop(i)
        dets = []
        if i > 0:
            xyz = xyzlut(range_image.astype(np.uint32)) * 4
            if args.input == "signal":
                rgb = np.stack((sig, sig, sig), axis=-1)
                rgb = ((rgb - rgb.min()) / (rgb.max() - rgb.min()) * 255).astype(np.uint8)
                boxes = model(rgb).xyxy[0]
            else:
                rgb = (np.stack((nir, sig, ref), axis=-1) >> 8).astype(np.uint8)  # BGR, as in training
                # pad 16 grey rows above/below = Ultralytics' validation letterbox (1024x160), the input its
                # mAP is measured on (17_train_yolo11n_comb4.predict); boxes shifted back to image rows
                img = cv2.copyMakeBorder(rgb, 16, 16, 0, 0, cv2.BORDER_CONSTANT, value=(114, 114, 114))
                boxes = model.predict(img, imgsz=args.imgsz, conf=args.conf, iou=0.7, max_det=300,
                                      verbose=False)[0].boxes.xyxy.cpu().numpy()
                boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]] - 16, 0, rgb.shape[0])
            for bbox in boxes:
                x_min, y_min, x_max, y_max = map(int, bbox[:4])
                gx, gy, nd = gu.extract_nearest_point_in_bounding_box_region(
                    x_min, x_max, y_min, y_max, rgb, {i: range_image}, i)
                if gx is None or gy is None or nd is None:
                    continue
                c = xyz[int(gy), int(gx), :].astype(float)
                new_distance = gu.calculate_distance(c, gnss_to_lidar_offset)
                az, _ = gu.calculate_azimuth_elevation_from_gnss(c[0], c[1], c[2], gnss_to_lidar_offset)
                dets.append((c, float(new_distance), float(-az)))
        frames[i] = dets
        if i % 500 == 0:
            print(f"frame {i}: {len(dets)} boxes  ({time.time() - t0:.0f}s)", file=sys.stderr, flush=True)

    import contextlib, io
    with AnyReader([Path(args.bag)]) as r:
        for conn, t_ns, raw in r.messages():
            topic, t = conn.topic, t_ns / 1e9
            if topic == "/ouster/signal_image":
                signal[n_sig] = img16(r.deserialize(raw, conn.msgtype)); n_sig += 1
            elif topic == "/ouster/range_image":
                rng[n_rng] = img16(r.deserialize(raw, conn.msgtype)); ts_range.append(t); n_rng += 1
            elif topic == "/ouster/reflec_image" and args.input == "comb4":
                reflec[n_ref] = img16(r.deserialize(raw, conn.msgtype)); n_ref += 1
            elif topic == "/ouster/nearir_image" and args.input == "comb4":
                nearir[n_nir] = img16(r.deserialize(raw, conn.msgtype)); n_nir += 1
            elif topic == "/gps_left_position":
                m = r.deserialize(raw, conn.msgtype); left.append((m.latitude, m.longitude)); ts_left.append(t)
            elif topic == "/gps_right_position":
                m = r.deserialize(raw, conn.msgtype); right.append((m.latitude, m.longitude)); ts_right.append(t)
            else:
                continue
            for i in [k for k in list(rng) if k in signal and (args.input == "signal" or (k in reflec and k in nearir))]:
                with contextlib.redirect_stdout(io.StringIO()):  # helpers print per call
                    process(i)

    n = len(ts_range)
    out = {"frames": [frames[i] for i in range(n)], "timestamps_range": ts_range,
           "timestamps_left_gnss": ts_left, "left_gnss": left, "right_gnss": right}
    with open(args.out, "wb") as f:
        pickle.dump(out, f)
    nb = sum(len(v) for v in out["frames"])
    print(f"cached {n} frames, {nb} boxes, {len(ts_left)} GNSS fixes -> {args.out} "
          f"in {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
