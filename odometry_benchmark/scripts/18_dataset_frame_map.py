"""Map every SnowPole Detection dataset image to its frame in the SnowPole bag.

  python scripts/18_dataset_frame_map.py     # -> results/detector_dataset_frame_map.csv (+ summary on stdout)

Each combined_color PNG is checked byte-for-byte against the bag's /ouster/{nearir,signal,reflec}_image
stacked as (B, G, R). This (1) proves 03_cache_detections.py --input comb4 rebuilds the training input
exactly, and (2) records which localization frames the detector was trained / validated / tested on.
"""
import hashlib
from pathlib import Path

import cv2
import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader

ROOT = Path(__file__).resolve().parent.parent
DS = ROOT.parent / ("SnowPole Detection A Comprehensive Dataset for Detection and Localization Using LiDAR "
                    "Imaging in Nordic Winter Conditions") / "SnowPole_Detection_Dataset" / "combined_color"
BAG = ROOT.parent / "snow_pole_geo_localization_data" / "2024-02-28-12-59-51_no_unwanted_topics.bag"


def main():
    idx = {}
    for f in sorted(DS.glob("*/*.png")):
        a = cv2.imread(str(f), cv2.IMREAD_UNCHANGED)
        idx.setdefault(hashlib.md5(np.ascontiguousarray(a[..., 1]).tobytes()).hexdigest(), []).append((f, a))
    rows, buf, n = [], {}, {}
    with AnyReader([BAG]) as r:
        cons = [c for c in r.connections if c.topic in ("/ouster/signal_image", "/ouster/reflec_image",
                                                         "/ouster/nearir_image")]
        for c, _, raw in r.messages(connections=cons):
            k = c.topic.split("/")[2].split("_")[0]
            i = n.get(k, 0); n[k] = i + 1
            m = r.deserialize(raw, c.msgtype)
            buf.setdefault(i, {})[k] = np.frombuffer(np.ascontiguousarray(m.data).tobytes(),
                                                     dtype=np.uint16).reshape((m.height, m.width))
            if len(buf[i]) < 3:
                continue
            b = buf.pop(i)
            for f, a in idx.get(hashlib.md5(np.ascontiguousarray(b["signal"]).tobytes()).hexdigest(), []):
                rows.append({"frame": i, "split": f.parent.name, "image": f.name,
                             "bgr_identical": bool(np.array_equal(a, np.stack([b["nearir"], b["signal"],
                                                                                b["reflec"]], -1)))})
    df = pd.DataFrame(rows).sort_values("frame")
    df.to_csv(ROOT / "results" / "detector_dataset_frame_map.csv", index=False)
    print(f"{len(df)} of {sum(len(v) for v in idx.values())} dataset images matched to bag frames "
          f"(of {max(n.values())}); byte-identical BGR stack: {int(df.bgr_identical.sum())}")
    for s, g in df.groupby("split"):
        print(f"  {s}: {len(g)} images, frames {g.frame.min()}-{g.frame.max()}")


if __name__ == "__main__":
    main()
