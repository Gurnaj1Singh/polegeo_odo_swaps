# Pole detections over the SnowPole drive

Frame 0 is skipped by the pipeline. Distance = LiDAR distance to the box's nearest point (boxes <= 5 m are later dropped by the sweep's gate).

| Run | Frames | Frames with >= 1 box | Boxes | Boxes/frame | Median distance m | Boxes <= 5 m |
|---|---|---|---|---|---|---|
| cache: all frames | 5419 | 3386 (62.5 %) | 4633 | 0.85 | 8.5 | 767 |
| cache_comb4: all frames | 5419 | 3909 (72.1 %) | 5721 | 1.06 | 7.9 | 1056 |
| cache: in detector dataset (844-2999) | 2156 | 1611 (74.7 %) | 2334 | 1.08 | 8.2 | 361 |
| cache_comb4: in detector dataset (844-2999) | 2156 | 1757 (81.5 %) | 2745 | 1.27 | 7.6 | 478 |
| cache: outside detector dataset | 3263 | 1775 (54.4 %) | 2299 | 0.70 | 8.8 | 406 |
| cache_comb4: outside detector dataset | 3263 | 2152 (66.0 %) | 2976 | 0.91 | 8.3 | 578 |
