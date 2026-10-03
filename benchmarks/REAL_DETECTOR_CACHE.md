# Real detector offline cache v1

`real_detector_cache.py` reads map-anchored real MCAP sessions. It does not
start ROS nodes or publish messages. The October 3 map-anchored recordings are
under `/home/eddyswens/ROS/hsl2026Extra/recordings_last8`; the full new
`20261003T094252.665737Z-autonomous/bag` has recorded raw and filtered clouds,
`/map`, `/odom`, and `map→odom`. Run it inside the ROS 2 Humble real image with
the bags mounted read-only and select map-anchored sessions explicitly:

```sh
docker run --rm --network none \
  --mount type=bind,src="$PWD",dst=/work \
  --mount type=bind,src=/home/eddyswens/ROS/hsl2026Extra,dst=/bags,readonly \
  -w /work jr_real_image:latest bash -lc \
  'source /opt/ros/humble/setup.bash; PYTHONDONTWRITEBYTECODE=1 python3 benchmarks/real_detector_cache.py --root /bags --output results/real-detector-cache-v1 --session recordings_last8/20261003T094252.665737Z-autonomous'
```

`--session <relative-session-path>` may be repeated to process a subset. Use
the separate local extractor below for the five raw-only sessions, which have
no recorded map or map transform. Some historical bag sources are no longer
present on disk; their existing versioned cache remains valid and must be
preserved.
`--cloud-mode filtered` is available if disk space is constrained. The default
stores *both complete original streams* and does not skip scans. A development
run with `--limit-clouds` is marked `truncated` and has a distinct signature.

Each session directory contains:

- `clouds.zstbin`: one independent zstd frame per PointCloud2 `data` field,
  with no point conversion or reordering. This preserves the Livox `timestamp`
  FLOAT64 field byte for byte.
- `frames.tsv`: one row per cloud in `rosbag2_py.SequentialReader.read_next()`
  order. It records topic (`raw` or `filtered`), bag timestamp, exact header
  timestamp (both integer nanoseconds), frame, dimensions, field offsets,
  zstd offset and sizes, and the derived sensor pose. A blank pose value with
  `pose_status` other than `ok` is intentionally unusable.
- `pairs.tsv`: raw and filtered record indexes sharing a header timestamp.
  Unmatched frames remain in `frames.tsv`.
- `map.bin`: signed int8 row-major occupancy cells; `map.tsv` and `map.json`
  preserve resolution, dimensions and full origin quaternion plus yaw.
- `poses.jsonl`, `amcl_pose.jsonl`, `tf.jsonl`: recorded inputs with bag and
  header timestamps. TF is stored by individual parent/child edge and retains
  its dynamic or static topic. Old FAST-LIO and `/navigation/self` stay separate.
- `manifest.json`: SHA256 of source MCAP/metadata, extractor and LiDAR filter
  code/config, plus output files; layouts with ROS PointField datatypes, counts,
  pose policy, gap bounds and missing-pose reasons.

The fixed comparison pose uses timestamped `/odom` (`odom→base_footprint`)
composed with the recorded `map→odom` AMCL TF, then base-to-sensor TF. The
extractor linearly interpolates translation and uses shortest-arc quaternion
slerp only between two samples. Maximum brackets are 150 ms for wheel odometry
and 300 ms for `map→odom` or dynamic sensor TF. There is no extrapolation or
latest-pose fallback. A frame before the recorded `/map` is marked unavailable.
The cache never combines `map→lio_odom` with the old `map→odom` edge. The pose
variance is approximate: the larger of wheel odometry and nearby `/amcl_pose`
diagonals, with 5 cm and 5° floors. It is not ground truth for localization.

An existing session cache is reused only when the input and processing hashes
and cloud mode match. Otherwise the extractor stops so that the caller can
select a new output directory. Generated caches belong under ignored `results/`.

## Paired baseline

Run the existing Python detector against every cached filtered frame and the
same derived sensor pose used by the C++ runner:

```sh
docker run --rm --network none \
  --mount type=bind,src="$PWD",dst=/work -w /work jr_real_image:latest bash -lc \
  'source /opt/ros/humble/setup.bash; export PYTHONPATH=/work/src/jr_perception:/work/src/jr_map:$PYTHONPATH; PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 benchmarks/replay_cached_baseline.py results/real-detector-cache-v1/* --output results/real-detector-baseline-v1'
```

`replay_cached_baseline.py` calls the existing `RobotDetector.process` without
altering detection or tracking. The map and PointCloud2 bytes come from the
cache; sensor pose is the `map_from_sensor` on the same `frames.tsv` row. It
does not perform per-point deskew because the existing detector does not.
There is one JSONL row per filtered cloud, including missing-pose frames.
Selected old tracks are split into newly measured outputs and predictions by
`track.last_update` versus the cloud timestamp. Each JSONL has a manifest with
the cache signature and hashes of the profile, detector sources and output.
These outputs are observations for comparison, not semantic ground truth.
For the newly added full `094252` bag, the cache has 2,099 raw and 2,105
filtered frames. Nine filtered frames lack a strictly bracketed map pose;
the old detector processed 2,096 and produced 903 measured outputs and 90
predictions. The cache and baseline manifests contain the exact source and
output hashes.

## October 3 raw-only recordings: local odom variant

The five short recordings `095154`, `095433`, `095724`, `095806`, `095829`
exist at `/home/eddyswens/ROS/hsl2026Extra/recordings_last8/<session>/bag/`.
They contain `/livox/lidar`, `/odom`, static base-to-Livox TF, and dynamic
`odom→base_footprint`, but **no** `/map`, `/amcl_pose`, `map→odom`, or recorded
`/sensing/lidar/points_filtered`. The `odom` origin is local to each bag and
must not be treated as the shared `map` frame. Their descriptions from the
recorder are in the task conversation: `095433` is a rotation among boxes;
the final three contain a moving opponent while our robot stands still.

`real_detector_local_cache.py` makes an explicitly separate
`real-detector-cache-local-v1` variant. It applies the same field-preserving
LiDAR filter algorithm and `config/lidar_filter.yaml` settings to each raw
cloud offline, keeps the original raw payload, and stores a derived filtered
payload next to it. It strictly interpolates recorded `/odom` at the cloud
header time and composes the recorded base-to-sensor TF. The manifest records
source MCAP, extractor, filter implementation/config hashes, `world_frame=odom`,
`map_available=false`, and `pose_quality=local_wheel_odom_unanchored`. The
legacy `frames.tsv` column names `map_sensor_*` mean **odom→sensor** only in
this separate schema; `pose_source` is `recorded_odom_local`. No fake map file
or map transform is written.

```sh
docker run --rm --network none \
  --mount type=bind,src="$PWD",dst=/work \
  --mount type=bind,src=/home/eddyswens/ROS/hsl2026Extra,dst=/bags,readonly \
  -w /work jr_real_image:latest bash -lc \
  'source /opt/ros/humble/setup.bash; PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 benchmarks/real_detector_local_cache.py --root /bags --output results/real-detector-cache-local-v1'
```

The offline filter oracle was checked against **all 2,099 paired raw/recorded
filtered clouds** in the full `094252` real bag: 2,099 were byte-identical,
including original PointCloud2 point fields and Livox timestamps. The report is
`results/real-detector-cache-local-v1/filter_equivalence_on_094252.json`.
Reproduce it with:

```sh
docker run --rm --network none \
  --mount type=bind,src="$PWD",dst=/work -w /work jr_real_image:latest bash -lc \
  'source /opt/ros/humble/setup.bash; export PYTHONPATH=/work/src/jr_perception:/work/src/jr_map:$PYTHONPATH; PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 benchmarks/verify_real_filter_cache.py results/real-detector-cache-v1/20261003T094252.665737Z-autonomous'
```

`replay_cached_baseline.py` accepts local caches but labels the result
`mapless_local_odom_ablation`: it calls the old detector's unchanged geometry
and tracking path with map background/visibility disabled. This is a fair
same-input comparison for the mapless C++ core, **not** the old real launch
quality with a map. Its output is under
`results/real-detector-baseline-local-v1/` and uses the same one-row-per-frame
format as the map-backed baseline.

| Local session | Filtered frames | Old measured | Old predicted | Old none |
| --- | ---: | ---: | ---: | ---: |
| `095154` | 675 | 365 | 160 | 150 |
| `095433` | 800 | 434 | 262 | 104 |
| `095724` | 236 | 210 | 24 | 2 |
| `095806` | 117 | 25 | 10 | 82 |
| `095829` | 243 | 169 | 50 | 24 |

All local frames have a strict interpolated odom pose. These output counts
are **not** true/false detection counts without manual visibility labels.

```sh
docker run --rm --network none \
  --mount type=bind,src="$PWD",dst=/work -w /work jr_real_image:latest bash -lc \
  'source /opt/ros/humble/setup.bash; export PYTHONPATH=/work/src/jr_perception:/work/src/jr_map:$PYTHONPATH; PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 python3 benchmarks/replay_cached_baseline.py results/real-detector-cache-local-v1/20261003T*-bag --output results/real-detector-baseline-local-v1'
```
