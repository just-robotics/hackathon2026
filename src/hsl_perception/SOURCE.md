# Detector source

`segmentation.py` and `tracker.py` are copied from `origin/feature/detector`,
commit b985515 (Just Robotics, same repository/license). The ROS adapter is
adapted to the existing hsl interfaces: timestamped map cloud and own pose,
static occupancy background instead of world SDF, local-frame velocity,
fresh detections vs coasting tracks, diagnostic JSON. No opponent ground truth
or unknown-box poses enter the detector. Original C++ detector is removed;
Git checkpoint 4f9168c preserves it.

Simulation launch enables the existing `robot.max_gap_share=0.12` shape test
after full-cloud replay exposed narrow-box corner false positives. The real
launch retains the source default (1.0) pending hardware validation. Minimal
cluster extent stays disabled: it rejected occluded robots in that replay.
NumPy/BLAS uses one thread per process; scan-time TF is awaited in a bounded
queue. `segmentation.py` and `tracker.py` remain byte-identical to the source.
Simulation also uses `robot.line_ratio=0.35`: a strong circle fit must improve
over a straight line substantially, rather than confirming noisy box faces.
Partial straight fragments stay weak candidates and can continue an existing
track. The real profile retains the source ratio 0.7 pending hardware testing.

Additional simulation confidence checks in the adapter (not in source files):
- `strong_arc_min_span_deg=90.0`: short arcs can continue, not initialize a track.
- `allow_merged_strong=false`: a robot extracted from an oversized cluster is
  weak evidence until separately observed. A known track can still use it.
- `strong_min_inlier_fraction=0.95`: a new circle must explain nearly all rim
  points, not just a handpicked subset of an unknown box face. With 0.05m
  radial tolerance against 0.02m simulated range noise, the expected fraction
  for a circular body is about 99%; 95% permits a few outliers.
The real launch retains defaults 0/true/0 until hardware checks. These checks
reduce initial detections under occlusion; they are not a semantic classifier.

Real sensor_frame is configurable and set to livox to match hardware TF.
A separate upstream real_lidar_filter removes measured near shadows of the
four own rods and low-confidence Livox returns before AMCL/navigation.
Source segmentation.py/tracker.py are unchanged.
