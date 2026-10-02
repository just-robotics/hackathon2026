# Detector source

`segmentation.py` and `tracker.py` are copied from `origin/feature/detector`,
commit b985515 (Just Robotics, same repository/license). The ROS adapter is
adapted to the existing hsl interfaces: timestamped map cloud and own pose,
static occupancy background instead of world SDF, local-frame velocity,
fresh detections vs coasting tracks, diagnostic JSON. No opponent ground truth
or unknown-box poses enter the detector. Original C++ detector is removed;
Git checkpoint 4f9168c preserves it.

Simulation launch enables the existing `robot.max_gap_share=0.12` shape test
after full-cloud replay exposed narrow-box corner false positives. Simulation minimal cluster extent stays disabled: it rejected occluded robots
in that replay. The separately configured real profile is described below.
NumPy/BLAS uses one thread per process; scan-time TF is awaited in a bounded
queue. `segmentation.py` and `tracker.py` remain byte-identical to the source.
Simulation also uses `robot.line_ratio=0.35`: a strong circle fit must improve
over a straight line substantially, rather than confirming noisy box faces.
Partial straight fragments stay weak candidates and can continue an existing
track. Real parameters are separately configured in profiles.py.

Additional simulation confidence checks in the adapter (not in source files):
- `strong_arc_min_span_deg=90.0`: short arcs can continue, not initialize a track.
- `allow_merged_strong=false`: a robot extracted from an oversized cluster is
  weak evidence until separately observed. A known track can still use it.
- `strong_min_inlier_fraction=0.95`: a new circle must explain nearly all rim
  points, not just a handpicked subset of an unknown box face. With 0.05m
  radial tolerance against 0.02m simulated range noise, the expected fraction
  for a circular body is about 99%; 95% permits a few outliers.
These checks
reduce initial detections under occlusion; they are not a semantic classifier.

The real launch now uses max_gap_share=0.25, line_ratio=0.50,
strong_min_extent=0.25m, strong_arc_min_span_deg=75,
allow_merged_strong=false, strong_min_inlier_fraction=0.80. Minimum extent
only prevents new strong births; short partial weak candidates can maintain
a track. Offline replay reduced overlap with independently fitted narrow-box
hypotheses; the bags have no semantic labels, so these are not precision/recall
measurements. New hardware runs are required. Both profiles reject fits with
centers outside known-free static cells and omit unknown-map foreground.
The real adapter uses configurable sensor_frame=livox, matching hardware TF.
Raw real input is filtered upstream for measured near shadows of the four own
rods and low-confidence Livox returns before AMCL and navigation. Neither
source segmentation.py nor tracker.py is changed.

Simulation additionally compares the already fitted circular rim with edges
of a rotated rectangle (`strong_rectangle_ratio=0.70`). A candidate with
rectangle edge MSE below 70% of circle MSE becomes weak, so a box corner
cannot open a track. No box dimensions/poses are inputs. Default is disabled;
the real profile does not enable this new check yet. Synthetic L-corner
regression reproduces an old false birth; captured simulation replay keeps
136 peer-near estimates unchanged. Actual-box false positives and visibility
recall still need repeated simulation/hardware validation.
