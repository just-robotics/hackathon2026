# Native detector and shared geometry

Production robot detection and Kalman tracking live exclusively in
`hsl_perception_cpp` (C++/Eigen), adapted from `origin/feature/detector`
commit b985515, same repository/license. Python robot detection, tracker,
circle fitting and oracle were removed at the user's request on 03.10.2026.

This Python package now contains only cloud decoding, shared static-map masking,
XY clustering used by SmallBoxFilter, and ROS configuration constants.
`geometry.py` retains the existing masking/clustering calculations unchanged.
Small-box recognition remains in `hsl_planning/obstacle_filter.py`.

Offline detector tools call the native `detector_replay` executable. Their
Python code transports data and reports diagnostics; it does not implement
robot detection. Source the ROS workspace or set HSL_DETECTOR_REPLAY explicitly.

Native track confirmation requires three strong hits. Weak observations can
bridge a partial view only until max_coast since the last strong observation.
This restriction is experimental: simulation replay removed false continuations
but lost some partial true-robot observations. Full autonomous validation remains
incomplete. Simulation/real confidence profiles differ; no unknown fixture
coordinates or navigation ground truth enter either profile.

Historical Python/C++ comparison on 761 full real clouds (five repeats,
i7-10510U, one core) measured mean4.188/0.268ms per detector step (15.62x).
Outputs agreed to numerical precision. This excludes ROS/filter/TF/I/O and
proves neither full-stack speed nor semantic recognition accuracy.
