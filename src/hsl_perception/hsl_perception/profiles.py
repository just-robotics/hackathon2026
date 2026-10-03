"""ROS configuration constants for the native C++ detector; no Python detector."""

REAL_PARAMETERS = {
    'robot.max_gap_share': 0.25,
    'robot.line_ratio': 0.50,
    'strong_min_extent': 0.25,
    'strong_arc_min_span_deg': 75.0,
    'allow_merged_strong': False,
    'strong_min_inlier_fraction': 0.80,
}
