"""Real-cloud birth checks; incomplete observations may maintain known tracks.

Validated by offline replay of October 2 bags, without semantic ground truth.
Keep imported segmentation/tracker defaults separate from ROS launch policy.
"""

REAL_PARAMETERS = {
    'robot.max_gap_share': 0.25,
    'robot.line_ratio': 0.50,
    'strong_min_extent': 0.25,
    'strong_arc_min_span_deg': 75.0,
    'allow_merged_strong': False,
    'strong_min_inlier_fraction': 0.80,
}


def real_detector(max_height=.46):
    """Construct the same non-ROS profile used by the real launch for replay."""
    from .core import Detector
    from .segmentation import RobotModel
    model = {key.removeprefix('robot.'): value for key, value in REAL_PARAMETERS.items()
             if key.startswith('robot.')}
    checks = {key: value for key, value in REAL_PARAMETERS.items()
              if not key.startswith('robot.')}
    return Detector(RobotModel(max_height=max_height, **model), **checks)
