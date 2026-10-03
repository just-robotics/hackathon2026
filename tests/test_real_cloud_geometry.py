"""Dense observation transport keeps measured faces and exact self masking."""
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/hsl_real'),str(ROOT/'src/hsl_sim_adapter')]
from hsl_real.cloud_geometry import map_xyz
from hsl_sim_adapter.cloud import transform


def test_full_cloud_transport_preserves_sparse_objects_and_matches_scalar_rotation():
    rng=np.random.default_rng(47)
    cloud=rng.normal(size=(20003,3))
    quaternion=np.array([.2,-.3,.4,.5]);quaternion/=np.linalg.norm(quaternion)
    translation=np.array([1.2,2.3,.3])
    expected=np.array([transform(p,translation,quaternion) for p in cloud])
    expected=expected[np.linalg.norm(expected[:,:2]-[1.2,2.3],axis=1)>=.25]
    actual=map_xyz(cloud,translation,quaternion,[1.2,2.3])
    assert len(actual)>6000
    np.testing.assert_allclose(actual,expected,atol=1e-14)


def test_nonfinite_and_body_returns_removed_without_blind_zone_expansion():
    cloud=np.array([[.249,0,0],[.251,0,.4],[1,0,.4],[np.nan,1,0]])
    actual=map_xyz(cloud,[0,0,0],[0,0,0,1],[0,0])
    np.testing.assert_array_equal(actual,cloud[[1,2]])
