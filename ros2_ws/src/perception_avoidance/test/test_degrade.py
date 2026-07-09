"""degrade_pointcloud_node의 순수 열화 변환(degrade) 테스트."""

import numpy as np
import pytest
from rclpy.parameter import Parameter

from perception_avoidance.degrade_pointcloud_node import DegradePointcloudNode


def make_node(**params):
    overrides = [Parameter(k, value=v) for k, v in params.items()]
    return DegradePointcloudNode(parameter_overrides=overrides)


def test_passthrough_by_default():
    node = make_node()
    pts = np.random.default_rng(0).uniform(-5, 5, (1000, 3)).astype(np.float32)
    out = node.degrade(pts.copy())
    np.testing.assert_array_equal(out, pts)
    node.destroy_node()


def test_density_downsample_ratio():
    node = make_node(density_keep_ratio=0.2)
    pts = np.ones((10000, 3), dtype=np.float32)
    out = node.degrade(pts)
    assert out.shape[0] == pytest.approx(2000, rel=0.15)
    node.destroy_node()


def test_dropout_combines_with_density():
    node = make_node(density_keep_ratio=0.5, dropout_ratio=0.2)
    pts = np.ones((10000, 3), dtype=np.float32)
    out = node.degrade(pts)
    assert out.shape[0] == pytest.approx(4000, rel=0.15)   # 0.5 * 0.8
    node.destroy_node()


def test_range_noise_is_radial_with_expected_std():
    node = make_node(range_noise_std=0.03)
    n = 20000
    dirs = np.random.default_rng(1).normal(size=(n, 3))
    dirs /= np.linalg.norm(dirs, axis=1)[:, None]
    pts = (dirs * 5.0).astype(np.float32)                  # all at range 5m
    out = node.degrade(pts)
    r_out = np.linalg.norm(out, axis=1)
    assert np.mean(r_out) == pytest.approx(5.0, abs=0.01)
    assert np.std(r_out) == pytest.approx(0.03, rel=0.1)
    # direction preserved (radial-only noise)
    dirs_out = out / r_out[:, None]
    assert np.abs(dirs_out - dirs).max() < 1e-3
    node.destroy_node()


def test_empty_cloud_ok():
    node = make_node(density_keep_ratio=0.2, range_noise_std=0.03)
    out = node.degrade(np.empty((0, 3), dtype=np.float32))
    assert out.shape[0] == 0
    node.destroy_node()
