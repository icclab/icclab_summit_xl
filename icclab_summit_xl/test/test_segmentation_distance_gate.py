#!/usr/bin/env python3
"""Offline test of the distance gate in segmentation_node_remote.py (no ROS graph needed).

Run: python3 -m pytest test/test_segmentation_distance_gate.py
"""

import importlib.util
import pathlib
import sys
from unittest import mock

import numpy as np

# select_mask needs only numpy; stub the node's ROS and vision imports.
for _name in ('rclpy', 'rclpy.node', 'rclpy.qos', 'sensor_msgs', 'sensor_msgs.msg', 'std_msgs',
              'std_msgs.msg', 'cv_bridge', 'cv2', 'message_filters', 'open3d', 'requests'):
    sys.modules.setdefault(_name, mock.MagicMock())

_SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'scripts' / 'segmentation_node_remote.py'
_spec = importlib.util.spec_from_file_location('segmentation_node_remote', _SCRIPT)
_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_module)
select_mask = _module.select_mask


def _mask(rows):
    m = np.zeros((10, 10), dtype=bool)
    m[rows, 2:5] = True
    return m


def _depth_m(near=1.5, far=5.0):
    """Rows 0-3 at `near`, rows 6-9 at `far`, rows 4-5 without depth (NaN)."""
    d = np.full((10, 10), np.nan, dtype=np.float32)
    d[0:4] = near
    d[6:10] = far
    return d


NEAR, FAR, NO_DEPTH = _mask(slice(0, 4)), _mask(slice(6, 10)), _mask(slice(4, 6))


def test_gate_off_keeps_first_mask():
    chosen, medians = select_mask([FAR, NEAR], _depth_m(), 0.0)
    assert chosen == 0
    assert medians == [5.0, 1.5]


def test_far_first_mask_is_skipped():
    chosen, _ = select_mask([FAR, NEAR], _depth_m(), 3.0)
    assert chosen == 1


def test_confidence_order_kept_within_range():
    chosen, _ = select_mask([NEAR, _mask(slice(1, 3))], _depth_m(), 3.0)
    assert chosen == 0


def test_all_out_of_range():
    chosen, _ = select_mask([FAR], _depth_m(), 3.0)
    assert chosen is None


def test_mask_without_depth_is_skipped():
    chosen, medians = select_mask([NO_DEPTH, NEAR], _depth_m(), 3.0)
    assert chosen == 1
    assert medians[0] is None


def test_no_depth_on_any_mask():
    chosen, medians = select_mask([NO_DEPTH], _depth_m(), 3.0)
    assert chosen is None
    assert medians == [None]


def test_uint16_millimetres():
    depth_mm = np.nan_to_num(_depth_m() * 1000.0).astype(np.uint16)
    chosen, medians = select_mask([FAR, NEAR], depth_mm, 3.0)
    assert chosen == 1
    assert medians == [5.0, 1.5]
