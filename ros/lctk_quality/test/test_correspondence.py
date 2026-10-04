import numpy as np
from lctk_quality.correspondence import transform_marker_corners


def test_marker_corner_order_and_nonunit_xyzw_pose_are_preserved():
    local = np.array([[1, 0, 0], [0, 1, 0], [-1, 0, 0], [0, -1, 0]])
    pixels = np.array([[20, 10], [10, 0], [0, 10], [10, 20]])
    world, observed = transform_marker_corners(local, [2, 3, 4], [0, 0, 0, 2], pixels)
    np.testing.assert_array_equal(world, [[3, 3, 4], [2, 4, 4], [1, 3, 4], [2, 2, 4]])
    np.testing.assert_array_equal(observed, pixels)
