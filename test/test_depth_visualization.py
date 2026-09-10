import numpy as np
import pytest

from humanoid_camera.depth_visualization import depth_color_preview, depth_color_legend


def test_distance_colors_invalid_pixels_and_clipping_preserve_source():
    depth = np.array([[0, 100, 200, 1100, 2000, 65535]], dtype=np.uint16)
    original = depth.copy()
    colors, scale = depth_color_preview(depth)
    np.testing.assert_array_equal(colors[0], [[0, 0, 0], [255, 0, 0], [255, 0, 0],
                                            [0, 255, 0], [0, 0, 255], [0, 0, 255]])
    np.testing.assert_array_equal(depth, original)
    assert scale['range_mode'] == 'fixed' and scale['meters_per_raw_unit'] == .001
    legend = np.array(depth_color_legend())
    assert tuple(legend[25, 12]) == (255, 0, 0)
    assert tuple(legend[25, -13]) == (0, 0, 255)


def test_same_distance_has_same_color_across_frames():
    a, _ = depth_color_preview(np.array([[0, 800]], dtype=np.uint16))
    b, _ = depth_color_preview(np.array([[800, 65000]], dtype=np.uint16))
    np.testing.assert_array_equal(a[0, 1], b[0, 0])


@pytest.mark.parametrize('low,high', [(1, 1), (-1, 2), (0, float('nan')), (2, 1), (0, 66)])
def test_invalid_range_is_rejected(low, high):
    with pytest.raises(ValueError):
        depth_color_preview(np.zeros((2, 2), dtype=np.uint16), low, high)
