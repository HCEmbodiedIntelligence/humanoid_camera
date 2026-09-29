import numpy as np
import pytest
import pyrealsense2 as rs

from humanoid_camera.depth_visualization import depth_color_preview, depth_color_legend


def sdk_pixels(pixels, *, fixed=False, scheme=0):
    device = rs.software_device()
    sensor = device.add_sensor('official independent test input')
    stream = rs.video_stream()
    stream.type, stream.fmt = rs.stream.depth, rs.format.z16
    stream.width, stream.height, stream.fps, stream.bpp = pixels.shape[1], pixels.shape[0], 30, 2
    stream_profile = sensor.add_video_stream(stream)
    queue = rs.frame_queue(1)
    colorizer = rs.colorizer()
    assert colorizer.get_option(rs.option.visual_preset) == 0
    assert colorizer.get_option(rs.option.color_scheme) == 0
    assert colorizer.get_option(rs.option.histogram_equalization_enabled) == 1
    if scheme:
        colorizer.set_option(rs.option.color_scheme, scheme)
    if fixed:
        colorizer.set_option(rs.option.histogram_equalization_enabled, 0)
        colorizer.set_option(rs.option.min_distance, .2)
        colorizer.set_option(rs.option.max_distance, 2.)
    sensor.open(stream_profile)
    sensor.start(queue)
    try:
        frame = rs.software_video_frame()
        frame.pixels, frame.bpp, frame.stride = pixels.data, 2, pixels.shape[1] * 2
        frame.profile, frame.depth_units = stream_profile.as_video_stream_profile(), .001
        sensor.on_video_frame(frame)
        return np.asanyarray(colorizer.colorize(queue.wait_for_frame(1000)).get_data()).copy()
    finally:
        sensor.stop()
        sensor.close()


@pytest.mark.parametrize('scheme', [0, 2])
@pytest.mark.parametrize('fixed', [False, True])
@pytest.mark.parametrize('big_endian', [False, True])
def test_distance_colors_invalid_pixels_and_clipping_preserve_source(scheme, fixed, big_endian):
    depth = np.array([[0, 100, 200, 1100, 2000, 65535]], dtype=np.uint16)
    expected = sdk_pixels(depth, fixed=fixed, scheme=scheme)
    if big_endian:
        depth = depth.astype('>u2')
    original = depth.copy()
    colors, scale = depth_color_preview(depth, .2 if fixed else None, 2. if fixed else None,
                                       scale_m_per_unit=.001 if fixed else None, color_scheme=scheme)
    np.testing.assert_array_equal(colors, expected)
    np.testing.assert_array_equal(depth, original)
    assert scale['range_mode'] == ('fixed' if fixed else 'per_frame_histogram')
    assert scale['meters_per_raw_unit'] == (.001 if fixed else None)
    assert scale['implementation'] == 'pyrealsense2.colorizer'
    assert not colors[0, 0].any()
    legend = np.array(depth_color_legend())
    assert legend.shape == (76, 640, 3)


def test_same_distance_has_same_color_across_frames():
    a, _ = depth_color_preview(np.array([[0, 800]], dtype=np.uint16), .2, 2., scale_m_per_unit=.001)
    b, _ = depth_color_preview(np.array([[800, 65000]], dtype=np.uint16), .2, 2., scale_m_per_unit=.001)
    np.testing.assert_array_equal(a[0, 1], b[0, 0])


def test_unknown_source_units_cannot_use_metric_display_and_sdk_mismatch_has_no_fallback(monkeypatch):
    from humanoid_camera import depth_visualization
    pixels = np.zeros((2, 2), dtype=np.uint16)
    with pytest.raises(ValueError, match='源深度单位'):
        depth_color_preview(pixels, .2, 2.)
    np.testing.assert_array_equal(depth_color_preview(pixels)[0], 0)
    monkeypatch.setattr(depth_visualization, 'version', lambda _: 'unsupported')
    with pytest.raises(RuntimeError, match='SDK'):
        depth_color_preview(pixels)


@pytest.mark.parametrize('low,high', [(1, 1), (-1, 2), (0, float('nan')), (2, 1), (0, 66)])
def test_invalid_range_is_rejected(low, high):
    with pytest.raises(ValueError):
        depth_color_preview(np.zeros((2, 2), dtype=np.uint16), low, high)
