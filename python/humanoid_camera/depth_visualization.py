"""Official RealSense colorizer on offline ROS pixels; no custom palette fallback.

Only software_device is requested. No physical acquisition starts; the SDK may
initialize its udev monitor while constructing this in-memory input device.
Dynamic colorization needs no physical unit assumption. Fixed-range rendering
requires an explicit verified scale from the caller.
"""
from importlib.metadata import version
import math

import numpy as np
from PIL import Image, ImageDraw

SDK_VERSION = '2.58.4.10922'


def color_scale(min_m=None, max_m=None):
    dynamic = min_m is None and max_m is None
    if not dynamic and (any(isinstance(v, bool) or not isinstance(v, (int, float))
                            or not math.isfinite(v) for v in (min_m, max_m))
                        or not 0 <= min_m < max_m <= 65.535):
        raise ValueError('固定深度预览须同时提供有效米范围；默认使用官方动态直方图')
    return {'implementation': 'pyrealsense2.colorizer', 'sdk_version': SDK_VERSION,
            'min_m': min_m, 'max_m': max_m, 'colormap': 'Jet',
            'visual_preset': 'Dynamic' if dynamic else 'SDK fixed range',
            'histogram_equalization_enabled': dynamic,
            'range_mode': 'per_frame_histogram' if dynamic else 'fixed',
            'zero': 'invalid_black', 'meters_per_raw_unit': None,
            'unit_basis': 'unknown unless verified producer metadata is supplied'}


def depth_color_preview(depth, min_m=None, max_m=None, *, scale_m_per_unit=None, color_scheme=0):
    profile = color_scale(min_m, max_m)
    pixels = np.asarray(depth)
    if pixels.ndim != 2 or pixels.dtype.kind != 'u' or pixels.dtype.itemsize != 2 or not pixels.size:
        raise ValueError('官方 Z16 显示输入必须为二维 uint16 深度码值')
    if scale_m_per_unit is not None and (isinstance(scale_m_per_unit, bool)
            or not isinstance(scale_m_per_unit, (int, float))
            or not math.isfinite(scale_m_per_unit) or scale_m_per_unit <= 0):
        raise ValueError('深度显示尺度必须为正有限数')
    if profile['range_mode'] == 'fixed' and scale_m_per_unit is None:
        raise ValueError('固定米范围预览需要已确认的源深度单位')
    if color_scheme not in (0, 2):
        raise ValueError('只支持官方 Jet 或 White to Black')
    if version('pyrealsense2') != SDK_VERSION:
        raise RuntimeError('RealSense SDK 版本与显示 profile 不同；不能回退到自定义色带')
    import pyrealsense2 as rs
    pixels = np.ascontiguousarray(pixels, dtype=np.uint16)
    device = rs.software_device()
    sensor = device.add_sensor('HC offline camera evidence')
    stream = rs.video_stream()
    stream.type, stream.fmt = rs.stream.depth, rs.format.z16
    stream.width, stream.height = pixels.shape[1], pixels.shape[0]
    stream.fps, stream.bpp = 30, 2
    stream_profile = sensor.add_video_stream(stream)
    queue = rs.frame_queue(1)
    colorizer = rs.colorizer()
    colorizer.set_option(rs.option.visual_preset, 0)
    colorizer.set_option(rs.option.color_scheme, color_scheme)
    if profile['range_mode'] == 'fixed':
        colorizer.set_option(rs.option.histogram_equalization_enabled, 0)
        colorizer.set_option(rs.option.min_distance, min_m)
        colorizer.set_option(rs.option.max_distance, max_m)
    sensor.open(stream_profile)
    sensor.start(queue)
    try:
        frame = rs.software_video_frame()
        frame.pixels, frame.bpp, frame.stride = pixels.data, 2, pixels.shape[1] * 2
        frame.profile = stream_profile.as_video_stream_profile()
        # The SDK API requires positive units even for histogram rendering.
        # A placeholder is display-only, never recorded as physical source units.
        frame.depth_units = scale_m_per_unit if scale_m_per_unit is not None else 1.
        sensor.on_video_frame(frame)
        output = np.asanyarray(colorizer.colorize(queue.wait_for_frame(1000)).get_data()).copy()
    finally:
        sensor.stop()
        sensor.close()
    profile.update(meters_per_raw_unit=scale_m_per_unit,
                   colormap=colorizer.get_option_value_description(rs.option.color_scheme, color_scheme),
                   display_only_units_placeholder=scale_m_per_unit is None,
                   unit_basis='explicit verified source metadata' if scale_m_per_unit is not None else profile['unit_basis'])
    return output, profile


def depth_color_legend(min_m=None, max_m=None, width=640):
    profile = color_scale(min_m, max_m)
    image = Image.new('RGB', (max(320, int(width)), 76), 'white')
    draw = ImageDraw.Draw(image)
    draw.text((8, 4), 'Intel RealSense SDK colorizer / Jet', fill='black')
    if profile['range_mode'] == 'per_frame_histogram':
        draw.text((8, 25), 'Dynamic histogram: no fixed distance/color scale.', fill='black')
    else:
        draw.text((8, 25), f'SDK fixed range: {min_m:g} to {max_m:g} m.', fill='black')
    draw.text((8, 47), 'Read source depth for values. Black = invalid.', fill='black')
    return image
