"""Conversions between configured microseconds and vendor parameter units."""
import math

D435_RGB_MODELS = frozenset({'d435', 'd435i', 'd435f', 'd435if'})
D435_EXPOSURE_US = 3900


def metadata_exposure_scale_us(device_type, stream):
    """D435 USB RGB metadata returns md_rgb_control.manual_exp in UVC ticks.

    librealsense ds_color_common::register_metadata forwards this field without
    unit conversion (verified in 2.55.1, 2.56.5 and 2.58.3). This is separate
    from the microsecond depth metadata and from all timestamp fields.
    """
    return 100 if device_type.lower() in D435_RGB_MODELS and stream == 'rgb' else 1


def metadata_exposure_us(device_type, stream, raw):
    value = raw['actual_exposure']
    if isinstance(value, bool):
        raise ValueError('invalid actual_exposure')
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('invalid actual_exposure')
    return value * metadata_exposure_scale_us(device_type, stream)


def override_driver_parameters(parameters, overrides):
    """Override dotted and nested ROS spellings consistently."""
    result = dict(parameters)
    for name, value in overrides.items():
        group, _, child = name.partition('.')
        if child and isinstance(result.get(group), dict):
            result[group] = {**result[group], child: value}
        result[name] = value
    return result


def rgb_exposure_parameter(device_type, exposure_us):
    """D435 UVC RGB exposure uses 100 us ticks; depth exposure uses microseconds.

    librealsense's uvc_pu_option forwards the RGB value unchanged to
    V4L2_CID_EXPOSURE_ABSOLUTE. Round down so quantization cannot exceed the
    configured duration. Other providers retain their existing parameter units.
    """
    if device_type.lower() in D435_RGB_MODELS:
        if exposure_us < 100:
            raise ValueError('D435 RGB 手动曝光不能小于 100 μs（驱动步长为 100 μs）')
        return int(exposure_us) // 100
    return int(exposure_us)
