"""Conversions between configured microseconds and vendor parameter units."""

D435_RGB_MODELS = frozenset({'d435', 'd435i', 'd435f', 'd435if'})
D435_EXPOSURE_US = 3900


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
