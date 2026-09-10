"""Choose IR outputs explicitly when bypassing the vendor's rs_launch.py.

The C++ video profile manager enables available streams by default; the vendor
launch file disables infrared outputs. Keep RGBD launches equally explicit.
"""
from .exposure import D435_RGB_MODELS


INFRARED_DEFAULTS = {'enable_infra1': False, 'enable_infra2': False}


def infrared_parameters(camera):
    overrides = camera.get('parameters', {})
    result = {key: overrides.get(key, value) for key, value in INFRARED_DEFAULTS.items()}
    if camera['device_type'].lower() in D435_RGB_MODELS and camera.get('depth_auto_gain', True):
        result['enable_infra1'] = True
    return result
