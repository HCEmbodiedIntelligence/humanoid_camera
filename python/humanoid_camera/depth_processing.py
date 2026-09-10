"""Acquisition uses spatial depth filtering without temporal history."""

DEPTH_FILTER_PARAMETERS = {'temporal_filter.enable': False, 'spatial_filter.enable': True}


def acquisition_filters(parameters):
    """Override both ROS parameter spellings without changing other settings."""
    result = dict(parameters)
    for key, enabled in DEPTH_FILTER_PARAMETERS.items():
        group = key.split('.')[0]
        if isinstance(result.get(group), dict):
            result[group] = {**result[group], 'enable': enabled}
        result[key] = enabled
    return result
