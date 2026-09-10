"""Depth temporal filtering is disabled for acquisition at startup."""


def without_temporal_filter(parameters):
    """Override both ROS parameter spellings without changing other settings."""
    result = dict(parameters)
    if isinstance(result.get('temporal_filter'), dict):
        result['temporal_filter'] = {**result['temporal_filter'], 'enable': False}
    result['temporal_filter.enable'] = False
    return result
