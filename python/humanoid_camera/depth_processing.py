"""Depth processing policy, including the driver's implicit SDK syncer path."""

DEPTH_FILTER_PARAMETERS = {'temporal_filter.enable': False, 'spatial_filter.enable': True}
INDEPENDENT_FILTER_PARAMETERS = {name + '.enable': False for name in (
    'decimation_filter', 'hdr_merge', 'filter_by_sequence_id', 'disparity_filter', 'disparity_to_depth',
    'spatial_filter', 'temporal_filter', 'hole_filling_filter', 'rotation_filter',
    'pointcloud', 'align_depth', 'colorizer')}


def acquisition_filters(parameters, *, sdk_sync=True):
    """Override both ROS parameter spellings without changing other settings."""
    result = dict(parameters)
    # The ROS driver invokes the SDK syncer when ANY processing filter is on,
    # even with enable_sync=false. Independent capture must disable that path.
    settings = DEPTH_FILTER_PARAMETERS if sdk_sync else INDEPENDENT_FILTER_PARAMETERS
    for key, enabled in settings.items():
        group = key.split('.')[0]
        if isinstance(result.get(group), dict):
            result[group] = {**result[group], 'enable': enabled}
        result[key] = enabled
    return result
