"""Verified ROS driver output contract, not a guess about SDK raw Z16 units."""

DRIVER_REFERENCE = (
    'https://github.com/realsenseai/realsense-ros/blob/4.57.7/'
    'realsense2_camera/src/base_realsense_node.cpp#L252-L287'
)


def installed_driver_version():
    from pathlib import Path
    from xml.etree import ElementTree
    from ament_index_python.packages import get_package_share_directory
    try:
        path = Path(get_package_share_directory('realsense2_camera')) / 'package.xml'
        return ElementTree.parse(path).findtext('version')
    except (LookupError, OSError, ElementTree.ParseError):
        return None


def realsense_ros_depth_profile(driver_version, source_format):
    # Verified against this exact release: fillCVMatImageAndReturnStatus calls
    # fix_depth_scale for depth frames, then publishes Z16 as ROS 16UC1 in mm.
    # Unsupported versions/formats retain the raw image with unknown units.
    if driver_version != '4.57.7' or source_format != 'Z16':
        return None
    return {
        'schema_version': 'hc-depth-source/1', 'encoding': '16UC1', 'unit': 'mm',
        'scale_m_per_unit': .001, 'invalid_values': [0], 'invalid_nonfinite': True,
        'registered_to_rgb': False,
        'registration_evidence': 'original depth/image_rect_raw optical grid; no RGB registration',
        'evidence': {'kind': 'driver_contract', 'reference': DRIVER_REFERENCE,
                     'version': 'realsense2_camera/4.57.7'},
    }
