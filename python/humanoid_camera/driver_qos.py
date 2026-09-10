"""Explicit ROS publisher defaults; SYSTEM_DEFAULT can keep only one image."""


def driver_qos_parameters(overrides=None):
    # Official ProfilesManager uses root-level stream names on D405 and D435.
    # Exposure/profile parameters are sensor-prefixed, but these QoS names are not.
    parameters = overrides or {}
    return {stream + suffix: parameters.get(stream + suffix, 'DEFAULT')
            for stream in ('depth', 'color') for suffix in ('_qos', '_info_qos')}
