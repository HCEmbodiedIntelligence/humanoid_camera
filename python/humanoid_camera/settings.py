"""Device instance values and plugin-owned schemas; no vendor class knowledge."""
import copy
import math
import re

from .errors import CameraConfigurationError as DeploymentError

SETTINGS_DEFAULTS = {'instance_parameters': {}, 'startup': [], 'capabilities': {}}
REFERENCE = re.compile(r'\$\{([A-Za-z_][A-Za-z0-9_]*)\}')


def settings_from_manifest(manifest):
    return {key: copy.deepcopy(manifest.get(key, default)) for key, default in SETTINGS_DEFAULTS.items()}


def validate_settings(settings):
    if not isinstance(settings, dict) or set(settings) - SETTINGS_DEFAULTS.keys():
        raise DeploymentError('device settings contain unknown fields')
    result = {**copy.deepcopy(SETTINGS_DEFAULTS), **copy.deepcopy(settings)}
    values = result['instance_parameters']
    if not isinstance(values, dict) or len(values) > 128:
        raise DeploymentError('instance_parameters must be a mapping of at most 128 strings')
    for name, value in values.items():
        if (not isinstance(name, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]*', name)
                or not isinstance(value, str) or len(value) > 4096 or '\x00' in value
                or '$(' in value or '${' in value):
            raise DeploymentError('instance_parameters require literal string values and valid names')
    from robot_bringup.startup import validate_startup
    from robot_bringup.errors import HardwareConfigurationError
    try:
        validate_startup(expand(result['startup'], values))
    except HardwareConfigurationError as error:
        raise DeploymentError(str(error)) from error
    capabilities = result['capabilities']
    if not isinstance(capabilities, dict):
        raise DeploymentError('plugin capabilities must be a mapping')
    grippers = capabilities.get('grippers', {})
    if not isinstance(grippers, dict):
        raise DeploymentError('capabilities.grippers must be a mapping')
    for name, targets in grippers.items():
        if not isinstance(name, str) or not isinstance(targets, dict):
            raise DeploymentError('gripper capabilities require named target mappings')
        tolerance_keys = ('position_tolerance', 'open_position_tolerance', 'closed_position_tolerance')
        for key in ('open_position', 'closed_position', 'max_effort', *tolerance_keys):
            if key in targets and (isinstance(targets[key], bool) or not isinstance(targets[key], (int, float))
                                   or not math.isfinite(targets[key])):
                raise DeploymentError(f'{name}.{key} must be finite')
        if targets.get('max_effort', 0) < 0:
            raise DeploymentError(f'{name}.max_effort must be nonnegative')
        for key in tolerance_keys:
            if key in targets and targets[key] <= 0:
                raise DeploymentError(f'{name}.{key} must be positive')
    return result


def expand(value, parameters):
    if isinstance(value, str):
        def substitute(match):
            name = match.group(1)
            if name not in parameters:
                raise DeploymentError(f'undefined device instance parameter: {name}')
            return parameters[name]
        return REFERENCE.sub(substitute, value)
    if isinstance(value, dict):
        result = {}
        for key, item in value.items():
            resolved_key = expand(key, parameters) if isinstance(key, str) else key
            if resolved_key in result:
                raise DeploymentError(f'device instance expansion creates duplicate key: {resolved_key}')
            result[resolved_key] = expand(item, parameters)
        return result
    if isinstance(value, list):
        return [expand(item, parameters) for item in value]
    return copy.deepcopy(value)


def resolved_document(document, manifest):
    return expand(document, manifest.get('instance_parameters', {}))
