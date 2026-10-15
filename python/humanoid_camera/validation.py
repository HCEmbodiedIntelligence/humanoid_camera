"""Camera parameter scalar checks migrated from the manager."""
import math
from .errors import CameraConfigurationError as DeploymentError

def validate_values(value, path=""):
    """Reject unsafe scalar values in addition to cross-resource validation."""
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise DeploymentError(f"{path}: 配置字段名必须是字符串")
            if key != 'instance_parameters':
                validate_values(item, f"{path}/{key}")
            if key.endswith(("_hz", "_ms", "_s", "_rad_s", "_rad_s2", "_rad_s3", "_m_s", "_m_s2", "_m_s3")) and not isinstance(item, (dict, list)):
                if isinstance(item, bool) or not isinstance(item, (int, float)) or item < 0:
                    raise DeploymentError(f"{path}/{key}: 必须是非负数")
            if key in {"control_frequency_hz", "diagnostic_frequency_hz"} and not 0 < item <= 1000:
                raise DeploymentError(f"{key}: 频率必须在 0–1000 Hz 之间（不含 0）")
    elif isinstance(value, list):
        for item in value:
            validate_values(item, path)
    elif isinstance(value, float) and not math.isfinite(value):
        raise DeploymentError(f"{path}: 必须是有限数值")
