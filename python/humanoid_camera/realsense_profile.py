"""RealSense provider, including compatibility with pre-profile camera documents."""
import copy
import math
import re
from humanoid_manager.deployment import DeploymentError
from humanoid_manager.configuration import validate_values
from .identity import normalize_camera_identity
from .exposure import D435_RGB_MODELS, D435_EXPOSURE_US, rgb_exposure_parameter
from .depth_processing import acquisition_filters

def validate_cameras(value):
    """Validate robot-owned camera definitions without touching camera hardware."""
    if not isinstance(value, list) or len(value) > 16:
        raise DeploymentError("相机配置必须是列表，且最多 16 台")
    result, identifiers, endpoints, serials = [], set(), set(), set()
    ros_name = re.compile(r"[A-Za-z_][A-Za-z0-9_]{0,63}")
    ros_topic = re.compile(r"/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*")
    for index, raw in enumerate(value):
        if not isinstance(raw, dict):
            raise DeploymentError("每台相机配置必须是对象")
        camera = copy.deepcopy(raw)
        ident = normalize_camera_identity(camera, index, identifiers)
        camera.setdefault("enabled", True)
        camera.setdefault("backend", "realsense")
        camera.setdefault("required", True)
        camera.setdefault("pointcloud", False)
        camera.setdefault("device_type", "d405" if camera["backend"] == "realsense" else "custom_rgbd")
        camera.setdefault("serial_no", "")
        camera.setdefault("sync_rgb_depth", True)
        camera.setdefault("timestamp_alignment", camera.get("normalize_timestamps", True))
        camera.pop("normalize_timestamps", None)
        camera.setdefault("max_actual_exposure_us", 5000)
        camera.setdefault("rgbd_max_midpoint_skew_ms", 1.0)
        camera.setdefault("camera_max_error_ms", 1.0)
        for key in ("enabled", "required", "pointcloud", "sync_rgb_depth", "timestamp_alignment"):
            if type(camera[key]) is not bool:
                raise DeploymentError(f"{ident}.{key} 必须为布尔值")
        if not isinstance(camera["device_type"], str) or not re.fullmatch(r"[A-Za-z0-9_.-]{1,64}", camera["device_type"]):
            raise DeploymentError(f"{ident}.device_type 无效")
        if not isinstance(camera["serial_no"], str) or len(camera["serial_no"]) > 128 or any(c.isspace() for c in camera["serial_no"]):
            raise DeploymentError(f"{ident}.serial_no 无效")
        # Official ROS driver strips this legacy string-typing prefix.
        if camera['backend'] == 'realsense':
            camera['serial_no'] = camera['serial_no'].removeprefix('_')
        if camera["enabled"] and camera["serial_no"]:
            if camera["serial_no"] in serials:
                raise DeploymentError(f"相机序列号重复: {camera['serial_no']}")
            serials.add(camera["serial_no"])
        exposure = camera["max_actual_exposure_us"]
        if type(exposure) is not int or not 1 <= exposure <= 5000:
            raise DeploymentError(f"{ident}.max_actual_exposure_us 必须是 1–5000 的整数")
        for key in ("rgbd_max_midpoint_skew_ms", "camera_max_error_ms"):
            number = camera[key]
            if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number) or not 0 < number <= 1000:
                raise DeploymentError(f"{ident}.{key} 必须是 0–1000 的有限正数")
        if camera["backend"] not in {"realsense", "ros_topics"}:
            raise DeploymentError(f"{ident}: backend 只支持 realsense 或 ros_topics")
        if camera["backend"] == "ros_topics":
            camera.setdefault("fps", 30)
            if type(camera["fps"]) is not int or not 1 <= camera["fps"] <= 240:
                raise DeploymentError(f"{ident}.fps 必须是 1–240 的整数")
            camera.setdefault("rgbd_topic", f"/{ident}/normalized/rgbd")
            camera.setdefault("metadata_topic", f"/{ident}/normalized/metadata")
            camera.setdefault("pointcloud_topic", f"/{ident}/normalized/points")
            camera.setdefault("pointcloud_metadata_topic", f"/{ident}/normalized/points_metadata")
            for key in ("rgbd_topic", "metadata_topic"):
                if not isinstance(camera[key], str) or not ros_topic.fullmatch(camera[key]):
                    raise DeploymentError(f"{ident}.{key} 必须是绝对 ROS 话题")
            if camera["pointcloud"]:
                for key in ("pointcloud_topic", "pointcloud_metadata_topic"):
                    if not isinstance(camera[key], str) or not ros_topic.fullmatch(camera[key]):
                        raise DeploymentError(f"{ident}.{key} 必须是绝对 ROS 话题")
            result.append(camera)
            continue
        manual_d435 = camera['device_type'].lower() in D435_RGB_MODELS
        initial_exposure = D435_EXPOSURE_US if manual_d435 else 4500
        defaults = {
            "namespace": ident, "camera_name": "camera", "width": 640, "height": 480, "fps": 30,
            "color_format": "RGB8", "depth_format": "Z16", "align_depth": False,
            "depth_auto_exposure": not manual_d435, "depth_exposure_us": initial_exposure, "depth_gain": 64,
            "depth_auto_exposure_limit_us": 4500, "depth_auto_gain_limit": 128 if manual_d435 else 64,
            "color_auto_exposure": False, "color_exposure_us": initial_exposure,
            "depth_auto_gain": manual_d435, "color_auto_gain": manual_d435, "color_auto_gain_limit": 128,
            "color_gain": 64, "parameters": {},
        }
        for key, default in defaults.items():
            camera.setdefault(key, default)
        for key in ("namespace", "camera_name"):
            if not isinstance(camera[key], str) or not ros_name.fullmatch(camera[key]):
                raise DeploymentError(f"{ident}.{key} 不是有效 ROS 名称")
        endpoint = (camera["namespace"], camera["camera_name"])
        if endpoint in endpoints:
            raise DeploymentError(f"相机 ROS 命名空间与节点名重复: /{endpoint[0]}/{endpoint[1]}")
        endpoints.add(endpoint)
        for key in ("align_depth", "depth_auto_exposure", "color_auto_exposure", "depth_auto_gain", "color_auto_gain"):
            if type(camera[key]) is not bool:
                raise DeploymentError(f"{ident}.{key} 必须为布尔值")
        # Move older D435 auto-depth configurations to the paired 3.9 ms preset.
        if manual_d435:
            if camera['depth_auto_exposure']:
                camera['depth_exposure_us'] = camera['color_exposure_us'] = D435_EXPOSURE_US
            camera['depth_auto_exposure'] = False
            camera['color_auto_exposure'] = False
            camera['sync_rgb_depth'] = True
        elif camera['depth_auto_gain'] or camera['color_auto_gain']:
            raise DeploymentError(f'{ident}: 程序自动增益目前只支持 D435 系列')
        for key, low, high in (("width", 1, 8192), ("height", 1, 8192), ("fps", 1, 240),
                               ("depth_exposure_us", 1, 5000), ("depth_gain", 0, 10000),
                               ("depth_auto_exposure_limit_us", 1, 5000), ("depth_auto_gain_limit", 1, 10000),
                               ("color_exposure_us", 1, 5000), ("color_gain", 0, 10000),
                               ("color_auto_gain_limit", 1, 10000)):
            number = camera[key]
            if type(number) is not int or not low <= number <= high:
                raise DeploymentError(f"{ident}.{key} 必须是 {low}–{high} 的整数")
        depth_setting = camera["depth_auto_exposure_limit_us"] if camera["depth_auto_exposure"] else camera["depth_exposure_us"]
        if depth_setting > camera["max_actual_exposure_us"]:
            raise DeploymentError(f"{ident}: 深度曝光设置不能超过曝光要求上限")
        if camera["device_type"].lower() != "d405" and not camera["color_auto_exposure"] and camera["color_exposure_us"] > camera["max_actual_exposure_us"]:
            raise DeploymentError(f"{ident}: RGB 手动曝光不能超过曝光要求上限")
        if not camera['color_auto_exposure']:
            try:
                rgb_exposure_parameter(camera['device_type'], camera['color_exposure_us'])
            except ValueError as error:
                raise DeploymentError(f'{ident}: {error}') from error
        for key in ("color_format", "depth_format"):
            if not isinstance(camera[key], str) or not re.fullmatch(r"[A-Za-z0-9_]{1,32}", camera[key]):
                raise DeploymentError(f"{ident}.{key} 无效")
        if not isinstance(camera["parameters"], dict):
            raise DeploymentError(f"{ident}.parameters 必须为对象")
        validate_values(camera["parameters"], f"/cameras/{ident}/parameters")
        camera['parameters'] = acquisition_filters(camera['parameters'])
        if camera['device_type'].lower() in D435_RGB_MODELS:
            for group in ('rgb_camera', 'depth_module'):
                for leaf in ('enable_auto_exposure', 'exposure', 'gain'):
                    camera['parameters'].pop(group + '.' + leaf, None)
                    if isinstance(camera['parameters'].get(group), dict):
                        camera['parameters'][group].pop(leaf, None)
        result.append(camera)
    active_realsense = [x for x in result if x["backend"] == "realsense" and x["enabled"]]
    if len(active_realsense) > 1 and any(not x["serial_no"] for x in active_realsense):
        raise DeploymentError("同时启用多台 RealSense 时，每台都必须填写唯一序列号")
    return result
