"""Generic camera inputs. Device drivers supply startup steps and normalized ROS outputs."""
import copy
import math
import re
from humanoid_manager.deployment import DeploymentError
from humanoid_manager.plugin_metadata import validate_settings, resolved_document
from .identity import normalize_camera_identity


def validate_cameras(value):
    if not isinstance(value, list) or len(value) > 16:
        raise DeploymentError('相机配置必须是列表，且最多 16 台')
    # Optional providers describe legacy formats; other brands use the common contract.
    from .profiles import VALIDATORS
    groups, result, identifiers, topics = {}, [], set(), set()
    for index, raw in enumerate(value):
        if not isinstance(raw, dict):
            raise DeploymentError('每台相机配置必须是对象')
        camera = copy.deepcopy(raw)
        ident = normalize_camera_identity(camera, index, identifiers)
        backend = camera.get('backend', 'realsense')  # Legacy documents omitted this field.
        if not isinstance(backend, str) or not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', backend):
            raise DeploymentError('无效相机 backend')
        camera['backend'] = backend
        if backend in VALIDATORS:
            groups.setdefault(backend, []).append(camera)
        else:
            # Standard topic inputs do not depend on a vendor package.
            camera.setdefault('enabled', True)
            camera.setdefault('required', True)
            camera.setdefault('pointcloud', False)
            camera.setdefault('fps', 30)
            for key, suffix in [('rgbd_topic', 'rgbd'), ('metadata_topic', 'metadata'),
                                ('pointcloud_topic', 'points'), ('pointcloud_metadata_topic', 'points_metadata')]:
                camera.setdefault(key, f'/{ident}/normalized/{suffix}')
            if type(camera['fps']) is not int or not 1 <= camera['fps'] <= 240:
                raise DeploymentError('相机 fps 必须为 1–240 的整数')
        result.append(camera)
    for backend, cameras in groups.items():
        normalized = VALIDATORS[backend](cameras)
        for old, new in zip(cameras, normalized):
            old.clear()
            old.update(new)
    for camera in result:
        ident = camera['id']
        for key in ('enabled', 'required', 'pointcloud'):
            if type(camera[key]) is not bool:
                raise DeploymentError(f'{ident}.{key} 必须为布尔值')
        camera.setdefault('max_actual_exposure_us', 5000)
        exposure = camera['max_actual_exposure_us']
        if type(exposure) is not int or not 1 <= exposure <= 5000:
            raise DeploymentError(f'{ident}.max_actual_exposure_us 必须是 1–5000 的整数')
        for key in ('sync_rgb_depth', 'timestamp_alignment'):
            camera.setdefault(key, True)
            if type(camera[key]) is not bool:
                raise DeploymentError(f'{ident}.{key} 必须为布尔值')
        for key in ('rgbd_max_midpoint_skew_ms', 'camera_max_error_ms'):
            number = camera.setdefault(key, 1.0)
            if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number) or not 0 < number <= 1000:
                raise DeploymentError(f'{ident}.{key} 必须是有限正数')
        if camera['backend'] in VALIDATORS:
            for key, suffix in [('rgbd_topic', 'rgbd'), ('metadata_topic', 'metadata'),
                                ('pointcloud_topic', 'points'), ('pointcloud_metadata_topic', 'points_metadata')]:
                camera.setdefault(key, f"/{camera['namespace']}/normalized/{suffix}")
            for key, suffix in [('rgb_topic', 'color/image_raw'), ('depth_topic', 'depth/image_rect_raw')]:
                camera.setdefault(key, f"/{camera['namespace']}/{camera['camera_name']}/{suffix}")
        settings = validate_settings({key: camera[key] for key in ('startup', 'instance_parameters') if key in camera})
        if 'startup' in camera or 'instance_parameters' in camera:
            camera.update({key: settings[key] for key in ('startup', 'instance_parameters')})
        resolved = resolved_document(camera, settings)
        keys = ['rgbd_topic', 'metadata_topic']
        if camera['backend'] == 'realsense':
            keys += ['rgb_topic', 'depth_topic']
        if camera['pointcloud']:
            keys += ['pointcloud_topic', 'pointcloud_metadata_topic']
        for key in keys:
            topic = resolved.get(key)
            if topic is None and camera['backend'] in VALIDATORS:
                continue  # Provider declares its normalized outputs.
            if not isinstance(topic, str) or not re.fullmatch(r'/(?:[A-Za-z_][A-Za-z0-9_]*)(?:/[A-Za-z_][A-Za-z0-9_]*)*', topic):
                raise DeploymentError(f'{ident}.{key} 必须是绝对 ROS 话题')
            if camera['enabled'] and topic in topics:
                raise DeploymentError(f'相机输出话题重复: {topic}')
            if camera['enabled']:
                topics.add(topic)
    return result
