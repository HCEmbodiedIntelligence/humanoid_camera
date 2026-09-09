"""Camera IDs are normalized consistently without changing explicit ROS outputs."""
import copy
import json

import pytest

from humanoid_camera.configuration import validate_cameras
from humanoid_camera.realsense_profile import validate_cameras as validate_realsense
from humanoid_manager.deployment import DeploymentError


@pytest.fixture(params=[validate_cameras, validate_realsense])
def validate(request):
    return request.param


@pytest.mark.parametrize('ident', ['camera_left', ' \tcamera_left\n', '\u00a0camera_left\u3000'])
@pytest.mark.parametrize('backend', ['realsense', 'ros_topics'])
def test_pasted_id_is_trimmed_before_deriving_outputs(validate, ident, backend):
    values = [{'id': ident, 'backend': backend, 'namespace': ident}]
    original = copy.deepcopy(values)
    camera = validate(values)[0]
    assert camera['id'] == camera['namespace'] == 'camera_left'
    if backend == 'ros_topics' or validate is validate_cameras:
        assert camera['rgbd_topic'] == '/camera_left/normalized/rgbd'
    assert values == original
    assert validate([camera]) == [camera]


def test_explicit_namespace_and_topics_are_preserved(validate):
    camera = validate([{'id': ' camera_left ', 'namespace': 'left_optical',
                        'rgbd_topic': '/left/rgbd'}])[0]
    assert camera['id'] == 'camera_left'
    assert camera['namespace'] == 'left_optical'
    assert camera['rgbd_topic'] == '/left/rgbd'


@pytest.mark.parametrize('ident', ['', ' \t\n', None, 42, 'Camera_left', '1camera',
                                   'camera left', 'camera-left', 'camera_left\u200b',
                                   'camera＿left', 'a' * 65])
def test_invalid_id_identifies_the_actual_camera_and_invisible_characters(validate, ident):
    with pytest.raises(DeploymentError) as caught:
        validate([{'id': 'camera_left', 'enabled': False}, {'id': ident}])
    message = str(caught.value)
    assert '第 2 台相机 ID（cameras[1].id）' in message
    if isinstance(ident, str):
        assert json.dumps(ident, ensure_ascii=True) in message


def test_duplicate_ids_are_checked_after_trimming(validate):
    with pytest.raises(DeploymentError, match='重复: camera_left'):
        validate([{'id': 'camera_left'}, {'id': ' camera_left '}])


def test_id_rules_cover_other_camera_backends():
    camera = validate_cameras([{'id': ' camera_left ', 'backend': 'custom_vendor'}])[0]
    assert camera['id'] == 'camera_left'
    assert camera['rgbd_topic'] == '/camera_left/normalized/rgbd'
    with pytest.raises(DeploymentError, match='重复: camera_left'):
        validate_cameras([{'id': 'camera_left'}, {'id': ' camera_left ', 'backend': 'ros_topics'}])


def test_maximum_length_id_is_accepted(validate):
    assert validate([{'id': 'a' * 64}])[0]['id'] == 'a' * 64
