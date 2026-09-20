from pathlib import Path

from humanoid_camera.permission_guard import device_nodes, permission_problem


def test_only_realsense_devices_are_checked_including_usb_and_video(tmp_path):
    sys_root, dev_root = tmp_path/'sys', tmp_path/'dev'
    camera = sys_root/'devices/usb/4-2'
    camera.mkdir(parents=True)
    for key, value in {'idVendor':'8086', 'product':'Intel RealSense D405', 'busnum':'4', 'devnum':'5'}.items():
        (camera/key).write_text(value)
    bus = sys_root/'bus/usb/devices'; bus.mkdir(parents=True)
    (bus/'4-2').symlink_to(camera, target_is_directory=True)
    video = camera/'4-2:1.0/video4linux/video3'; video.mkdir(parents=True)
    klass = sys_root/'class/video4linux'; klass.mkdir(parents=True)
    (klass/'video3').symlink_to(video, target_is_directory=True)
    unrelated = sys_root/'devices/usb/4-3'; unrelated.mkdir()
    (unrelated/'idVendor').write_text('8086'); (unrelated/'product').write_text('Intel unrelated device')
    (bus/'4-3').symlink_to(unrelated, target_is_directory=True)
    assert device_nodes(sys_root, dev_root) == sorted([dev_root/'video3', dev_root/'bus/usb/004/005'])
    assert permission_problem(device_nodes(sys_root, dev_root), access=lambda *_: True) is None
    error = permission_problem([dev_root/'video3'], access=lambda *_: False)
    assert str(dev_root/'video3') in error and '缺少读写权限' in error
    assert '未检测到' in permission_problem([])


def test_guard_waits_without_starting_driver_and_limits_logs(monkeypatch, capsys):
    from humanoid_camera import permission_guard as guard
    now = [0.]
    monkeypatch.setattr(guard.sys, 'argv', ['guard', 'driver', '--ros-args'])
    monkeypatch.setattr(guard.signal, 'signal', lambda *_: None)
    monkeypatch.setattr(guard.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(guard.time, 'sleep', lambda seconds: now.__setitem__(0, now[0]+seconds))
    monkeypatch.setattr(guard, 'device_nodes', lambda: [Path('/dev/video3')])
    monkeypatch.setattr(guard, 'permission_problem', lambda _: 'denied' if now[0]<65 else None)
    calls=[]
    class Executed(Exception): pass
    def execute(*args):
        calls.append(args);raise Executed
    monkeypatch.setattr(guard.os, 'execvp', execute)
    import pytest
    with pytest.raises(Executed): guard.main()
    assert now[0] == 65
    assert len(capsys.readouterr().out.splitlines()) == 2
    assert calls == [('driver', ['driver', '--ros-args'])]
