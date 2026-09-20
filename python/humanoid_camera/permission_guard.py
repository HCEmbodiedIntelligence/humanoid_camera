"""Wait for accessible RealSense nodes before executing the unmodified vendor driver."""
import os
from pathlib import Path
import signal
import sys
import time


def _read(path):
    try:
        return path.read_text().strip()
    except OSError:
        return ''


def realsense_parent(path):
    for parent in (path, *path.parents):
        if _read(parent / 'idVendor') == '8086' and 'RealSense' in _read(parent / 'product'):
            return parent
    return None


def device_nodes(sys_root=Path('/sys'), dev_root=Path('/dev')):
    nodes = set()
    for device in (sys_root / 'bus/usb/devices').glob('*'):
        if realsense_parent(device.resolve()) != device.resolve():
            continue
        bus, address = _read(device / 'busnum'), _read(device / 'devnum')
        if bus.isdigit() and address.isdigit():
            nodes.add(dev_root / f'bus/usb/{int(bus):03d}/{int(address):03d}')
    for kind in ('video4linux', 'media', 'hidraw'):
        for device in (sys_root / 'class' / kind).glob('*'):
            if realsense_parent(device.resolve()) is not None:
                nodes.add(dev_root / device.name)
    return sorted(nodes)


def permission_problem(nodes, access=os.access):
    if not nodes:
        return '未检测到 RealSense 设备；等待连接'
    denied = [str(path) for path in nodes if not access(path, os.R_OK | os.W_OK)]
    if denied:
        return f'RealSense 设备缺少读写权限：{", ".join(denied[:8])}（共 {len(denied)} 个）；请停止节点后执行 install_camera_permissions.sh'
    return None


def main():
    if len(sys.argv) < 2:
        raise SystemExit('usage: python -m humanoid_camera.permission_guard DRIVER [ARGS...]')
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    last_log = float('-inf')
    while True:
        problem = permission_problem(device_nodes())
        if problem is None:
            os.execvp(sys.argv[1], sys.argv[1:])
        now = time.monotonic()
        if now - last_log >= 60:
            print(f'[camera permission check] {problem}；每 5 秒检查，日志最多每 60 秒一次', flush=True)
            last_log = now
        time.sleep(5)


if __name__ == '__main__':
    main()
