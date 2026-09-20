#!/usr/bin/env bash
set -euo pipefail
if [[ $EUID -ne 0 ]]; then
  echo '请先停止机器人/相机节点，再使用 sudo 执行本脚本。' >&2
  exit 1
fi
if pgrep -f '(^|/)realsense2_camera_node([[:space:]]|$)' >/dev/null; then
  echo '检测到 RealSense 节点或权限等待进程，请先停止机器人/相机节点。' >&2
  exit 1
fi
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
rule_file="$script_dir/../rules/99-humanoid-realsense.rules"
if [[ ! -f "$rule_file" ]]; then
  rule_file="$script_dir/../../share/humanoid_camera/rules/99-humanoid-realsense.rules"
fi
getent group plugdev >/dev/null
install -m 0644 "$rule_file" /etc/udev/rules.d/99-humanoid-realsense.rules
udevadm control --reload-rules
for device in /sys/bus/usb/devices/*; do
  [[ -r "$device/idVendor" && -r "$device/product" ]] || continue
  [[ $(<"$device/idVendor") == 8086 && $(<"$device/product") == *RealSense* ]] || continue
  device_path=$(readlink -f "$device")
  udevadm trigger --action=change "$device_path"
  udevadm trigger --action=change --parent-match="$device_path"
done
udevadm settle --timeout=10
echo 'RealSense 权限规则已安装并应用。未启动或重启任何节点。'
echo '运行用户需属于 plugdev 组；如刚加入该组，请重新登录。现在可手动启动节点。'
