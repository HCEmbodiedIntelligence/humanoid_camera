"""Shared camera identity validation for documents and direct provider launches."""
import json
import re

from humanoid_manager.deployment import DeploymentError


def normalize_camera_identity(camera, index, identifiers):
    """Normalize pasted whitespace before deriving namespaces or output topics."""
    raw = camera.get('id', '')
    label = f'第 {index + 1} 台相机 ID（cameras[{index}].id）'
    if not isinstance(raw, str):
        raise DeploymentError(f'{label} 必须是字符串')
    ident = raw.strip()
    received = json.dumps(raw[:100], ensure_ascii=True)
    if not ident:
        raise DeploymentError(f'{label} 不能为空；收到：{received}')
    if not re.fullmatch(r'[a-z][a-z0-9_]{0,63}', ident):
        raise DeploymentError(
            f'{label} 需为 1–64 个字符，以小写字母开头，只允许小写字母、数字和下划线；'
            f'收到：{received}'
        )
    if ident in identifiers:
        raise DeploymentError(f'{label} 重复: {ident}')
    identifiers.add(ident)
    camera['id'] = ident
    # The editor follows ID changes for the default namespace only.
    if camera.get('namespace') == raw:
        camera['namespace'] = ident
    return ident
