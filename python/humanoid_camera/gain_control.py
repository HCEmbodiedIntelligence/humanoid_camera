"""Bounded gain feedback; exposure and image pixels are never changed here."""
import math


def image_brightness(message):
    channels = {'mono8': 1, '8UC1': 1, 'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4}
    count = channels.get(message.encoding)
    if count is None:
        raise ValueError(f'不支持的亮度反馈图像编码: {message.encoding}')
    width, height, step = message.width, message.height, message.step
    if width <= 0 or height <= 0 or step < width * count or len(message.data) < step * height:
        raise ValueError('亮度反馈图像尺寸或数据无效')
    data = memoryview(message.data)
    total = samples = 0
    for y in range(0, height, max(1, math.ceil(height / 64))):
        for x in range(0, width, max(1, math.ceil(width / 64))):
            offset = y * step + x * count
            if count == 1:
                value = data[offset]
            else:
                r, g, b = data[offset:offset + 3]
                if message.encoding.startswith('bgr'):
                    r, b = b, r
                value = (77 * r + 150 * g + 29 * b) / 256
            total += value
            samples += 1
    return total / samples


def next_gain(current, brightness, minimum, maximum, step=1):
    """Target mid-gray with a dead band and at most four hardware steps per update."""
    if not all(math.isfinite(x) for x in (current, brightness, minimum, maximum, step)):
        raise ValueError('无效的增益反馈数值')
    if step <= 0 or maximum < minimum:
        raise ValueError('相机增益范围与配置上限不兼容')
    if current < minimum or current > maximum:
        proposed = min(maximum, max(minimum, current))
    elif abs(128 - brightness) <= 10:
        return int(current)
    else:
        proposed = current + (4 * step if brightness < 128 else -4 * step)
    # The configured maximum need not lie on a supported hardware step.
    highest = minimum + math.floor((maximum - minimum) / step) * step
    return int(min(highest, max(minimum, minimum + round((proposed - minimum) / step) * step)))
