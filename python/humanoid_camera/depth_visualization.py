"""False-color plots of ROS uint16 millimeter depth; never alter measured data."""
import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def color_scale(min_m=.2, max_m=2.):
    if not all(math.isfinite(v) for v in (min_m, max_m)) or not 0 <= min_m < max_m <= 65.535:
        raise ValueError('深度预览范围必须满足 0 ≤ 最小值 < 最大值 ≤ 65.535 m')
    return {'min_m': min_m, 'max_m': max_m, 'meters_per_raw_unit': .001,
            'colormap': 'red_yellow_green_cyan_blue', 'near': 'red', 'far': 'blue',
            'zero': 'invalid_black', 'out_of_range': 'clipped_to_endpoint_color',
            'range_mode': 'fixed', 'unit_basis': 'ROS 16UC1 millimeters; not an independent accuracy calibration'}


def rainbow(values):
    anchors = np.array([[255, 0, 0], [255, 255, 0], [0, 255, 0], [0, 255, 255], [0, 0, 255]])
    return np.stack([np.interp(values, np.linspace(0, 1, 5), anchors[:, c]) for c in range(3)], axis=-1).astype(np.uint8)


def depth_color_preview(depth, min_m=.2, max_m=2.):
    scale = color_scale(min_m, max_m)
    if depth.ndim != 2 or depth.dtype.kind not in 'ui' or np.any(depth < 0) or np.any(depth > 65535):
        raise ValueError('彩色预览需要 ROS 16 位深度数值')
    fraction = np.clip((depth.astype(np.float64) * .001 - min_m) / (max_m - min_m), 0, 1)
    preview = rainbow(fraction)
    preview[depth == 0] = 0
    return preview, scale


def depth_color_legend(min_m=.2, max_m=2., width=640):
    color_scale(min_m, max_m)
    width = max(320, int(width))
    legend = Image.new('RGB', (width, 76), 'white')
    draw = ImageDraw.Draw(legend)
    font = ImageFont.load_default()
    draw.text((12, 3), 'Depth (m): RED = near, BLUE = far; BLACK = invalid', fill='black')
    bar_width = width - 24
    bar = np.repeat(rainbow(np.linspace(0, 1, bar_width))[None, :, :], 20, axis=0)
    legend.paste(Image.fromarray(bar), (12, 20))
    for index, value in enumerate(np.linspace(min_m, max_m, 5)):
        label = f'{value:.2f}'
        x = 12 + int(index / 4 * (bar_width - 1))
        draw.line((x, 40, x, 44), fill='black')
        label_width = font.getbbox(label)[2] if hasattr(font, 'getbbox') else font.getsize(label)[0]
        draw.text((min(width-label_width-4, max(4, x-label_width//2)), 46), label, fill='black')
    draw.text((12, 62), 'Fixed display range; outside values use endpoint colors.', fill='black')
    return legend
