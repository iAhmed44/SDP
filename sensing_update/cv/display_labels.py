"""Presentation labels only; semantic classes remain unchanged for fusion."""
import colorsys
import hashlib
from collections import defaultdict


def label_color(name):
    """Stable bright BGR class color across processes and runs."""
    value = int.from_bytes(hashlib.sha256(name.encode('utf-8')).digest()[:4], 'big')
    r, g, b = colorsys.hsv_to_rgb((value % 360) / 360, .72, .95)
    return tuple(round(255 * c) for c in (b, g, r))


def numbered_detections(detections):
    """Number each class left-to-right in this result, without mutating caches.

    Numbers are display order, not persistent physical-object identities.
    """
    result = [dict(d) for d in detections]
    groups = defaultdict(list)
    for i, d in enumerate(result):
        groups[d['name']].append(i)
    for name, indices in groups.items():
        indices.sort(key=lambda i: ((result[i]['bbox'][0] + result[i]['bbox'][2]) / 2,
                                    result[i]['bbox'][1], i))
        for number, i in enumerate(indices, 1):
            result[i]['display_label'] = f'{name}{number}'
            result[i]['class_color_bgr'] = label_color(name)
    return result
