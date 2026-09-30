#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test helper
#
# Details:  The machinery of the graphics regression test (#2704): what a scene looks like as a
#           FINGERPRINT, taken from SC.renderer.GetGraphicsData() without a window, and how two
#           fingerprints are compared.
#
#           A fingerprint holds, per item - or per item type for a model of more than 32 items -
#           and per kind of element (lines, spheres, circles, texts, triangles):
#             - the NUMBER of elements, which must agree exactly;
#             - min, max and mean of the points per coordinate, of the colors per channel, of the
#               radii, the sphere resolutions, the circle segments, the font sizes and the triangle
#               normals, which must agree to a relative 1e-5;
#             - the texts themselves, which must agree exactly.
#           So a difference reads as "object 3: triangles 12 -> 10" or "the mean z of the triangles
#           of object 0 moved", and the reference is a JSON file a human can read in a diff.
#
#           The references are in python/testing/graphicsReferences/, one file per case. A case
#           whose reference is missing, or every case when EXUDYN_RECORD_GRAPHICS_REFERENCES=1 is
#           set, writes its reference and the test says so and fails, so that nothing is recorded
#           unnoticed.
#
#           A case with VARIANTS - one model drawn with different settings - stores the fingerprint
#           of the default and, per variant, only the groups that differ from it, so that thirty
#           settings on one model stay a file of the size of one: CheckVariantsAgainstReference.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-27
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import json
import os

import numpy as np

referenceDirectory = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'graphicsReferences')
relativeTolerance = 1e-5              #maintainer, 2026-09-27
itemLimit = 32                        #per item up to this many items, per item type above
kinds = ['lines', 'spheres', 'circles', 'texts', 'triangles']
#the scalar quantities of an element, under the keys GetGraphicsData() gives them
scalars = {'spheres': ['radius', 'resolution'], 'circles': ['radius', 'numberOfSegments'], 'texts': ['fontSize']}
quantities = ['points', 'colors', 'radius', 'resolution', 'numberOfSegments', 'fontSize', 'normals']
itemTypeNames = {0: 'None', 1: 'Node', 2: 'Object', 3: 'Marker', 4: 'Load', 5: 'Sensor'}


def GroupKey(item, perItem):
    """'Object 3', or 'Object' per item type; a static element of the scene is 'static <code>'"""
    (system, itemType, index) = (int(item[0]), int(item[1]), int(item[2]))
    if system < 0:
        return 'static ' + str(index)
    name = itemTypeNames.get(itemType, str(itemType))
    if system != 0:
        name = 'system ' + str(system) + ' ' + name
    return name + ' ' + str(index) if perItem else name


def Statistics(values):
    """min, max and mean over the first axis, per component, as plain lists; to 8 digits, which is
    what the float data carries and far below the tolerance, and keeps the references small"""
    values = np.asarray(values, dtype=float)
    values = values.reshape(values.shape[0], -1) if values.ndim > 1 else values.reshape(-1, 1)
    return {name: [float('%.8g' % value) for value in statistic]
            for (name, statistic) in [('min', values.min(axis=0)), ('max', values.max(axis=0)),
                                      ('mean', values.mean(axis=0))]}


def Fingerprint(data, perItem=None):
    """the fingerprint of the dictionary SC.renderer.GetGraphicsData() returns; perItem=None groups
    per item up to itemLimit items and per item type above, True or False forces it"""
    allItems = np.concatenate([data[kind]['items'] for kind in kinds if len(data[kind]['items'])]) \
        if any(len(data[kind]['items']) for kind in kinds) else np.zeros((0, 3), dtype=int)
    distinctItems = set(map(tuple, allItems.tolist()))
    if perItem is None:
        perItem = len(distinctItems) <= itemLimit
    groups = {}
    for kind in kinds:
        entry = data[kind]
        keys = [GroupKey(item, perItem) for item in entry['items']]
        for key in sorted(set(keys)):
            mask = np.array([k == key for k in keys])
            group = groups.setdefault(key, {})
            summary = {'count': int(mask.sum())}
            points = np.asarray(entry['points'])[mask].reshape(-1, 3)
            summary['points'] = Statistics(points)
            colors = np.asarray(entry['colors'])[mask].reshape(-1, 4)
            summary['colors'] = Statistics(colors)
            for quantity in scalars.get(kind, []):
                summary[quantity] = Statistics(np.asarray(entry[quantity])[mask])
            if kind == 'triangles':
                summary['normals'] = Statistics(np.asarray(entry['normals'])[mask].reshape(-1, 3))
            if kind == 'texts':
                summary['text'] = sorted(text for (text, m) in zip(entry['text'], mask) if m)
            group[kind] = summary
    return {'formatVersion': 1, 'perItem': perItem, 'items': len(distinctItems), 'groups': groups}


def GroupDifferences(key, a, b, tolerance=relativeTolerance):
    """what differs in the group key between two fingerprints; a or b is None where it is missing"""
    if a is None and b is None:
        return []
    if a is None or b is None:
        return [key + ': ' + ('new' if a is None else 'gone')]
    lines = []
    for kind in kinds:
        (x, y) = (a.get(kind), b.get(kind))
        if x is None and y is None:
            continue
        if x is None or y is None or x['count'] != y['count']:
            lines.append(key + ': ' + kind + ' ' + str(x['count'] if x else 0) + ' -> '
                         + str(y['count'] if y else 0))
            continue
        if x.get('text') != y.get('text'):
            lines.append(key + ': texts ' + str(x.get('text')) + ' -> ' + str(y.get('text')))
        for quantity in quantities:
            if quantity not in x or quantity not in y:
                continue
            for statistic in ['min', 'max', 'mean']:
                for (component, (u, v)) in enumerate(zip(x[quantity][statistic], y[quantity][statistic])):
                    if abs(u - v) > tolerance * (1 + max(abs(u), abs(v))):
                        lines.append(key + ': ' + kind + ' ' + quantity + ' ' + statistic + '['
                                     + str(component) + '] ' + repr(u) + ' -> ' + repr(v))
    return lines


def Differences(reference, current, tolerance=relativeTolerance):
    """what differs between two fingerprints, as readable lines; [] if they agree"""
    lines = []
    if reference.get('perItem') != current.get('perItem'):
        lines.append('grouped per item: ' + str(reference.get('perItem')) + ' -> ' + str(current.get('perItem')))
    for key in sorted(set(reference['groups']) | set(current['groups'])):
        lines += GroupDifferences(key, reference['groups'].get(key), current['groups'].get(key), tolerance)
    return lines


def VariantDelta(default, variant):
    """what of the fingerprint variant differs from default, per group and kind of element: the
    group or the kind as it is in variant, or None where it is gone"""
    delta = {}
    for key in sorted(set(default['groups']) | set(variant['groups'])):
        (a, b) = (default['groups'].get(key), variant['groups'].get(key))
        if not GroupDifferences(key, a, b):
            continue
        if a is None or b is None:
            delta[key] = b
        else:
            delta[key] = {kind: b.get(kind) for kind in kinds
                          if GroupDifferences(key, {kind: a[kind]} if kind in a else {},
                                              {kind: b[kind]} if kind in b else {})}
    return delta


def ApplyDelta(default, delta):
    """the fingerprint of a variant, from the default and what VariantDelta gave"""
    groups = dict(default['groups'])
    for (key, change) in delta.items():
        if change is None:
            groups.pop(key, None)
        elif key not in groups:
            groups[key] = change
        else:
            group = dict(groups[key])
            for (kind, summary) in change.items():
                if summary is None:
                    group.pop(kind, None)
                else:
                    group[kind] = summary
            groups[key] = group
    return dict(default, groups=groups)


def ReferencePath(case):
    return os.path.join(referenceDirectory, case + '.json')


def _ReadOrRecord(case, content):
    """(reference, []) - or (None, [a note]) after writing content as the reference, if it is
    missing or EXUDYN_RECORD_GRAPHICS_REFERENCES is set"""
    path = ReferencePath(case)
    record = os.environ.get('EXUDYN_RECORD_GRAPHICS_REFERENCES', '') not in ['', '0']
    if record or not os.path.exists(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)   #a case may name a subfolder: 'miniExamples/...'
        with open(path, 'w', encoding='utf-8', newline='\n') as file:
            json.dump(content, file, indent=1, sort_keys=True)
            file.write('\n')
        return (None, ['reference written: ' + os.path.relpath(path) + ' - review it, and run again'])
    with open(path, encoding='utf-8') as file:
        return (json.load(file), [])


def CheckAgainstReference(case, fingerprint):
    """the differences to the stored reference; writes the reference if it is missing or if
    EXUDYN_RECORD_GRAPHICS_REFERENCES is set, and then reports that it did"""
    (reference, lines) = _ReadOrRecord(case, fingerprint)
    return lines if reference is None else Differences(reference, fingerprint)


def CheckVariantsAgainstReference(case, default, variants):
    """the same for one model drawn with different settings: default is its fingerprint with the
    default settings, variants is {name: fingerprint}; a line of a variant starts with its name"""
    content = {'formatVersion': 1, 'default': default,
               'variants': {name: VariantDelta(default, variant) for (name, variant) in variants.items()}}
    (reference, lines) = _ReadOrRecord(case, content)
    if reference is None:
        return lines
    lines = Differences(reference['default'], default)
    for name in sorted(set(reference['variants']) | set(variants)):
        if name not in variants or name not in reference['variants']:
            lines.append(name + ': ' + ('new' if name in variants else 'gone'))
            continue
        expected = ApplyDelta(reference['default'], reference['variants'][name])
        lines += [name + ' - ' + line for line in Differences(expected, variants[name])]
    return lines


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#LOW-RESOLUTION RAYTRACER IMAGES (#2704): what the graphics data cannot say - faces, face edges,
#transparency, lighting - is in the image of the software raytracer, which needs no window. The
#references are PNG files a human can open; written and read here with zlib alone, so that the test
#needs nothing beyond numpy. An image agrees if few pixels differ by more than a few levels:
#floating point differs between compilers, and a pixel on an edge may flip
pixelTolerance = 24              #levels of 255 a pixel may differ by without counting
pixelFractionTolerance = 0.01    #the fraction of pixels that may differ by more


def WritePNG(fileName, image):
    """an 8-bit RGB PNG of an array (height, width, 3) of uint8"""
    import struct                                                           # noqa: PLC0415
    import zlib                                                             # noqa: PLC0415
    image = np.ascontiguousarray(image, dtype=np.uint8)
    (height, width) = image.shape[:2]
    raw = b''.join(b'\x00' + image[row].tobytes() for row in range(height))   #filter type 0

    def Chunk(kind, data):
        return (struct.pack('>I', len(data)) + kind + data
                + struct.pack('>I', zlib.crc32(kind + data) & 0xffffffff))
    with open(fileName, 'wb') as file:
        file.write(b'\x89PNG\r\n\x1a\n'
                   + Chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0))
                   + Chunk(b'IDAT', zlib.compress(raw, 9)) + Chunk(b'IEND', b''))


def ReadPNG(fileName):
    """the image WritePNG wrote, as an array (height, width, 3) of uint8"""
    import struct                                                           # noqa: PLC0415
    import zlib                                                             # noqa: PLC0415
    with open(fileName, 'rb') as file:
        data = file.read()
    (position, header, compressed) = (8, None, b'')
    while position < len(data):
        (length,) = struct.unpack('>I', data[position:position + 4])
        kind = data[position + 4:position + 8]
        content = data[position + 8:position + 8 + length]
        if kind == b'IHDR':
            header = struct.unpack('>IIBBBBB', content)
        elif kind == b'IDAT':
            compressed += content
        position += 12 + length
    (width, height) = header[:2]
    rows = np.frombuffer(zlib.decompress(compressed), dtype=np.uint8).reshape(height, 1 + 3*width)
    assert (rows[:, 0] == 0).all(), fileName + ': only unfiltered rows, as WritePNG writes them'
    return rows[:, 1:].reshape(height, width, 3).copy()


def ImageDifferences(name, reference, image):
    """what differs between two images, as readable lines; [] if they agree"""
    if reference.shape != image.shape:
        return [name + ': image size ' + str(reference.shape) + ' -> ' + str(image.shape)]
    difference = np.abs(reference.astype(int) - image.astype(int)).max(axis=2)
    fraction = float((difference > pixelTolerance).mean())
    if fraction > pixelFractionTolerance:
        return [name + ': ' + str(round(100*fraction, 2)) + '% of the pixels differ by more than '
                + str(pixelTolerance) + ' levels (mean difference '
                + str(round(float(difference.mean()), 2)) + ')']
    return []


def CheckImageAgainstReference(case, image):
    """the differences of a raytracer image to its reference PNG; writes the reference if it is
    missing or EXUDYN_RECORD_GRAPHICS_REFERENCES is set, and then reports that it did"""
    path = os.path.join(referenceDirectory, case + '.png')
    record = os.environ.get('EXUDYN_RECORD_GRAPHICS_REFERENCES', '') not in ['', '0']
    if record or not os.path.exists(path):
        os.makedirs(referenceDirectory, exist_ok=True)
        WritePNG(path, image)
        return ['reference written: ' + os.path.relpath(path) + ' - look at it, and run again']
    return ImageDifferences(case, ReadPNG(path), image)
