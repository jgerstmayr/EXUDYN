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
#               radii and of the triangle normals, which must agree to a relative 1e-5;
#             - the texts themselves, which must agree exactly.
#           So a difference reads as "object 3: triangles 12 -> 10" or "the mean z of the triangles
#           of object 0 moved", and the reference is a JSON file a human can read in a diff.
#
#           The references are in python/testing/graphicsReferences/, one file per case. A case
#           whose reference is missing, or every case when EXUDYN_RECORD_GRAPHICS_REFERENCES=1 is
#           set, writes its reference and the test says so and fails, so that nothing is recorded
#           unnoticed.
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
    """min, max and mean over the first axis, per component, as plain lists"""
    values = np.asarray(values, dtype=float)
    values = values.reshape(values.shape[0], -1) if values.ndim > 1 else values.reshape(-1, 1)
    return {'min': values.min(axis=0).tolist(), 'max': values.max(axis=0).tolist(),
            'mean': values.mean(axis=0).tolist()}


def Fingerprint(data):
    """the fingerprint of the dictionary SC.renderer.GetGraphicsData() returns"""
    allItems = np.concatenate([data[kind]['items'] for kind in kinds if len(data[kind]['items'])]) \
        if any(len(data[kind]['items']) for kind in kinds) else np.zeros((0, 3), dtype=int)
    distinctItems = set(map(tuple, allItems.tolist()))
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
            if 'radius' in entry:
                summary['radius'] = Statistics(np.asarray(entry['radius'])[mask])
            if kind == 'triangles':
                summary['normals'] = Statistics(np.asarray(entry['normals'])[mask].reshape(-1, 3))
            if kind == 'texts':
                summary['text'] = sorted(text for (text, m) in zip(entry['text'], mask) if m)
            group[kind] = summary
    return {'formatVersion': 1, 'perItem': perItem, 'items': len(distinctItems), 'groups': groups}


def Differences(reference, current, tolerance=relativeTolerance):
    """what differs between two fingerprints, as readable lines; [] if they agree"""
    lines = []
    if reference.get('perItem') != current.get('perItem'):
        lines.append('grouped per item: ' + str(reference.get('perItem')) + ' -> ' + str(current.get('perItem')))
    for key in sorted(set(reference['groups']) | set(current['groups'])):
        (a, b) = (reference['groups'].get(key), current['groups'].get(key))
        if a is None or b is None:
            lines.append(key + ': ' + ('new' if a is None else 'gone'))
            continue
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
            for quantity in ['points', 'colors', 'radius', 'normals']:
                if quantity not in x:
                    continue
                for statistic in ['min', 'max', 'mean']:
                    for (component, (u, v)) in enumerate(zip(x[quantity][statistic], y[quantity][statistic])):
                        if abs(u - v) > tolerance * (1 + max(abs(u), abs(v))):
                            lines.append(key + ': ' + kind + ' ' + quantity + ' ' + statistic + '['
                                         + str(component) + '] ' + repr(u) + ' -> ' + repr(v))
    return lines


def ReferencePath(case):
    return os.path.join(referenceDirectory, case + '.json')


def CheckAgainstReference(case, fingerprint):
    """the differences to the stored reference; writes the reference if it is missing or if
    EXUDYN_RECORD_GRAPHICS_REFERENCES is set, and then reports that it did"""
    path = ReferencePath(case)
    record = os.environ.get('EXUDYN_RECORD_GRAPHICS_REFERENCES', '') not in ['', '0']
    if record or not os.path.exists(path):
        os.makedirs(referenceDirectory, exist_ok=True)
        with open(path, 'w', encoding='utf-8', newline='\n') as file:
            json.dump(fingerprint, file, indent=1, sort_keys=True)
            file.write('\n')
        return ['reference written: ' + os.path.relpath(path) + ' - review it, and run again']
    with open(path, encoding='utf-8') as file:
        reference = json.load(file)
    return Differences(reference, fingerprint)
