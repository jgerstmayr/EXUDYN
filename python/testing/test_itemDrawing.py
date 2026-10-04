#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The section Drawing of the item pages against the C++ (#2840): every visualization setting the drawing
#           of an item reads (its UpdateGraphics, CallUserFunction and the helpers they call, as
#           tools/itemDrawingReport.py reads them) is named by the kind of the item (itemKindDefinitions.py,
#           drawingSettings) or by the item itself (drawingSettings of its definition), and an item names no setting
#           its drawing does not read; an item without drawing code says that it draws nothing.
#
# Usage:    pytest python/testing/test_itemDrawing.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-04
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import importlib.util
import os
import sys

import pytest

root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
spec = importlib.util.spec_from_file_location('itemDrawingReport', os.path.join(root, 'tools', 'itemDrawingReport.py'))
report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(report)       #puts definitions/ and tools/generators/ on the path
import definitionLoader                                                     # noqa: E402

kinds = dict((entry['kind'], entry) for entry in __import__('itemKindDefinitions').definitions)
functions = report.DrawingFunctions()
items = [definition for moduleName in definitionLoader.itemModules for definition in __import__(moduleName).definitions]


def Kind(definition):
    classType = definition.get('classType', '')
    return 'Objects (' + definition.get('objectType', '') + ')' if classType == 'Object' else classType + 's'


def ItemName(definition):
    className = definition['className']
    return className if className.startswith(definition.get('classType', '')) else definition['classType'] + className


@pytest.mark.parametrize('definition', items, ids=lambda definition: definition['className'])
def testTheDrawingSettingsAreThoseOfTheCode(definition):
    function = functions.get(ItemName(definition))
    declared = set(definition.get('drawingSettings') or [])
    if function is None:
        assert definition.get('drawing') == 'The item draws nothing.' and not declared
        return
    read = report.Settings(function[2])
    common = set(kinds[Kind(definition)]['drawingSettings'])
    assert read - common - declared == set(), 'read by the drawing, but named neither by the kind nor by the item'
    assert declared - read == set(), 'named by the item, but not read by its drawing'


def testEveryKindHasItsDrawing():
    for kind in kinds.values():
        assert kind['drawing'].strip() != '', kind['kind']
