#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The node types a node marker needs are DECLARED in its definition, requestedNodeTypes,
#           and the pages of the reference manual say from it which nodes and markers fit (#2725).
#           The check itself is C++ - CSystem::CheckSystemIntegrity for position and orientation,
#           MainMarkerNodeRotationCoordinate::CheckPreAssembleConsistency for the rotation
#           coordinate. This test keeps the two in agreement (#2727): every node marker is attached to
#           every node, and what Assemble() accepts must be what the declaration says.
#
# Usage:    pytest python/testing/test_itemCompatibility.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-28
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys

import pytest

import exudyn as exu

repositoryRoot = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(repositoryRoot, 'tools', 'generators'))
sys.path.insert(0, os.path.join(repositoryRoot, 'definitions'))
import itemCompatibility                                                    # noqa: E402

items = itemCompatibility.LoadItems()
nodes = [item for item in items if item.kind == 'Node']
nodeMarkers = [item for item in items if item.kind == 'Marker' and item.requestedNodeTypes]

#what a node needs to be added at all; the rest takes the defaults
nodeArguments = {
    'NodeGenericODE2': {'numberOfODE2Coordinates': 3, 'referenceCoordinates': [0, 0, 0],
                        'initialCoordinates': [0, 0, 0], 'initialCoordinates_t': [0, 0, 0]},
    'NodeGenericODE1': {'numberOfODE1Coordinates': 3, 'referenceCoordinates': [0, 0, 0],
                        'initialCoordinates': [0, 0, 0]},
    'NodeGenericAE': {'numberOfAECoordinates': 3, 'referenceCoordinates': [0, 0, 0],
                      'initialCoordinates': [0, 0, 0]},
    'NodeGenericData': {'numberOfDataCoordinates': 3, 'initialCoordinates': [0, 0, 0]},
    'NodeRigidBodyEP': {'referenceCoordinates': [0, 0, 0, 1, 0, 0, 0]},
    }
markerArguments = {'MarkerNodeRotationCoordinate': {'rotationCoordinate': 0}}


def test_everyMarkerWithAPositionOrOrientationOnANodeDeclaresItsNodeTypes():
    """a new node marker that measures a position or an orientation must declare which nodes it
    needs, or its page and the pages of the nodes are silent about it"""
    undeclared = [item.name for item in items if item.kind == 'Marker' and 'Node' in item.provided
                  and ('Position' in item.provided or 'Orientation' in item.provided)
                  and 'Body' not in item.provided and not item.requestedNodeTypes]
    assert undeclared == []
    assert len(nodeMarkers) >= 3


@pytest.mark.parametrize('marker', nodeMarkers, ids=[m.name for m in nodeMarkers])
def test_theDeclaredNodeTypesAreWhatAssembleAccepts(marker):
    disagree = []
    for node in nodes:
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        nodeNumber = mbs.AddNode(dict({'nodeType': node.name[len('Node'):]}, **nodeArguments.get(node.name, {})))
        accepted = True
        try:
            mbs.AddMarker(dict({'markerType': marker.name[len('Marker'):], 'nodeNumber': nodeNumber},
                               **markerArguments.get(marker.name, {})))
            mbs.Assemble()
        except Exception:                                                   # noqa: BLE001
            accepted = False
        if accepted != marker.AcceptsNode(node):
            disagree.append(node.name + ': Assemble ' + ('accepts' if accepted else 'refuses')
                            + ', the declaration ' + ('accepts' if marker.AcceptsNode(node) else 'refuses'))
    assert disagree == [], marker.name + '\n' + '\n'.join(disagree)
