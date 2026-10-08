#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The node types a node marker needs are DECLARED in its definition, requestedNodeTypes,
#           and the pages of the reference manual say from it which nodes and markers fit (#2725).
#           The declaration also generates the check of CSystem::CheckSystemIntegrity and the answer
#           of mbs.Inspect (#2817). This test keeps the documentation and the C++ in agreement (#2727):
#           every node marker is attached to every node, what Assemble() accepts must be what the
#           declaration says, and mbs.Inspect answers the declaration.
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


@pytest.mark.parametrize('marker', nodeMarkers, ids=[m.name for m in nodeMarkers])
def test_inspectAnswersTheDeclaredNodeTypes(marker):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    nodeNumber = mbs.AddNode(exu.itemInterface.NodeRigidBodyEP(referenceCoordinates=[0, 0, 0, 1, 0, 0, 0]))
    markerNumber = mbs.AddMarker(dict({'markerType': marker.name[len('Marker'):], 'nodeNumber': nodeNumber},
                                      **markerArguments.get(marker.name, {})))
    answer = mbs.Inspect(markerNumber, exu.InspectType.RequestedNodeTypes)
    names = [[[nodeType.name for nodeType in alternatives] for alternatives in perNode] for perNode in answer]
    assert names == [marker.requestedNodeTypes]


def _SuperElementAndTree(mbs):
    """an ObjectGenericODE2 on two points and an ObjectKinematicTree with one link"""
    nodes = [mbs.AddNode(exu.itemInterface.NodePoint(referenceCoordinates=[i, 0, 0])) for i in range(2)]
    oSuper = mbs.AddObject(exu.itemInterface.ObjectGenericODE2(nodeNumbers=nodes, massMatrix=[[float(i == j) for j in range(6)] for i in range(6)]))
    nTree = mbs.AddNode(exu.itemInterface.NodeGenericODE2(numberOfODE2Coordinates=1, referenceCoordinates=[0],
                                                          initialCoordinates=[0], initialCoordinates_t=[0]))
    oTree = mbs.AddObject(exu.itemInterface.ObjectKinematicTree(nodeNumber=nTree, jointTypes=[exu.JointType.RevoluteZ],
                          linkParents=[-1], jointTransformations=exu.Matrix3DList([[[1, 0, 0], [0, 1, 0], [0, 0, 1]]]),
                          jointOffsets=exu.Vector3DList([[0, 0, 0]]),
                          linkInertiasCOM=exu.Matrix3DList([[[1, 0, 0], [0, 1, 0], [0, 0, 1]]]),
                          linkCOMs=exu.Vector3DList([[0, 0, 0]]), linkMasses=[1.]))
    return {'ObjectGenericODE2': oSuper, 'ObjectKinematicTree': oTree}


#the markers placed on bodies, with what they need besides the body
bodyMarkerArguments = {
    'MarkerBodyPosition': lambda o: {'bodyNumber': o},
    'MarkerBodyRigid': lambda o: {'bodyNumber': o},
    'MarkerBodyMass': lambda o: {'bodyNumber': o},
    'MarkerSuperElementPosition': lambda o: {'bodyNumber': o, 'meshNodeNumbers': [0], 'weightingFactors': [1]},
    'MarkerKinematicTreeRigid': lambda o: {'objectNumber': o, 'linkNumber': 0},
    }


@pytest.mark.parametrize('objectName', ['ObjectGenericODE2', 'ObjectKinematicTree'])
def test_theBodyMarkersAssembleAcceptsAreTheDeclaredOnes(objectName):
    """ObjectGenericODE2 and ObjectKinematicTree serve only their own markers; Assemble() refuses the
    general body markers, which their access functions do not provide, and the marker of the other
    kind (#2734) - as the declarations, and so the pages, say"""
    byName = dict((item.name, item) for item in items)
    disagree = []
    for (markerName, arguments) in bodyMarkerArguments.items():
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        objectNumber = _SuperElementAndTree(mbs)[objectName]
        accepted = True
        try:
            mbs.AddMarker(dict({'markerType': markerName[len('Marker'):]}, **arguments(objectNumber)))
            mbs.Assemble()
        except Exception:                                                   # noqa: BLE001
            accepted = False
        declared = byName[objectName].CarriesMarker(byName[markerName])
        if accepted != declared:
            disagree.append(markerName + ': Assemble ' + ('accepts' if accepted else 'refuses')
                            + ', the declaration ' + ('accepts' if declared else 'refuses'))
    assert disagree == [], objectName + '\n' + '\n'.join(disagree)


def _Assembles(build):
    """True if the model build(mbs, bodies) adds passes Assemble()"""
    ii = exu.itemInterface
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    nRigid = mbs.AddNode(ii.NodeRigidBody2D(referenceCoordinates=[0, 0, 0]))
    nPoint = mbs.AddNode(ii.NodePoint(referenceCoordinates=[0, 0, 0]))
    from exudyn.beams import GenerateBeamElementsAlongLine
    bodies = {'ObjectRigidBody2D': mbs.AddObject(ii.ObjectRigidBody2D(nodeNumber=nRigid, mass=1, inertia=1)),
              'ObjectMassPoint': mbs.AddObject(ii.ObjectMassPoint(nodeNumber=nPoint, mass=1)),
              'ObjectANCFCable2D': GenerateBeamElementsAlongLine(mbs, [0, 0, 0], [1, 0, 0], 1,
                  ii.ObjectANCFCable2D(massPerLength=1, bendingStiffness=1, axialStiffness=1))['elements'][0],
              'ObjectANCFCable': GenerateBeamElementsAlongLine(mbs, [0, 0, 0], [1, 0, 0], 1,
                  ii.ObjectANCFCable(massPerLength=1, bendingStiffness=1, axialStiffness=1))['elements'][0],
              'ObjectGround': mbs.AddObject(ii.ObjectGround())}
    try:
        build(mbs, bodies)
        mbs.Assemble()
    except Exception:                                                       # noqa: BLE001
        return False
    return True


@pytest.mark.parametrize('markerName, bodies', [('MarkerBodyCable2DShape', ['ObjectANCFCable2D']),
                                                ('MarkerBodyCable2DCoordinates', ['ObjectANCFCable2D']),
                                                ('MarkerBodyBeamShape', ['ObjectANCFCable'])])
def test_theShapeMarkersTakeTheirCablesOnly(markerName, bodies):
    """a shape marker computes with the shape functions of its cable element; on another body Assemble()
    refuses it (#2731) - it cast the body to a cable before"""
    for body in ['ObjectRigidBody2D', 'ObjectANCFCable2D', 'ObjectANCFCable']:
        accepted = _Assembles(lambda mbs, b: mbs.AddMarker({'markerType': markerName[len('Marker'):], 'bodyNumber': b[body]}))
        assert accepted == (body in bodies), markerName + ' on ' + body


def test_theRelativeMarkersAreCoordinateMarkersOnRigidBodies():
    """the relative coordinate markers feed coordinate connectors; a position connector refuses them, and
    they refuse a body without orientation where they need one (#2731)"""
    ii = exu.itemInterface
    translation = lambda b0, b1: {'markerType': 'BodiesRelativeTranslationCoordinate', 'bodyNumbers': [b0, b1]}
    def WithConnector(connector, b0='ObjectGround', b1='ObjectRigidBody2D'):
        def Build(mbs, b):
            m = mbs.AddMarker(translation(b[b0], b[b1]))
            mGround = mbs.AddMarker(ii.MarkerBodyPosition(bodyNumber=b['ObjectGround']))
            mbs.AddObject(connector(mGround, m))
        return Build
    assert not _Assembles(WithConnector(lambda m0, m1: ii.ObjectConnectorSpringDamper(markerNumbers=[m0, m1], stiffness=1)))
    #body 0 needs an orientation: a mass point as body 0 is refused, as body 1 accepted
    assert not _Assembles(lambda mbs, b: mbs.AddMarker(translation(b['ObjectMassPoint'], b['ObjectRigidBody2D'])))
    assert _Assembles(lambda mbs, b: mbs.AddMarker(translation(b['ObjectRigidBody2D'], b['ObjectMassPoint'])))
