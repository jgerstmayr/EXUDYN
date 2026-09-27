#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Type information of the items and queries on it: which nodes an object accepts, which
#           markers can be attached to an object, which connectors and loads accept a marker. The
#           data (exudyn.types.items) is generated from definitions/; the rules here are the ones
#           mbs.Assemble() checks in C++ (CSystem::CheckSystemIntegrity), so a query is a pre-check:
#           the checks at assembly and in CheckPreAssembleConsistency stay authoritative.
#           Nothing here is imported by exudyn.utilities.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-15 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from exudyn.types.items import items

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'ItemNames', 'ItemInfo', 'Parameters', 'NodesForObject', 'MarkersForObject', 'ObjectsForMarker',
    'ConnectorsForMarkers', 'LoadsForMarker',
    ]

#marker bits that need an access function of the object, and the access function (C++ CheckSystemIntegrity
#for Position and Orientation; super element and kinematic tree markers need their access function)
_markerAccessFunctions = {'Position': 'TranslationalVelocity_qt', 'Orientation': 'AngularVelocity_qt',
                          'SuperElement': 'SuperElement', 'KinematicTree': 'KinematicTree'}


#markers whose C++ code casts the object to one specific class; not expressed by type bits
_markerObjects = {'MarkerBodyCable2DShape': ['ObjectANCFCable2D', 'ObjectALEANCFCable2D'],
                  'MarkerBodyCable2DCoordinates': ['ObjectANCFCable2D', 'ObjectALEANCFCable2D'],
                  'MarkerBodyBeamShape': ['ObjectANCFCable']}


def _Item(name):
    if name not in items:
        raise ValueError('exudyn.types: unknown item ' + repr(name) + '; use the class name, e.g. ObjectMassPoint')
    return items[name]


def _Contains(types, requested):
    return set(requested) <= set(types)


def ItemNames(kind=None):
    """Names of the items with a Python interface.

    Args:
        kind: 'Node', 'Object', 'Marker', 'Load' or 'Sensor'; None for all

    Returns:
        list of class names, e.g. ['ObjectMassPoint', ...]
    """
    return [name for name, data in items.items() if kind is None or data['kind'] == kind]


def ItemInfo(name):
    """Type information of one item: kind, type bits, requested node and marker types, access
    function types, output variables, parameters and visualization parameters.

    Args:
        name: class name, e.g. 'ObjectMassPoint'

    Returns:
        dict (the generated data; do not modify it)
    """
    return _Item(name)


def Parameters(name):
    """Parameters of an item with type, size, range, default (Python source text), mustBeGiven and
    description.

    Args:
        name: class name, e.g. 'ObjectMassPoint'

    Returns:
        dict parameter name -> dict
    """
    return _Item(name)['parameters']


def NodesForObject(objectName):
    """Nodes whose type contains the node type the object requests.

    Args:
        objectName: object class name, e.g. 'ObjectRigidBody'

    Returns:
        list of node class names; all nodes if the object requests no specific type
    """
    requested = _Item(objectName).get('requestedNodeTypes', [])
    return [name for name in ItemNames('Node') if _Contains(items[name]['types'], requested)]


def _MarkerFitsObject(markerName, objectName):
    markerData, objectData = items[markerName], items[objectName]
    if 'Body' not in markerData['types'] or 'Body' not in objectData['types']:
        return False
    if markerName in _markerObjects and objectName not in _markerObjects[markerName]:
        return False
    accessFunctions = objectData.get('accessFunctionTypes', [])
    return all(accessFunction in accessFunctions for bit, accessFunction in _markerAccessFunctions.items()
               if bit in markerData['types'])


def MarkersForObject(objectName):
    """Body markers that can be attached to the object: the object is a body and provides the access
    functions of the marker (position, orientation, super element, kinematic tree).

    Args:
        objectName: object class name, e.g. 'ObjectRigidBody'

    Returns:
        list of marker class names
    """
    _Item(objectName)
    return [name for name in ItemNames('Marker') if _MarkerFitsObject(name, objectName)]


def ObjectsForMarker(markerName):
    """Objects a body marker can be attached to (the inverse of MarkersForObject).

    Args:
        markerName: marker class name, e.g. 'MarkerBodyRigid'

    Returns:
        list of object class names; empty for node markers
    """
    _Item(markerName)
    return [name for name in ItemNames('Object') if _MarkerFitsObject(markerName, name)]


def ConnectorsForMarkers(markerName0, markerName1=None):
    """Objects that request marker types contained in the types of both markers (connectors,
    constraints, contacts). Objects that request no specific type (different types for their two
    markers) are not listed; types added under a condition (e.g. Orientation if dynamicFriction != 0)
    are not required.

    Args:
        markerName0: marker class name, e.g. 'MarkerBodyPosition'
        markerName1: second marker class name; None: the same as markerName0

    Returns:
        list of object class names
    """
    types0 = _Item(markerName0)['types']
    types1 = _Item(markerName1 if markerName1 is not None else markerName0)['types']
    result = []
    for name in ItemNames('Object'):
        requested = items[name].get('requestedMarkerTypes', [])
        if requested and _Contains(types0, requested) and _Contains(types1, requested):
            result.append(name)
    return result


def LoadsForMarker(markerName):
    """Loads whose requested marker type is contained in the marker's type.

    Args:
        markerName: marker class name, e.g. 'MarkerBodyMass'

    Returns:
        list of load class names
    """
    types = _Item(markerName)['types']
    return [name for name in ItemNames('Load') if _Contains(types, items[name].get('requestedMarkerTypes', []))]
