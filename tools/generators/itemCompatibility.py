#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# itemCompatibility - which items fit together, said in words on the page of each item (#2725)
#
# Every item declares types in its definition: what a node or a marker provides (ItemTypes), what
# an object, a connector or a load requests of its nodes or markers (GetRequestedNodeType,
# GetRequestedMarkerType), which access functions a body offers (ItemAccessFunctionTypes), and -
# for the node markers - which node types they need (requestedNodeTypes). Those declarations ARE
# the compatibility rules of a model, checked by CSystem::CheckSystemIntegrity. This module turns
# them into the lines of the Interface block of an item page: "Node markers: MarkerNodePosition,
# ...", "Used by: ...", instead of a type bit that the reader has to match by hand.
#
# Only what is declared is said. A marker whose object is named in its own description (the cable
# and beam shape markers) and a connector that requests no marker type are not listed.
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import definitionLoader

#the access function a body must offer for a body marker of a type - the rule of
#CSystem::CheckSystemIntegrity (src/Main/CSystem.cpp) and of the markers' own checks
bodyMarkerAccess = {'Position': 'TranslationalVelocity_qt', 'Orientation': 'AngularVelocity_qt',
                    'BodyMass': 'DisplacementMassIntegral_q', 'SuperElement': 'SuperElement',
                    'KinematicTree': 'KinematicTree'}
#the marker types a user chooses by; the others are flags of the implementation
markerTypesShown = ['Position', 'Orientation', 'Coordinate', 'Coordinates', 'BodyMass', 'Beam3DShape']


def _Member(definition, pythonName):
    for member in definition['members']:
        if member.get('pythonName') == pythonName:
            return member
    return None


def _List(member, key):
    return list(member.get(key) or []) if member is not None else []


class Item:
    """the declared types of one item"""

    def __init__(self, definition):
        self.name = definition['className']
        self.kind = definition.get('classType', '')
        self.objectType = definition.get('objectType') or ''
        self.provided = _List(_Member(definition, 'GetType'), 'itemTypes')
        requestedMarker = _Member(definition, 'GetRequestedMarkerType')
        self.requestedMarker = _List(requestedMarker, 'requestedTypes')
        self.conditionalMarker = _List(requestedMarker, 'conditionalTypes')
        self.requestedNode = _List(_Member(definition, 'GetRequestedNodeType'), 'requestedTypes')
        accessMember = _Member(definition, 'GetAccessFunctionTypes')
        self.access = _List(accessMember, 'accessFunctionTypes')
        #False where the types serve the object's own markers only (#2734)
        self.bodyMarkers = True if accessMember is None else accessMember.get('bodyMarkers', True)
        #node markers: a list of requirements, each a list of alternatives
        self.requestedNodeTypes = [list(alternatives) for alternatives in
                                   (definition.get('requestedNodeTypes') or [])]
        objects = _Member(definition, 'GetNumberOfObjects')
        self.numberOfObjects = 2 if objects is not None and 'return 2' in (objects.get('implementation') or '') else 1

    def Link(self):
        return '[](#sec-item-' + self.name.lower() + ')'

    def IsBodyMarker(self):
        return self.kind == 'Marker' and 'Body' in self.provided

    def BodyMarkerNeeds(self):
        """the access functions this body marker needs, or None if it is not a general body marker"""
        if not self.IsBodyMarker() or 'Coordinate' in self.provided or 'Coordinates' in self.provided \
                or 'Beam3DShape' in self.provided:
            return None
        needs = [bodyMarkerAccess[t] for t in self.provided if t in bodyMarkerAccess]
        return needs if needs else None

    def AcceptsNode(self, node):
        """this node marker can be attached to node"""
        return (self.kind == 'Marker' and self.requestedNodeTypes != []
                and all(any(t in node.provided for t in alternatives)
                        for alternatives in self.requestedNodeTypes))

    def RequestsNode(self, node):
        """this object takes node as its node"""
        return (self.kind == 'Object' and self.requestedNode != []
                and all(t in node.provided for t in self.requestedNode))

    def AcceptsMarker(self, marker):
        """this connector, joint or load can use marker"""
        return (self.requestedMarker != [] and marker.kind == 'Marker'
                and all(t in marker.provided for t in self.requestedMarker))

    def CarriesMarker(self, marker):
        """marker can be placed on this body"""
        needs = marker.BodyMarkerNeeds()
        ownMarker = 'SuperElement' in marker.provided or 'KinematicTree' in marker.provided
        return (self.kind == 'Object' and needs is not None and self.access != []
                and all(n in self.access for n in needs) and (self.bodyMarkers or ownMarker))


def LoadItems():
    items = []
    for moduleName in definitionLoader.itemModules:
        items += [Item(d) for d in __import__(moduleName).definitions]
    return items


def _Names(items):
    return ', '.join(item.Link() for item in items)


def InterfaceLines(item, items):
    """the lines that say in words what fits to item"""
    lines = []
    if item.kind == 'Node':
        lines.append('Provides: ' + ', '.join('`' + t + '`' for t in item.provided))
        markers = [m for m in items if m.AcceptsNode(item)]
        if markers:
            lines.append('Node markers that can be attached: ' + _Names(markers))
        objects = [o for o in items if o.RequestsNode(item)]
        if objects:
            lines.append('Objects that take this node: ' + _Names(objects))
    elif item.kind == 'Object':
        if item.requestedNode:
            nodes = [n for n in items if n.kind == 'Node' and item.RequestsNode(n)]
            lines.append('Nodes it takes: ' + (_Names(nodes) if nodes else
                                               'nodes providing ' + ', '.join(item.requestedNode)))
        markers = [m for m in items if m.kind == 'Marker' and item.CarriesMarker(m)]
        if markers:
            lines.append('Body markers that can be placed on it: ' + _Names(markers))
    if item.requestedMarker:
        requested = ' and '.join('`' + t + '`' for t in item.requestedMarker)
        condition = ''.join(' (and `' + t + '` if `' + parameter + '` is not zero)'
                            for (t, parameter) in item.conditionalMarker)
        markers = [m for m in items if item.AcceptsMarker(m)]
        lines.append('Markers it acts on: those providing ' + requested + condition
                     + (': ' + _Names(markers) if markers else ''))
    if item.kind == 'Marker':
        shown = [t for t in item.provided if t in markerTypesShown]
        if shown:
            lines.append('Provides: ' + ', '.join('`' + t + '`' for t in shown))
        if item.requestedNodeTypes:
            nodes = [n for n in items if n.kind == 'Node' and item.AcceptsNode(n)]
            lines.append('Nodes it can be attached to: ' + _Names(nodes))
        if item.BodyMarkerNeeds() is not None:
            bodies = [o for o in items if o.CarriesMarker(item)]
            lines.append('Bodies it can be placed on: ' + _Names(bodies))
        users = [u for u in items if u.AcceptsMarker(item)]
        if users:
            lines.append('Connectors, constraints and loads that can use it: ' + _Names(users))
    return lines


def AttachedTo(marker):
    """what a marker sits on, in words"""
    if 'KinematicTree' in marker.provided:
        return 'a link of a kinematic tree'
    if 'SuperElement' in marker.provided:
        return 'mesh nodes of a super element'
    if 'Body' in marker.provided:
        return 'two bodies' if marker.numberOfObjects == 2 else 'a body'
    if marker.requestedNodeTypes:
        return 'a node with ' + ' and '.join(' or '.join('`' + t + '`' for t in alternatives)
                                             for alternatives in marker.requestedNodeTypes)
    return 'a node'


def MarkerTable(items):
    """the table of all markers for the page of the markers: what each sits on, what it provides and
    how many connectors, constraints and loads can use it - generated, like the Interface block"""
    from autoGenerateHelper import PdfColumnWidths
    lines = [PdfColumnWidths([0.3, 0.3, 0.2, 0.2]).rstrip('\n'),
             '| marker | attached to | provides | usable by |', '|---|---|---|---|']
    for marker in [item for item in items if item.kind == 'Marker']:
        shown = [t for t in marker.provided if t in markerTypesShown]
        users = [u for u in items if u.AcceptsMarker(marker)]
        lines.append('| ' + marker.Link() + ' | ' + AttachedTo(marker) + ' | '
                     + (', '.join('`' + t + '`' for t in shown) or '-') + ' | '
                     + (str(len(users)) + (' items' if len(users) > 1 else ' item') if users else 'the items that name it') + ' |')
    return '\n'.join(lines) + '\n'
