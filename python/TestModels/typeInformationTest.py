#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test of exudyn.types: the generated type information agrees
#           with the C++ module (enum values, item parameters), and the query functions agree with
#           mbs.Assemble() for marker/object and connector/marker combinations that are built here.
#           The result is the number of disagreements (0).
#
# Author:   Johannes Gerstmayr, Claude-JG
# Date:     2026-09-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
import exudyn.itemInterface as itemInterface
import exudyn.types as types
import contextlib
import inspect
import io

testIsActive = exu.sys.get('testIsActive', False)

errors = 0
def Check(condition, text):
    global errors
    if not condition:
        errors += 1
        exu.Print('typeInformationTest: ' + text)

#++++++++++++++++++++++++++++++++++++++++++++++++++
#data agrees with the C++ module and itemInterface.py
enumNames = {'Node': set(exu.NodeType.__members__), 'Marker': set(exu.MarkerType.__members__)}
for name in types.ItemNames():
    info = types.ItemInfo(name)
    if info['kind'] in enumNames:
        Check(set(info['types']) <= enumNames[info['kind']], name + ': unknown type bits ' + str(info['types']))
    for key in ('requestedNodeTypes', 'requestedMarkerTypes'):
        kind = 'Node' if key == 'requestedNodeTypes' else 'Marker'
        Check(set(info.get(key, [])) <= enumNames[kind], name + ': unknown ' + key)
    Check(set(info.get('accessFunctionTypes', [])) <= set(exu.AccessFunctionType.__members__),
          name + ': unknown access function types')
    signature = inspect.signature(getattr(itemInterface, name)).parameters
    #the class also takes the old names of renamed parameters, as its last keywords (#2589)
    Check(set(types.Parameters(name)) == set(signature) - {'visualization'} - set(info.get('deprecatedParameters', {})),
          name + ': parameters differ from itemInterface.py')
    Check(set(info['visualization']) == set(signature['visualization'].default),
          name + ': visualization parameters differ from itemInterface.py')

#++++++++++++++++++++++++++++++++++++++++++++++++++
#queries agree with mbs.Assemble(): markers on bodies, connectors on markers
def AssembleWorks(Build):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    printToConsole = exu.config.printToConsole
    exu.config.printToConsole = False
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            Build(mbs)
            mbs.Assemble()
        return True
    except Exception:
        return False
    finally:
        exu.config.printToConsole = printToConsole

def AddBody(mbs, objectName):
    if objectName == 'ObjectMassPoint':
        n = mbs.AddNode(itemInterface.NodePoint())
        return mbs.AddObject(itemInterface.ObjectMassPoint(mass=1, nodeNumber=n))
    if objectName == 'ObjectRigidBody':
        n = mbs.AddNode(itemInterface.NodeRigidBodyEP(referenceCoordinates=[0,0,0, 1,0,0,0]))
        return mbs.AddObject(itemInterface.ObjectRigidBody(mass=1, inertia=[1,1,1,0,0,0], nodeNumber=n))
    return mbs.AddObject(itemInterface.ObjectGround())

markerBuilders = {'MarkerBodyPosition': lambda b: itemInterface.MarkerBodyPosition(bodyNumber=b),
                  'MarkerBodyRigid': lambda b: itemInterface.MarkerBodyRigid(bodyNumber=b),
                  'MarkerBodyMass': lambda b: itemInterface.MarkerBodyMass(bodyNumber=b)}

for objectName in ['ObjectMassPoint', 'ObjectRigidBody', 'ObjectGround']:
    for markerName, MarkerBuilder in markerBuilders.items():
        expected = markerName in types.MarkersForObject(objectName)
        works = AssembleWorks(lambda mbs: mbs.AddMarker(MarkerBuilder(AddBody(mbs, objectName))))
        Check(expected == works, markerName + ' on ' + objectName + ': query ' + str(expected) + ', Assemble ' + str(works))
        Check((objectName in types.ObjectsForMarker(markerName)) == expected, markerName + ': ObjectsForMarker inconsistent')

connectorBuilders = {'ObjectConnectorSpringDamper': lambda m: itemInterface.ObjectConnectorSpringDamper(markerNumbers=m, stiffness=1),
                     'ObjectConnectorRigidBodySpringDamper': lambda m: itemInterface.ObjectConnectorRigidBodySpringDamper(markerNumbers=m),
                     'ObjectJointSpherical': lambda m: itemInterface.ObjectJointSpherical(markerNumbers=m)}
for markerName in ['MarkerBodyPosition', 'MarkerBodyRigid']:
    for connectorName, ConnectorBuilder in connectorBuilders.items():
        expected = connectorName in types.ConnectorsForMarkers(markerName)
        def Build(mbs):
            m0 = mbs.AddMarker(markerBuilders[markerName](AddBody(mbs, 'ObjectGround')))
            m1 = mbs.AddMarker(markerBuilders[markerName](AddBody(mbs, 'ObjectRigidBody')))
            mbs.AddObject(ConnectorBuilder([m0, m1]))
        works = AssembleWorks(Build)
        Check(expected == works, connectorName + ' on two ' + markerName + ': query ' + str(expected) + ', Assemble ' + str(works))

Check('NodeRigidBodyEP' in types.NodesForObject('ObjectRigidBody') and
      'NodePoint' not in types.NodesForObject('ObjectRigidBody'), 'NodesForObject(ObjectRigidBody)')
Check('LoadForceVector' in types.LoadsForMarker('MarkerBodyPosition') and
      'LoadTorqueVector' not in types.LoadsForMarker('MarkerBodyPosition'), 'LoadsForMarker(MarkerBodyPosition)')

exu.Print('typeInformationTest: ' + str(len(types.ItemNames())) + ' items, ' + str(errors) + ' disagreements')
exu.sys['testResult'] = errors
