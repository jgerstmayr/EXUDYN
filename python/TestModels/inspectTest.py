#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  mbs.Inspect(itemIndex, what) asks an item what it provides and requests (#2203): what is a
#           member of exu.InspectType, the answer a list of the enumeration members Exudyn exports
#           (OutputVariableType, ObjectType, NodeType, MarkerType, AccessFunctionType); with what=None
#           the answer is a dict of all that apply to the item. Shown on a mass point on a spring-damper,
#           a rigid body, a load and a sensor:
#           (1) the output variables of a body, a connector, a node and a marker - the potential energy of
#               a spring-damper only as long as no user function defines its force;
#           (2) the types of an object, a node and a marker;
#           (3) what a connector and a load request of their markers, what an object requests of its
#               nodes, and the access functions of a body, which decide the markers it takes;
#           (4) what Inspect refuses: a plain int instead of a typed index, and a what that does not
#               apply to the item.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()
I = exu.InspectType

oGround = mbs.AddObject(ObjectGround())
nMass = mbs.AddNode(NodePoint(referenceCoordinates=[1, 0, 0]))
oMass = mbs.AddObject(MassPoint(nodeNumber=nMass, mass=2))
mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround))
mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
oSpring = mbs.AddObject(SpringDamper(markerNumbers=[mGround, mMass], referenceLength=1, stiffness=100))
lForce = mbs.AddLoad(LoadForceVector(markerNumber=mMass, loadVector=[1, 0, 0]))
sSpring = mbs.AddSensor(SensorObject(objectNumber=oSpring, outputVariableType=exu.OutputVariableType.Force, storeInternal=True))
oRigid = mbs.CreateRigidBody(referencePosition=[0, 1, 0], inertia=InertiaCuboid(density=1000, sideLengths=[0.2, 0.1, 0.1]))
mbs.Assemble()

def Names(answer):
    """the names of the members, for printing"""
    return [member.name for member in answer]

#(1) output variables; the spring-damper lists PotentialEnergy, and no longer does with a force user function
outputVariables = {name: mbs.Inspect(item, I.OutputVariables) for (name, item) in
                   [('mass point', oMass), ('spring-damper', oSpring), ('node', nMass), ('marker', mMass)]}
for (name, answer) in outputVariables.items():
    exu.Print('output variables of the', name + ':', Names(answer))
mbs.SetObjectParameter(oSpring, 'springForceUserFunction', lambda mbs, t, itemNumber, u, v, k, d, f: k*u)
withUserFunction = mbs.Inspect(oSpring, I.OutputVariables)
exu.Print('with a springForceUserFunction, PotentialEnergy is listed:', exu.OutputVariableType.PotentialEnergy in withUserFunction)
mbs.SetObjectParameter(oSpring, 'springForceUserFunction', 0)

#(2) the types, as single flags
exu.Print('object types of the rigid body:', Names(mbs.Inspect(oRigid, I.ObjectType)))
exu.Print('node types of the mass point node:', Names(mbs.Inspect(nMass, I.NodeType)))
exu.Print('marker types of the node marker:', Names(mbs.Inspect(mMass, I.MarkerType)))

#(3) what is requested, per marker or per node, and the access functions; and everything of one item
exu.Print('the spring-damper requests per marker:', [Names(perMarker) for perMarker in mbs.Inspect(oSpring, I.RequestedMarkerTypes)])
exu.Print('the load requests per marker:', [Names(perMarker) for perMarker in mbs.Inspect(lForce, I.RequestedMarkerTypes)])
exu.Print('the mass point requests per node:', [Names(perNode) for perNode in mbs.Inspect(oMass, I.RequestedNodeTypes)])
exu.Print('access functions of the rigid body:', Names(mbs.Inspect(oRigid, I.AccessFunctions)))
everything = mbs.Inspect(oSpring)
exu.Print('everything of the spring-damper:', {what.name: answer for (what, answer) in everything.items()})
exu.Print('a sensor has nothing to inspect:', mbs.Inspect(sSpring))

#(4) refused, with the reason
refused = 0
for (itemIndex, what) in [(1, I.OutputVariables), (oSpring, I.AccessFunctions)]:
    try:
        mbs.Inspect(itemIndex, what)
    except (TypeError, ValueError) as error:
        exu.Print('refused:', str(error).split(' [Python file')[0])
        refused += 1

testResult = (sum(len(answer) for answer in outputVariables.values()) + len(withUserFunction) + len(everything)
              + len(mbs.Inspect(oRigid, I.AccessFunctions)) + refused)
exu.Print('solution of inspectTest=', testResult)
exu.sys['testResult'] = testResult
