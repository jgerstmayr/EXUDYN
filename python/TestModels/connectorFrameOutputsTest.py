#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The frame outputs of joints and connectors (#2870): HomogeneousTransformation is the joint frame J0,
#           HomogeneousTransformationLocal the joint frame J1 relative to J0, Displacement the global vector between
#           the markers. Three rigid bodies on a revolute, a generic and a prismatic joint, the last one with a linear
#           spring-damper, and a fourth on a rigid-body spring-damper move under gravity; then J0 times the local
#           frame must give J1 as the markers and the rotationMarker1 of the joint say, the translation of the local
#           frame must equal DisplacementLocal, and a SensorObject stores the 16 values. The 2D joints: a pendulum on
#           JointRevolute2D (Position, Force), a body on JointPrismatic2D (ForceLocal, DisplacementLocal against
#           Distance), and a body on a cable with JointSliding2D (DisplacementLocal, its drift; SlidingCoordinate, #2872).
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

OVT = exu.OutputVariableType
g = 9.81
inertia = InertiaCuboid(density=1000, sideLengths=[0.2,0.1,0.1])
checks = [] #(name, passed)

def Check(name, passed):
    checks.append((name, bool(passed)))
    if not passed:
        exu.Print('connectorFrameOutputsTest: FAILED', name)

def Settings(endTime, steps):
    simulationSettings = exu.SimulationSettings()
    simulationSettings.solution.file.write = False
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.timeIntegration.numberOfSteps = steps
    simulationSettings.timeIntegration.endTime = endTime
    simulationSettings.timeIntegration.newton.useModifiedNewton = False
    return simulationSettings

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the 3D joints and the two rigid-body connectors
SC = exu.SystemContainer()
mbs = SC.AddSystem()
oGround = mbs.CreateGround()
gravity = [0,-g,0]
oRevolute = mbs.CreateRigidBody(inertia=inertia, referencePosition=[1,0,0], gravity=gravity)
jRevolute = mbs.CreateRevoluteJoint(itemNumbers=[oGround, oRevolute], position=[0.5,0,0], axis=[0,0.2,1])
oGeneric = mbs.CreateRigidBody(inertia=inertia, referencePosition=[1,2,0], gravity=gravity,
                               referenceRotationMatrix=RotationMatrixX(0.4))
jGeneric = mbs.CreateGenericJoint(itemNumbers=[oGround, oGeneric], position=[0.5,2,0], constrainedAxes=[1,1,1,1,0,0],
                                  rotationMatrixAxes=RotationMatrixZ(0.3))
oPrismatic = mbs.CreateRigidBody(inertia=inertia, referencePosition=[1,4,0], gravity=gravity)
jPrismatic = mbs.CreatePrismaticJoint(itemNumbers=[oGround, oPrismatic], position=[1,4,0], axis=[1,-1,0])
cLinear = mbs.CreateLinearSpringDamper(itemNumbers=[oGround, oPrismatic], position=[1,4,0], axis=[1,-1,0],
                                       stiffness=2e3, damping=10)
oSpring = mbs.CreateRigidBody(inertia=inertia, referencePosition=[1,6,0], gravity=gravity,
                              initialAngularVelocity=[1,2,3])
cRigid = mbs.CreateRigidBodySpringDamper(itemNumbers=[oGround, oSpring], localPosition0=[1,6.1,0], localPosition1=[0,0.1,0],
                                         stiffness=np.diag([2e3,2e3,2e3,50,50,50]), damping=np.diag([5,5,5,0.5,0.5,0.5]),
                                         rotationMatrixJoint=RotationMatrixY(0.2))
sJointHT = mbs.AddSensor(SensorObject(objectNumber=jRevolute, storeInternal=True, outputVariableType=OVT.HomogeneousTransformationLocal))

mbs.Assemble()
mbs.SolveDynamic(Settings(0.5, 500))

def MarkerFrame(markerNumber, rotationMarker):
    """the frame of a marker times rotationMarker of the item"""
    H = mbs.GetMarkerOutput(markerNumber, OVT.HomogeneousTransformation).HT44()
    return HomogeneousTransformation(H[0:3,0:3] @ np.array(rotationMarker), H[0:3,3])

for (name, item, rotationMarkers, hasHT) in [('JointRevoluteZ', jRevolute, True, True),
                                             ('JointGeneric', jGeneric, True, True),
                                             ('JointPrismaticX', jPrismatic, True, True),
                                             ('ConnectorRigidBodySpringDamper', cRigid, True, False),
                                             ('ConnectorLinearSpringDamper', cLinear, False, False)]:
    data = mbs.GetObject(item)
    [m0, m1] = data['markerNumbers']
    H0 = MarkerFrame(m0, data['rotationMarker0'] if rotationMarkers else np.eye(3))
    H1 = MarkerFrame(m1, data['rotationMarker1'] if rotationMarkers else np.eye(3))
    HLocal = mbs.GetObjectOutput(item, OVT.HomogeneousTransformationLocal).HT44()
    Check(name + ' local frame', np.abs(HLocal - np.linalg.inv(H0) @ H1).max() < 1e-12)
    Check(name + ' Displacement', np.abs(mbs.GetObjectOutput(item, OVT.Displacement) - (H1[0:3,3] - H0[0:3,3])).max() < 1e-12
          if name.startswith('Connector') else True)
    if hasHT:
        Check(name + ' frame J0', np.abs(mbs.GetObjectOutput(item, OVT.HomogeneousTransformation).HT44() - H0).max() < 1e-12)
        Check(name + ' DisplacementLocal', np.abs(HLocal[0:3,3] - mbs.GetObjectOutput(item, OVT.DisplacementLocal)).max() < 1e-12)

#the revolute joint: the local frame turns about z by Rotation[2], up to the drift
HLocal = mbs.GetObjectOutput(jRevolute, OVT.HomogeneousTransformationLocal).HT44()
angle = mbs.GetObjectOutput(jRevolute, OVT.Rotation)[2]
Check('JointRevoluteZ turned', abs(angle) > 0.1 and np.abs(HLocal[0:3,0:3] - RotationMatrixZ(angle)).max() < 1e-8)
Check('sensor of the local frame', np.abs(mbs.GetSensorStoredData(sJointHT)[-1,1:] - HLocal.flatten()).max() < 1e-12)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the 2D joints at rest: the reaction force carries the weight; as for every joint, the force is the one on marker 0 (the ground)
mass = 2.
SC2 = exu.SystemContainer()
mbs2 = SC2.AddSystem()
oGround2 = mbs2.AddObject(ObjectGround())
nPendulum = mbs2.AddNode(NodeRigidBody2D(referenceCoordinates=[0,-1,0]))
oPendulum = mbs2.AddObject(ObjectRigidBody2D(nodeNumber=nPendulum, mass=mass, inertia=0.1))
mbs2.AddLoad(LoadMassProportional(markerNumber=mbs2.AddMarker(MarkerBodyMass(bodyNumber=oPendulum)), loadVector=[0,-g,0]))
mPendulumTop = mbs2.AddMarker(MarkerBodyPosition(bodyNumber=oPendulum, localPosition=[0,1,0]))
jRevolute2D = mbs2.AddObject(ObjectJointRevolute2D(markerNumbers=[mbs2.AddMarker(MarkerBodyPosition(bodyNumber=oGround2)), mPendulumTop]))

nSlider = mbs2.AddNode(NodeRigidBody2D(referenceCoordinates=[3,0,0], initialVelocities=[1,0,0]))
oSlider = mbs2.AddObject(ObjectRigidBody2D(nodeNumber=nSlider, mass=mass, inertia=0.1))
mbs2.AddLoad(LoadMassProportional(markerNumber=mbs2.AddMarker(MarkerBodyMass(bodyNumber=oSlider)), loadVector=[0,-g,0]))
mGroundSlider = mbs2.AddMarker(MarkerBodyRigid(bodyNumber=oGround2, localPosition=[2,0,0]))
jPrismatic2D = mbs2.AddObject(ObjectJointPrismatic2D(markerNumbers=[mGroundSlider, mbs2.AddMarker(MarkerBodyRigid(bodyNumber=oSlider))],
                                                     axisMarker0=[2,0,0], normalMarker1=[0,1,0]))
mbs2.Assemble()
mbs2.SolveDynamic(Settings(0.2, 200))

force = mbs2.GetObjectOutput(jRevolute2D, OVT.Force)
Check('JointRevolute2D Force', abs(force[1] + mass*g) < 1e-8 and abs(force[0]) < 1e-8 and force[2] == 0)
Check('JointRevolute2D Position', np.abs(mbs2.GetObjectOutput(jRevolute2D, OVT.Position)).max() == 0)
Check('JointRevolute2D Velocity', np.abs(mbs2.GetObjectOutput(jRevolute2D, OVT.Velocity)).max() == 0)
forceLocal = mbs2.GetObjectOutput(jPrismatic2D, OVT.ForceLocal)
Check('JointPrismatic2D ForceLocal', abs(forceLocal[1] + mass*g) < 1e-8 and abs(forceLocal[0]) < 1e-8)
displacementLocal = mbs2.GetObjectOutput(jPrismatic2D, OVT.DisplacementLocal)
Check('JointPrismatic2D DisplacementLocal', abs(displacementLocal[0] - mbs2.GetObjectOutput(jPrismatic2D, OVT.Distance)) < 1e-12
      and abs(displacementLocal[0] - 1.2) < 1e-8)
Check('JointPrismatic2D frames', np.abs(mbs2.GetObjectOutput(jPrismatic2D, OVT.HomogeneousTransformationLocal).HT44()[0:3,3]
                                        - displacementLocal).max() < 1e-12
      and np.abs(mbs2.GetObjectOutput(jPrismatic2D, OVT.HomogeneousTransformation).HT44()[0:3,3] - [2,0,0]).max() == 0)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#JointSliding2D: a body hanging from a cable, as in SlidingJoint2DTest; the sliding point stays on marker 0 up to the drift
SC3 = exu.SystemContainer()
mbs3 = SC3.AddSystem()
L = 2; nElements = 3; lElem = L/nElements
E = 2.07e11; rho = 7800; A = 1e-6; I = 1e-12/12
nGround3 = mbs3.AddNode(NodePointGround())
mGround3 = mbs3.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround3, coordinate=0))
cableList = []
nc0 = mbs3.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
for i in range(nElements):
    mbs3.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
    cableList += [mbs3.AddObject(Cable2D(length=lElem, massPerLength=rho*A, bendingStiffness=E*I, axialStiffness=E*A,
                                         nodeNumbers=[int(nc0)+i, int(nc0)+i+1]))]
    mbs3.AddLoad(Gravity(markerNumber=mbs3.AddMarker(MarkerBodyMass(bodyNumber=cableList[-1])), loadVector=[0,-g,0]))
for coordinate in [0,1,3]:
    mbs3.AddObject(CoordinateConstraint(markerNumbers=[mGround3, mbs3.AddMarker(MarkerNodeCoordinate(nodeNumber=nc0, coordinate=coordinate))]))
a = 0.1
massRigid = 0.12
nRigid = mbs3.AddNode(Rigid2D(referenceCoordinates=[lElem*1.5,-a,0]))
oRigid = mbs3.AddObject(RigidBody2D(mass=massRigid, inertia=massRigid/12*(2*a)**2, nodeNumber=nRigid))
markerRigidTop = mbs3.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[0.,a,0.]))
mbs3.AddLoad(Force(markerNumber=mbs3.AddMarker(MarkerBodyPosition(bodyNumber=oRigid)), loadVector=[massRigid*g*0.1, -massRigid*g, 0]))
cableMarkers = [mbs3.AddMarker(MarkerBodyCable2DCoordinates(bodyNumber=item)) for item in cableList]
nodeData = mbs3.AddNode(NodeGenericData(initialCoordinates=[1, lElem*1.5], numberOfDataCoordinates=2))
jSliding = mbs3.AddObject(ObjectJointSliding2D(markerNumbers=[markerRigidTop, cableMarkers[1]], slidingMarkerNumbers=cableMarkers,
                                               slidingMarkerOffsets=[i*lElem for i in range(nElements)],
                                               nodeNumber=nodeData, useClassicalFormulation=False))
mbs3.Assemble()
drift0 = mbs3.GetObjectOutput(jSliding, OVT.DisplacementLocal)
settings3 = Settings(0.05, 50)
settings3.timeIntegration.newton.relativeTolerance = 1e-6
settings3.timeIntegration.newton.absoluteTolerance = 1e-8
mbs3.SolveDynamic(settings3)
drift = mbs3.GetObjectOutput(jSliding, OVT.DisplacementLocal)
#the tangential component from the sliding coordinate of the step, counted once (#2872)
Check('JointSliding2D DisplacementLocal', np.abs(drift0).max() < 1e-14 and abs(drift[0]) < 1e-8 and abs(drift[1]) < 1e-8
      and drift[2] == 0)
Check('JointSliding2D SlidingCoordinate', mbs3.GetObjectOutput(jSliding, OVT.SlidingCoordinate) == mbs3.GetNodeOutput(nodeData, OVT.Coordinates)[1])

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
u = sum(int(passed) for (name, passed) in checks)
exu.Print(u, 'of', len(checks), 'checks passed')
exu.Print('solution of connectorFrameOutputsTest=', u)

exu.sys['testResult'] = u
