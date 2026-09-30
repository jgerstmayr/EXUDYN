#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Kinetic and potential energy as output variables (#2202): several simple, independent
#           mechanisms without gravity, each read through sensors of KineticEnergy and PotentialEnergy.
#           The undamped ones keep their total energy, the damped one loses it, and a rigid body with
#           an offset center of mass has the kinetic energy of its center of mass plus its rotation.
#           (1) a mass point on a Cartesian spring-damper, oscillating in x and y;
#           (2) a mass point on a distance spring-damper, swinging and stretching;
#           (3) a planar rigid body on a coordinate spring-damper, with a spin;
#           (4) a rotor (ObjectRotationalMass1D) on a torsional spring;
#           (5) a 1D mass on a coordinate spring-damper with damping - the energy decreases;
#           (6) a free rigid body whose reference point is not its center of mass;
#           (7) a rigid body on a torsional spring with a constant torque, and (8) one on a linear
#           spring-damper with a constant force - the constant load is part of the potential;
#           (9) an ObjectGenericODE2 with a constant force vector; (10) a rigid body on a rigid-body
#           spring-damper, in small motion; (11) ANCF cables and geometrically exact beams in a rigid
#           translation, whose kinetic energy comes from their mass matrix; (12) a system with gravity
#           and a constant force, whose total energy SystemEnergy (exudyn.advancedUtilities) records.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities

import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround = mbs.AddObject(ObjectGround())
nGround = mbs.AddNode(NodePointGround())
mGroundCoordinate = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
KE = exu.OutputVariableType.KineticEnergy
PE = exu.OutputVariableType.PotentialEnergy
mechanisms = {}     #name: ([kinetic energy sensors], [potential energy sensors])

def EnergySensors(name, bodies, connectors):
    mechanisms[name] = ([mbs.AddSensor(SensorBody(bodyNumber=b, outputVariableType=KE, storeInternal=True)) for b in bodies],
                        [mbs.AddSensor(SensorObject(objectNumber=c, outputVariableType=PE, storeInternal=True)) for c in connectors])

#(1) mass point on a Cartesian spring-damper
n1 = mbs.AddNode(NodePoint(referenceCoordinates=[0, 0, 0], initialCoordinates=[0.1, 0, 0], initialVelocities=[0, 0.5, 0]))
o1 = mbs.AddObject(MassPoint(nodeNumber=n1, physicsMass=2))
c1 = mbs.AddObject(CartesianSpringDamper(markerNumbers=[mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround)),
                                                        mbs.AddMarker(MarkerNodePosition(nodeNumber=n1))],
                                         stiffness=[200, 100, 300], damping=[0, 0, 0]))
EnergySensors('Cartesian spring', [o1], [c1])

#(2) mass point on a distance spring-damper, swinging and stretching
n2 = mbs.AddNode(NodePoint(referenceCoordinates=[1, 0, 0], initialCoordinates=[0.1, 0, 0], initialVelocities=[0, 1, 0.2]))
o2 = mbs.AddObject(MassPoint(nodeNumber=n2, physicsMass=1))
c2 = mbs.AddObject(SpringDamper(markerNumbers=[mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[2, 0, 0])),
                                               mbs.AddMarker(MarkerNodePosition(nodeNumber=n2))],
                                referenceLength=1, stiffness=100, damping=0))
EnergySensors('distance spring', [o2], [c2])

#(3) planar rigid body on a coordinate spring-damper in x, spinning freely
n3 = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0, 2, 0], initialCoordinates=[0.05, 0, 0], initialVelocities=[0, 0, 2]))
o3 = mbs.AddObject(RigidBody2D(nodeNumber=n3, physicsMass=3, physicsInertia=0.2))
c3 = mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mGroundCoordinate,
                                                         mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n3, coordinate=0))],
                                          stiffness=50, damping=0, offset=0.01))
EnergySensors('planar body', [o3], [c3])

#(4) rotor on a torsional spring (rotor about the local z-axis)
n4 = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0], initialCoordinates=[0.2], initialCoordinates_t=[0], numberOfODE2Coordinates=1))
o4 = mbs.AddObject(ObjectRotationalMass1D(nodeNumber=n4, physicsInertia=0.5, referencePosition=[0, 3, 0]))
c4 = mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mGroundCoordinate,
                                                         mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n4, coordinate=0))],
                                          stiffness=20, damping=0))
EnergySensors('rotor', [o4], [c4])

#(5) damped 1D mass: the energy decreases
n5 = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0], initialCoordinates=[0.1], initialCoordinates_t=[0], numberOfODE2Coordinates=1))
o5 = mbs.AddObject(ObjectMass1D(nodeNumber=n5, physicsMass=1, referencePosition=[0, 4, 0]))
c5 = mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mGroundCoordinate,
                                                         mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n5, coordinate=0))],
                                          stiffness=100, damping=1))
EnergySensors('damped mass', [o5], [c5])

#(6) free rigid body, center of mass away from the reference point: T is the one of the center of mass
inertia = InertiaCuboid(density=500, sideLengths=[0.4, 0.2, 0.1]).Translated([0.2, 0.1, 0.05])
omega0 = np.array([1., 2., 3.])
v0 = np.array([0.3, 0, 0])
n6 = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0, 5, 0] + list(eulerParameters0),
                                 initialVelocities=list(v0) + list(AngularVelocity2EulerParameters_t(omega0, eulerParameters0))))
o6 = mbs.AddObject(ObjectRigidBody(nodeNumber=n6, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D(),
                                   physicsCenterOfMass=inertia.COM()))
EnergySensors('free body with offset center of mass', [o6], [])

#(7) rigid body on a torsional spring about z, with a constant torque
n7 = mbs.AddNode(NodeRigidBodyRxyz(referenceCoordinates=[0, 6, 0, 0, 0, 0], initialVelocities=[0, 0, 0, 0, 0, 1]))
inertia7 = InertiaCuboid(density=500, sideLengths=[0.4, 0.2, 0.1])
o7 = mbs.AddObject(ObjectRigidBody(nodeNumber=n7, physicsMass=inertia7.Mass(), physicsInertia=inertia7.GetInertia6D()))
c7 = mbs.AddObject(TorsionalSpringDamper(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 6, 0])),
                                                        mbs.AddMarker(MarkerNodeRigid(nodeNumber=n7))],
                                         stiffness=2, damping=0, torque=0.1))
EnergySensors('torsional spring with a constant torque', [o7], [c7])

#(8) rigid body on a linear spring-damper along x, with a constant force
n8 = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0, 7, 0] + list(eulerParameters0), initialVelocities=[0.4, 0, 0, 0, 0, 0, 0]))
o8 = mbs.AddObject(ObjectRigidBody(nodeNumber=n8, physicsMass=inertia7.Mass(), physicsInertia=inertia7.GetInertia6D()))
c8 = mbs.AddObject(ObjectConnectorLinearSpringDamper(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 7, 0])),
                                                                    mbs.AddMarker(MarkerNodeRigid(nodeNumber=n8))],
                                                     stiffness=30, damping=0, force=0.5))
EnergySensors('linear spring with a constant force', [o8], [c8])

#(9) two masses on springs as an ObjectGenericODE2: M, K and a constant force vector
n9 = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0, 0], initialCoordinates=[0.1, -0.05], initialCoordinates_t=[0, 0.3],
                                 numberOfODE2Coordinates=2))
o9 = mbs.AddObject(ObjectGenericODE2(nodeNumbers=[n9], massMatrix=np.diag([1., 2.]),
                                     stiffnessMatrix=np.array([[200., -100.], [-100., 100.]]), forceVector=[0., 1.]))
mechanisms['GenericODE2 with a constant force'] = ([mbs.AddSensor(SensorBody(bodyNumber=o9, outputVariableType=KE, storeInternal=True))],
                                                   [mbs.AddSensor(SensorBody(bodyNumber=o9, outputVariableType=PE, storeInternal=True))])

#(10) rigid body on a rigid-body spring-damper, small motion in all six directions
n10 = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0, 8, 0] + list(eulerParameters0),
                                  initialVelocities=[0.02, 0.01, -0.01] + list(AngularVelocity2EulerParameters_t([0.02, -0.01, 0.03], eulerParameters0))))
o10 = mbs.AddObject(ObjectRigidBody(nodeNumber=n10, physicsMass=inertia7.Mass(), physicsInertia=inertia7.GetInertia6D()))
c10 = mbs.AddObject(RigidBodySpringDamper(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 8, 0])),
                                                         mbs.AddMarker(MarkerNodeRigid(nodeNumber=n10))],
                                          stiffness=np.diag([100, 200, 300, 2, 3, 4]), damping=np.zeros((6, 6))))
EnergySensors('rigid-body spring-damper, small motion', [o10], [c10])

#(11) the kinetic energy from the mass matrix: flexible bodies in a rigid translation, 1/2 m v^2
v11 = 0.5
nodes11 = [mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[i*0.5, 10, 1, 0], initialVelocities=[v11, 0, 0, 0])) for i in range(3)]
cables11 = [mbs.AddObject(ObjectANCFCable2D(nodeNumbers=[nodes11[i], nodes11[i+1]], physicsLength=0.5, physicsMassPerLength=2,
                                            physicsBendingStiffness=1, physicsAxialStiffness=100)) for i in range(2)]
section12 = exu.BeamSection()
section12.stiffnessMatrix = np.diag([100, 100, 100, 1, 1, 1]); section12.inertia = np.diag([2e-3, 1e-3, 1e-3]); section12.massPerLength = 2
nodes12 = [mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[i*0.5, 11, 0] + list(eulerParameters0),
                                       initialVelocities=[0, v11, 0, 0, 0, 0, 0])) for i in range(3)]
beams12 = [mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=[nodes12[i], nodes12[i+1]], physicsLength=0.5, sectionData=section12))
           for i in range(2)]
sRigidMotion = [mbs.AddSensor(SensorBody(bodyNumber=b, outputVariableType=KE, storeInternal=True)) for b in cables11 + beams12]

mbs.Assemble()

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 2000
simulationSettings.timeIntegration.endTime = 2
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.solutionSettings.sensorsWritePeriod = 0.01
mbs.SolveDynamic(simulationSettings)

testResult = 0
for (name, (kinetic, potential)) in mechanisms.items():
    total = sum(mbs.GetSensorStoredData(s)[:, 1] for s in kinetic + potential)
    exu.Print('%-40s total energy %.8f -> %.8f, relative change %.2e' % (name, total[0], total[-1], (total[-1]-total[0])/total[0]))
    testResult += total[-1]

#(6) against the kinetic energy of the center of mass and the rotation about it, at the start
comVelocity = v0 + np.cross(omega0, inertia.COM())
Jcom = inertia.Translated(-inertia.COM()).Inertia()
T6 = 0.5*inertia.Mass()*comVelocity @ comVelocity + 0.5*omega0 @ Jcom @ omega0
exu.Print('free body: T from the output variable', mbs.GetSensorStoredData(mechanisms['free body with offset center of mass'][0][0])[0, 1],
          ', from the center of mass', T6)

#(11) at the start: each element of mass 2*0.5 = 1 moving with 0.5 has T = 0.125
rigidMotion = [mbs.GetSensorStoredData(s)[0, 1] for s in sRigidMotion]
exu.Print('flexible bodies in a rigid translation: T =', rigidMotion, '(exact: 0.125 each)')
testResult += sum(rigidMotion)

#(12) the energy of a whole system with loads, SystemEnergy: a rigid body with an offset center of mass under
#gravity and a constant force, on a spring; the total of kinetic, elastic and load potential energy is kept
from exudyn.advancedUtilities import SystemEnergy
SC2 = exu.SystemContainer()
mbs2 = SC2.AddSystem()
oGround2 = mbs2.AddObject(ObjectGround())
inertia12 = InertiaCuboid(density=500, sideLengths=[0.4, 0.2, 0.1]).Translated([0.1, 0, 0])
n12 = mbs2.AddNode(NodeRigidBodyEP(referenceCoordinates=[1, 0, 0] + list(eulerParameters0),
                                   initialVelocities=[0, 0.5, 0] + list(AngularVelocity2EulerParameters_t([0, 0, 1], eulerParameters0))))
o12 = mbs2.AddObject(ObjectRigidBody(nodeNumber=n12, physicsMass=inertia12.Mass(), physicsInertia=inertia12.GetInertia6D(),
                                     physicsCenterOfMass=inertia12.COM()))
mbs2.AddObject(SpringDamper(markerNumbers=[mbs2.AddMarker(MarkerBodyPosition(bodyNumber=oGround2)),
                                           mbs2.AddMarker(MarkerBodyPosition(bodyNumber=o12))],
                            referenceLength=0.8, stiffness=50, damping=0))
mbs2.AddLoad(LoadMassProportional(markerNumber=mbs2.AddMarker(MarkerBodyMass(bodyNumber=o12)), loadVector=[0, -9.81, 0]))
mbs2.AddLoad(LoadForceVector(markerNumber=mbs2.AddMarker(MarkerBodyPosition(bodyNumber=o12, localPosition=[0.2, 0, 0])),
                             loadVector=[0.5, 0, 0]))
mbs2.Assemble()
systemEnergy = SystemEnergy(mbs2)
sEnergy = systemEnergy.AddSensor()
mbs2.Assemble()
exu.Print('SystemEnergy: kinetic', len(systemEnergy.kineticObjects), 'potential', len(systemEnergy.potentialObjects),
          'loads', len(systemEnergy.loads), 'unavailable', systemEnergy.unavailable)
mbs2.SolveDynamic(simulationSettings)
energies = mbs2.GetSensorStoredData(sEnergy)
exu.Print('system with loads: total energy %.8f -> %.8f, relative change %.2e' % (energies[0, 4], energies[-1, 4],
          (energies[-1, 4]-energies[0, 4])/energies[0, 4]))
testResult += energies[-1, 4]

exu.Print('solution of energiesTest=', testResult)
exu.sys['testResult'] = testResult
