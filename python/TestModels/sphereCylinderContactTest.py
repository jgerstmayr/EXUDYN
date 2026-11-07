#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ObjectContactSphereCylinder (#3320): the output Distance of a deactivated contact against
#           exudyn.cUtilities.Distance for a sphere beside the side, above a cap, beside a rim and inside a tilted
#           cylinder; a ball at rest on the cap and on the side of a cylinder, m g / k into it; a ball sliding along
#           the top of a cylinder with friction, with the friction torque mu m g r about +y, rolls at 5/7 of its
#           initial velocity. The result is the sum of the checks and of the final positions. Run directly, the
#           SolutionViewer shows the drops and the rolling ball, whose edges show its rotation.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-07
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
from exudyn.cUtilities import ConvexShape, Distance
import numpy as np
import exudyn.graphics as graphics

testIsActive = exu.sys.get('testIsActive', False)

r = 0.1             #ball radius
rC, L = 0.2, 1.     #cylinder radius and length
k, d = 1e5, 1e3     #contact stiffness and damping
g = 9.81
checks = []

def Model(position, velocity=[0, 0, 0], cylinderHT=np.eye(4), friction=0., active=True, gravity=True):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.CreateGround()
    inertia = InertiaSphere(mass=1, radius=r)
    node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=list(position) + eulerParameters0,
                                       initialVelocities=list(velocity) + [0]*4))
    ball = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D(),
                                         visualization=VObjectRigidBody(graphicsData=[graphics.Sphere(radius=r, color=graphics.color.orange, nTiles=16, addEdges=True)])))
    if gravity:
        mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=ball)), loadVector=[0, 0, -g]))
    mCylinder = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localHT=cylinderHT))
    mBall = mbs.AddMarker(MarkerBodyRigid(bodyNumber=ball))
    nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=3, initialCoordinates=[1, 0, 0])) #no contact at the start
    contact = mbs.AddObject(ObjectContactSphereCylinder(markerNumbers=[mBall, mCylinder], nodeNumber=nData, sphereRadius=r,
                                                        cylinderRadius=rC, cylinderLength=L, contactStiffness=k, contactDamping=d,
                                                        dynamicFriction=friction, frictionProportionalZone=1e-3, activeConnector=active,
                                                        visualization=VObjectContactSphereCylinder(show=True)))
    mbs.Assemble()
    SC.visualizationSettings.contact.show.cylinders = True
    return mbs, node, contact

#the distance of a deactivated contact against cUtilities.Distance, with a tilted and shifted cylinder
cylinderHT = HTtranslate([0.1, -0.2, 0.3]) @ HTrotateX(0.4) @ HTrotateY(-0.3)
H1 = exu.HT(rotation=cylinderHT[0:3, 0:3], translation=cylinderHT[0:3, 3])
error = 0.
for localPosition in [[0.5, 0.1, 0.2], [0.05, -0.02, 0.9], [0.3, 0.3, -0.7], [0.05, 0.02, 0.1], [0.01, 0.15, -0.45]]: #side, cap, rim, inside
    position = (cylinderHT @ np.array(localPosition + [1]))[0:3]
    mbs, node, contact = Model(position, cylinderHT=cylinderHT, active=False)
    dItem = mbs.GetObjectOutput(contact, exu.OutputVariableType.Distance)
    nItem = mbs.GetObjectOutput(contact, exu.OutputVariableType.Director1)
    (dExact, p0, p1, nExact) = Distance(ConvexShape('Sphere', radius=r), exu.HT(translation=position),
                                        ConvexShape('Cylinder', radius=rC, length=L), H1)
    error = max(error, abs(dItem - dExact), np.max(np.abs(nItem - nExact)))
exu.Print('Distance of the deactivated contact against cUtilities.Distance:', error)
checks += [error < 1e-14]

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 2000
simulationSettings.timeIntegration.endTime = 1
simulationSettings.solution.file.write = not testIsActive #for the SolutionViewer

#at rest on the cap (axis vertical) and on the side (axis along x)
zEnd = []
for (cylinderHT, zRest) in [(np.eye(4), 0.5*L + r - g/k), (HTrotateY(0.5*np.pi), rC + r - g/k)]:
    mbs, node, contact = Model([0.05, 0, zRest + 0.05], cylinderHT=cylinderHT)
    mbs.SolveDynamic(simulationSettings)
    if not testIsActive: mbs.SolutionViewer()
    z = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[2]
    exu.Print('at rest: z =', z, ', expected', zRest)
    checks += [abs(z - zRest) < 1e-8]
    zEnd += [z]

#sliding along the top of a cylinder lying along x: friction makes the ball roll at 5/7 of its initial velocity
v0 = 1.
mbs, node, contact = Model([-0.4, 0, rC + r - g/k], velocity=[v0, 0, 0], cylinderHT=HTrotateY(0.5*np.pi), friction=0.5)
sTorque = mbs.AddSensor(SensorObject(objectNumber=contact, outputVariableType=exu.OutputVariableType.Torque, storeInternal=True))
mbs.Assemble()
simulationSettings.timeIntegration.endTime = 0.4
simulationSettings.timeIntegration.numberOfSteps = 4000
mbs.SolveDynamic(simulationSettings)
#while it slides (until t = 2 v0/(7 mu g) = 0.058 s), the friction torque on the ball is mu m g r about +y (#3328)
torqueSliding = mbs.GetSensorStoredData(sTorque)[2] #t = 0.02
exu.Print('friction torque while sliding:', torqueSliding[1:4], ', mu m g r =', 0.5*g*r)
checks += [abs(torqueSliding[2] - 0.5*g*r) < 1e-3*0.5*g*r, abs(torqueSliding[1]) + abs(torqueSliding[3]) < 1e-10]
if not testIsActive: mbs.SolutionViewer()
vx = mbs.GetNodeOutput(node, exu.OutputVariableType.Velocity)[0]
omega = mbs.GetNodeOutput(node, exu.OutputVariableType.AngularVelocity)[1]
exu.Print('rolling: vx =', vx, ', 5/7 v0 =', 5/7*v0, ', omega r =', omega*r)
checks += [abs(vx - 5/7*v0) < 2e-3, abs(vx - omega*r) < 2e-3]

u = sum(checks) + sum(zEnd) + vx
exu.Print('checks:', checks)
exu.Print('solution of sphereCylinderContactTest=', u)
exu.sys['testResult'] = u
