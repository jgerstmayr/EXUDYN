#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The option infinitePlane of ObjectContactSphereTriangle (#3323): the Distance of a deactivated contact
#           beside the triangle - to its edge without the option, to its plane with it - and with the center behind
#           the plane; a ball sliding with friction over the edge of a small triangle rolls on in the plane at the
#           height r - m g / k with the option, and falls off the triangle without it. Run directly, the
#           SolutionViewer shows both balls. The result is the sum of the checks and of the final heights.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-07
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
from exudyn import cMath
import exudyn.graphics as graphics

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()
oGround = mbs.CreateGround()

r = 0.1
k, d = 1e5, 1e3
g = 9.81
trianglePoints = exu.Vector3DList([[-0.5, -0.5, 0], [0.5, -0.5, 0], [-0.5, 0.5, 0]]) #its edge from [0.5,-0.5] to [-0.5,0.5] is the line x + y = 0
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))

def Ball(position, velocity, color):
    inertia = InertiaSphere(mass=1, radius=r)
    node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=position + eulerParameters0, initialVelocities=velocity + [0]*4))
    ball = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D(),
                                         visualization=VObjectRigidBody(graphicsData=[graphics.Sphere(radius=r, color=color, nTiles=16, addEdges=True)])))
    mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=ball)), loadVector=[0, 0, -g]))
    return node, mbs.AddMarker(MarkerBodyRigid(bodyNumber=ball))

def Contact(mBall, infinitePlane, active=True):
    nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=4, initialCoordinates=[0, 0, 0, 0]))
    return mbs.AddObject(ObjectContactSphereTriangle(markerNumbers=[mBall, mGround], nodeNumber=nData, sphereRadius=r,
                                                     trianglePoints=trianglePoints, infinitePlane=infinitePlane,
                                                     contactStiffness=k, contactDamping=d, dynamicFriction=0.3,
                                                     activeConnector=active, visualization=VObjectContactSphereTriangle(show=True)))

#two balls on the triangle, sliding towards its edge x + y = 0 and rolling over it (x > 0 after 1 s); the first in contact with the plane, the second with the triangle
zRest = r - g/k
v0 = [0.5, 0.5, 0]
nodePlane, mPlane = Ball([-0.2, -0.2, zRest], v0, graphics.color.orange)
nodeTriangle, mTriangle = Ball([-0.2, -0.2, zRest + 1], v0, graphics.color.steelblue) #shifted up: an own triangle below it
contactPlane = Contact(mPlane, True)
mGroundUp = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 0, 1]))
nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=4, initialCoordinates=[0, 0, 0, 0]))
contactTriangle = mbs.AddObject(ObjectContactSphereTriangle(markerNumbers=[mTriangle, mGroundUp], nodeNumber=nData, sphereRadius=r,
                                                            trianglePoints=trianglePoints, contactStiffness=k, contactDamping=d,
                                                            dynamicFriction=0.3, visualization=VObjectContactSphereTriangle(show=True)))
#deactivated, for the distances: a third ball at rest beside the edge
nodeBeside, mBeside = Ball([0.3, 0.3, 0.25], [0, 0, 0], graphics.color.grey)
mbs.SetNodeParameter(nodeBeside, 'referenceCoordinates', [0.3, 0.3, 0.25] + eulerParameters0)
distancePlane = Contact(mBeside, True, active=False)
distanceTriangle = Contact(mBeside, False, active=False)
mbs.Assemble()

checks = []
dPlane = mbs.GetObjectOutput(distancePlane, exu.OutputVariableType.Distance)
dTriangle = mbs.GetObjectOutput(distanceTriangle, exu.OutputVariableType.Distance)
exu.Print('beside the triangle: Distance to the plane', dPlane, ', to the triangle', dTriangle)
checks += [abs(dPlane - (0.25 - r)) < 1e-14, abs(dTriangle - (cMath.sqrt(2*0.3**2 + 0.25**2) - r)) < 1e-14]

#the center behind the plane: the penetration is the depth of the center plus the radius
mbs.SetNodeParameter(nodeBeside, 'referenceCoordinates', [0.3, 0.3, -0.05] + eulerParameters0)
mbs.Assemble()
dBehind = mbs.GetObjectOutput(distancePlane, exu.OutputVariableType.Distance)
nBehind = mbs.GetObjectOutput(distancePlane, exu.OutputVariableType.Director1)
exu.Print('center behind the plane: Distance', dBehind, ', normal', nBehind)
checks += [abs(dBehind - (-0.05 - r)) < 1e-14, abs(nBehind[2] + 1) < 1e-14]
mbs.SetNodeParameter(nodeBeside, 'referenceCoordinates', [0.3, 0.3, 0.25] + eulerParameters0) #gravity lets it fall; no contact is active
mbs.Assemble()

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 2000
simulationSettings.timeIntegration.endTime = 1
simulationSettings.solution.file.write = not testIsActive #for the SolutionViewer
mbs.SolveDynamic(simulationSettings)

zPlane = mbs.GetNodeOutput(nodePlane, exu.OutputVariableType.Position)[2]
zTriangle = mbs.GetNodeOutput(nodeTriangle, exu.OutputVariableType.Position)[2] - 1
xPlane = mbs.GetNodeOutput(nodePlane, exu.OutputVariableType.Position)[0]
exu.Print('after 1 s: the ball on the plane at x =', xPlane, ', z =', zPlane, '; the ball on the triangle at z =', zTriangle)
checks += [xPlane > 0.1, abs(zPlane - zRest) < 1e-6, zTriangle < -0.5]

if not testIsActive:
    SC.visualizationSettings.contact.showTriangles = True
    mbs.SolutionViewer()

u = sum(checks) + zPlane + zTriangle
exu.Print('checks:', checks)
exu.Print('solution of sphereTrianglePlaneTest=', u)
exu.sys['testResult'] = u
