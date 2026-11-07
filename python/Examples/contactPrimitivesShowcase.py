#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The contacts between primitive shapes side by side: a ball rolling over a cylinder (ObjectContactSphereCylinder)
#           onto the floor (ObjectContactSphereTriangle with infinitePlane), a roller rolling down an inclined plane
#           (ObjectContactCylinderTriangle with infinitePlane), a truncated cone
#           rolling in a circle on the plane (ObjectContactConeTriangle), a cube dropped tilted onto a box, tipping onto
#           its face (ObjectContactConvexConvex), a shaft falling into a short ring (ObjectContactCylinderCylinder with
#           isHollowCylinder1) and a rotor floating in a lubricated journal bearing (ObjectJournalBearing). The shapes
#           of the contacts are drawn by the contact settings; the solution is shown in the SolutionViewer.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-07
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
import numpy as np
from exudyn import cMath

SC = exu.SystemContainer()
mbs = SC.AddSystem()
oGround = mbs.CreateGround()

k, d, mu, g = 1e5, 500., 0.4, 9.81
law = dict(contactStiffness=k, contactDamping=d, dynamicFriction=mu)

def Body(position, inertia, graphicsList, rotation=np.eye(3), velocity=[0, 0, 0], angularVelocity=[0, 0, 0]):
    """a rigid body under gravity; returns its marker at the center"""
    ep = RotationMatrix2EulerParameters(rotation)
    node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=list(position) + list(ep),
                                       initialVelocities=list(velocity) + list(AngularVelocity2EulerParameters_t(angularVelocity, ep))))
    body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D(),
                                         visualization=VObjectRigidBody(graphicsData=graphicsList)))
    mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=body)), loadVector=[0, 0, -g]))
    return body

def DataNode(n):
    return mbs.AddNode(NodeGenericData(numberOfDataCoordinates=n, initialCoordinates=([1, 0, 0]*(n//3))))

def GroundMarker(HT):
    return mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localHT=HT))

plane = exu.Vector3DList([[-10, -10, 0], [10, -10, 0], [0, 10, 0]])

#a ball rolling over a cylinder lying along y
r = 0.08
ball = Body([-0.3, 0, 0.35], InertiaSphere(mass=1, radius=r), [graphics.Sphere(radius=r, color=graphics.color.orange, nTiles=16, addEdges=True)],
            velocity=[0.6, 0, 0])
mbs.AddObject(ObjectContactSphereCylinder(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=ball)), GroundMarker(HTtranslate([0, 0, 0.15]) @ HTrotateX(0.5*np.pi))],
                                          nodeNumber=DataNode(3), sphereRadius=r, cylinderRadius=0.15, cylinderLength=0.6,
                                          visualization=VObjectContactSphereCylinder(show=True), **law))
mbs.AddObject(ObjectContactSphereTriangle(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=ball)), GroundMarker(np.eye(4))],
                                          nodeNumber=mbs.AddNode(NodeGenericData(numberOfDataCoordinates=4, initialCoordinates=[1, 0, 0, 0])),
                                          sphereRadius=r, trianglePoints=plane, infinitePlane=True, **law)) #the floor
#a roller rolling down a plane inclined by 10 degrees
incline = HTtranslate([0, 1, 0]) @ HTrotateY(-10/180*np.pi)
roller = Body([-0.6, 1, 0.1 + 0.11*cMath.sin(10/180*np.pi)], InertiaCylinder(density=1000, length=0.15, outerRadius=0.1, axis=1),
              [graphics.Cylinder(pAxis=[0, -0.075, 0], vAxis=[0, 0.15, 0], radius=0.1, color=graphics.color.steelblue, nTiles=24, addEdges=True)],
              rotation=RotationMatrixY(-10/180*np.pi))
mbs.AddObject(ObjectContactCylinderTriangle(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=roller, localHT=HTrotateX(0.5*np.pi))), GroundMarker(incline)],
                                            nodeNumber=DataNode(12), cylinderRadius=0.1, cylinderLength=0.15, trianglePoints=plane,
                                            infinitePlane=True, visualization=VObjectContactCylinderTriangle(show=True), **law))
#a truncated cone rolling in a circle, lying on its side line
r0, r1, L = 0.1, 0.05, 0.2
alpha = cMath.atan((r0 - r1)/L)
rm = 0.5*(r0 + r1)
cone = Body([0, 2, rm*cMath.cos(alpha)], InertiaCylinder(density=1000, length=L, outerRadius=rm, axis=2), [],
            rotation=RotationMatrixY(0.5*np.pi + alpha), angularVelocity=[-8*cMath.cos(alpha), 0, 8*cMath.sin(alpha)])
mbs.AddObject(ObjectContactConeTriangle(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=cone)), GroundMarker(np.eye(4))],
                                        nodeNumber=DataNode(12), coneRadii=[r0, r1], coneLength=L, trianglePoints=plane, infinitePlane=True,
                                        visualization=VObjectContactConeTriangle(show=True), **law))
#a cube dropped tilted onto a box
a = 0.15
def Box(size):
    points = [[sx*size[0]/2, sy*size[1]/2, sz*size[2]/2] for sz in [-1, 1] for sy in [-1, 1] for sx in [-1, 1]]
    triangles = [[0, 2, 1], [1, 2, 3], [4, 5, 6], [5, 7, 6], [0, 1, 4], [1, 5, 4], [2, 6, 3], [3, 6, 7], [0, 4, 2], [2, 4, 6], [1, 3, 5], [3, 7, 5]]
    return np.array(points), np.array(triangles)
cube = Body([0, 3, 0.1 + a + 0.1], InertiaCuboid(density=1000, sideLengths=[a]*3), [graphics.Brick(size=[a]*3, color=graphics.color.green, addEdges=True)],
            rotation=RotationMatrixX(0.6) @ RotationMatrixY(0.3))
mbs.AddObject(ObjectGround(referencePosition=[0, 3, 0], visualization=VObjectGround(graphicsData=[graphics.Brick(size=[0.8, 0.8, 0.2], color=graphics.color.lightgrey)])))
points0, triangles0 = Box([a]*3)
points1, triangles1 = Box([0.8, 0.8, 0.2])
mbs.AddObject(ObjectContactConvexConvex(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=cube)), GroundMarker(HTtranslate([0, 3, 0]))],
                                        nodeNumber=DataNode(24), points0=points0, triangles0=triangles0, points1=points1, triangles1=triangles1,
                                        **law))
#a shaft falling into a short ring along x
shaft = Body([0, 4, 0.03], InertiaCylinder(density=1000, length=0.5, outerRadius=0.05, axis=0),
             [graphics.Cylinder(pAxis=[-0.25, 0, 0], vAxis=[0.5, 0, 0], radius=0.05, color=graphics.color.red, nTiles=24, addEdges=True)])
mbs.AddObject(ObjectContactCylinderCylinder(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=shaft, localHT=HTrotateY(0.5*np.pi))),
                                                           GroundMarker(HTtranslate([0, 4, 0]) @ HTrotateY(0.5*np.pi))],
                                            nodeNumber=DataNode(24), cylindersRadii=[0.05, 0.07], cylindersLengths=[0.5, 0.1], isHollowCylinder1=True,
                                            visualization=VObjectContactCylinderCylinder(show=True), **law))
#a rotor floating in a lubricated journal bearing, turning at 100 rad/s
rotor = Body([0, 5, 0], InertiaCylinder(density=7800, length=0.01, outerRadius=0.02, axis=2), [], angularVelocity=[0, 0, 100])
mbs.AddObject(ObjectJournalBearing(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=rotor, localHT=HTrotateX(0.5*np.pi))),
                                                  GroundMarker(HTtranslate([0, 5, 0]) @ HTrotateX(0.5*np.pi))],
                                   journalRadius=0.02 - 1e-4, bearingRadius=0.02, bearingLength=0.01, viscosity=0.04, contactStiffness=1e8,
                                   visualization=VObjectJournalBearing(show=True)))
mbs.Assemble()

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 4000
simulationSettings.timeIntegration.endTime = 2
simulationSettings.solution.file.writePeriod = 0.01
mbs.SolveDynamic(simulationSettings)

SC.visualizationSettings.contact.showSpheres = True
SC.visualizationSettings.contact.showCylinders = True
SC.visualizationSettings.contact.showCones = True
SC.visualizationSettings.contact.showTriangles = False
mbs.SolutionViewer()
