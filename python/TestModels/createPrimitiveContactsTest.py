#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The Create functions of the contacts between primitive shapes (#3342), each with a body resting on a fixed
#           shape where its height is known: a ball on a cylinder lying along x (CreateSphereCylinderContact, r + R -
#           m g/k), a roller along y on the floor (CreateCylinderTriangleContact, on the 2 ends of its line, r - m g/(2k)),
#           a shaft along x in a fixed ring (CreateCylinderCylinderContact with isHollowCylinder1, on the 2 rims of the
#           ring), a cone standing on the floor (CreateConeTriangleContact, on 3 points of its rim), a cone standing on
#           the top of a fixed cone (CreateConeConeContact), a cube on a slab from the points and triangles of their
#           graphics (CreateConvexConvexContact, on its 4 corners), a ball on the plane of a triangle given as a list
#           (CreateSphereTriangleContact with infinitePlane, #3347). The size of the data nodes, also with the plastic
#           model, and two arguments refused. Run directly, the SolutionViewer shows the bodies. The result is the sum of
#           the checks and of the heights.
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

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()
oGround = mbs.CreateGround()

k, d, g = 1e5, 500., 9.81
law = dict(contactStiffness=k, contactDamping=d, show=True)
gravity = [0, 0, -g]
checks = []

def Body(inertia, position, graphicsList):
    return mbs.CreateRigidBody(inertia=inertia, referencePosition=position, gravity=gravity, graphicsDataList=graphicsList)

#a ball on a cylinder lying along x
r, R = 0.1, 0.2
ball = InertiaSphere(mass=1, radius=r)
oBall = Body(ball, [0, 0, R + r], [graphics.Sphere(radius=r, color=graphics.color.orange, nTiles=16)])
mbs.CreateSphereCylinderContact(itemNumbers=[oBall, oGround], localPosition1=[0, 0, 0], localAxis1=[1, 0, 0],
                                sphereRadius=r, cylinderRadius=R, cylinderLength=1, **law)
expected = {oBall: R + r - ball.Mass()*g/k}

#a roller along y on the floor
roller = InertiaCylinder(density=1000, length=0.2, outerRadius=0.1, axis=1)
oRoller = Body(roller, [0, 1, 0.1], [graphics.Cylinder(pAxis=[0, -0.1, 0], vAxis=[0, 0.2, 0], radius=0.1, color=graphics.color.steelblue)])
mbs.CreateCylinderTriangleContact(itemNumbers=[oRoller, oGround], localAxis0=[0, 1, 0], cylinderRadius=0.1, cylinderLength=0.2,
                                  trianglePoints=[[0, 0, 0], [1, 0, 0], [0, 1, 0]], infinitePlane=True, **law)
expected[oRoller] = 0.1 - roller.Mass()*g/(2*k)

#a shaft along x in a fixed ring of radius 0.06 at the height 0.3
shaft = InertiaCylinder(density=1000, length=0.5, outerRadius=0.05, axis=0)
oShaft = Body(shaft, [0, 2, 0.29], [graphics.Cylinder(pAxis=[-0.25, 0, 0], vAxis=[0.5, 0, 0], radius=0.05, color=graphics.color.red)])
mbs.CreateCylinderCylinderContact(itemNumbers=[oShaft, oGround], localAxis0=[1, 0, 0], localPosition1=[0, 2, 0.3], localAxis1=[1, 0, 0],
                                  cylindersRadii=[0.05, 0.06], cylindersLengths=[0.5, 0.1], isHollowCylinder1=True, **law)
expected[oShaft] = 0.29 - shaft.Mass()*g/(2*k)

#a cone standing on the floor, its base of radius 0.1 at the start of its axis
cone = InertiaCylinder(density=1000, length=0.2, outerRadius=0.07, axis=2) #an inertia close to the cone's
oCone = Body(cone, [0, 3, 0.1], [graphics.SolidOfRevolution(pAxis=[0, 0, -0.1], vAxis=[0, 0, 1], contour=[[0, 0], [0, 0.1], [0.2, 0.03], [0.2, 0]],
                                                           color=graphics.color.green)])
mbs.CreateConeTriangleContact(itemNumbers=[oCone, oGround], coneRadii=[0.1, 0.03], coneLength=0.2,
                              trianglePoints=[[0, 0, 0], [1, 0, 0], [0, 1, 0]], infinitePlane=True, **law)
expected[oCone] = 0.1 - cone.Mass()*g/(3*k)

#a small cone standing on the top of a fixed cone, whose top of radius 0.1 is at the height 0.2
small = InertiaCylinder(density=1000, length=0.1, outerRadius=0.04, axis=2)
oSmall = Body(small, [0, 4, 0.25], [graphics.SolidOfRevolution(pAxis=[0, 0, -0.05], vAxis=[0, 0, 1], contour=[[0, 0], [0, 0.05], [0.1, 0.02], [0.1, 0]],
                                                              color=graphics.color.yellow)])
mbs.CreateConeConeContact(itemNumbers=[oSmall, oGround], localPosition1=[0, 4, 0.1], conesRadii0=[0.05, 0.02], conesRadii1=[0.2, 0.1],
                          conesLengths=[0.1, 0.2], **law)
expected[oSmall] = 0.25 - small.Mass()*g/(3*k)

#a cube on a slab, from the points and triangles of their graphics; the slab moved down by localPosition1
a = 0.2
cubeGraphics = graphics.Brick(size=[a]*3, color=graphics.color.blue)
cube = InertiaCuboid(density=1000, sideLengths=[a]*3)
oCube = Body(cube, [0, 5, a/2], [cubeGraphics])
(points0, triangles0) = graphics.ToPointsAndTrigs(cubeGraphics)
(points1, triangles1) = graphics.ToPointsAndTrigs(graphics.Brick(size=[1, 1, 0.1]))
oContactCube = mbs.CreateConvexConvexContact(itemNumbers=[oCube, oGround], points0=points0, triangles0=triangles0,
                                             points1=points1, triangles1=triangles1, localPosition1=[0, 5, -0.05], **law)
expected[oCube] = a/2 - cube.Mass()*g/(4*k)

#a ball on the plane of a triangle given as a list, far outside the triangle (#3347)
oBall2 = Body(ball, [3, 6, r], [graphics.Sphere(radius=r, color=graphics.color.orange, nTiles=16)])
mbs.CreateSphereTriangleContact(itemNumbers=[oBall2, oGround], sphereRadius=r, trianglePoints=[[0, 0, 0], [1, 0, 0], [0, 1, 0]],
                                infinitePlane=True, contactStiffness=k, contactDamping=d)
expected[oBall2] = r - ball.Mass()*g/k

#the data nodes: 3 coordinates per contact point, 4 with the plastic model
nodeSize = lambda contact: mbs.GetNode(mbs.GetObject(contact)['nodeNumber'])['numberOfDataCoordinates']
oPlastic = mbs.CreateConvexConvexContact(itemNumbers=[oCube, oGround], points0=points0, triangles0=triangles0, points1=points1,
                                         triangles1=triangles1, contactPlasticityRatio=0.5, activeConnector=False)
checks += [nodeSize(oContactCube) == 24, nodeSize(oPlastic) == 32]
#arguments refused: a radius of zero, an axis of a marker other than its z axis
refused = 0
for arguments in [dict(sphereRadius=0, cylinderRadius=0.1, cylinderLength=1), dict(sphereRadius=0.1, cylinderRadius=0.1, cylinderLength=1, localAxis1=[1, 0, 0])]:
    try:
        mbs.CreateSphereCylinderContact(itemNumbers=[oBall, mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))], **arguments)
    except ValueError:
        refused += 1
checks += [refused == 2]
mbs.Assemble()

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
simulationSettings.timeIntegration.endTime = 1
simulationSettings.solution.file.write = not testIsActive #for the SolutionViewer
mbs.SolveDynamic(simulationSettings)

heights = []
for (body, height) in expected.items():
    z = mbs.GetObjectOutputBody(body, exu.OutputVariableType.Position)[2]
    heights += [z]
    checks += [abs(z - height) < 1e-9]
    exu.Print(mbs.GetObject(body)['name'] if mbs.GetObject(body)['name'] else body, 'at', z, 'against', height)

if not testIsActive:
    SC.visualizationSettings.contact.show.spheres = True
    SC.visualizationSettings.contact.show.cylinders = True
    SC.visualizationSettings.contact.show.cones = True
    mbs.SolutionViewer()

u = sum(checks) + sum(heights)
exu.Print('checks:', checks)
exu.Print('solution of createPrimitiveContactsTest=', u)
exu.sys['testResult'] = u
