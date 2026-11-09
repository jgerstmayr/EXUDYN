#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Capsules in CreateCylinderCylinderContact (isCapsule0, isCapsule1, #3365) and CreatePrimitiveConvexContact
#           (ObjectContactPrimitiveConvex, #3364), each with a body resting where its height is known: two parallel
#           capsules on the 2 ends of their line, two crossed capsules on one point, a capsule on the cap of a cylinder
#           on the 2 ends of its segment, a capsule on the side of a parallel cylinder; a sphere, capsule, cylinder and cone
#           on a box given by points and triangles, on 1, 2, 3 and 3 points. Also the output Distance of an inactive
#           contact, graphics.Capsule (#3366) by the volume and the extent of its surface, and the arguments refused. Run
#           directly, the SolutionViewer shows the bodies. The result is the sum of the checks and of the heights.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-08
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
expected = {}

r, L = 0.05, 0.2
capsuleInertia = InertiaCylinder(density=1000, length=L + 2*r, outerRadius=r, axis=0)
mass = capsuleInertia.Mass()

def Body(inertia, position, graphicsList):
    return mbs.CreateRigidBody(inertia=inertia, referencePosition=position, gravity=gravity, graphicsDataList=graphicsList)

def CapsuleBody(position, axis=[L, 0, 0]):
    start = -0.5*np.array(axis)
    return Body(capsuleInertia, position, [graphics.Capsule(pAxis=list(start), vAxis=axis, radius=r, color=graphics.color.orange)])

#two parallel capsules along x, on the 2 ends of the line
oParallel = CapsuleBody([0, 0, 2*r])
mbs.CreateCylinderCylinderContact(itemNumbers=[oParallel, oGround], localAxis0=[1, 0, 0], localPosition1=[0, 0, 0], localAxis1=[1, 0, 0],
                                  cylindersRadii=[r, r], cylindersLengths=[L, L], isCapsule0=True, isCapsule1=True, **law)
expected[oParallel] = 2*r - mass*g/(2*k)

#two crossed capsules, on one point
oCrossed = CapsuleBody([0, 1, 2*r])
mbs.CreateCylinderCylinderContact(itemNumbers=[oCrossed, oGround], localAxis0=[1, 0, 0], localPosition1=[0, 1, 0], localAxis1=[0, 1, 0],
                                  cylindersRadii=[r, r], cylindersLengths=[L, L], isCapsule0=True, isCapsule1=True, **law)
expected[oCrossed] = 2*r - mass*g/k

#a capsule lying on the cap of a fixed cylinder along z, whose top is at z = 0.2
oOnCap = CapsuleBody([0, 2, 0.2 + r])
mbs.CreateCylinderCylinderContact(itemNumbers=[oOnCap, oGround], localAxis0=[1, 0, 0], localPosition1=[0, 2, 0.1], localAxis1=[0, 0, 1],
                                  cylindersRadii=[r, 0.3], cylindersLengths=[L, 0.2], isCapsule0=True, **law)
expected[oOnCap] = 0.2 + r - mass*g/(2*k)

#a capsule on the side of a parallel fixed cylinder of radius 0.1
oOnSide = CapsuleBody([0, 3, 0.1 + r])
mbs.CreateCylinderCylinderContact(itemNumbers=[oOnSide, oGround], localAxis0=[1, 0, 0], localPosition1=[0, 3, 0], localAxis1=[1, 0, 0],
                                  cylindersRadii=[r, 0.1], cylindersLengths=[L, 1], isCapsule0=True, **law)
expected[oOnSide] = 0.1 + r - mass*g/(2*k)

#the primitives on a box, whose top is at z = 0, by its points and triangles
(points1, triangles1) = graphics.ToPointsAndTrigs(graphics.Brick(size=[1, 1, 0.1]))

def OnBox(body, y, **primitive):
    return mbs.CreatePrimitiveConvexContact(itemNumbers=[body, oGround], points1=points1, triangles1=triangles1, localPosition1=[0, y, -0.05], **primitive, **law)

R = 0.1
ball = InertiaSphere(mass=1, radius=R)
oSphere = Body(ball, [0, 4, R], [graphics.Sphere(radius=R, color=graphics.color.orange, nTiles=16)])
OnBox(oSphere, 4, primitiveType=exu.ContactPrimitiveType.Sphere, radius=R)
expected[oSphere] = R - ball.Mass()*g/k

oCapsule = CapsuleBody([0, 5, r])
OnBox(oCapsule, 5, primitiveType=exu.ContactPrimitiveType.Capsule, radius=r, length=L, localAxis0=[1, 0, 0])
expected[oCapsule] = r - mass*g/(2*k)

cylinder = InertiaCylinder(density=1000, length=0.2, outerRadius=R, axis=2)
oCylinder = Body(cylinder, [0, 6, 0.1], [graphics.Cylinder(pAxis=[0, 0, -0.1], vAxis=[0, 0, 0.2], radius=R, color=graphics.color.steelblue)])
OnBox(oCylinder, 6, primitiveType=exu.ContactPrimitiveType.Cylinder, radius=R, length=0.2)
expected[oCylinder] = 0.1 - cylinder.Mass()*g/(3*k)

cone = InertiaCylinder(density=1000, length=0.2, outerRadius=0.07, axis=2) #an inertia close to the cone's
oCone = Body(cone, [0, 7, 0.1], [graphics.SolidOfRevolution(pAxis=[0, 0, -0.1], vAxis=[0, 0, 1], contour=[[0, 0], [0, R], [0.2, 0.03], [0.2, 0]],
                                                           color=graphics.color.green)])
OnBox(oCone, 7, primitiveType=3, radius=R, radius1=0.03, length=0.2) #the integer of Cone
expected[oCone] = 0.1 - cone.Mass()*g/(3*k)

#the distance of an inactive contact: a sphere of radius R with its center 0.5 above the top of the box
oDistance = mbs.CreatePrimitiveConvexContact(itemNumbers=[oGround, oGround], primitiveType=exu.ContactPrimitiveType.Sphere, radius=R, localPosition0=[0, 8, 0.5],
                                             points1=points1, triangles1=triangles1, localPosition1=[0, 8, -0.05], activeConnector=False)

#the data node of 4 points; a radius of zero is refused
nodeSize = lambda contact: mbs.GetNode(mbs.GetObject(contact)['nodeNumber'])['numberOfDataCoordinates']
checks += [nodeSize(oDistance) == 12]
if not exu.__useExudynFast: #the fast module does not check arguments
    try:
        mbs.CreatePrimitiveConvexContact(itemNumbers=[oSphere, oGround], radius=0, points1=points1, triangles1=triangles1)
        checks += [False]
    except ValueError:
        checks += [True]
else:
    checks += [True]
mbs.Assemble()
checks += [abs(mbs.GetObjectOutput(oDistance, exu.OutputVariableType.Distance) - (0.5 - R)) < 1e-10]

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
simulationSettings.timeIntegration.endTime = 1
simulationSettings.solution.file.write = not testIsActive #for the SolutionViewer
mbs.SolveDynamic(simulationSettings)

heights = []
for (body, height) in expected.items():
    z = mbs.GetObjectOutputBody(body, exu.OutputVariableType.Position)[2]
    heights += [z]
    checks += [abs(z - height) < 1e-8]
    exu.Print('body', int(body), 'at', z, 'against', height)

#isHollowCylinder1 together with a capsule is refused at Assemble
SC2 = exu.SystemContainer()
mbs2 = SC2.AddSystem()
oGround2 = mbs2.CreateGround()
mbs2.CreateCylinderCylinderContact(itemNumbers=[oGround2, oGround2], cylindersRadii=[0.1, 0.2], cylindersLengths=[1, 1], isHollowCylinder1=True, isCapsule0=True)
try:
    mbs2.Assemble()
    checks += [False]
except Exception:
    checks += [True]

#graphics.Capsule: the volume of the closed surface and its extent along the axis
(a, b) = (0.1, 0.5)
(points, triangles) = graphics.ToPointsAndTrigs(graphics.Capsule(pAxis=[0, 0, 0], vAxis=[0, 0, b], radius=a, nTiles=64))
points = np.array(points)
volume = sum(np.dot(points[t[0]], np.cross(points[t[1]], points[t[2]])) for t in triangles)/6
exact = np.pi*a**2*b + 4/3*np.pi*a**3
checks += [abs(abs(volume)/exact - 1) < 0.02, abs(points[:, 2].min() + a) < 1e-12, abs(points[:, 2].max() - (a + b)) < 1e-12]
checks += [len(graphics.ToPointsAndTrigs(graphics.Capsule(vAxis=[0, 0, 0], radius=a))[0]) > 0] #a sphere

if not testIsActive:
    SC.visualizationSettings.contact.show.spheres = True
    SC.visualizationSettings.contact.show.cylinders = True
    SC.visualizationSettings.contact.show.cones = True
    SC.visualizationSettings.contact.show.triangles = True
    mbs.SolutionViewer()

u = sum(checks) + sum(heights)
exu.Print('checks:', checks)
exu.Print('solution of capsulePrimitiveContactTest=', u)
exu.sys['testResult'] = u
