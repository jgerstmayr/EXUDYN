#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  All contacts between primitive shapes at once (#3343): two spheres, two cylinders, two cones and two
#           polytopes (a cube and an icosahedron) on a table with borders, which is shaken - lifted, pushed sideways and
#           tilted in a smooth pulse of 0.2 s, so that the bodies jump - and then rests. The table top and the borders are
#           planes for the spheres, cylinders and cones (SphereTriangle, CylinderTriangle, ConeTriangle with
#           infinitePlane) and boxes for the polytopes (ConvexConvex); every pair of bodies whose shapes have a contact item
#           is coupled: sphere-sphere, sphere-cylinder, cylinder-cylinder, cone-cone, cylinder-cone (a cone of equal
#           radii) and the two polytopes. The checks: the solver finishes, no body leaves the table or sinks into it, the
#           bodies jump, and, run directly for 2 s, they come nearly to rest (spheres and cylinders roll on). As a test only the first 0.4 s are computed, through the
#           pulse and the first impacts. Run directly, the SolutionViewer shows the table with the contact forces.
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

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()
oGround = mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0, 0, -0.5], size=3)])

g = 9.81
Lx, Ly, wall, height = 1.0, 0.7, 0.05, 0.15 #inner size of the table, thickness and height of the borders
law = dict(contactStiffness=1e5, contactDamping=100, dynamicFriction=0.3, frictionProportionalZone=0.01, show=True)
checks = []

#the table, moved by a generic joint to the ground: a smooth pulse of 0.2 s from t0, up, sideways and tilted about x
oTable = mbs.CreateRigidBody(inertia=InertiaCuboid(density=1000, sideLengths=[Lx, Ly, 0.05]), referencePosition=[0, 0, 0],
                             graphicsDataList=[graphics.Brick(centerPoint=[0, 0, -0.025], size=[Lx + 2*wall, Ly + 2*wall, 0.05], color=graphics.color.lightgrey)]
                             + [graphics.Brick(centerPoint=[sx*(Lx + wall)/2, 0, height/2], size=[wall, Ly + 2*wall, height], color=graphics.color.grey) for sx in [-1, 1]]
                             + [graphics.Brick(centerPoint=[0, sy*(Ly + wall)/2, height/2], size=[Lx, wall, height], color=graphics.color.grey) for sy in [-1, 1]])
t0, T = 0.05, 0.2
amplitudes = [0.02, 0., 0.05, 0.12, 0., 0.] #x, y, z and the rotations about x, y, z

def Pulse(t):
    """the shape of the pulse, 0 outside [t0, t0 + T], and its derivative: sin^2 of a full period, smooth"""
    if t < t0 or t > t0 + T:
        return 0., 0.
    phase = np.pi*(t - t0)/T
    return cMath.sin(phase)**2, 2*cMath.sin(phase)*cMath.cos(phase)*np.pi/T

def UFoffset(mbs, t, itemNumber, offsetUserFunctionParameters):
    return [a*Pulse(t)[0] for a in amplitudes]

def UFoffset_t(mbs, t, itemNumber, offsetUserFunctionParameters):
    return [a*Pulse(t)[1] for a in amplitudes]

oJoint = mbs.CreateGenericJoint(itemNumbers=[oGround, oTable], position=[0, 0, 0], constrainedAxes=[1]*6)
mbs.SetObjectParameter(oJoint, 'offsetUserFunction', UFoffset)
mbs.SetObjectParameter(oJoint, 'offsetUserFunction_t', UFoffset_t)

#the planes of the table top and the borders, in the frame of the table, normal to the inside
def Plane(point, normal):
    t1, t2 = ComputeOrthonormalBasisVectors(normal)[1:]
    if np.dot(np.cross(t1, t2), normal) < 0:
        t1, t2 = t2, t1
    return [list(point), list(np.array(point) + t1), list(np.array(point) + t2)]
planes = [Plane([0, 0, 0], [0, 0, 1]), Plane([Lx/2, 0, 0], [-1, 0, 0]), Plane([-Lx/2, 0, 0], [1, 0, 0]),
          Plane([0, Ly/2, 0], [0, -1, 0]), Plane([0, -Ly/2, 0], [0, 1, 0])]
#the boxes of the table top and the borders, for the polytopes
def BoxPoints(center, size):
    return np.array([[center[0] + sx*size[0]/2, center[1] + sy*size[1]/2, center[2] + sz*size[2]/2] for sx in [-1, 1] for sy in [-1, 1] for sz in [-1, 1]])
boxTriangles = np.array([[0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3], [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6], [0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5]])
boxes = [BoxPoints([0, 0, -0.025], [Lx + 2*wall, Ly + 2*wall, 0.05])] \
    + [BoxPoints([sx*(Lx + wall)/2, 0, height/2], [wall, Ly + 2*wall, height]) for sx in [-1, 1]] \
    + [BoxPoints([0, sy*(Ly + wall)/2, height/2], [Lx, wall, height]) for sy in [-1, 1]]

def Body(inertia, position, graphicsList, rotation=np.eye(3)):
    return mbs.CreateRigidBody(inertia=inertia, referencePosition=position, referenceRotationMatrix=rotation, gravity=[0, 0, -g],
                               graphicsDataList=graphicsList)

#the bodies, resting on the table
r = 0.05
spheres = [Body(InertiaSphere(mass=0.5, radius=r), [x, -0.2, r], [graphics.Sphere(radius=r, color=graphics.color.orange, nTiles=16)]) for x in [-0.3, -0.18]]
rc, Lc = 0.05, 0.12 #cylinders lying along y
cylinders = [Body(InertiaCylinder(density=1000, length=Lc, outerRadius=rc, axis=1), [x, 0., rc],
                  [graphics.Cylinder(pAxis=[0, -Lc/2, 0], vAxis=[0, Lc, 0], radius=rc, color=graphics.color.steelblue, nTiles=24)]) for x in [-0.3, -0.12]]
cr0, cr1, Lk = 0.06, 0.03, 0.12 #cones standing on their bases
cones = [Body(InertiaCylinder(density=1000, length=Lk, outerRadius=0.5*(cr0 + cr1), axis=2), [x, 0.2, Lk/2],
              [graphics.SolidOfRevolution(pAxis=[0, 0, -Lk/2], vAxis=[0, 0, 1], contour=[[0, 0], [0, cr0], [Lk, cr1], [Lk, 0]],
                                          color=graphics.color.red, nTiles=24)]) for x in [-0.3, -0.15]]
a = 0.1
cubePoints = BoxPoints([0, 0, 0], [a]*3)
phi = 0.5*(1 + cMath.sqrt(5))
icoPoints = 0.06/cMath.sqrt(1 + phi*phi)*np.array([[0, s1, s2*phi] for s1 in [-1, 1] for s2 in [-1, 1]] + [[s1, s2*phi, 0] for s1 in [-1, 1] for s2 in [-1, 1]]
                                                    + [[s1*phi, 0, s2] for s1 in [-1, 1] for s2 in [-1, 1]])
#the 20 faces of the icosahedron: the triangles of points at the edge length from each other, outward
edge = min(cMath.sqrt((p - q) @ (p - q)) for p in icoPoints for q in icoPoints if (p - q) @ (p - q) > 0)
icoTriangles = []
for i in range(12):
    for j in range(i+1, 12):
        for k in range(j+1, 12):
            if all(abs(cMath.sqrt((icoPoints[m] - icoPoints[n]) @ (icoPoints[m] - icoPoints[n])) - edge) < 1e-12 for (m, n) in [(i, j), (j, k), (i, k)]):
                normal = np.cross(icoPoints[j] - icoPoints[i], icoPoints[k] - icoPoints[i])
                icoTriangles += [[i, j, k] if normal @ icoPoints[i] > 0 else [i, k, j]]
icoInRadius = cMath.sqrt(np.mean(icoPoints[icoTriangles[0]], axis=0) @ np.mean(icoPoints[icoTriangles[0]], axis=0))
icoRotation = RotationMatrixZ(0) #a face down: the rotation of the normal of the first face to -z
faceNormal = np.mean(icoPoints[icoTriangles[0]], axis=0)/icoInRadius
axis = np.cross(faceNormal, [0, 0, -1])
icoRotation = RotationVector2RotationMatrix(axis/cMath.sqrt(axis @ axis)*cMath.acos(-faceNormal[2]))
polytopes = [Body(InertiaCuboid(density=1000, sideLengths=[a]*3), [0.15, -0.15, a/2], [graphics.FromPointsAndTrigs(cubePoints, boxTriangles, color=graphics.color.green)]),
             Body(RigidBodyInertia(mass=0.6, inertiaTensor=np.diag([0.6*0.4*0.05**2]*3)), [0.15, 0.15, icoInRadius],
                  [graphics.FromPointsAndTrigs(icoPoints, icoTriangles, color=graphics.color.yellow)], rotation=icoRotation)]
shapes = [(cubePoints, boxTriangles), (icoPoints, np.array(icoTriangles))]

#the contacts with the table
for b in spheres:
    for p in planes:
        mbs.CreateSphereTriangleContact(itemNumbers=[b, oTable], sphereRadius=r, trianglePoints=p, infinitePlane=True, **law)
for b in cylinders:
    for p in planes:
        mbs.CreateCylinderTriangleContact(itemNumbers=[b, oTable], localAxis0=[0, 1, 0], cylinderRadius=rc, cylinderLength=Lc, trianglePoints=p,
                                          infinitePlane=True, **law)
for b in cones:
    for p in planes:
        mbs.CreateConeTriangleContact(itemNumbers=[b, oTable], coneRadii=[cr0, cr1], coneLength=Lk, trianglePoints=p, infinitePlane=True, **law)
for (b, (points, triangles)) in zip(polytopes, shapes):
    for box in boxes:
        mbs.CreateConvexConvexContact(itemNumbers=[b, oTable], points0=points, triangles0=triangles, points1=box, triangles1=boxTriangles, **law)
#the contacts between the bodies
mbs.CreateSphereSphereContact(itemNumbers=spheres, spheresRadii=[r, r], **law)
for s in spheres:
    for c in cylinders:
        mbs.CreateSphereCylinderContact(itemNumbers=[s, c], localAxis1=[0, 1, 0], sphereRadius=r, cylinderRadius=rc, cylinderLength=Lc, **law)
mbs.CreateCylinderCylinderContact(itemNumbers=cylinders, localAxis0=[0, 1, 0], localAxis1=[0, 1, 0], cylindersRadii=[rc, rc], cylindersLengths=[Lc, Lc], **law)
mbs.CreateConeConeContact(itemNumbers=cones, conesRadii0=[cr0, cr1], conesRadii1=[cr0, cr1], conesLengths=[Lk, Lk], **law)
for c in cylinders:
    for k in cones:
        mbs.CreateConeConeContact(itemNumbers=[c, k], localAxis0=[0, 1, 0], conesRadii0=[rc, rc], conesRadii1=[cr0, cr1], conesLengths=[Lc, Lk], **law)
mbs.CreateConvexConvexContact(itemNumbers=polytopes, points0=shapes[0][0], triangles0=shapes[0][1], points1=shapes[1][0], triangles1=shapes[1][1], **law)

bodies = spheres + cylinders + cones + polytopes
sensors = [mbs.AddSensor(SensorBody(bodyNumber=b, outputVariableType=exu.OutputVariableType.Position, storeInternal=True)) for b in bodies]
mbs.Assemble()
exu.Print('contact items:', sum(1 for i in range(mbs.systemData.NumberOfObjects()) if 'Contact' in mbs.GetObject(i)['objectType']))

tEnd = 0.4 if testIsActive else 2
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = int(tEnd*1000)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solution.sensors.writePeriod = 0.01
simulationSettings.solution.file.write = not testIsActive #for the SolutionViewer
simulationSettings.solution.file.writePeriod = 0.01
simulationSettings.timeIntegration.generalizedAlpha.checkInitialConstraints = False #the offset of the joint has no velocity level
try:
    mbs.SolveDynamic(simulationSettings, showHints=False)
    checks += [True]
except Exception as e:
    exu.Print("the solver failed at t =", mbs.systemData.GetTime(), ":", e)
    checks += [False]

#no body leaves the table or sinks into it: inside the borders and above the table top, in the frame of the table
lift = 0.
for (b, s) in zip(bodies, sensors):
    data = mbs.GetSensorStoredData(s)
    lift = max(lift, data[:, 3].max())
    p = mbs.GetObjectOutputBody(b, exu.OutputVariableType.Position)
    inside = abs(p[0]) < Lx/2 and abs(p[1]) < Ly/2 and 0 < p[2] < 0.3
    if not inside:
        exu.Print('body', b, 'lost at', p)
    checks += [inside]
exu.Print('the bodies rose up to', lift)
checks += [lift > 0.1] #they jumped
if not testIsActive: #at rest at the end
    velocity = max(cMath.norm(mbs.GetObjectOutputBody(b, exu.OutputVariableType.Velocity)) for b in bodies)
    exu.Print('the largest velocity at the end:', velocity)
    checks += [velocity < 0.05] #spheres and cylinders roll on without a rolling resistance

if not testIsActive:
    SC.visualizationSettings.contact.showContactForces = True
    SC.visualizationSettings.contact.contactForcesFactor = 0.01
    SC.visualizationSettings.connectors.show = False
    SC.visualizationSettings.openGL.light0.shadow = 0.3
    mbs.SolutionViewer()

u = sum(checks) + sum(mbs.GetObjectOutputBody(b, exu.OutputVariableType.Position)[2] for b in bodies)
exu.Print('checks:', checks)
exu.Print('solution of contactPrimitivesTableTest=', u)
exu.sys['testResult'] = u
exu.sys['testTolerance'] = 1e-7 #many impacts: the last digits depend on the platform
