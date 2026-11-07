#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ObjectContactConeCone and ObjectContactConeTriangle (#3332): the Distance of deactivated contacts of a tilted
#           truncated cone against exudyn.cUtilities.Distance; then five truncated cones side by side, each in its own
#           contact, which show the contact points: standing on its larger cap on the infinite plane and upside down
#           on its smaller cap on a triangle, on 3 points of the rim (L/2 - m g/(3k)); lying on its side line on the
#           plane, on the 2 ends of the line (r_m cos(alpha), with the mean radius r_m and the half angle alpha, less
#           the sinking of the line at the center by the penetrations that balance the weight); standing on a fixed
#           truncated cone; lying on its side line on a fixed cylinder, the line contact of two cones. Run directly,
#           the SolutionViewer shows the five cones. The result is the sum of the checks and of the final heights.
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
from exudyn import cMath
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()
oGround = mbs.CreateGround()

k, d = 1e5, 1e3     #contact stiffness and damping
g = 9.81
r0, r1, L = 0.1, 0.05, 0.2  #the radii at z = -L/2 and z = +L/2 and the length of the moving cones
alpha = cMath.atan((r0 - r1)/L) #the half angle of the cone
checks = []

def Cone(position, rotation=np.eye(3)):
    """a rigid truncated cone, its axis the local z axis (the inertia of a cylinder of the mean radius is enough here)"""
    inertia = InertiaCylinder(density=1000, length=L, outerRadius=0.5*(r0 + r1), axis=2)
    node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=list(position) + list(RotationMatrix2EulerParameters(rotation))))
    body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D()))
    mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=body)), loadVector=[0, 0, -g]))
    return node, inertia.Mass(), mbs.AddMarker(MarkerBodyRigid(bodyNumber=body))

def Law(active=True):
    return dict(nodeNumber=mbs.AddNode(NodeGenericData(numberOfDataCoordinates=12, initialCoordinates=[1, 0, 0]*4)),
                contactStiffness=k, contactDamping=d, dynamicFriction=0.3, activeConnector=active)

plane = exu.Vector3DList([[-5, -5, 0], [5, -5, 0], [0, 5, 0]])
mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))

#the Distance of deactivated contacts: a tilted cone against a fixed cone and against a triangle
rotation = RotationMatrixX(0.5) @ RotationMatrixY(0.3)
mFixedCone = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 10, 0]))
trianglePoints = [[-0.5, 9.6, -0.1], [0.5, 9.7, 0.], [0., 10.5, -0.05]]
items = []
for position in [[0.1, 10.05, 0.32], [0.05, 9.95, 0.2], [0.2, 10.1, 0.15]]:
    node, _, marker = Cone(position, rotation)
    cc = mbs.AddObject(ObjectContactConeCone(markerNumbers=[marker, mFixedCone], conesRadii0=[r0, r1], conesRadii1=[0.3, 0.2],
                                             conesLengths=[L, 0.2], **Law(active=False)))
    ct = mbs.AddObject(ObjectContactConeTriangle(markerNumbers=[marker, mGround], coneRadii=[r0, r1], coneLength=L,
                                                 trianglePoints=exu.Vector3DList(trianglePoints), **Law(active=False)))
    items += [(cc, ct, position)]
mbs.Assemble()
errorDistance = 0.
cone, fixedCone = ConvexShape('Cone', radius=r0, radius1=r1, length=L), ConvexShape('Cone', radius=0.3, radius1=0.2, length=0.2)
triangle = ConvexShape('Triangle', points=trianglePoints)
for (cc, ct, position) in items:
    H0 = HomogeneousTransformation(rotation, position)
    errorDistance = max(errorDistance,
                        abs(mbs.GetObjectOutput(cc, exu.OutputVariableType.Distance) - Distance(cone, H0, fixedCone, HTtranslate([0, 10, 0]))[0]),
                        abs(mbs.GetObjectOutput(ct, exu.OutputVariableType.Distance) - Distance(cone, H0, triangle, np.eye(4))[0]))
exu.Print('Distance of the deactivated contacts against cUtilities.Distance:', errorDistance)
checks += [errorDistance < 1e-14]

#five cones side by side
nStand, mStand, marker = Cone([0, 0, 0.5*L + 0.01])
mbs.AddObject(ObjectContactConeTriangle(markerNumbers=[marker, mGround], coneRadii=[r0, r1], coneLength=L, trianglePoints=plane,
                                        infinitePlane=True, visualization=VObjectContactConeTriangle(show=True), **Law()))
nUpside, mUpside, marker = Cone([0, 0.6, 0.5*L + 0.01], RotationMatrixX(np.pi))
mbs.AddObject(ObjectContactConeTriangle(markerNumbers=[marker, mGround], coneRadii=[r0, r1], coneLength=L,
                                        trianglePoints=exu.Vector3DList([[-0.3, 0.3, 0], [0.3, 0.3, 0], [0, 0.9, 0]]),
                                        visualization=VObjectContactConeTriangle(show=True), **Law()))
rm = 0.5*(r0 + r1)
nLying, mLying, marker = Cone([0, 1.2, rm*cMath.cos(alpha) + 0.005], RotationMatrixY(0.5*np.pi + alpha)) #its lowest side line horizontal
mbs.AddObject(ObjectContactConeTriangle(markerNumbers=[marker, mGround], coneRadii=[r0, r1], coneLength=L, trianglePoints=plane,
                                        infinitePlane=True, visualization=VObjectContactConeTriangle(show=True), **Law()))
nOnCone, mOnCone, marker = Cone([0, 1.8, 0.1 + 0.5*L + 0.01])
mbs.AddObject(ObjectContactConeCone(markerNumbers=[marker, mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 1.8, 0]))],
                                    conesRadii0=[r0, r1], conesRadii1=[0.3, 0.2], conesLengths=[L, 0.2],
                                    visualization=VObjectContactConeCone(show=True), **Law()))
rC = 0.15 #a fixed cylinder along x, as a cone of equal radii
nOnCylinder, mOnCylinder, marker = Cone([0, 2.4, rC + rm*cMath.cos(alpha) + 0.005], RotationMatrixY(0.5*np.pi + alpha))
mbs.AddObject(ObjectContactConeCone(markerNumbers=[marker, mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localHT=HTtranslate([0, 2.4, 0]) @ HTrotateY(0.5*np.pi)))],
                                    conesRadii0=[r0, r1], conesRadii1=[rC, rC], conesLengths=[L, 1.],
                                    visualization=VObjectContactConeCone(show=True), **Law()))
mbs.Assemble()

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 2000
simulationSettings.timeIntegration.endTime = 1
simulationSettings.solution.file.write = not testIsActive #for the SolutionViewer
mbs.SolveDynamic(simulationSettings)

heights = [mbs.GetNodeOutput(n, exu.OutputVariableType.Position)[2] for n in [nStand, nUpside, nLying, nOnCone, nOnCylinder]]
#a cone on its side line rests on the 2 ends of the line, at x1 and x2 from its center: their penetrations balance the
#moment of the weight about the center, so that the line sinks at the center by m g (x1^2 + x2^2)/(k (x2 - x1)^2)
x1 = -r0*cMath.sin(alpha) - 0.5*L*cMath.cos(alpha)
x2 = -r1*cMath.sin(alpha) + 0.5*L*cMath.cos(alpha)
def Sink(m):
    return m*g*(x1**2 + x2**2)/(k*(x2 - x1)**2)
expected = [0.5*L - mStand*g/(3*k), 0.5*L - mUpside*g/(3*k), rm*cMath.cos(alpha) - Sink(mLying), 0.1 + 0.5*L - mOnCone*g/(3*k),
            rC + rm*cMath.cos(alpha) - Sink(mOnCylinder)]
exu.Print('heights', heights, '\n expected', expected)
checks += [abs(h - e) < 1e-6 for h, e in zip(heights, expected)]

if not testIsActive:
    SC.visualizationSettings.contact.show.cones = True
    SC.visualizationSettings.contact.show.triangles = True
    SC.visualizationSettings.contact.color.triangles = [0.6, 0.6, 0.6, 0.5]
    mbs.SolutionViewer()

u = sum(checks) + sum(heights)
exu.Print('checks:', checks)
exu.Print('solution of coneContactTest=', u)
exu.sys['testResult'] = u
