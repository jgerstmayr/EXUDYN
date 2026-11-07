#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The contact inside a hollow cylinder and a hollow cone (#3334), isHollowCylinder1 of
#           ObjectContactCylinderCylinder and isHollowCone1 of ObjectContactConeCone: three bodies, each inside its fixed
#           hollow shape, which show the contact points: a shaft lying in a horizontal bore rests on the lowest points of
#           its two rims (its center R - r + m g/(2k) below the axis of the bore); a long shaft through a short ring rests
#           on the two edges of the ring, the same; a truncated cone in a conical seat of the same taper rests on its
#           two rims all around (6 points), lower than where its surface fits the seat by m g (1 + R'^2)/(6 k R'^2) with
#           the slope R' of the radii; and the refusal of an inner shape that does not fit. Run directly, the
#           SolutionViewer shows the three bodies in their hollow shapes. The result is the sum of the checks and of the
#           final heights.
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

k, d = 1e5, 1e3     #contact stiffness and damping
g = 9.81
checks = []

def Body(position, radius, length, axisHT, color):
    """a rigid body with the inertia of a cylinder, its contact shape along the local z axis of axisHT"""
    inertia = InertiaCylinder(density=1000, length=length, outerRadius=radius, axis=2).Rotated(axisHT[0:3, 0:3])
    node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=list(position) + eulerParameters0))
    body = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=inertia.Mass(), inertia=inertia.GetInertia6D()))
    mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=body)), loadVector=[0, 0, -g]))
    return node, inertia.Mass(), mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localHT=axisHT))

def DataNode():
    return mbs.AddNode(NodeGenericData(numberOfDataCoordinates=24, initialCoordinates=[1, 0, 0]*8)) #apart at the start

law = dict(contactStiffness=k, contactDamping=d)
alongX = HTrotateY(0.5*np.pi)

#a shaft in a bore along x
r, R = 0.05, 0.06
nShaft, mShaft, marker = Body([0, 0, 0], r, 0.4, alongX, graphics.color.orange)
mBore = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localHT=alongX))
mbs.AddObject(ObjectContactCylinderCylinder(markerNumbers=[marker, mBore], nodeNumber=DataNode(), cylindersRadii=[r, R],
                                            cylindersLengths=[0.4, 0.5], isHollowCylinder1=True,
                                            visualization=VObjectContactCylinderCylinder(show=True), **law))
#a long shaft through a short ring
nLong, mLong, marker = Body([0, 0.5, 0], r, 0.6, alongX, graphics.color.steelblue)
mRing = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localHT=HTtranslate([0, 0.5, 0]) @ alongX))
mbs.AddObject(ObjectContactCylinderCylinder(markerNumbers=[marker, mRing], nodeNumber=DataNode(), cylindersRadii=[r, R],
                                            cylindersLengths=[0.6, 0.1], isHollowCylinder1=True,
                                            visualization=VObjectContactCylinderCylinder(show=True), **law))
#a truncated cone in a conical seat of the same taper, axis vertical
cone, seat, L = [0.05, 0.1], [0.04, 0.09], 0.2
slope = (cone[1] - cone[0])/L
zFit = (cone[0] - seat[0])/slope #where the surface of the cone fits the seat
nCone, mCone, marker = Body([0, 1.0, zFit + 0.002], 0.5*(cone[0] + cone[1]), L, np.eye(4), graphics.color.green)
mSeat = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0, 1.0, 0]))
mbs.AddObject(ObjectContactConeCone(markerNumbers=[marker, mSeat], nodeNumber=DataNode(), conesRadii0=cone, conesRadii1=seat,
                                    conesLengths=[L, L], isHollowCone1=True, visualization=VObjectContactConeCone(show=True), **law))
mbs.Assemble()

#an inner cylinder that does not fit is refused at Assemble
mbs2 = exu.SystemContainer().AddSystem()
g2 = mbs2.CreateGround()
n2 = mbs2.AddNode(NodeRigidBodyEP(referenceCoordinates=[0, 0, 0] + eulerParameters0))
b2 = mbs2.AddObject(ObjectRigidBody(nodeNumber=n2, mass=1, inertia=[1, 1, 1, 0, 0, 0]))
mbs2.AddObject(ObjectContactCylinderCylinder(markerNumbers=[mbs2.AddMarker(MarkerBodyRigid(bodyNumber=b2)), mbs2.AddMarker(MarkerBodyRigid(bodyNumber=g2))],
                                             nodeNumber=mbs2.AddNode(NodeGenericData(numberOfDataCoordinates=24, initialCoordinates=[0]*24)),
                                             cylindersRadii=[0.07, R], cylindersLengths=[0.4, 0.5], isHollowCylinder1=True, **law))
try:
    mbs2.Assemble()
    refused = False
except Exception as e:
    refused = 'must not be larger' in str(e)
exu.Print('a cylinder larger than the bore is refused:', refused)
checks += [refused]

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 2000
simulationSettings.timeIntegration.endTime = 1
simulationSettings.solution.file.write = not testIsActive #for the SolutionViewer
mbs.SolveDynamic(simulationSettings)

heights = [mbs.GetNodeOutput(n, exu.OutputVariableType.Position)[2] for n in [nShaft, nLong, nCone]]
expected = [-(R - r) - mShaft*g/(2*k), -(R - r) - mLong*g/(2*k), zFit - mCone*g*(1 + slope**2)/(6*k*slope**2)]
exu.Print('heights', heights, '\n expected', expected)
checks += [abs(h - e) < 1e-6 for h, e in zip(heights, expected)]

if not testIsActive:
    SC.visualizationSettings.contact.show.cylinders = True
    SC.visualizationSettings.contact.show.cones = True
    mbs.SolutionViewer()

u = sum(checks) + sum(heights)
exu.Print('checks:', checks)
exu.Print('solution of hollowContactTest=', u)
exu.sys['testResult'] = u
