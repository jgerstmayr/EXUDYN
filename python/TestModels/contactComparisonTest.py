#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Three contact objects, one drop: a ball of radius r falls onto the ground with the same
#           linear penalty law (contact stiffness k, damping d). ObjectContactCoordinate sees it as
#           one coordinate, ObjectContactSphereSphere as a hit on a large ground sphere,
#           ObjectContactSphereTriangle as a hit on a ground triangle - the motion is the same, so the
#           three give the same height z(t) (#2749), a cross-check of three implementations.
#           Measured: sphere and triangle agree to 1e-13; all three agree to 1e-10 until the ball
#           leaves the ground, the deepest point 0.0963898 against 0.0963927 of an independent RK4
#           integration of the same law; after the release the coordinate contact differs by 4e-5,
#           of first order in the step size, because its post Newton step recommends another step.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-29
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities

import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

r = 0.1         #radius of the ball
m = 1           #mass
k = 1e5         #contact stiffness
d = 100         #contact damping
g = 9.81
z0 = 0.2        #initial height of the center: falls 0.1 before contact
tEnd = 0.3      #one contact and the rebound
nSteps = 20000


def Drop(kind):
    """the height of the ball's center over time, for one of the three contact objects"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.AddObject(ObjectGround())
    if kind == 'coordinate':
        #the coordinate is the height; gap = q - offset
        node = mbs.AddNode(Node1D(referenceCoordinates=[0], initialCoordinates=[z0]))
        mbs.AddObject(ObjectMass1D(nodeNumber=node, physicsMass=m))
        mBall = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=0))
        mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=mbs.AddNode(NodePointGround()), coordinate=0))
        nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=1, initialCoordinates=[z0 - r]))
        mbs.AddObject(ObjectContactCoordinate(markerNumbers=[mGround, mBall], nodeNumber=nData,
                                              contactStiffness=k, contactDamping=d, offset=r))
        mbs.AddLoad(LoadCoordinate(markerNumber=mBall, load=-m*g))
        sensor = mbs.AddSensor(SensorNode(nodeNumber=node, outputVariableType=exu.OutputVariableType.Coordinates,
                                          storeInternal=True, writeToFile=False))
        component = 1
    else:
        node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.1, 0.1, z0] + eulerParameters0))
        ball = mbs.AddObject(ObjectRigidBody(nodeNumber=node, physicsMass=m,
                                             physicsInertia=InertiaSphere(mass=m, radius=r).GetInertia6D()))
        mBall = mbs.AddMarker(MarkerBodyRigid(bodyNumber=ball))
        mbs.AddLoad(LoadForceVector(markerNumber=mBall, loadVector=[0, 0, -m*g]))
        nData = mbs.AddNode(NodeGenericData(numberOfDataCoordinates=4, initialCoordinates=[0, 0, 0, 0]))
        if kind == 'sphere':
            R = 1   #a ground sphere whose top is at z=0, right below the ball
            mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0.1, 0.1, -R]))
            mbs.AddObject(ObjectContactSphereSphere(markerNumbers=[mGround, mBall], nodeNumber=nData,
                                                    spheresRadii=[R, r], contactStiffness=k, contactDamping=d))
        else:
            #the sphere is marker 0, the triangle marker 1
            mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
            mbs.AddObject(ObjectContactSphereTriangle(markerNumbers=[mBall, mGround], nodeNumber=nData, radiusSphere=r,
                                                      trianglePoints=exu.Vector3DList([[-1, -1, 0], [1, -1, 0], [0, 1, 0]]),
                                                      contactStiffness=k, contactDamping=d))
        sensor = mbs.AddSensor(SensorNode(nodeNumber=node, outputVariableType=exu.OutputVariableType.Position,
                                          storeInternal=True, writeToFile=False))
        component = 3
    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = nSteps
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solutionSettings.sensorsWritePeriod = tEnd/nSteps
    simulationSettings.solutionSettings.writeSolutionToFile = False
    mbs.SolveDynamic(simulationSettings)
    data = mbs.GetSensorStoredData(sensor)
    #on one time grid: the adaptive step control puts the sensor times of the release step at different places
    return np.interp(np.linspace(0, tEnd, nSteps+1), data[:, 0], data[:, component])


z = dict((kind, Drop(kind)) for kind in ['coordinate', 'sphere', 'triangle'])
t = np.linspace(0, tEnd, nSteps+1)
release = np.argmax((t > 0.14) & (z['coordinate'] > r))    #the ball leaves the ground again

exu.Print('deepest point   :', z['coordinate'].min(), z['sphere'].min(), z['triangle'].min(), '(RK4 of the same law: 0.0963927)')
exu.Print('sphere-triangle :', np.abs(z['sphere'] - z['triangle']).max())
exu.Print('until release   :', np.abs(z['coordinate'][:release] - z['sphere'][:release]).max())
exu.Print('after release   :', np.abs(z['coordinate'] - z['sphere']).max())

#the deepest points of the three and the heights at the end - one number that moves if any of the
#three changes
testResult = sum(z[kind].min() + z[kind][-1] for kind in z)
exu.Print('solution of contactComparisonTest=', testResult)
exu.sys['testResult'] = testResult
