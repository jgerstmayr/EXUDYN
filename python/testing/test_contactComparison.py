#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  Three contact objects, one drop: a ball of radius r falls onto the ground with the same
#           linear penalty law (contact stiffness k, damping d). ObjectContactCoordinate sees it as
#           one coordinate, ObjectContactSphereSphere as a hit on a large ground sphere,
#           ObjectContactSphereTriangle as a hit on a ground triangle - the motion is the same, so the
#           three must give the same height z(t) (#2749). A cross-check of three implementations.
#
# Usage:    pytest python/testing/test_contactComparison.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-29
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

import exudyn as exu
from exudyn.itemInterface import *                                          # noqa: F403
from exudyn.rigidBodyUtilities import InertiaSphere, eulerParameters0

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
    return (data[:, 0], data[:, component])


@pytest.fixture(scope='module')
def drops():
    """z(t) of the three, on one time grid: the adaptive step control puts the sensor times of the
    release step at different places"""
    grid = np.linspace(0, tEnd, nSteps+1)
    result = {}
    for kind in ['coordinate', 'sphere', 'triangle']:
        (time, z) = Drop(kind)
        result[kind] = np.interp(grid, time, z)
    result['t'] = grid
    return result


def test_sphereAndTriangleAgree(drops):
    #the same kind of contact computation: equal to round-off
    assert np.abs(drops['sphere'] - drops['triangle']).max() < 1e-10


def test_thePenetrationAgrees(drops):
    """up to the release of the first contact all three are equal; the deepest point agrees with an
    independent RK4 integration of the same 1D contact law (0.096393)"""
    (t, zCoordinate, zSphere) = (drops['t'], drops['coordinate'], drops['sphere'])
    release = np.argmax((t > 0.14) & (zCoordinate > r))    #the ball leaves the ground again
    assert np.abs(zCoordinate[:release] - zSphere[:release]).max() < 1e-10
    assert zCoordinate.min() == pytest.approx(0.09639, abs=1e-5)


def test_theReboundAgrees(drops):
    """after the release the step size control of the coordinate contact differs from the sphere
    contacts; the difference is of first order in the step size (6e-4 at 5000 steps, 4e-5 at 20000)"""
    assert np.abs(drops['coordinate'] - drops['sphere']).max() < 1e-4
