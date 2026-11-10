#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The viscous rolling friction of ObjectConnectorRollingDiscPenalty for any orientation of the disc axis
#           (#926): a coin, tilted by 0.4 rad against the ground and turning about the plane normal. The output
#           variable Velocity, the velocity of the trail, is the time derivative of the contact point, which the
#           rotation of the disc about its axis does not move (a central difference); the rolling friction adds the force
#           -mu_r |f_n| v_r on the disc, v_r the trail velocity in the plane, which for the tilted coin differs from
#           the velocity of its center; for an upright disc both are the same. Then the tilted coin rolls on a circle
#           for 1 s with and without rolling friction, which slows it down.
#           The result is the number of checks that pass plus the speed of the coin at the end.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-11
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics
from exudyn import cMath #the math functions of the C++ code, the same bits on every platform
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

testError = 0
numberOfChecks = 0
def Check(condition, message):
    global testError, numberOfChecks
    numberOfChecks += 1
    if not condition:
        exu.Print('rollingDiscRollingFrictionTest: ' + message)
        testError += 1

r = 0.2       #radius of the coin
mass = 2.
muRolling = 0.5
g = [0, 0, -9.81]
inertia = InertiaCylinder(density=mass/(np.pi*r**2*0.02), length=0.02, outerRadius=r, axis=0)

def Coin(tilt, omegaYaw, speed, rollingFriction, penetration=0.):
    """a coin with axis x, tilted about its forward axis y by tilt, rolling forward with speed and turning about z;
    returns [mbs, SC, oRolling, nCoin]"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.CreateGround(graphicsDataList=[graphics.CheckerBoard(point=[0, 0, 0], size=4, nTiles=8)])
    A = np.array(RotationMatrixY(-tilt)) #the axis w1 = A[:,0] tilted out of the plane: a rotation about the forward direction y
    w1 = A[:, 0]
    w3 = np.cross(w1, np.cross(w1, [0, 0, 1])/cMath.norm(np.cross(w1, [0, 0, 1]))) #from the center to the contact point
    center = -r*w3 + np.array([0, 0, -penetration])
    omega = speed/r*(-w1) + np.array([0, 0, omegaYaw]) #rolling forward in +y, turning about z
    vCenter = np.cross(omega, center - (center + r*w3)) #rolling without slip about the contact point
    body = mbs.CreateRigidBody(inertia=inertia, nodeType=exu.NodeType.RotationEulerParameters,
                               referencePosition=list(center), referenceRotationMatrix=A,
                               initialVelocity=list(vCenter), initialAngularVelocity=list(omega), gravity=g,
                               graphicsDataList=[graphics.Cylinder(pAxis=[-0.01, 0, 0], vAxis=[0.02, 0, 0], radius=r,
                                                                   color=graphics.color.orange, nTiles=32),
                                                 graphics.Brick(size=[0.022, 0.3*r, 0.3*r], color=graphics.color.grey)],
                               returnDict=True)
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
    mCoin = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body['bodyNumber']))
    #the data coordinates: slip velocities and the gap of the last discontinuous iteration, which switches the contact on
    nData = mbs.AddNode(NodeGenericData(initialCoordinates=[0, 0, -penetration], numberOfDataCoordinates=3))
    oRolling = mbs.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[mGround, mCoin], nodeNumber=nData,
                                                               discRadius=r, discAxis=[1, 0, 0], planeNormal=[0, 0, 1],
                                                               dryFrictionProportionalZone=1e-2, dryFriction=[0.8, 0.8],
                                                               contactStiffness=1e5, contactDamping=1e3,
                                                               rollingViscousFriction=rollingFriction))
    mbs.Assemble()
    return [mbs, SC, oRolling, body['nodeNumber']]

def RHS(mbs):
    simulationSettings = exu.SimulationSettings()
    simulationSettings.solution.file.write = False
    solver = exu.MainSolverImplicitSecondOrder()
    solver.InitializeSolver(mbs, simulationSettings)
    solver.ComputeODE2RHS(mbs)
    return np.array(solver.GetSystemResidual()[0:mbs.systemData.ODE2Size()])

#%%+++ the trail velocity and the force, at a state with contact
for (tilt, case) in [(0.4, 'tilted coin'), (0., 'upright disc')]:
    [mbs, SC, oRolling, nCoin] = Coin(tilt, omegaYaw=3., speed=1., rollingFriction=0., penetration=1e-4)
    vTrail = np.array(mbs.GetObjectOutput(oRolling, exu.OutputVariableType.Velocity))
    vCenter = np.array(mbs.GetNodeOutput(nCoin, exu.OutputVariableType.Velocity))
    fn = mbs.GetObjectOutput(oRolling, exu.OutputVariableType.ForceLocal)[2]

    #the trail: the motion of the contact point, which the spin of the coin about its axis does not move
    q = np.array(mbs.systemData.GetODE2Coordinates())
    q_t = np.array(mbs.systemData.GetODE2Coordinates_t())
    dt = 1e-6
    positions = []
    for sign in [1, -1]:
        mbs.systemData.SetODE2Coordinates(q + sign*dt*q_t)
        positions += [np.array(mbs.GetObjectOutput(oRolling, exu.OutputVariableType.Position))]
    mbs.systemData.SetODE2Coordinates(q)
    vDifference = (positions[0] - positions[1])/(2*dt)
    Check(cMath.norm(vDifference - vTrail) < 1e-6, case + ': the trail velocity against the derivative of the contact point: '
          + str(vDifference) + ' / ' + str(vTrail))

    rhs0 = RHS(mbs)
    mbs.SetObjectParameter(oRolling, 'rollingViscousFriction', muRolling)
    fRolling = (RHS(mbs) - rhs0)[0:3]
    vPlane = vTrail - vTrail[2]*np.array([0, 0, 1])
    Check(cMath.norm(fRolling + muRolling*abs(fn)*vPlane) < 1e-10*abs(fn), case + ': the rolling friction force: '
          + str(fRolling) + ' / ' + str(-muRolling*abs(fn)*vPlane))
    vCenterPlane = vCenter - vCenter[2]*np.array([0, 0, 1])
    exu.Print('rollingDiscRollingFrictionTest:', case, ': trail velocity', vPlane, ', center velocity', vCenterPlane)
    if tilt == 0.:
        Check(cMath.norm(vPlane - vCenterPlane) < 1e-12, case + ': the trail moves as the center')
    else:
        Check(cMath.norm(vPlane - vCenterPlane) > 0.1, case + ': the trail moves unlike the center')

#%%+++ the tilted coin rolling on a circle, slowed down by the rolling friction
speeds = []
for rollingFriction in [0., 0.05]:
    [mbs, SC, oRolling, nCoin] = Coin(0.4, omegaYaw=3., speed=1., rollingFriction=rollingFriction)
    h = 1e-3
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 1000
    simulationSettings.timeIntegration.endTime = 1000*h
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solution.file.write = rollingFriction != 0 and not testIsActive
    simulationSettings.solution.file.writePeriod = 0.01
    mbs.SolveDynamic(simulationSettings)
    speeds += [cMath.norm(mbs.GetNodeOutput(nCoin, exu.OutputVariableType.Velocity))]
exu.Print('rollingDiscRollingFrictionTest: speed after 1 s without / with rolling friction:', speeds)
Check(speeds[1] < 0.95*speeds[0], 'the rolling friction slows the coin down: ' + str(speeds))

if not testIsActive:
    SC.visualizationSettings.connectors.show = True
    mbs.SolutionViewer()

u = numberOfChecks - 10*testError + speeds[1]
exu.Print('rollingDiscRollingFrictionTest: errors', testError)
exu.Print('solution of rollingDiscRollingFrictionTest=', u)
exu.sys['testResult'] = u
