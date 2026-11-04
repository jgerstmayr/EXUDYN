#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The automatic step size of SolveTrapezoidalGGL of exudyn.symbolics.advancedSolvers (#3243) on two models
#           of changing time scales: a ball falling onto the ground with Hertz contact and Hunt-Crossley damping, free
#           flights and contacts of 10 ms, where the controlled step needs a tenth of the steps of a constant step for
#           a tenth of its error; and a chain of 10 mass points on distance constraints, fixed at one end and starting
#           horizontally, which falls and whips, against mbs.SolveDynamic with fine steps - the controlled step is as
#           good as a constant one there, its error follows the tolerance. The PI controller rejects few steps.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-04
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import NodePoint2D, MassPoint2D, MarkerNodePosition, MarkerBodyPosition, ObjectGround, ObjectConnectorDistance, LoadForceVector
from exudyn.symbolics.systemModel import NumpySystem
from exudyn.symbolics.advancedSolvers import SolveTrapezoidalGGL
import exudyn.cMath as cMath
import numpy as np
from scipy.integrate import solve_ivp

testError = 0
def Check(condition, message):
    global testError
    if not condition:
        exu.Print('symbolicTrapezoidalGGLStepSizeTest: ' + message)
        testError += 1

def Settings(steps, endTime, automatic=False, tolerance=1e-6):
    settings = exu.SimulationSettings()
    settings.timeIntegration.endTime = endTime
    settings.timeIntegration.numberOfSteps = steps
    settings.timeIntegration.automaticStepSize = automatic
    settings.timeIntegration.absoluteTolerance = tolerance
    settings.timeIntegration.relativeTolerance = tolerance
    settings.timeIntegration.newton.relativeTolerance = 1e-10
    settings.timeIntegration.newton.absoluteTolerance = 1e-12
    settings.timeIntegration.verboseMode = 0
    settings.solution.file.write = False
    settings.solution.file.writePeriod = endTime
    return settings

#a ball of radius 0.1 falling from 1 m: contact force k p^1.5 (1 - d v) for the penetration p, two contacts in 1.5 s
(radius, stiffness, damping, g) = (0.1, 1e6, 0.05, 9.81)
def BallForces(q, q_t, t):
    p = radius - q[1]
    f = np.array([0., -g])
    if p > 0:
        f[1] += stiffness*cMath.pow(p, 1.5)*(1. - damping*q_t[1])
    return f
ball = NumpySystem(np.eye(2), BallForces, initialState=([0., 1.], [0.2, 0.]))
endTime = 1.5
(q0, v0) = ball.InitialState()
reference = solve_ivp(lambda t, x: np.concatenate([x[2:], BallForces(x[:2], x[2:], t)]), [0, endTime], np.concatenate([q0, v0]),
                      method='DOP853', rtol=1e-12, atol=1e-12, max_step=1e-3).y[:2, -1]
constant = SolveTrapezoidalGGL(ball, Settings(4000, endTime))
errorConstant = np.max(np.abs(constant['q'][-1] - reference))
automatic = SolveTrapezoidalGGL(ball, Settings(10, endTime, automatic=True, tolerance=1e-6))
errorAutomatic = np.max(np.abs(automatic['q'][-1] - reference))
exu.Print('ball: 4000 constant steps, error', errorConstant, '; automatic step size, tolerance 1e-6:', automatic['steps'],
          'steps,', automatic['rejectedStepsError'], 'rejected, error', errorAutomatic)
Check(automatic['steps'] < 600 and errorAutomatic < errorConstant/5 and automatic['rejectedStepsError'] < 20,
      'the automatic step size of the ball')
u = automatic['steps'] + round(float(automatic['q'][-1][1]), 6)

#a chain of 10 mass points, length 1, horizontal, its first point on a distance constraint to the origin
(n, length, angle, mass) = (10, 1., 0., 0.1)
l = length/n
direction = np.array([cMath.cos(angle*np.pi/180), cMath.sin(angle*np.pi/180)])
positions0 = np.concatenate([(i + 1)*l*direction for i in range(n)])
def Differences(q):
    p = q.reshape(n, 2)
    return np.vstack([p[:1], p[1:] - p[:-1]])
def ChainJacobian(q, q_t, y, t):
    d = Differences(q)
    C = np.zeros((n, 2*n))
    for i in range(n):
        C[i, 2*i:2*i + 2] = d[i]
        if i:
            C[i, 2*i - 2:2*i] = -d[i]
    return C
gravity = np.tile([0., -mass*g], n)
chain = NumpySystem(mass*np.eye(2*n), lambda q, q_t, t: gravity, initialState=(positions0, np.zeros(2*n)),
                    constraints=lambda q, q_t, y, z, t: 0.5*(np.sum(Differences(q)**2, axis=1) - l**2),
                    constraintJacobian=ChainJacobian, numberOfConstraints=n)

#the same chain in Exudyn, with fine steps; the free end reaches 7.5 m/s at 0.46 s
SC = exu.SystemContainer()
mbs = SC.AddSystem()
previous = mbs.AddMarker(MarkerBodyPosition(bodyNumber=mbs.AddObject(ObjectGround()), localPosition=[0, 0, 0]))
for i in range(n):
    node = mbs.AddNode(NodePoint2D(referenceCoordinates=list(positions0[2*i:2*i + 2])))
    mbs.AddObject(MassPoint2D(mass=mass, nodeNumber=node))
    marker = mbs.AddMarker(MarkerNodePosition(nodeNumber=node))
    mbs.AddObject(ObjectConnectorDistance(markerNumbers=[previous, marker], distance=l))
    mbs.AddLoad(LoadForceVector(markerNumber=marker, loadVector=[0, -mass*g, 0]))
    previous = marker
mbs.Assemble()
endTime = 0.5
settings = Settings(80000, endTime)
settings.timeIntegration.generalizedAlpha.spectralRadius = 0.9 #undamped generalized-alpha is unstable with index 3
mbs.SolveDynamic(settings)
reference = np.array(mbs.systemData.GetODE2Coordinates()) + positions0
constant = SolveTrapezoidalGGL(chain, Settings(200, endTime))
errorConstant = np.max(np.abs(constant['q'][-1] - reference))
automatic = SolveTrapezoidalGGL(chain, Settings(10, endTime, automatic=True, tolerance=1e-5))
errorAutomatic = np.max(np.abs(automatic['q'][-1] - reference))
exu.Print('whipping chain: 200 constant steps, error', errorConstant, '; automatic step size, tolerance 1e-5:',
          automatic['steps'], 'steps,', automatic['rejectedStepsError'], 'rejected, error', errorAutomatic)
Check(automatic['steps'] < 300 and errorAutomatic < errorConstant and automatic['rejectedStepsError'] < 10, 'the whipping chain')
u += automatic['steps']

u = round(u, 6) + testError #rounded: the last bits of numpy differ between platforms
exu.Print('solution of symbolicTrapezoidalGGLStepSizeTest=', u)

exu.sys['testResult'] = u
