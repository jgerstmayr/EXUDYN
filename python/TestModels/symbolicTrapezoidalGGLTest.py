#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  SolveTrapezoidalGGL of exudyn.symbolics.advancedSolvers (#3242), the trapezoidal rule with the GGL
#           stabilization of index-3 constraints: a pendulum of a NumpySystem with constant steps, of order 2 in the
#           positions, velocities, accelerations and the multiplier, the constraints on position and velocity level
#           fulfilled, converging to the solution of SolveGeneralizedAlpha; a slider crank of rigid bodies on the joints
#           of UserItemCreator as SymbolicSystem, of order 2 against a fine solution of mbs.SolveDynamic; ODE1
#           coordinates of order 2; the step size controlled by the local error, its steps and error with two
#           tolerances; an algebraic equation of the velocities refused.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-04
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import InertiaCuboid
from exudyn.symbolics.userItems import UserItemCreator
from exudyn.symbolics.systemModel import SymbolicSystem, NumpySystem
from exudyn.symbolics.solvers import SolveGeneralizedAlpha
from exudyn.symbolics.advancedSolvers import SolveTrapezoidalGGL
import exudyn.cMath as cMath
import numpy as np

testError = 0
def Check(condition, message):
    global testError
    if not condition:
        exu.Print('symbolicTrapezoidalGGLTest: ' + message)
        testError += 1

def Settings(steps, endTime=1., automatic=False, tolerance=1e-6):
    settings = exu.SimulationSettings()
    settings.timeIntegration.endTime = endTime
    settings.timeIntegration.numberOfSteps = steps
    settings.timeIntegration.automaticStepSize = automatic
    settings.timeIntegration.absoluteTolerance = tolerance
    settings.timeIntegration.relativeTolerance = tolerance
    settings.timeIntegration.newton.relativeTolerance = 1e-12
    settings.timeIntegration.newton.absoluteTolerance = 1e-12
    settings.timeIntegration.verboseMode = 0
    settings.solution.file.write = False
    settings.solution.file.writePeriod = endTime
    return settings

def Order(errors):
    return [float(np.log2(e0/e1)) for (e0, e1) in zip(errors[0], errors[1])]

#a pendulum of a mass point, c = (q^T q - L^2)/2, released horizontally
L, g = 1., 9.81
pendulum = NumpySystem(np.eye(2), lambda q, q_t, t: np.array([0., -g]), initialState=([L, 0.], [0., 0.]),
                       constraints=lambda q, q_t, y, z, t: np.array([0.5*(q @ q - L**2)]),
                       constraintJacobian=lambda q, q_t, y, t: np.array([q]), numberOfConstraints=1)
reference = SolveTrapezoidalGGL(pendulum, Settings(1600))
names = ['q', 'q_t', 'q_tt', 'z']
errors = []
u = 0
for steps in [100, 200]:
    result = SolveTrapezoidalGGL(pendulum, Settings(steps))
    errors.append([np.max(np.abs(result[name][-1] - reference[name][-1])) for name in names])
    (q, v) = (result['q'][-1], result['q_t'][-1])
    Check(abs(0.5*(q @ q - L**2)) < 1e-12 and abs(q @ v) < 1e-12, 'the constraints of the pendulum')
    u += float(np.sum(np.abs(q)))
order = Order(errors)
exu.Print('pendulum: errors with 100 and 200 steps', errors[1], ', order of q, q_t, q_tt, z', order)
Check(min(order) > 1.9, 'the order of the pendulum: ' + str(order))
settings = Settings(1600)
settings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
generalizedAlpha = SolveGeneralizedAlpha(pendulum, settings)
difference = np.max(np.abs(generalizedAlpha['q'][-1] - reference['q'][-1]))
Check(difference < 2e-5, 'against SolveGeneralizedAlpha: ' + str(difference))

#a slider crank: crank and rod on revolute joints, a slider mass point on a spherical joint at the rod, its height fixed
SC = exu.SystemContainer()
mbs = SC.AddSystem()
creator = UserItemCreator(mbs)
ground = creator.CreateGround()
gravity = [0, -9.81, 0]
c = 0.2*np.sqrt(0.5) #the crank at 45 degrees
xSlider = c + np.sqrt(0.5**2 - c**2)
crank = creator.CreateRigidBody(inertia=InertiaCuboid(1000, [0.02, 0.2, 0.02]), referencePosition=[0.5*c, 0.5*c, 0], gravity=gravity,
                                nodeType=exu.NodeType.RotationRxyz)
rod = creator.CreateRigidBody(inertia=InertiaCuboid(1000, [0.5, 0.02, 0.02]), referencePosition=[0.5*(c + xSlider), 0.5*c, 0],
                              gravity=gravity, nodeType=exu.NodeType.RotationRxyz)
slider = creator.CreateMassPoint(referencePosition=[xSlider, 0, 0], mass=0.5, gravity=gravity)
creator.CreateRevoluteJoint(itemNumbers=[ground, crank], position=[0, 0, 0], axis=[0, 0, 1])
creator.CreateRevoluteJoint(itemNumbers=[crank, rod], position=[c, c, 0], axis=[0, 0, 1])
creator.CreateSphericalJoint(itemNumbers=[rod, slider], position=[xSlider, 0, 0])
creator.CreateCoordinateConstraint(itemNumbers=[slider, None], coordinates=[1, None])
creator.Record()
mbs.Assemble()
system = SymbolicSystem(mbs)
endTime = 0.25
settings = Settings(5000, endTime)
settings.timeIntegration.generalizedAlpha.spectralRadius = 1.
mbs.SolveDynamic(settings)
referenceCrank = [np.array(mbs.systemData.GetODE2Coordinates()), np.array(mbs.systemData.GetODE2Coordinates_t())]
errors = []
for steps in [25, 50]:
    result = SolveTrapezoidalGGL(system, Settings(steps, endTime))
    errors.append([np.max(np.abs(result[name][-1] - value)) for (name, value) in zip(['q', 'q_t'], referenceCrank)])
    constraints = system.AlgebraicEquations(result['q'][-1], result['q_t'][-1], None, None, endTime)
    Check(np.max(np.abs(constraints)) < 1e-12, 'the constraints of the slider crank')
    u += float(np.sum(np.abs(result['q'][-1])))
order = Order(errors)
exu.Print('slider crank: errors with 50 steps', errors[1], ', order of q, q_t', order)
Check(min(order) > 1.8, 'the order of the slider crank: ' + str(order))

#ODE1 coordinates: y_t = -y beside the pendulum, y = exp(-t)
withODE1 = NumpySystem(np.eye(2), lambda q, q_t, t, y: np.array([0., -g]), initialState=([L, 0.], [0., 0.]),
                       ode1RHS=lambda q, q_t, y, t: -y, initialStateODE1=[1.],
                       constraints=lambda q, q_t, y, z, t: np.array([0.5*(q @ q - L**2)]),
                       constraintJacobian=lambda q, q_t, y, t: np.array([q]), numberOfConstraints=1)
errorsODE1 = [abs(SolveTrapezoidalGGL(withODE1, Settings(steps))['y'][-1][0] - cMath.exp(-1.)) for steps in [50, 100]]
orderODE1 = float(np.log2(errorsODE1[0]/errorsODE1[1]))
Check(orderODE1 > 1.9, 'the order of the ODE1 coordinates: ' + str(orderODE1))

#the step size by the local error: smaller tolerances, more steps and a smaller error; the end time reached
adaptive = []
for tolerance in [1e-5, 1e-7]:
    result = SolveTrapezoidalGGL(pendulum, Settings(10, automatic=True, tolerance=tolerance))
    adaptive.append((result['steps'], result['rejectedStepsError'], np.max(np.abs(result['q'][-1] - reference['q'][-1]))))
    Check(abs(result['time'][-1] - 1.) < 1e-12, 'the end time of the adaptive steps')
exu.Print('automatic step size, tolerances 1e-5 and 1e-7: (steps, rejected, error)', adaptive)
ratio = adaptive[1][0]/adaptive[0][0]
Check(3 < ratio < 7 and adaptive[1][2] < adaptive[0][2]/10, 'the automatic step size: ' + str(adaptive))
u += adaptive[0][0] + adaptive[1][0]

#an algebraic equation of the velocities is refused
velocityConstraint = NumpySystem(np.eye(2), lambda q, q_t, t: np.array([0., -g]), initialState=([L, 0.], [0., 0.]),
                                 constraints=lambda q, q_t, y, z, t: np.array([q_t[0]]),
                                 constraintJacobian=lambda q, q_t, y, t: np.array([[1., 0.]]), numberOfConstraints=1)
try:
    SolveTrapezoidalGGL(velocityConstraint, Settings(10))
    Check(False, 'an equation of the velocities is not refused')
except ValueError:
    pass

u = round(u, 8) + testError #rounded: the last bits of numpy differ between platforms
exu.Print('solution of symbolicTrapezoidalGGLTest=', u)

exu.sys['testResult'] = u
