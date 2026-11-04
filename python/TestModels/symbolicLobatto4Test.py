#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  SolveImplicitRungeKutta of exudyn.symbolics.advancedSolvers (#3244, #3246, #3247), the Lobatto and Radau
#           methods for index-3 constraints: on a pendulum, LobattoIIIA4 and LobattoIIIC4-GGL of order 4 in the
#           positions and velocities, Radau3 of order 3, all of order 2 in the accelerations and the multiplier, the
#           constraints on both levels kept; the methods agree with fine steps, and the 2-stage Lobatto IIIA with GGL is
#           SolveTrapezoidalGGL; on a slider crank of rigid bodies on the joints of UserItemCreator, LobattoIIIA4 of order
#           4 against mbs.SolveDynamic with fine steps, and the automatic step size by the error estimate of the order of
#           the method, its error of the size of the tolerance; on a pendulum with a mass on a stiff spring at its bob,
#           LobattoIIIC4-GGL and Radau3 damp the fast oscillation of the spring at a step of h omega = 70, LobattoIIIA4
#           keeps it and the energy at a smaller step.
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
from exudyn.symbolics.advancedSolvers import SolveImplicitRungeKutta, SolveTrapezoidalGGL, implicitRungeKuttaMethods
import exudyn.cMath as cMath
import numpy as np

testError = 0
def Check(condition, message):
    global testError
    if not condition:
        exu.Print('symbolicLobatto4Test: ' + message)
        testError += 1

def Settings(steps, endTime=1., writePeriod=None):
    settings = exu.SimulationSettings()
    settings.timeIntegration.endTime = endTime
    settings.timeIntegration.numberOfSteps = steps
    settings.timeIntegration.automaticStepSize = False #for SolveTrapezoidalGGL
    settings.timeIntegration.newton.relativeTolerance = 1e-13
    settings.timeIntegration.newton.absoluteTolerance = 1e-14
    settings.timeIntegration.verboseMode = 0
    settings.solution.file.write = False
    settings.solution.file.writePeriod = endTime if writePeriod is None else writePeriod
    return settings

def Order(errors):
    return [round(float(np.log2(e0/e1)), 2) for (e0, e1) in zip(errors[0], errors[1])]

#a pendulum of a mass point, released horizontally
L, g = 1., 9.81
pendulum = NumpySystem(np.eye(2), lambda q, q_t, t: np.array([0., -g]), initialState=([L, 0.], [0., 0.]),
                       constraints=lambda q, q_t, y, z, t: np.array([0.5*(q @ q - L**2)]),
                       constraintJacobian=lambda q, q_t, y, t: np.array([q]), numberOfConstraints=1)
reference = SolveImplicitRungeKutta(pendulum, Settings(400), 'LobattoIIIA4')
difference = max(np.max(np.abs(SolveImplicitRungeKutta(pendulum, Settings(800), method)['q'][-1] - reference['q'][-1]))
                 for method in implicitRungeKuttaMethods)
Check(difference < 1e-7, 'the methods with fine steps (Radau3 of order 3): ' + str(difference))
u = 0
expected = {'LobattoIIIA4': [3.8, 3.8, 1.8, 1.8], 'LobattoIIIC4-GGL': [3.5, 3.8, 1.8, 1.8], 'Radau3': [2.8, 2.8, 1.8, 1.8]}
for variant in implicitRungeKuttaMethods:
    errors = []
    for steps in [25, 50]:
        result = SolveImplicitRungeKutta(pendulum, Settings(steps), variant)
        errors.append([np.max(np.abs(result[name][-1] - reference[name][-1])) for name in ['q', 'q_t', 'q_tt', 'z']])
        (q, v) = (result['q'][-1], result['q_t'][-1])
        Check(abs(0.5*(q @ q - L**2)) < 1e-12 and abs(q @ v) < 1e-12, 'the constraints of the pendulum, ' + variant)
    order = Order(errors)
    exu.Print('pendulum, ' + variant + ': errors of q, q_t, q_tt, z with 50 steps', ['%.2e' % e for e in errors[1]], ', orders', order,
              ', velocity constraint', q @ v)
    Check(all(o > e for (o, e) in zip(order, expected[variant])), 'the order of ' + variant + ': ' + str(order))
    u += float(np.sum(np.abs(q)))


#the 2-stage Lobatto IIIA with GGL, the trapezoidal rule
import exudyn.symbolics.advancedSolvers as advancedSolvers
difference = np.max(np.abs(advancedSolvers._SolveRungeKutta(pendulum, Settings(50), advancedSolvers._rungeKuttaTableaus['Trapezoidal'],
                                                            'GGL', 'test')['q'][-1]
                           - SolveTrapezoidalGGL(pendulum, Settings(50))['q'][-1]))
Check(difference < 1e-13, 'the 2-stage IIIA-GGL against SolveTrapezoidalGGL: ' + str(difference))

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
settings = Settings(20000, endTime)
settings.timeIntegration.generalizedAlpha.spectralRadius = 0.9
settings.timeIntegration.newton.relativeTolerance = 1e-10
mbs.SolveDynamic(settings)
referenceCrank = [np.array(mbs.systemData.GetODE2Coordinates()), np.array(mbs.systemData.GetODE2Coordinates_t())]
errors = []
for steps in [8, 16]:
    result = SolveImplicitRungeKutta(system, Settings(steps, endTime), 'LobattoIIIA4')
    errors.append([np.max(np.abs(result[name][-1] - value)) for (name, value) in zip(['q', 'q_t'], referenceCrank)])
    u += float(np.sum(np.abs(result['q'][-1])))
order = Order(errors)
exu.Print('slider crank, LobattoIIIA4: errors of q, q_t with 16 steps', errors[1], ', orders', order)
Check(min(order) > 3.7, 'the order of the slider crank: ' + str(order))

#the automatic step size by the error of the quadrature against the polynomial through the stages of two steps, of
#the order of the method (#3250): the error is of the size of the tolerance
settings = Settings(4, endTime)
settings.timeIntegration.automaticStepSize = True
settings.timeIntegration.absoluteTolerance = 1e-6
settings.timeIntegration.relativeTolerance = 1e-6
result = SolveImplicitRungeKutta(system, settings, 'LobattoIIIA4')
error = np.max(np.abs(result['q'][-1] - referenceCrank[0]))
exu.Print('slider crank, LobattoIIIA4, automatic step size, tolerance 1e-6:', result['steps'], 'steps,', result['rejectedStepsError'],
          'rejected, error', error)
Check(error < 5e-6 and result['steps'] < 20 and abs(result['time'][-1] - endTime) < 1e-12, 'the automatic step size')
u += result['steps']
for method in implicitRungeKuttaMethods: #the pendulum: the steps follow the order of the method
    steps = []
    for tolerance in [1e-5, 1e-7]:
        settings = Settings(4)
        settings.timeIntegration.automaticStepSize = True
        settings.timeIntegration.absoluteTolerance = tolerance
        settings.timeIntegration.relativeTolerance = tolerance
        result = SolveImplicitRungeKutta(pendulum, settings, method)
        steps.append(result['steps']) #the local errors of about the tolerance add up over the steps:
        error = np.max(np.abs(result['q'][-1] - reference['q'][-1]))
        Check(error < 10*result['steps']*tolerance, 'the automatic step size of ' + method + ', tolerance ' + str(tolerance) + ': error ' + str(error))
    localOrder = 2*cMath.log(10.)/cMath.log(steps[1]/steps[0]) #the steps grow by 100^(1/(p+1)) for a local error of order p+1
    exu.Print('pendulum, ' + method + ', automatic step size, tolerances 1e-5 and 1e-7:', steps, 'steps, local order', localOrder)
    Check(localOrder > (3.3 if method == 'Radau3' else 3.8), 'the steps of the automatic step size of ' + method + ': ' + str(steps))

#a pendulum with a mass on a stiff linear spring at its bob, displaced by 1 mm: a fast mode of omega = 1414/s
stiffness = 1e6
def Forces(q, q_t, t):
    f = stiffness*(q[2:] - q[:2])
    return np.array([0., -g, 0., -g]) + np.concatenate([f, -f])
stiff = NumpySystem(np.eye(4), Forces, initialState=([L, 0., L + 0.001, 0.], [0., 0., 0., 0.]),
                    constraints=lambda q, q_t, y, z, t: np.array([0.5*(q[:2] @ q[:2] - L**2)]),
                    constraintJacobian=lambda q, q_t, y, t: np.array([[q[0], q[1], 0., 0.]]), numberOfConstraints=1)
def Energy(q, v):
    d = q[2:] - q[:2]
    return 0.5*v @ v + g*(q[1] + q[3]) + 0.5*stiffness*(d @ d)
energy0 = Energy(*stiff.InitialState())
settings = Settings(40, 2.)
settings.timeIntegration.newton.relativeTolerance = 1e-8 #the residual of the stiff spring is not below 1e-13
settings.timeIntegration.newton.absoluteTolerance = 1e-10
for method in ['LobattoIIIC4-GGL', 'Radau3']:
    result = SolveImplicitRungeKutta(stiff, settings, method) #h omega = 70
    stretch = cMath.norm(result['q'][-1][2:] - result['q'][-1][:2])
    energyLost = energy0 - Energy(result['q'][-1], result['q_t'][-1])
    exu.Print('stiff spring, ' + method + ', h = 0.05: the stretch of the spring', stretch, '(static', g/stiffness,
              '), the energy lost', energyLost)
    Check(stretch < 1e-4 and 0.49 < energyLost < 0.7, method + ' damps the fast mode') #Radau3 also damps the pendulum by 0.09
settings.timeIntegration.numberOfSteps = 200
result = SolveImplicitRungeKutta(stiff, settings, 'LobattoIIIA4') #h omega = 14
energyChange = Energy(result['q'][-1], result['q_t'][-1]) - energy0
exu.Print('stiff spring, LobattoIIIA4, h = 0.01: the energy changed by', energyChange)
Check(abs(energyChange) < 1e-3, 'LobattoIIIA4 keeps the energy')

u = round(u, 8) + testError #rounded: the last bits of numpy differ between platforms
exu.Print('solution of symbolicLobatto4Test=', u)

exu.sys['testResult'] = u
