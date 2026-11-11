#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  LinearSolverType.PardisoMKL (#3431) and the workers that sleep during a factorization (#3428). A chain of
#           mass points on springs (an ODE system) and a chain of rigid bodies on revolute joints (index 3) are solved
#           with EigenSparse as the reference. Where MKL is installed (exu.special.solver.PardisoInfo()), both are
#           solved with PardisoMKL and compared with the reference; a residual tolerance of 0 forces the fallback to
#           SparseLU, which gives the reference and counts in special.solver.pardisoFallbacks, and pardisoFallback=False
#           raises instead. Without MKL, selecting PardisoMKL raises a ValueError that says how to install it. In any
#           case, the revolute chain is solved on 4 threads with the workers sleeping during each factorization
#           (special.solver.suspendWorkersDuringFactorization), which gives the reference. The test result is the
#           same with and without MKL. Run alone, it shows the revolute chain in the SolutionViewer.
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
import numpy as np
from exudyn import cMath #the math functions of the C++ code, the same bits on every platform
testIsActive = exu.sys.get('testIsActive', False) #run alone: the renderer, the solution file and the SolutionViewer

errors = 0 #the number of failed checks

def Check(condition, text):
    global errors
    if not condition:
        errors += 1
        exu.Print('pardisoSolverTest: FAILED:', text)

def Model(kind, n=10):
    """a chain of n mass points on springs ('springs') or of n rigid bodies on revolute joints ('revolute')"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.CreateGround()
    L = 0.5
    previous = oGround
    for i in range(n):
        if kind == 'springs':
            body = mbs.CreateMassPoint(referencePosition=[L*(i+1), 0, 0], mass=1, gravity=[0, -9.81, 0],
                                       graphicsDataList=[graphics.Sphere(radius=0.05, color=graphics.color.red)])
            mbs.CreateSpringDamper(itemNumbers=[previous, body], stiffness=1e4, damping=10)
        else:
            body = mbs.CreateRigidBody(referencePosition=[L*(i+0.5), 0, 0], inertia=InertiaCuboid(1000, [L, 0.05, 0.05]),
                                       gravity=[0, -9.81, 0],
                                       graphicsDataList=[graphics.Brick(size=[L, 0.05, 0.05], color=graphics.color.steelblue)])
            mbs.CreateRevoluteJoint(itemNumbers=[previous, body], position=[L*i, 0, 0], axis=[0, 0, 1],
                                    axisRadius=0.02, axisLength=0.08)
        previous = body
    sensor = mbs.AddSensor(SensorBody(bodyNumber=previous, storeInternal=True, outputVariableType=exu.OutputVariableType.Position))
    mbs.Assemble()
    return SC, mbs, sensor

def Solve(kind, solverType, numberOfThreads=1, show=False):
    """the position of the last body after 0.5 s, full Newton"""
    SC, mbs, sensor = Model(kind)
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 500
    simulationSettings.timeIntegration.endTime = 0.5
    simulationSettings.timeIntegration.newton.useModifiedNewton = False
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.linearSolver.solverType = solverType
    simulationSettings.parallel.numberOfThreads = numberOfThreads
    simulationSettings.solution.file.write = show
    simulationSettings.solution.sensors.writePeriod = 0.5
    mbs.SolveDynamic(simulationSettings)
    if show:
        mbs.SolutionViewer()
    return np.array(mbs.GetSensorValues(sensor))

special = exu.special.solver
settingsBefore = (special.pardisoResidualTolerance, special.pardisoFallback, special.suspendWorkersDuringFactorization,
                  special.suspendWorkersMinimumSize)

reference = {kind: Solve(kind, exu.LinearSolverType.EigenSparse) for kind in ['springs', 'revolute']}
exu.Print('pardisoSolverTest: reference positions', reference)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#PARDISO where MKL is installed, the refusal otherwise
info = special.PardisoInfo()
exu.Print('pardisoSolverTest: PardisoInfo', info)
if info['available']:
    for kind in ['springs', 'revolute']:
        position = Solve(kind, exu.LinearSolverType.PardisoMKL)
        Check(cMath.norm(position - reference[kind]) < 1e-10, kind + ': PardisoMKL against EigenSparse')

    fallbacksBefore = special.pardisoFallbacks
    special.pardisoResidualTolerance = 0. #every check fails: the fallback to SparseLU
    position = Solve('revolute', exu.LinearSolverType.PardisoMKL)
    Check(special.pardisoFallbacks > fallbacksBefore, 'the fallback is counted')
    Check(cMath.norm(position - reference['revolute']) < 1e-10, 'the fallback gives the reference')

    special.pardisoFallback = False #an error instead of the fallback
    try:
        Solve('revolute', exu.LinearSolverType.PardisoMKL)
        Check(False, 'pardisoFallback=False raises')
    except Exception as e:
        Check('pardisoFallback is False' in str(e), 'the error names special.solver.pardisoFallback')
    special.pardisoResidualTolerance, special.pardisoFallback = settingsBefore[0], settingsBefore[1]
else:
    try:
        Solve('springs', exu.LinearSolverType.PardisoMKL)
        Check(False, 'PardisoMKL without MKL raises')
    except ValueError as e:
        Check('pip install mkl' in str(e), 'the error says how to install MKL')
    Check(len(info['message']) > 0, 'PardisoInfo gives the reason')

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the workers sleep during each factorization
special.suspendWorkersDuringFactorization = True
special.suspendWorkersMinimumSize = 0
position = Solve('revolute', exu.LinearSolverType.EigenSparse, numberOfThreads=4)
Check(cMath.norm(position - reference['revolute']) < 1e-12, 'sleeping workers give the reference')
special.suspendWorkersDuringFactorization, special.suspendWorkersMinimumSize = settingsBefore[2], settingsBefore[3]

if not testIsActive:
    Solve('revolute', exu.LinearSolverType.PardisoMKL if info['available'] else exu.LinearSolverType.EigenSparse, show=True)

u = float(sum(reference['springs']) + sum(reference['revolute'])) + errors
exu.Print('pardisoSolverTest: failed checks:', errors)
exu.Print('solution of pardisoSolverTest=', u)
exu.sys['testResult'] = u
