#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A simulation continued from its restart file computes what the uninterrupted one computes
#           (#2850). A double pendulum of rigid bodies with revolute joints (algebraic coordinates,
#           Lie group nodes) and a mass point in sphere-sphere contact (data coordinates)
#           runs to t = 1 in one go, and again stopped by a post-step function at t = 0.6 and started
#           a second time with solution.restart.continueIfAvailable, after the restart file of t = 0.6
#           was replaced by the one of t = 0.4 - as if the job had been killed after 0.4. The rows of
#           the solution and sensor files and the final state are identical to the uninterrupted run;
#           the same for the explicit RK44 solver, with springs instead of the joints. A restart file of another system raises; without a
#           file the simulation starts at its start time.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import numpy as np
import os
import shutil

testIsActive = exu.sys.get('testIsActive', False)

directory = 'solution/restartFileTest/'
errors = 0
def Check(condition, text):
    global errors
    if not condition:
        errors += 1
        exu.Print('restartFileTest failed:', text)

def Model(explicit, addNode=False):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    oGround = mbs.CreateGround()
    inertia = InertiaCuboid(density=1000, sideLengths=[0.4,0.05,0.05])
    nodeType = exu.NodeType.RotationRotationVector if explicit else exu.NodeType.RotationEulerParameters
    b0 = mbs.CreateRigidBody(inertia=inertia, referencePosition=[0.2,0,0], gravity=[0,-9.81,0], nodeType=nodeType)
    b1 = mbs.CreateRigidBody(inertia=inertia, referencePosition=[0.6,0,0], gravity=[0,-9.81,0], initialAngularVelocity=[0,0,2],
                             nodeType=nodeType)
    if explicit: #an explicit solver takes no joints: stiff springs
        for (items, position) in [([oGround, b0], [-0.2,0,0]), ([b0, b1], [0.2,0,0])]:
            mbs.CreateCartesianSpringDamper(itemNumbers=items, localPosition1=position, stiffness=[1e4]*3, damping=[10]*3)
    else:
        mbs.CreateRevoluteJoint(itemNumbers=[oGround, b0], position=[0,0,0], axis=[0,0,1])
        mbs.CreateRevoluteJoint(itemNumbers=[b0, b1], position=[0.4,0,0], axis=[0,0,1])
    oBall = mbs.CreateMassPoint(referencePosition=[1,0.3,0], initialVelocity=[0,-1,0], mass=0.5, gravity=[0,-9.81,0])
    mbs.CreateSphereSphereContact(itemNumbers=[oGround, oBall], localPosition0=[1,-0.6,0], spheresRadii=[0.5,0.1],
                                  contactStiffness=1e4, contactDamping=20)
    if addNode:
        mbs.AddNode(NodePointGround())
    nBall = mbs.GetObject(oBall)['nodeNumber']
    mbs.AddSensor(SensorBody(bodyNumber=b1, localPosition=[0.2,0,0], fileName=directory+'sensorTip.txt',
                             outputVariableType=exu.OutputVariableType.Position))
    mbs.AddSensor(SensorNode(nodeNumber=nBall, fileName=directory+'sensorBall.txt',
                             outputVariableType=exu.OutputVariableType.Velocity))
    mbs.Assemble()
    return (SC, mbs)

def Settings(solverType, solutionFile, stopTime=None, continueIfAvailable=False):
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 500
    simulationSettings.timeIntegration.endTime = 1
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solution.file.name = directory + solutionFile
    simulationSettings.solution.file.writePeriod = 0.01
    simulationSettings.solution.sensors.writePeriod = 0.02
    simulationSettings.solution.precision = 16
    simulationSettings.solution.restart.write = True
    simulationSettings.solution.restart.name = directory + 'restart.txt'
    simulationSettings.solution.restart.writePeriod = 0.2
    simulationSettings.solution.restart.continueIfAvailable = continueIfAvailable
    simulationSettings.timeIntegration.newton.useModifiedNewton = False #the Jacobian of an earlier step is not in the restart file
    return simulationSettings

def Run(solverType, solutionFile, stopTime=None, continueIfAvailable=False, addNode=False):
    (SC, mbs) = Model(solverType != exu.DynamicSolverType.GeneralizedAlpha, addNode)
    if stopTime is not None:
        mbs.SetPostStepUserFunction(lambda mbs, t: t < stopTime - 1e-10)
    mbs.SolveDynamic(Settings(solverType, solutionFile, stopTime, continueIfAvailable), solverType=solverType)
    return (mbs, mbs.systemData.GetODE2Coordinates())

def Path(fileName):
    outputDirectory = exu.config.outputDirectory if hasattr(exu.config, 'outputDirectory') else ''
    return os.path.join(outputDirectory, fileName) if outputDirectory else fileName

def Rows(fileName):
    with open(Path(fileName)) as f:
        return [line for line in f if line[0] != '#']

def RemoveRestartFiles():
    for ending in ['', '.bck', '.tmp']:
        if os.path.isfile(Path(directory + 'restart.txt' + ending)):
            os.remove(Path(directory + 'restart.txt' + ending))

u = 0
for solverType in [exu.DynamicSolverType.GeneralizedAlpha, exu.DynamicSolverType.RK44]:
    RemoveRestartFiles()
    (mbsFull, qFull) = Run(solverType, 'full.txt')
    rowsFull = [Rows(directory + name) for name in ['full.txt', 'sensorTip.txt', 'sensorBall.txt']]

    RemoveRestartFiles()
    (mbsStopped, qStopped) = Run(solverType, 'continued.txt', stopTime=0.6)
    Check(0.6 - 1e-10 <= mbsStopped.systemData.GetTime() < 0.62, 'stopped at ' + str(mbsStopped.systemData.GetTime())) #the step size adapts at contact
    #as if killed after the restart state of t = 0.4: the files hold rows up to 0.6 that are written again
    shutil.copyfile(Path(directory + 'restart.txt.bck'), Path(directory + 'restart.txt'))
    (mbsContinued, qContinued) = Run(solverType, 'continued.txt', continueIfAvailable=True)
    restartTime = mbsContinued.sys['dynamicSolver'].output.restartTime
    Check(0.4 - 1e-10 <= restartTime < 0.42, 'restart time ' + str(restartTime))
    rowsContinued = [Rows(directory + name) for name in ['continued.txt', 'sensorTip.txt', 'sensorBall.txt']]

    Check(np.linalg.norm(qContinued - qFull) == 0, str(solverType) + ': final state differs by ' + str(np.linalg.norm(qContinued - qFull)))
    for (full, continued, name) in zip(rowsFull, rowsContinued, ['solution', 'sensor tip', 'sensor ball']):
        Check(full == continued, str(solverType) + ': ' + name + ' file: ' + str(len(full)) + ' rows against ' + str(len(continued)))
    u += np.sum(qFull)
    exu.Print(solverType, 'final state', np.sum(qFull), ', rows', [len(r) for r in rowsFull])

#a restart file of another system raises; continuing at the end time computes nothing
try:
    Run(exu.DynamicSolverType.RK44, 'other.txt', continueIfAvailable=True, addNode=True)
    Check(False, 'a restart file of another system is taken')
except Exception as error:
    Check('does not fit' in str(error), str(error))
(mbs, q) = Run(exu.DynamicSolverType.RK44, 'atEnd.txt', continueIfAvailable=True)
Check(np.linalg.norm(q - qFull) == 0 and abs(mbs.sys['dynamicSolver'].output.restartTime - 1) < 1e-12, 'continued at the end time')

#without a restart file, the simulation starts at its start time
RemoveRestartFiles()
(mbs, q) = Run(exu.DynamicSolverType.RK44, 'fresh.txt', continueIfAvailable=True)
Check(mbs.sys['dynamicSolver'].output.restartTime == -1 and np.linalg.norm(q - qFull) == 0, 'without a restart file')

exu.Print('restartFileTest: errors', errors)
#what the test checks is the equality within one platform (errors); the final state itself differs between platforms
#in the 10th digit through the contact (Linux, #2874), so it enters rounded
u = round(u, 6) + errors
exu.Print('solution of restartFileTest=', u)

exu.sys['testResult'] = u
