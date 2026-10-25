#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The outputs of items in symbolic user functions (#2984): in a function recorded by
#           CreateSymbolicUserFunction, mbs.GetNodeOutput, GetObjectOutput (of a connector),
#           GetObjectOutputBody, GetMarkerOutput and GetSensorValues are symbolic expressions, which C++ reads from the
#           MainSystem at every call. Loads on a mass point follow a rigid body through springs written with each of
#           them; the model is solved with the Python functions and with the symbolic ones, with 1 and with 4 threads,
#           and gives the same result. Every other attribute of mbs raises while recording, mbs.variables are
#           constants of the recording, an output of two dimensions is refused, UserFunction.Evaluate needs the mbs.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-25
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import (NodePoint, MassPoint, RigidBodyInertia, InertiaCuboid, ObjectGround, MarkerNodePosition,
                              MarkerBodyRigid, MarkerBodyPosition, ObjectConnectorSpringDamper, LoadForceVector,
                              SensorNode, SensorBody)
from exudyn.advancedUtilities import CreateSymbolicUserFunction
import numpy as np
import time
esym = exu.symbolic
ovt = exu.OutputVariableType

def Raises(function, errorType):
    """1 if function raises errorType, else 0"""
    try:
        function()
    except errorType:
        return 1
    return 0

def Model(symbolic, numberOfThreads=1):
    """a mass point pulled towards a rigid body by loads that read the body, a marker, a sensor and a connector"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    mbs.variables['k'] = 20.
    nMass = mbs.AddNode(NodePoint(referenceCoordinates=[1, 0, 0], initialVelocities=[0, 0.5, 0]))
    mbs.AddObject(MassPoint(mass=1, nodeNumber=nMass))
    mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    inertia = InertiaCuboid(density=1000, sideLengths=[0.2, 0.1, 0.1])
    body = mbs.CreateRigidBody(inertia=inertia, referencePosition=[0, 0, 0], initialAngularVelocity=[0, 0, 2],
                               create2D=False)
    nBody = mbs.GetObject(body)['nodeNumber']
    mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=[0.1, 0, 0]))
    mBodyPosition = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body))
    ground = mbs.AddObject(ObjectGround())
    mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=ground))
    spring = mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[mGround, mMass], stiffness=5, referenceLength=1,
                                                       damping=0.1))
    sBody = mbs.AddSensor(SensorBody(bodyNumber=body, localPosition=[0, 0.05, 0], outputVariableType=ovt.Velocity,
                                     storeInternal=True))

    def FollowNode(mbs, t, loadVector):
        """a spring towards the node of the body, and a damper with the velocity of the mass"""
        return mbs.variables['k']*(mbs.GetNodeOutput(nBody, ovt.Position) - mbs.GetNodeOutput(nMass, ovt.Position)) \
            - 0.2*mbs.GetNodeOutput(nMass, ovt.Velocity)

    def FollowBodyPoint(mbs, t, loadVector):
        """towards a point of the body, and along the velocity the body has at a sensor"""
        return 2.*(mbs.GetObjectOutputBody(body, ovt.Position, [0, 0.1, 0]) - mbs.GetMarkerOutput(mMass, ovt.Position)) \
            + 0.1*mbs.GetSensorValues(sBody)

    def FollowMarker(mbs, t, loadVector):
        """towards a marker of the body, scaled by the length of the spring, a scalar output"""
        distance = mbs.GetObjectOutput(spring, ovt.Distance)
        return (1. + 0.1*distance)*(mbs.GetMarkerOutput(mBody, ovt.Position) - mbs.GetMarkerOutput(mMass, ovt.Position)) \
            + 0.001*mbs.GetObjectOutputBody(body, ovt.Velocity, [0.1, 0, 0]) + loadVector

    functions = [FollowNode, FollowBodyPoint, FollowMarker]
    loads = [mbs.AddLoad(LoadForceVector(markerNumber=mMass, loadVector=[0, 0, -0.1], loadVectorUserFunction=f))
             for f in functions]
    loadsBody = [mbs.AddLoad(LoadForceVector(markerNumber=mBodyPosition, loadVector=[0, 0, 0])) for f in functions]
    recorded = []
    if symbolic:
        mbs.Assemble() #the outputs are read once while recording, for their sizes
        for (load, function) in zip(loads, functions):
            recorded.append(CreateSymbolicUserFunction(mbs, function, 'loadVectorUserFunction', load))
            mbs.SetLoadParameter(load, 'loadVectorUserFunction', recorded[-1])
    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 500
    simulationSettings.timeIntegration.endTime = 1
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solution.file.write = False
    simulationSettings.parallel.numberOfThreads = numberOfThreads
    start = time.time()
    mbs.SolveDynamic(simulationSettings)
    duration = time.time() - start
    del loadsBody
    return (np.array(mbs.GetNodeOutput(nMass, ovt.Position)), np.array(mbs.GetNodeOutput(nBody, ovt.Position)),
            duration, mbs, recorded, (nMass, body, mMass))

failed = 0
python = Model(False)
symbolic = Model(True)
threads = Model(True, numberOfThreads=4)
difference = max(np.max(np.abs(python[0] - symbolic[0])), np.max(np.abs(python[1] - symbolic[1])),
                 np.max(np.abs(threads[0] - symbolic[0])))
exu.Print('largest difference of the symbolic functions to the Python ones and of 4 threads to 1:', difference)
exu.Print('time with Python functions: %.4f s, with symbolic functions: %.4f s' % (python[2], symbolic[2]))
failed += int(difference != 0.)
u = float(np.sum(symbolic[0]) + np.sum(symbolic[1]))

#UserFunction.Evaluate reads the outputs from the mbs it gets, and needs it
(mbs, recorded, (nMass, body, mMass)) = symbolic[3:]
value = np.array(recorded[0].Evaluate(mbs, 1., [0, 0, -0.1]))
nBody = mbs.GetObject(body)['nodeNumber']
expected = 20.*(mbs.GetNodeOutput(nBody, ovt.Position) - mbs.GetNodeOutput(nMass, ovt.Position)) \
    - 0.2*mbs.GetNodeOutput(nMass, ovt.Velocity)
failed += int(np.max(np.abs(value - expected)) > 1e-14)
failed += 1 - Raises(lambda: recorded[0].Evaluate(1., [0, 0, -0.1]), Exception)
u += float(np.sum(value))

#the text of the recording names the outputs
failed += int('mbs.GetNodeOutput(' not in recorded[0].GetFlatProgram())

#while recording, another attribute of mbs raises, and an output of two dimensions is refused
def ReadsObject(mbs, t, loadVector):
    return mbs.GetObject(body)['mass']*loadVector
def ReadsRotationMatrix(mbs, t, loadVector):
    return mbs.GetObjectOutputBody(body, ovt.RotationMatrix)[0:3]
def SetsAttribute(mbs, t, loadVector):
    mbs.x = 1
    return loadVector
load = mbs.AddLoad(LoadForceVector(markerNumber=mMass, loadVector=[0, 0, 0]))
mbs.Assemble()
failed += 1 - Raises(lambda: CreateSymbolicUserFunction(mbs, ReadsObject, 'loadVectorUserFunction', load), AttributeError)
failed += 1 - Raises(lambda: CreateSymbolicUserFunction(mbs, SetsAttribute, 'loadVectorUserFunction', load), AttributeError)
#a rotation matrix is a vector of 9 components; a homogeneous transformation, exu.HT, is refused
rotation = CreateSymbolicUserFunction(mbs, ReadsRotationMatrix, 'loadVectorUserFunction', load)
failed += int(np.max(np.abs(np.array(rotation.Evaluate(mbs, 0., [0, 0, 0]))
                            - np.array(mbs.GetObjectOutputBody(body, ovt.RotationMatrix))[0:3])) > 0)
def ReadsTransformation(mbs, t, loadVector):
    return mbs.GetObjectOutputBody(body, ovt.HomogeneousTransformation)
failed += 1 - Raises(lambda: CreateSymbolicUserFunction(mbs, ReadsTransformation, 'loadVectorUserFunction', load), ValueError)

exu.Print('failed checks:', failed)
u += failed
exu.Print('solution of symbolicItemOutputTest=', u)

exu.sys['testResult'] = u
