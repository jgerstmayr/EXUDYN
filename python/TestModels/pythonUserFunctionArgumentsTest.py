#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The vector arguments of Python user functions (#3008): q and q_t of the forceUserFunction and the
#           massMatrixUserFunction of ObjectGenericODE2 arrive as read-only numpy arrays that view the coordinates,
#           valid during the call - a function that keeps them must copy them; writing into them raises. The result may
#           be a numpy array or a list. The vectors and matrices of fixed size are numpy arrays as well (#3011), which
#           the user functions use in vectorized form: the load vector of a LoadForceVector, displacement, stiffness and
#           offset of a CartesianSpringDamper, the 6x6 stiffness and the 3x3 rotations of a RigidBodySpringDamper.
#           exudyn.special.deprecations.userFunctionListArguments = True gives lists, as before, and is reported as
#           deprecated. The solutions of all variants agree.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-16
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import numpy as np

n = 4
k = 100.
seen = {}

def ForceNumpy(mbs, t, itemNumber, q, q_t):
    seen['force'] = (type(q).__name__, q.flags.writeable if isinstance(q, np.ndarray) else None)
    if 'kept' not in seen:
        seen['kept'] = q.copy()  #a kept argument is copied: the array itself views the coordinates
    return -k*q - 0.1*q_t      #numpy arithmetic without np.array(q)

def ForceList(mbs, t, itemNumber, q, q_t):
    return [-k*q[i] - 0.1*q_t[i] for i in range(len(q))]

def Mass(mbs, t, itemNumber, q, q_t):
    seen['mass'] = type(q).__name__
    return np.eye(n)

def ForceWrites(mbs, t, itemNumber, q, q_t):
    q[0] = 0.                  #raises: the arguments are read-only
    return -k*q

def Solve(force, massUserFunction=False):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0]*n, initialCoordinates=[0.1*(i+1) for i in range(n)],
                                       initialCoordinates_t=[0]*n, numberOfODE2Coordinates=n))
    mbs.AddObject(ObjectGenericODE2(nodeNumbers=[node], massMatrix=np.eye(n), forceUserFunction=force,
                                    massMatrixUserFunction=Mass if massUserFunction else 0))
    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 100
    simulationSettings.timeIntegration.endTime = 0.1
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solution.file.write = False
    mbs.SolveDynamic(simulationSettings)
    return np.array(mbs.systemData.GetODE2Coordinates())

checks = []

#the vectors and matrices of fixed size, in vectorized form
def LoadUF(mbs, t, loadVector):
    seen['load'] = (type(loadVector).__name__, loadVector.shape, loadVector.flags.writeable)
    return loadVector*(1 + t)

def CartesianUF(mbs, t, itemNumber, u, v, k, d, offset):
    seen['cartesian'] = [type(x).__name__ for x in [u, v, k, d, offset]]
    return k*(u - offset) + d*v

def RigidUF(mbs, t, itemNumber, displacement, rotation, velocity, angularVelocity, stiffness, damping, rot0, rot1, offset):
    seen['rigid'] = (type(stiffness).__name__, stiffness.shape, type(rot0).__name__, rot0.shape, type(offset).__name__)
    return stiffness @ np.hstack([displacement, rotation]) + damping @ np.hstack([velocity, angularVelocity])

SC = exu.SystemContainer()
mbs = SC.AddSystem()
oGround = mbs.CreateGround()
oMass = mbs.CreateMassPoint(referencePosition=[1, 0, 0], mass=1)
mbs.CreateCartesianSpringDamper(itemNumbers=[oGround, oMass], stiffness=[100]*3, damping=[1]*3,
                                springForceUserFunction=CartesianUF)
mbs.CreateForce(itemNumber=oMass, loadVector=[0, -1, 0], loadVectorUserFunction=LoadUF)
oBody = mbs.CreateRigidBody(referencePosition=[0, 2, 0], inertia=InertiaCuboid(1000, [0.2, 0.2, 0.2]))
mbs.CreateRigidBodySpringDamper(itemNumbers=[oGround, oBody], localPosition0=[0, 2, 0], stiffness=100*np.eye(6),
                                damping=np.eye(6), springForceTorqueUserFunction=RigidUF)
mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 10
simulationSettings.timeIntegration.endTime = 0.01
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.solution.file.write = False
mbs.SolveDynamic(simulationSettings)
exu.Print('load vector', seen['load'], ', Cartesian spring-damper', seen['cartesian'], ', rigid body spring-damper',
          seen['rigid'])
checks += [seen['load'] == ('ndarray', (3,), False), seen['cartesian'] == ['ndarray']*5,
           seen['rigid'] == ('ndarray', (6, 6), 'ndarray', (3, 3), 'ndarray')]

qNumpy = Solve(ForceNumpy, massUserFunction=True)
exu.Print('force gets', seen['force'], ', mass matrix gets', seen['mass'], ', kept copy', seen['kept'])
checks += [seen['force'] == ('ndarray', False), seen['mass'] == 'ndarray', abs(seen['kept'][3] - 0.4) < 1e-15]

qList = Solve(ForceList)
checks += [max(abs(qList - qNumpy)) < 1e-14]

refused = ''
try:
    Solve(ForceWrites)
except Exception as error:
    refused = str(error)
exu.Print('writing into q:', 'read-only' in refused)
checks += ['read-only' in refused]

#the old behaviour, for the transition
exu.special.deprecations.userFunctionListArguments = True
qOld = Solve(ForceList, massUserFunction=True)
exu.special.deprecations.userFunctionListArguments = False
used = exu.sys.get('deprecationUse', {}).get('settings', {}).get('special.deprecations.userFunctionListArguments', 0)
exu.Print('userFunctionListArguments: mass matrix gets', seen['mass'], ', reported', used > 0)
checks += [seen['mass'] == 'list', used > 0, max(abs(qOld - qNumpy)) < 1e-14]

exu.Print('checks:', checks)
u = sum(qNumpy) + sum(checks) - len(checks)
exu.Print('solution of pythonUserFunctionArgumentsTest=', u)

exu.sys['testResult'] = u
