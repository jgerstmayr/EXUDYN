#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A library of small models for tests of the solvers and of other parts of Exudyn, each fast to simulate:
#           modelNames lists them, GetModel(model) creates one - by its index or name - and returns its assembled
#           mbs; mbs.variables['modelLibrary'] holds the name, the description and the end time and step size of a
#           fast run, SimulationSettings(mbs) gives settings of that run and Result(mbs) the state to compare
#           (the ODE2 and ODE1 coordinates). The models: a mass-spring-damper, a pendulum of mass points on distance
#           constraints, a chain of rigid bodies on revolute joints and the same chain on rigid body spring-dampers,
#           the IFToMM spatial slider-crank (a closed loop), a kinematic tree with PD control, a geometrically exact 3D
#           beam, an ObjectFFRFreducedOrder of a stored FEM mesh, and an ObjectGenericDAE with nonlinear user
#           functions. Not a test model itself: modelLibraryTest.py runs the models, python/testing/solverStudy.py
#           studies the solvers on them (#3257).
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-05
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics
import exudyn.cMath as cMath #the math functions of the C++ code, the same bits on every platform
from exudyn.FEM import FEMinterface
import numpy as np
import os

g = [0, -9.81, 0] #gravity of the planar models; the spatial ones have their own


def _MassOscillator(mbs):
    """a mass point on a spring-damper to ground, 1 coordinate moving"""
    oMass = mbs.CreateMassPoint(referencePosition=[1, 0, 0], initialDisplacement=[0.1, 0, 0], mass=2)
    oGround = mbs.CreateGround()
    mbs.CreateSpringDamper(itemNumbers=[oGround, oMass], stiffness=800, damping=4)
    return {'description': 'a mass of 2 kg on a spring-damper (800 N/m, 4 Ns/m) to ground, released 0.1 m off',
            'endTime': 1, 'stepSize': 2e-3}


def _NPendulum(mbs, n=4):
    """n mass points on distance constraints, a chain from ground, released horizontally"""
    oGround = mbs.CreateGround()
    previous = oGround
    for i in range(n):
        oMass = mbs.CreateMassPoint(referencePosition=[0.25*(i + 1), 0, 0], mass=0.5, gravity=g)
        mbs.CreateDistanceConstraint(itemNumbers=[previous, oMass])
        previous = oMass
    return {'description': 'a pendulum of ' + str(n) + ' mass points of 0.5 kg on distance constraints of 0.25 m, '
            'released horizontally under gravity',
            'endTime': 1, 'stepSize': 2e-3}


def _RigidBodyChain(mbs, useJoints, n=3):
    """n bricks in a chain from ground, on revolute joints (useJoints) or rigid body spring-dampers"""
    (L, w) = (0.4, 0.04)
    inertia = InertiaCuboid(density=1000, sideLengths=[L, w, w])
    oGround = mbs.CreateGround()
    previous = oGround
    stiffness = np.diag([1e5, 1e5, 1e5, 1e2, 1e2, 0]) #free rotation about z, as the revolute joint
    damping = 0.01*stiffness
    for i in range(n):
        body = mbs.CreateRigidBody(inertia=inertia, referencePosition=[L*(i + 0.5), 0, 0], gravity=g,
                                   graphicsDataList=[graphics.Brick(size=[L, w, w], color=graphics.colorList[i % len(graphics.colorList)])])
        if useJoints:
            mbs.CreateRevoluteJoint(itemNumbers=[previous, body], position=[L*i, 0, 0], axis=[0, 0, 1])
        else:
            mbs.CreateRigidBodySpringDamper(itemNumbers=[previous, body], localPosition0=[L*i, 0, 0] if i == 0 else [0.5*L, 0, 0],
                                            localPosition1=[-0.5*L, 0, 0], stiffness=stiffness, damping=damping)
        previous = body
    kind = 'revolute joints' if useJoints else 'rigid body spring-dampers (1e5 N/m, 1e2 Nm/rad, free about z)'
    return {'description': 'a chain of ' + str(n) + ' bricks of 0.4 m on ' + kind + ', released horizontally under '
            'gravity',
            'endTime': 0.5, 'stepSize': 2e-3}


def _SliderCrank3D(mbs):
    """the IFToMM spatial slider-crank, as sliderCrank3Dbenchmark.py"""
    gravity = [0, 0, -9.81]
    (lAB, lBC, yA, zA) = (0.08, 0.3, 0.1, 0.12)
    xD = cMath.sqrt(lBC**2 - yA**2 - (zA + lAB)**2)
    (pA, pB, pC) = ([0, yA, zA], [0, yA, zA + lAB], [xD, 0, 0])
    [xAxis1, zAxis1, unused] = GramSchmidt(np.array(pC) - pB, pB)
    rotMatBC = np.array([xAxis1, -np.cross(xAxis1, zAxis1), zAxis1]).T
    nodeType = exu.NodeType.RotationRotationVector
    bCrank = mbs.CreateRigidBody(inertia=RigidBodyInertia(mass=0.12, inertiaTensor=np.diag([1e-4, 1e-4, 1e-5])).Translated([0, 0, 0.5*lAB]),
                                 nodeType=nodeType, referencePosition=pA, initialAngularVelocity=[6, 0, 0], gravity=gravity)
    bRod = mbs.CreateRigidBody(inertia=RigidBodyInertia(mass=0.5, inertiaTensor=np.diag([4e-4, 4e-3, 4e-3])), nodeType=nodeType,
                               referencePosition=0.5*(np.array(pB) + pC), referenceRotationMatrix=rotMatBC,
                               initialVelocity=[0.12, -0.24, 0], initialAngularVelocity=[1.92, -0.96, 0.48], gravity=gravity)
    bSlider = mbs.CreateRigidBody(inertia=RigidBodyInertia(mass=2, inertiaTensor=np.diag([1e-4, 1e-4, 1e-4])), nodeType=nodeType,
                                  referencePosition=pC, initialVelocity=[0.24, 0, 0], gravity=gravity)
    oGround = mbs.CreateGround()
    markers = [mbs.AddMarker(MarkerBodyRigid(bodyNumber=body, localPosition=position)) for (body, position) in
               [(oGround, pA), (bCrank, [0, 0, 0]), (oGround, [0, 0, 0]), (bSlider, [0, 0, 0]), (bCrank, [0, 0, lAB]),
                (bRod, [-0.5*lBC, 0, 0]), (bSlider, [0, 0, 0]), (bRod, [0.5*lBC, 0, 0])]]
    for (i, axes) in enumerate([[1,1,1,0,1,1], [0,1,1,1,1,1], [1,1,1,0,0,0], [1,1,1,0,0,1]]):
        mbs.AddObject(GenericJoint(markerNumbers=markers[2*i:2*i + 2], constrainedAxes=axes))
    return {'description': 'the IFToMM spatial slider-crank: crank, connecting rod and slider on a revolute, a '
            'prismatic, a spherical and a universal joint, a closed loop, Lie group nodes',
            'endTime': 0.2, 'stepSize': 1e-3}


def _KinematicTree(mbs):
    """three links of a kinematic tree with PD control towards zero, released off"""
    (L, w) = (0.5, 0.05)
    inertia = InertiaCuboid(density=1000, sideLengths=[L, w, w]).Translated([0.5*L, 0, 0])
    links = [TreeLink(linkInertia=inertia, jointType=jointType, jointHT=exu.HT(translation=translation), PDcontrol=(2e3, 20))
             for (jointType, translation) in [(exu.JointType.RevoluteZ, [0, 0, 0]), (exu.JointType.RevoluteY, [L, 0, 0]),
                                              (exu.JointType.RevoluteZ, [L, 0, 0])]]
    mbs.CreateKinematicTree(listOfTreeLinks=links, initialCoordinates=[0.5, -0.4, 0.8], gravity=[0, 0, -9.81])
    return {'description': 'a kinematic tree of three links of 0.5 m (revolute z, y, z) with PD control towards zero, '
            'released from [0.5, -0.4, 0.8] rad under gravity',
            'endTime': 0.5, 'stepSize': 2e-3}


def _Beam3D(mbs, n=4):
    """a geometrically exact 3D beam, clamped at x = 0, under gravity and a tip load in z"""
    L = 1.
    section = exu.BeamSection()
    section.stiffnessMatrix = np.diag([1e5, 4e4, 4e4, 20, 30, 30]) #EA, GA, GA, GJ, EI, EI
    section.inertia = np.diag([2e-4, 1e-4, 1e-4])
    section.massPerLength = 1
    nodes = [mbs.AddNode(NodeRigidBodyRotVecLG(referenceCoordinates=[L*i/n, 0, 0, 0, 0, 0])) for i in range(n + 1)]
    for i in range(n):
        mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=[nodes[i], nodes[i + 1]], length=L/n, sectionData=section))
        mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=mbs.systemData.NumberOfObjects() - 1)),
                                         loadVector=g))
    oGround = mbs.CreateGround()
    mbs.AddObject(GenericJoint(markerNumbers=[mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround)),
                                              mbs.AddMarker(MarkerNodeRigid(nodeNumber=nodes[0]))]))
    mbs.AddLoad(Force(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=nodes[-1])), loadVector=[0, 0, -2]))
    return {'description': 'a geometrically exact 3D beam of 1 m, ' + str(n) + ' elements, clamped, under gravity in y '
            'and a tip load of 2 N in z',
            'endTime': 0.2, 'stepSize': 1e-3}


def _FFRFreducedOrder(mbs):
    """an ObjectFFRFreducedOrder of the stored mesh testData/netgenTestMesh.npz with 4 Hurty-Craig-Bampton modes, a
    pendulum on a revolute joint"""
    fem = FEMinterface()
    fem.LoadFromFile(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'testData', 'netgenTestMesh'), mode='NPZ')
    (L, a) = (1, 0.1)
    nodesLeft = fem.GetNodesInPlane([0, 0, 0], [-1, 0, 0])
    weightsLeft = fem.GetNodeWeightsFromSurfaceAreas(nodesLeft)
    fem.nodeSets = [{'Name': 'left', 'NodeNumbers': np.array(nodesLeft), 'NodeWeights': np.array(weightsLeft), 'type': 'boundary'}]
    fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=[nodesLeft], nEigenModes=4, useSparseSolver=False,
                                      boundaryNodesWeights=[weightsLeft])
    jointHT = exu.HT(rotation=RotationMatrixY(-0.5*cMath.pi), translation=[0.02, 0, 0]) #the joint axis in z of the marker
    createDict = mbs.CreateFFRFReducedOrderObject(name='pendulum', femInterface=fem, gravity=[0, -9.81, 0],
                                                  superElementRigidMarkersOffsets=[jointHT.translation],
                                                  superElementRigidMarkersLocalHTs=[jointHT])
    oGround = mbs.CreateGround()
    mGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localHT=jointHT))
    mbs.AddObject(GenericJoint(markerNumbers=[mGround, createDict['pendulum:left']], constrainedAxes=[1,1,1,0,1,1]))
    mbs.variables['fem'] = fem #the interface lives as long as the system
    return {'description': 'an ObjectFFRFreducedOrder of the stored mesh testData/netgenTestMesh.npz (a bar of 1 m, '
            '4 Hurty-Craig-Bampton modes) as a pendulum on a revolute joint under gravity',
            'endTime': 0.1, 'stepSize': 1e-3}


def _GenericDAE(mbs):
    """an ObjectGenericDAE: a pendulum in Cartesian coordinates by a nonlinear constraintUserFunction, a damper by
    the forceUserFunction and a first order lag of its horizontal position on an ODE1 coordinate"""
    (m, L, d, tau) = (2., 0.8, 0.3, 0.2)
    (x0, y0) = (L*cMath.sin(0.6), -L*cMath.cos(0.6))
    def UFforce(mbs, t, itemNumber, q, q_t, y, x, parameters):
        return [-d*q_t[0], -d*q_t[1]]
    def UFconstraint(mbs, t, itemNumber, q, q_t, y, z, x, velocityLevel, parameters):
        if velocityLevel: #its time derivative, for the solvers with velocity constraints
            return [2*(x0 + q[0])*q_t[0] + 2*(y0 + q[1])*q_t[1]]
        return [(x0 + q[0])**2 + (y0 + q[1])**2 - L**2]
    def UFrhs(mbs, t, itemNumber, q, q_t, y, x, parameters):
        return [((x0 + q[0])/L - y[0])/tau]
    def UFjacobian(mbs, t, itemNumber, q, q_t, y, z, x, factorODE2, factorODE2_t, factorODE1, parameters):
        J = np.zeros((4, 4)) #rows and columns q (2), y, z
        J[0, 0] = J[1, 1] = factorODE2_t*d #-df_u/dq_t
        J[2, 0] = factorODE2/(L*tau) #dg_u/dq
        J[2, 2] = -factorODE1/tau #dg_u/dy
        J[3, 0:2] = [factorODE2*2*(x0 + q[0]), factorODE2*2*(y0 + q[1])] #dc_u/dq
        return J
    n2 = mbs.AddNode(NodePoint2D(referenceCoordinates=[x0, y0]))
    n1 = mbs.AddNode(NodeGenericODE1(referenceCoordinates=[0], initialCoordinates=[0.], numberOfODE1Coordinates=1))
    nAE = mbs.AddNode(NodeGenericAE(referenceCoordinates=[0], initialCoordinates=[0], numberOfAECoordinates=1))
    mbs.AddObject(ObjectGenericDAE(nodeNumbersODE2=[n2], nodeNumbersODE1=[n1], nodeNumbersAE=[nAE], massMatrix=np.diag([m, m]),
                                   forceVector=[0, -m*9.81], forceUserFunction=UFforce, constraintUserFunction=UFconstraint,
                                   rhsUserFunction=UFrhs, jacobianUserFunction=UFjacobian))
    return {'description': 'an ObjectGenericDAE: a pendulum of 0.8 m in Cartesian coordinates by a nonlinear '
            'constraintUserFunction, a damper by the forceUserFunction, a first order lag of its horizontal position '
            'on an ODE1 coordinate by the rhsUserFunction, the jacobianUserFunction',
            'endTime': 1, 'stepSize': 5e-3}


_models = {'massOscillator': _MassOscillator,
           'nPendulum': _NPendulum,
           'rigidBodyJoints': lambda mbs: _RigidBodyChain(mbs, useJoints=True),
           'rigidBodySpringDampers': lambda mbs: _RigidBodyChain(mbs, useJoints=False),
           'sliderCrank3D': _SliderCrank3D,
           'kinematicTree': _KinematicTree,
           'beam3D': _Beam3D,
           'FFRFreducedOrder': _FFRFreducedOrder,
           'genericDAE': _GenericDAE,
           }

#the names of the models, in the order of their index
modelNames = list(_models)


def GetModel(model, SC=None):
    """create a model of the library in a new system and return its assembled mbs; model: the index into modelNames
    or the name; SC: the system container to add the system to, a new one if None; mbs.variables['modelLibrary'] holds
    'name', 'description', 'endTime' and 'stepSize' of a fast run"""
    name = modelNames[model] if isinstance(model, int) else model
    if name not in _models:
        raise ValueError('GetModel: unknown model ' + str(model) + '; the models are ' + str(modelNames))
    if SC is None:
        SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    info = _models[name](mbs)
    mbs.variables['modelLibrary'] = {'name': name, **info}
    mbs.Assemble()
    return mbs


def SimulationSettings(mbs, stepSize=None, endTime=None):
    """the settings of a fast run of a model of GetModel: its end time and step size unless given, no files, silent"""
    info = mbs.variables['modelLibrary']
    endTime = info['endTime'] if endTime is None else endTime
    stepSize = info['stepSize'] if stepSize is None else stepSize
    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.endTime = endTime
    simulationSettings.timeIntegration.numberOfSteps = int(round(endTime/stepSize))
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.solution.file.write = False
    simulationSettings.show.computationTime = False
    return simulationSettings


def Result(mbs, configuration=exu.ConfigurationType.Current):
    """the state of the model to compare: its ODE2 and ODE1 coordinates"""
    return np.array(list(mbs.systemData.GetODE2Coordinates(configuration=configuration))
                    + list(mbs.systemData.GetODE1Coordinates(configuration=configuration)))
