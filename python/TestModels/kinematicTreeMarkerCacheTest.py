#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Markers on every link of an ObjectKinematicTree, which share the frames of the links the tree
#           computes once per state (#3229): a chain of six revolute links, a CartesianSpringDamper from every
#           link to the ground and a spherical joint at the last link; solved with one and with four threads,
#           which give the same solution. A change of the base offset by SetObjectParameter moves the markers.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-02
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities

import numpy as np
from exudyn import cMath #the norm of the C++ code, the same bits on every platform

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()

nLinks = 6
L = 0.3
types = [exu.JointType.RevoluteZ, exu.JointType.RevoluteY, exu.JointType.RevoluteX]

def Solve(numberOfThreads):
    mbs.Reset()
    ground = mbs.CreateGround()
    node = mbs.AddNode(NodeGenericODE2(referenceCoordinates=[0]*nLinks, initialCoordinates=[0.1]*nLinks,
                                       initialCoordinates_t=[0]*nLinks, numberOfODE2Coordinates=nLinks))
    tree = mbs.AddObject(ObjectKinematicTree(nodeNumber=node, jointTypes=[types[i % 3] for i in range(nLinks)],
                         linkParents=list(range(-1, nLinks - 1)), jointTransformations=exu.Matrix3DList([np.eye(3)]*nLinks),
                         jointOffsets=exu.Vector3DList([[0, 0, 0]] + [[L, 0, 0]]*(nLinks - 1)),
                         linkInertiasCOM=exu.Matrix3DList([np.eye(3)*0.01]*nLinks), linkCOMs=exu.Vector3DList([[0.5*L, 0, 0]]*nLinks),
                         linkMasses=[1]*nLinks, gravity=[0, -9.81, 0]))
    markers = []
    for i in range(nLinks):
        m = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=tree, linkNumber=i, localHT=HTtranslate([L, 0, 0])))
        mG = mbs.AddMarker(MarkerBodyRigid(bodyNumber=ground, localHT=HTtranslate([L*(i + 1), 0.1, 0.])))
        mbs.AddObject(CartesianSpringDamper(markerNumbers=[mG, m], stiffness=[200]*3, damping=[2]*3))
        markers.append(m)
    mbs.Assemble()
    pEnd = mbs.GetMarkerOutput(markers[-1], exu.OutputVariableType.Position, exu.ConfigurationType.Initial)
    mEnd = mbs.AddMarker(MarkerBodyRigid(bodyNumber=ground, localHT=HTtranslate(pEnd)))
    mbs.AddObject(SphericalJoint(markerNumbers=[mEnd, markers[-1]]))
    mbs.Assemble()

    s = exu.SimulationSettings()
    s.timeIntegration.numberOfSteps = 100
    s.timeIntegration.endTime = 0.1
    s.timeIntegration.verboseMode = 0
    s.timeIntegration.newton.useModifiedNewton = False
    s.solution.file.write = False
    s.parallel.numberOfThreads = numberOfThreads
    s.parallel.multithreadedLowerLimitResiduals = 1
    s.parallel.multithreadedLowerLimitJacobians = 1
    s.parallel.multithreadedLowerLimitMassMatrices = 1
    s.parallel.multithreadedLowerLimitLoads = 1
    mbs.SolveDynamic(s, showHints=False)
    return (tree, markers, mbs.systemData.GetODE2Coordinates())

(tree, markers, q1) = Solve(1)
(tree, markers, q4) = Solve(4)
exu.Print('difference of one and four threads:', cMath.norm(q4 - q1))

#the markers follow a change of the parameters of the tree
p0 = mbs.GetMarkerOutput(markers[2], exu.OutputVariableType.Position)
mbs.SetObjectParameter(tree, 'baseOffset', [0, 0, 1])
p1 = mbs.GetMarkerOutput(markers[2], exu.OutputVariableType.Position)
exu.Print('marker moved by the base offset:', p1 - p0)

testResult = np.sum(q1) + (cMath.norm(q4 - q1) < 1e-10) + np.sum(p1 - p0)
exu.Print('solution of kinematicTreeMarkerCacheTest=', testResult)
exu.sys['testResult'] = testResult
