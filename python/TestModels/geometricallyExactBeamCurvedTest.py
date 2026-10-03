#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ObjectBeamGeometricallyExact with a curved reference configuration (#1494): the 45-degree
#           bend of Bathe and Bolourchi (1979), a cantilever on an eighth of a circle of radius 100 in the
#           x-y plane, loaded at the tip by a force P out of the plane. The element is stress-free in the
#           reference configuration of its nodes, so the unloaded bend keeps its shape. Tip position
#           with 8 elements at P = 300, 450, 600: (58.60, 22.14, 40.36), (52.05, 18.40, 48.57),
#           (46.98, 15.59, 53.46); Simo and Vu-Quoc (1986), 8 elements: (58.84, 22.33, 40.08),
#           (52.32, 18.62, 48.39), (47.23, 15.79, 53.37). With 32 elements at P = 600: (46.90, 15.56, 53.60).
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-29
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities

import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

R = 100                 #radius of the bend
E = 1e7; G = 0.5*E      #square cross section 1 x 1
A = 1; I = 1/12; J = 0.141
nElements = 8


def Bend(force):
    """the tip position of the bend under a tip force in z"""
    SC = exu.SystemContainer(); mbs = SC.AddSystem()
    nodes = []
    for i in range(nElements+1):
        phi = np.pi/4*i/nElements
        nodes.append(mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[R*np.sin(phi), R*(1-np.cos(phi)), 0]
                                                 + list(RotationMatrix2EulerParameters(RotationMatrixZ(phi))))))
    section = exu.BeamSection()
    section.stiffnessMatrix = np.diag([E*A, G*A, G*A, G*J, E*I, E*I])
    section.inertia = np.diag([2*I, I, I])
    section.massPerLength = A
    for i in range(nElements):
        mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=[nodes[i], nodes[i+1]], length=R*np.pi/4/nElements,
                                                   sectionData=section))
    #clamped: position and e1..e3; e0 follows from the norm constraint
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=mbs.AddNode(NodePointGround()), coordinate=0))
    for i in [0, 1, 2, 4, 5, 6]:
        mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround, mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodes[0], coordinate=i))]))
    mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=nodes[-1])), loadVector=[0, 0, force]))
    mbs.Assemble()
    simulationSettings = exu.SimulationSettings()
    simulationSettings.linearSolver.solverType = exu.LinearSolverType.EigenSparse
    simulationSettings.staticSolver.numberOfLoadSteps = 20
    simulationSettings.staticSolver.newton.relativeTolerance = 1e-10
    mbs.SolveStatic(simulationSettings)
    return mbs.GetNodeOutput(nodes[-1], exu.OutputVariableType.Position)


testResult = 0
for force in [0, 300, 450, 600]:
    pTip = Bend(force)
    exu.Print('P =', force, ': tip position', pTip.round(4))
    testResult += 1e-2*sum(pTip)

exu.Print('solution of geometricallyExactBeamCurvedTest=', testResult)
exu.sys['testResult'] = testResult
