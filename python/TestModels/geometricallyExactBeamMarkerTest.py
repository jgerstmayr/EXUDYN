#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Body markers on ObjectBeamGeometricallyExact (#2730, RG4.8.7 of revision2026b): loads through
#           MarkerBodyMass, MarkerBodyPosition and MarkerBodyRigid on the elements give the same result as
#           the equivalent loads on the nodes. (1) a flexible pendulum under gravity through
#           LoadMassProportional, against gravity as nodal forces; (2) a cantilever with a tip force
#           and a tip torque on the last element's end, against the same on the last node; (3) a force
#           at the middle of an element, against half of it on each of its nodes.
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

L = 0.5; nElements = 8; E = 1e8; rho = 1000; h = 0.002; b = 0.01; nu = 0.3
A = b*h; Izz = b*h**3/12; Iyy = h*b**3/12; G = E/(2*(1+nu)); g = 9.81
lElement = L/nElements


def Beam(mbs):
    """nodes and elements of a straight beam along x, and a marker for the first node's coordinates"""
    nodes = [mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[i*lElement, 0, 0] + eulerParameters0))
             for i in range(nElements+1)]
    section = exu.BeamSection()
    section.stiffnessMatrix = np.diag([E*A, G*A, G*A, G*(Iyy+Izz), E*Iyy, E*Izz])
    section.inertia = np.diag([rho*(Iyy+Izz), rho*Iyy, rho*Izz])
    section.massPerLength = rho*A
    elements = [mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=[nodes[i], nodes[i+1]], length=lElement,
                                                           sectionData=section)) for i in range(nElements)]
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=mbs.AddNode(NodePointGround()), coordinate=0))
    return (nodes, elements, mGround)


def Fix(mbs, mGround, node, coordinates):
    for i in coordinates:
        mbs.AddObject(ObjectConnectorCoordinate(markerNumbers=[mGround, mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node, coordinate=i))]))


#(1) the pendulum: gravity through the elements or on the nodes
def Pendulum(throughElements):
    SC = exu.SystemContainer(); mbs = SC.AddSystem()
    (nodes, elements, mGround) = Beam(mbs)
    if throughElements:
        for e in elements:
            mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=e)), loadVector=[0, -g, 0]))
    else:
        for i in range(nElements+1):
            weight = rho*A*lElement*(0.5 if i in [0, nElements] else 1)*g
            mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=nodes[i])), loadVector=[0, -weight, 0]))
    Fix(mbs, mGround, nodes[0], [0, 1, 2])
    mbs.Assemble()
    s = exu.SimulationSettings()
    s.timeIntegration.numberOfSteps = 200; s.timeIntegration.endTime = 0.5
    s.solution.file.write = False
    s.linearSolver.solverType = exu.LinearSolverType.EigenSparse
    mbs.SolveDynamic(s)
    return mbs.GetNodeOutput(nodes[-1], exu.OutputVariableType.Position)


#(2), (3) the cantilever, clamped (position and e1..e3; e0 follows from the norm constraint)
def Cantilever(case, throughElements):
    SC = exu.SystemContainer(); mbs = SC.AddSystem()
    (nodes, elements, mGround) = Beam(mbs)
    F = [0, -0.5*E*Izz/L**2, 0]; M = [0, 0, 0.3*E*Izz/L]
    if case == 'tip':
        if throughElements:
            mTip = mbs.AddMarker(MarkerBodyRigid(bodyNumber=elements[-1], localPosition=[0.5*lElement, 0, 0]))
        else:
            mTip = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nodes[-1]))
        mbs.AddLoad(LoadForceVector(markerNumber=mTip, loadVector=F))
        mbs.AddLoad(LoadTorqueVector(markerNumber=mTip, loadVector=M))
    else:   #the middle of the last element, or half on each of its nodes
        if throughElements:
            mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerBodyPosition(bodyNumber=elements[-1])), loadVector=F))
        else:
            for n in nodes[-2:]:
                mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=n)), loadVector=0.5*np.array(F)))
    Fix(mbs, mGround, nodes[0], [0, 1, 2, 4, 5, 6])
    mbs.Assemble()
    s = exu.SimulationSettings()
    s.staticSolver.numberOfLoadSteps = 10
    s.staticSolver.newton.relativeTolerance = 1e-10
    mbs.SolveStatic(s)
    return mbs.GetNodeOutput(nodes[-1], exu.OutputVariableType.Position)


testResult = 0
for (name, solve) in [('pendulum', lambda through: Pendulum(through)),
                      ('cantilever, tip', lambda through: Cantilever('tip', through)),
                      ('cantilever, element middle', lambda through: Cantilever('middle', through))]:
    pElements = solve(True)
    pNodes = solve(False)
    exu.Print(name + ': tip', pElements.round(8), ', difference element loads - node loads:', np.linalg.norm(pElements - pNodes))
    testResult += sum(pElements)

exu.Print('solution of geometricallyExactBeamMarkerTest=', testResult)
exu.sys['testResult'] = testResult
