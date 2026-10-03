#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  exu.special.beams.geometricallyExactLumpedMass (#2761): ObjectBeamGeometricallyExact
#           uses the element-consistent mass matrix by default and the lumped one of its nodes when
#           the switch is set - so that tests and comparisons can use both. Checked on the system
#           mass matrix of one element: the consistent one couples the positions of the two nodes
#           (rho A L / 6), the lumped one does not.
#
# Usage:    pytest python/testing/test_specialBeams.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-01
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
import pytest

import exudyn as exu
from exudyn.utilities import NodeRigidBodyEP, ObjectBeamGeometricallyExact, eulerParameters0

L = 2.
rhoA = 3.


def MassMatrix():
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    n0 = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0, 0, 0] + eulerParameters0))
    n1 = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[L, 0, 0] + eulerParameters0))
    section = exu.BeamSection()
    section.stiffnessMatrix = np.eye(6)
    section.inertia = np.eye(3) * 1e-3
    section.massPerLength = rhoA
    mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=[n0, n1], length=L, sectionData=section))
    mbs.Assemble()
    solver = exu.MainSolverImplicitSecondOrder()
    settings = exu.SimulationSettings()
    solver.InitializeSolver(mbs, settings)
    solver.ComputeMassMatrix(mbs)
    return solver.GetSystemMassMatrix()


def test_theDefaultIsTheConsistentMass():
    assert exu.special.beams.geometricallyExactLumpedMass is False
    M = MassMatrix()
    #x of node 0 (coordinate 0) and x of node 1 (coordinate 7, after 3 + 4 Euler parameters)
    assert M[0, 0] == pytest.approx(rhoA * L / 3)
    assert M[0, 7] == pytest.approx(rhoA * L / 6)


def test_theSwitchGivesTheLumpedMass():
    exu.special.beams.geometricallyExactLumpedMass = True
    try:
        M = MassMatrix()
    finally:
        exu.special.beams.geometricallyExactLumpedMass = False
    assert M[0, 0] == pytest.approx(rhoA * L / 2)
    assert M[0, 7] == 0.
