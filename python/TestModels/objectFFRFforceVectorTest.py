#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The applied forces and the mass matrix of ObjectFFRF against the nodal-based floating frame of reference
#           formulation of Zwoelfer and Gerstmayr 2019 (Co-rotational formulations for 3D flexible multibody systems:
#           a nodal-based approach, Eqs. (14.84) and (14.88)), #591: a body of 8 nodes with lumped masses and bars
#           between all nodes, at a rotated and deformed state at rest. The generalized forces of global nodal forces f
#           are L^T f = [Phi_t^T f, -G^T rf~^T A_bd^T f, A_bd^T f]; gravity applied by nodal loads gives them for
#           f = M Phi_t g. The forceVector [f_r, f_f] gives f_r and A_bd^T f_f without a rotational part: gravity
#           by forceVector is exact for a body whose center of mass is at the origin of its frame and stays there,
#           and differs by the rotational part otherwise. The mass matrix of the solver is L^T M L.
#           The result is the number of checks that pass.
#
# Author:   Johannes Gerstmayr
# Date:     2026-11-10
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import numpy as np
from exudyn import cMath #the math functions of the C++ code, the same bits on every platform

testIsActive = exu.sys.get('testIsActive', False)

testError = 0
numberOfChecks = 0
def Check(condition, message):
    global testError, numberOfChecks
    numberOfChecks += 1
    if not condition:
        exu.Print('objectFFRFforceVectorTest: ' + message)
        testError += 1

#%%+++ a body of 8 nodes: lumped masses, a bar between every two nodes
g = np.array([0.5, -9.81, 2.])
k = 1e4

def BodyData(center):
    """the nodes of a box around center, their masses and the stiffness matrix of the bars"""
    x = np.array([[i*0.4, j*0.2, l*0.1] for i in [-1, 1] for j in [-1, 1] for l in [-1, 1]]) + center
    masses = np.array([1., 2., 1.5, 1., 0.5, 1., 2.5, 1.])
    n = len(x)
    M = np.kron(np.diag(masses), np.eye(3))
    K = np.zeros((3*n, 3*n))
    for i in range(n):
        for j in range(i+1, n):
            e = (x[j] - x[i])/cMath.norm(x[j] - x[i])
            Kij = k*np.outer(e, e)
            for (a, b, s) in [(i, i, 1), (j, j, 1), (i, j, -1), (j, i, -1)]:
                K[3*a:3*a+3, 3*b:3*b+3] += s*Kij
    return [x, masses, M, K]

#the state: translated, rotated, deformed, at rest
qt = np.array([0.1, 0.2, -0.3])
A = RotXYZ2RotationMatrix([0.3, -0.4, 0.5])
ep = np.array(RotationMatrix2EulerParameters(A))
random = np.random.default_rng(591)
cf = 0.01*random.uniform(-1, 1, 24)

def SystemTerms(x, M, K, forceVector=[], nodalForces=None):
    """the mass matrix and the right-hand side of the solver for the ObjectFFRF of the body at the state"""
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    nRigid = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0, 0, 0] + list(eulerParameters0)))
    nodes = [mbs.AddNode(NodePoint(referenceCoordinates=list(p))) for p in x]
    massMatrix = exu.MatrixContainer(); massMatrix.SetWithDenseMatrix(M, useDenseMatrix=False)
    stiffnessMatrix = exu.MatrixContainer(); stiffnessMatrix.SetWithDenseMatrix(K, useDenseMatrix=False)
    oFFRF = mbs.AddObject(ObjectFFRF(nodeNumbers=[nRigid] + nodes, massMatrixFF=massMatrix,
                                     stiffnessMatrixFF=stiffnessMatrix, forceVector=forceVector))
    if nodalForces is not None: #a load at each mesh node, in global coordinates
        for (i, f) in enumerate(nodalForces):
            m = mbs.AddMarker(MarkerSuperElementPosition(bodyNumber=oFFRF, meshNodeNumbers=[i], weightingFactors=[1]))
            mbs.AddLoad(LoadForceVector(markerNumber=m, loadVector=list(f)))
    mbs.Assemble()

    simulationSettings = exu.SimulationSettings()
    simulationSettings.solution.file.write = False
    solver = exu.MainSolverImplicitSecondOrder()
    solver.InitializeSolver(mbs, simulationSettings)
    nODE2 = mbs.systemData.ODE2Size()
    mbs.systemData.SetODE2Coordinates(np.hstack([qt, ep - np.array(eulerParameters0), cf]))
    mbs.systemData.SetODE2Coordinates_t(np.zeros(nODE2))
    solver.ComputeMassMatrix(mbs)
    solver.ComputeODE2RHS(mbs)
    return [solver.GetSystemMassMatrix(), np.array(solver.GetSystemResidual()[0:nODE2])]

def PaperTerms(x, M):
    """L of Eq. (14.25) with the local G of Euler parameters, and L^T M L of Eq. (14.84)"""
    n = len(x)
    Phit = np.kron(np.ones((n, 1)), np.eye(3))
    Abd = np.kron(np.eye(n), A)
    rf = x.flatten() + cf
    rfTilde = ComputeSkewMatrix(rf) #the n skew matrices one below the other
    G = EulerParameters2GLocal(ep)
    L = np.hstack([Phit, -Abd @ rfTilde @ G, Abd])
    return [L, L.T @ M @ L, Phit, Abd]

for (case, center) in [('center of mass off the origin', np.array([0.3, -0.1, 0.2])), ('center of mass at the origin', None)]:
    [x, masses, M, K] = BodyData(np.zeros(3))
    if center is None: #nodes moved such that the center of mass is at the origin
        x = x - masses @ x/masses.sum()
    else:
        x = x + center
    [L, Mpaper, Phit, Abd] = PaperTerms(x, M)
    mass = masses.sum()
    fGravity = M @ Phit @ g #the global nodal forces of gravity

    [Msystem, rhs0] = SystemTerms(x, M, K)
    [_, rhsNodal] = SystemTerms(x, M, K, nodalForces=fGravity.reshape((-1, 3)))
    [_, rhsForceVector] = SystemTerms(x, M, K, forceVector=list(mass*g) + [0]*4 + list(fGravity))
    Qpaper = L.T @ fGravity
    Qnodal = rhsNodal - rhs0
    QforceVector = rhsForceVector - rhs0
    QrotPaper = Qpaper[3:7]

    Check(cMath.norm(Msystem - Mpaper) < 1e-10*cMath.norm(Mpaper), case + ': the mass matrix against L^T M L: '
          + str(cMath.norm(Msystem - Mpaper)))
    Check(cMath.norm(Qnodal - Qpaper) < 1e-10*cMath.norm(Qpaper), case + ': gravity by nodal loads against L^T f: '
          + str(cMath.norm(Qnodal - Qpaper)))
    Check(cMath.norm(Qpaper[0:3] - mass*g) < 1e-12*mass*cMath.norm(g), case + ': the translational part m g')
    QwithoutRotation = Qpaper.copy(); QwithoutRotation[3:7] = 0
    Check(cMath.norm(QforceVector - QwithoutRotation) < 1e-10*cMath.norm(Qpaper),
          case + ': forceVector against L^T f without its rotational part: ' + str(cMath.norm(QforceVector - QwithoutRotation)))
    #the rotational part: the moment of gravity about the origin of the frame, by the deformation only if the center
    #of mass is at the origin
    momentDeformation = cMath.norm(EulerParameters2GLocal(ep).T @ (ComputeSkewMatrix(cf).T @ M @ Phit @ A.T @ g))
    exu.Print('objectFFRFforceVectorTest:', case + ': rotational part of L^T f =', cMath.norm(QrotPaper),
              ', of the deformation =', momentDeformation)
    if center is None:
        Check(abs(cMath.norm(QrotPaper) - momentDeformation) < 1e-10, case + ': only the deformation gives a rotational part')
    else:
        Check(cMath.norm(QrotPaper) > 100*momentDeformation, case + ': the rotational part of gravity is missing in forceVector')

u = numberOfChecks - 10*testError
exu.Print('objectFFRFforceVectorTest: errors', testError)
exu.Print('solution of objectFFRFforceVectorTest=', u)
exu.sys['testResult'] = u
