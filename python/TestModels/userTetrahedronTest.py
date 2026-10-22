#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A user finite element (#2988): the linear tetrahedron as ObjectUserBody on four NodePoint nodes, with the
#           Saint Venant-Kirchhoff material (Green strain, large deformation) and the consistent mass matrix. A unit
#           cube of six tetrahedra in uniaxial tension (the patch test) has the exact homogeneous solution
#           F = diag(a, b, b) of the nonlinear material, which the static solution reproduces; a cube moving with a
#           uniform velocity has the kinetic energy of a rigid body.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-12
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.cMath import sqrt
import numpy as np
from itertools import permutations

SC = exu.SystemContainer()

youngsModulus = 1000
nu = 0.3
rho = 100
lam = youngsModulus*nu/((1 + nu)*(1 - 2*nu)) #Lame parameters
mu = youngsModulus/(2*(1 + nu))
force = 50 #nominal stress on the unit face x=1

#the data of each tetrahedron, by its object number: the gradients of the shape functions in the reference
#configuration (4 x 3) and the volume
tetData = {}

def UFmass(mbs, t, itemNumber, qRef, q, q_t, parameters):
    volume = tetData[itemNumber][1]
    return np.kron(rho*volume/20*(np.ones((4, 4)) + np.eye(4)), np.eye(3))

#the internal forces of the Saint Venant-Kirchhoff material, on the right-hand side
def UFforce(mbs, t, itemNumber, qRef, q, q_t, parameters):
    (gradN, volume) = tetData[itemNumber]
    F = np.array(q).reshape(4, 3).T @ gradN #deformation gradient
    E = 0.5*(F.T @ F - np.eye(3)) #Green strain
    S = lam*np.trace(E)*np.eye(3) + 2*mu*E #second Piola-Kirchhoff stress
    return -(volume*gradN @ (F @ S).T).flatten()

#localPosition: the coordinates (xi, eta, zeta) of the unit tetrahedron
def N(localPosition):
    (xi, eta, zeta) = localPosition
    return np.array([1 - xi - eta - zeta, xi, eta, zeta])

def UFposition(mbs, t, itemNumber, localPosition, qRef, q, q_t, parameters):
    return N(localPosition) @ np.array(q).reshape(4, 3)

def UFpositionJacobian(mbs, t, itemNumber, localPosition, qRef, q, q_t, parameters):
    return np.kron(N(localPosition), np.eye(3))

#a unit cube of six tetrahedra on its diagonal from node 0 to node 7; node i is at (i%2, (i//2)%2, i//4)
def AddCube(mbs, initialVelocity=[0, 0, 0]):
    points = [np.array([i % 2, (i//2) % 2, i//4], dtype=float) for i in range(8)]
    nodes = [mbs.AddNode(NodePoint(referenceCoordinates=list(p), initialVelocities=initialVelocity)) for p in points]
    tets = []
    for (a, b, c) in permutations([0, 1, 2]):
        corners = [0, 2**a, 2**a + 2**b, 7]
        X = np.array([points[i] for i in corners])
        D = (X[1:] - X[0]).T
        if np.linalg.det(D) < 0: #counterclockwise, positive volume
            corners[2:4] = corners[3:1:-1]
            X = np.array([points[i] for i in corners])
            D = (X[1:] - X[0]).T
        gradN = np.vstack([np.zeros(3), np.linalg.inv(D)])
        gradN[0] = -np.sum(gradN[1:], axis=0)
        tet = mbs.AddObject(ObjectUserBody(nodeNumbers=[nodes[i] for i in corners], massMatrixUserFunction=UFmass,
                                           forceUserFunction=UFforce, positionUserFunction=UFposition,
                                           positionJacobianUserFunction=UFpositionJacobian))
        tetData[int(tet)] = (gradN, np.linalg.det(D)/6)
        tets += [(tet, X)]
    return (nodes, points, tets)

#+++++++++++++++++++++++++++++++++++++++++++++++
#patch test: uniaxial tension, the faces x=0, y=0, z=0 held by the fewest constraints that keep the cube from moving
mbs = SC.AddSystem()
(nodes, points, tets) = AddCube(mbs)
nGround = mbs.AddNode(NodePointGround())
for (coordinate, held) in [(0, [0, 2, 4, 6]), (1, [0, 4]), (2, [0, 2])]:
    for i in held:
        mbs.AddObject(CoordinateConstraint(markerNumbers=[mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0)),
                                                          mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodes[i], coordinate=coordinate))]))
#the consistent nodal forces of the face x=1, whose two triangles share the diagonal from node 1 to node 7
for (i, share) in [(1, 1/3), (3, 1/6), (5, 1/6), (7, 1/3)]:
    mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerNodePosition(nodeNumber=nodes[i])), loadVector=[share*force, 0, 0]))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.staticSolver.newton.relativeTolerance = 1e-12
simulationSettings.staticSolver.verboseMode = 0
simulationSettings.solution.file.write = False
mbs.SolveStatic(simulationSettings)

#the exact solution: a*E*(a^2-1)/2 = force for the stretch a, and the lateral stretch b from S22 = 0
a = 1
for i in range(20):
    a -= (a*youngsModulus*(a**2 - 1)/2 - force)/(youngsModulus*(3*a**2 - 1)/2)
E11 = (a**2 - 1)/2
b = sqrt(1 - 2*lam*E11/(2*(lam + mu)))
stretch = np.diag([a, b, b])
error = max(np.max(np.abs(mbs.GetNodeOutput(nodes[i], exu.OutputVariableType.Position) - stretch @ points[i])) for i in range(8))
#a point inside each tetrahedron, through the body interface
for (tet, X) in tets:
    p = mbs.GetObjectOutputBody(tet, exu.OutputVariableType.Position, localPosition=[0.25, 0.25, 0.25])
    error = max(error, np.max(np.abs(p - stretch @ np.mean(X, axis=0))))
exu.Print('stretches a, b =', a, b, '; largest error of the patch test:', error)

#+++++++++++++++++++++++++++++++++++++++++++++++
#a cube in uniform motion has the kinetic energy of a rigid body, 1/2*rho*V*v^2
mbs2 = SC.AddSystem()
velocity = [1, 2, -0.5]
(nodes2, points2, tets2) = AddCube(mbs2, initialVelocity=velocity)
mbs2.Assemble()
kineticEnergy = sum(mbs2.GetObjectOutputBody(tet, exu.OutputVariableType.KineticEnergy, localPosition=[0, 0, 0]) for (tet, X) in tets2)
energyError = abs(kineticEnergy - 0.5*rho*np.dot(velocity, velocity))
exu.Print('kinetic energy', kineticEnergy, '; error', energyError)

u = a + b + kineticEnergy + int(error > 1e-10) + int(energyError > 1e-10)
exu.Print('solution of userTetrahedronTest=', u)

exu.sys['testResult'] = u
