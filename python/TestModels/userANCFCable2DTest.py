#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A user finite element (#2987): the 2D ANCF cable element as ObjectUserBody on two NodePoint2DSlope1 nodes,
#           minimal - mass matrix, axial strain and curvature with the integration rules of ObjectANCFCable2D, the
#           position and its Jacobian; a cantilever of four such elements under a tip force swings side by side with
#           a cantilever of ObjectANCFCable2D, and both must agree.
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

SC = exu.SystemContainer()
mbs = SC.AddSystem()

nElements = 4
length = 2
Le = length/nElements #length of one element
rhoA = 1    #mass per length
EA = 1000   #axial stiffness
EI = 2      #bending stiffness
tipForce = [0, -1, 0]

#the cubic Hermite shape functions of the element and their derivatives at x in [0, Le], as 2 x 8 matrices for the
#coordinates [x0, y0, x0', y0', x1, y1, x1', y1']
def S(x):
    xi = x/Le
    return np.kron([1 - 3*xi**2 + 2*xi**3, x - 2*x**2/Le + x*xi**2, 3*xi**2 - 2*xi**3, -x*xi + x*xi**2], np.eye(2))

def S_x(x):
    return np.kron([-6*x/Le**2 + 6*x**2/Le**3, 1 - 4*x/Le + 3*x**2/Le**2, 6*x/Le**2 - 6*x**2/Le**3, -2*x/Le + 3*x**2/Le**2], np.eye(2))

def S_xx(x):
    return np.kron([12*x/Le**3 - 6/Le**2, 6*x/Le**2 - 4/Le, 6/Le**2 - 12*x/Le**3, 6*x/Le**2 - 2/Le], np.eye(2))

#Gauss rules as in ObjectANCFCable2D: order 9 for the axial strain, order 5 for the curvature, order 7 for the mass
def GaussRule(n):
    (points, weights) = np.polynomial.legendre.leggauss(n)
    return [(0.5*Le*(p + 1), 0.5*Le*w) for (p, w) in zip(points, weights)]

axialRule = GaussRule(5)
bendingRule = GaussRule(3)
massMatrix = sum(w*rhoA*S(x).T @ S(x) for (x, w) in GaussRule(4))

def UFmass(mbs, t, itemNumber, q, q_t):
    return massMatrix

#the elastic forces, on the right-hand side; q includes the reference coordinates
def UFforce(mbs, t, itemNumber, q, q_t):
    q = np.array(q)
    f = np.zeros(8)
    for (x, w) in axialRule:
        rx = S_x(x) @ q
        rxNorm = sqrt(rx @ rx)
        f -= w*EA*(rxNorm - 1)/rxNorm * S_x(x).T @ rx
    for (x, w) in bendingRule:
        rx = S_x(x) @ q
        rxx = S_xx(x) @ q
        g = rx @ rx
        cross = rx[0]*rxx[1] - rx[1]*rxx[0]
        cross_q = S_x(x).T @ [rxx[1], -rxx[0]] + S_xx(x).T @ [-rx[1], rx[0]]
        curvature_q = cross_q/g - 2*cross/g**2 * S_x(x).T @ rx
        f -= w*EI*(cross/g) * curvature_q
    return f

def UFposition(mbs, t, itemNumber, localPosition, q):
    return list(S(localPosition[0]) @ q) + [0]

def UFpositionJacobian(mbs, t, itemNumber, localPosition, q):
    return np.vstack([S(localPosition[0]), np.zeros(8)])

nGround = mbs.AddNode(NodePointGround())
cables = []
for isUser in [False, True]:
    nodes = [mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[i*Le, 0, 1, 0])) for i in range(nElements + 1)]
    elements = []
    for i in range(nElements):
        if isUser:
            elements += [mbs.AddObject(ObjectUserBody(nodeNumbers=nodes[i:i+2], massMatrixUserFunction=UFmass,
                                                      forceUserFunction=UFforce, positionUserFunction=UFposition,
                                                      positionJacobianUserFunction=UFpositionJacobian))]
        else:
            elements += [mbs.AddObject(ObjectANCFCable2D(nodeNumbers=nodes[i:i+2], length=Le, massPerLength=rhoA,
                                                         bendingStiffness=EI, axialStiffness=EA))]
    #clamped at x=0: position and the y-component of the slope
    for coordinate in [0, 1, 3]:
        mbs.AddObject(CoordinateConstraint(markerNumbers=[mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0)),
                                                          mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodes[0], coordinate=coordinate))]))
    mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerBodyPosition(bodyNumber=elements[-1], localPosition=[Le, 0, 0])),
                                loadVector=tipForce))
    cables += [elements]

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 200
simulationSettings.timeIntegration.endTime = 0.2
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-10
simulationSettings.solution.file.write = False
mbs.SolveDynamic(simulationSettings)

#positions and velocities along both cantilevers
difference = 0
u = 0
for i in range(nElements):
    for x in [0.5*Le, Le]:
        for output in [exu.OutputVariableType.Position, exu.OutputVariableType.Velocity]:
            values = [np.array(mbs.GetObjectOutputBody(cable[i], output, localPosition=[x, 0, 0])) for cable in cables]
            difference = max(difference, np.max(np.abs(values[1] - values[0])))
            u += np.sum(values[1])
tip = mbs.GetObjectOutputBody(cables[1][-1], exu.OutputVariableType.Position, localPosition=[Le, 0, 0])
exu.Print('tip position of the user cable:', tip)
exu.Print('largest difference to ObjectANCFCable2D:', difference)

u += int(difference > 1e-8)
exu.Print('solution of userANCFCable2DTest=', u)

exu.sys['testResult'] = u
