#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The mass integral of a user body (#3063): with a massIntegralUserFunction, which returns the mass integral
#           of the position Jacobian int(rho J_pos dV), MarkerBodyMass works on ObjectUserBody and LoadMassProportional
#           applies gravity. (1) A cantilever of four user ANCF cable elements, as in userANCFCable2DTest.py, under
#           gravity against ObjectANCFCable2D; the mass integral is the integral of the shape functions times rhoA.
#           (2) A mass point as a user body against MassPoint, with a Python and a symbolic mass integral. (3) A
#           mass-proportional load on a user body without the function is refused.
#           Run alone, it draws the user bodies with their graphicsData and opens the SolutionViewer.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-23
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
from exudyn.advancedUtilities import CreateSymbolicUserFunction
from exudyn.cMath import sqrt
import exudyn.graphics as graphics
import numpy as np
esym = exu.symbolic
testIsActive = exu.sys.get('testIsActive', False) #run alone: the renderer, the solution file and the SolutionViewer

gravity = [0, -9.81, 0]
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 200
simulationSettings.timeIntegration.endTime = 0.2
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.timeIntegration.newton.relativeTolerance = 1e-10
simulationSettings.solution.file.write = False

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(1) the user ANCF cable element of userANCFCable2DTest.py, with its mass integral
nElements = 4
length = 2
Le = length/nElements #length of one element
rhoA = 1    #mass per length
EA = 1000   #axial stiffness
EI = 2      #bending stiffness

#the cubic Hermite shape functions and their derivatives at x in [0, Le], as 2 x 8 matrices
def S(x):
    xi = x/Le
    return np.kron([1 - 3*xi**2 + 2*xi**3, x - 2*x**2/Le + x*xi**2, 3*xi**2 - 2*xi**3, -x*xi + x*xi**2], np.eye(2))

def S_x(x):
    return np.kron([-6*x/Le**2 + 6*x**2/Le**3, 1 - 4*x/Le + 3*x**2/Le**2, 6*x/Le**2 - 6*x**2/Le**3, -2*x/Le + 3*x**2/Le**2], np.eye(2))

def S_xx(x):
    return np.kron([12*x/Le**3 - 6/Le**2, 6*x/Le**2 - 4/Le, 6/Le**2 - 12*x/Le**3, 6*x/Le**2 - 2/Le], np.eye(2))

def GaussRule(n):
    (points, weights) = np.polynomial.legendre.leggauss(n)
    return [(0.5*Le*(p + 1), 0.5*Le*w) for (p, w) in zip(points, weights)]

axialRule = GaussRule(5)
bendingRule = GaussRule(3)
massMatrix = sum(w*rhoA*S(x).T @ S(x) for (x, w) in GaussRule(4))
#int(rhoA J_pos dx): the position Jacobian [S(x); 0] does not depend on the coordinates
massIntegral = np.vstack([sum(w*rhoA*S(x) for (x, w) in GaussRule(4)), np.zeros(8)])

def UFmass(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return massMatrix

def UFmassIntegral(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return massIntegral

def UFforce(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    q = np.array(qTotal)
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

def UFposition(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return list(S(localPosition[0]) @ qTotal) + [0]

def UFpositionJacobian(mbs, t, itemNumber, localPosition, qRef, qTotal, q_t, parameters):
    return np.vstack([S(localPosition[0]), np.zeros(8)])

#the axis of the element, mapped by the positionUserFunction onto the deformed cable
axisGraphics = graphics.Lines([[x, 0, 0] for x in np.linspace(0, Le, 9)], color=graphics.color.red)

SC = exu.SystemContainer()
mbs = SC.AddSystem()
nGround = mbs.AddNode(NodePointGround())
cables = []
for isUser in [False, True]:
    nodes = [mbs.AddNode(NodePoint2DSlope1(referenceCoordinates=[i*Le, 0, 1, 0])) for i in range(nElements + 1)]
    elements = []
    for i in range(nElements):
        if isUser:
            element = mbs.AddObject(ObjectUserBody(nodeNumbers=nodes[i:i+2], massMatrixUserFunction=UFmass,
                                                   massIntegralUserFunction=UFmassIntegral, centerOfMass=[0.5*Le, 0, 0],
                                                   forceUserFunction=UFforce, positionUserFunction=UFposition,
                                                   positionJacobianUserFunction=UFpositionJacobian,
                                                   visualization=VObjectUserBody(graphicsData=[axisGraphics])))
        else:
            element = mbs.AddObject(ObjectANCFCable2D(nodeNumbers=nodes[i:i+2], length=Le, massPerLength=rhoA,
                                                      bendingStiffness=EI, axialStiffness=EA))
        mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=element)), loadVector=gravity))
        elements += [element]
    for coordinate in [0, 1, 3]: #clamped at x=0
        mbs.AddObject(CoordinateConstraint(markerNumbers=[mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0)),
                                                          mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nodes[0], coordinate=coordinate))]))
    cables += [elements]

mbs.Assemble()
if not testIsActive:
    simulationSettings.solution.file.write = True
    SC.renderer.Start()
mbs.SolveDynamic(simulationSettings)
if not testIsActive:
    SC.renderer.Stop()
    simulationSettings.solution.file.write = False
    mbs.SolutionViewer()

differenceCable = 0
u = 0
for i in range(nElements):
    for x in [0.5*Le, Le]:
        for output in [exu.OutputVariableType.Position, exu.OutputVariableType.Velocity]:
            values = [np.array(mbs.GetObjectOutputBody(cable[i], output, localPosition=[x, 0, 0])) for cable in cables]
            differenceCable = max(differenceCable, np.max(np.abs(values[1] - values[0])))
            u += np.sum(values[1])
exu.Print('cable under gravity: tip of the user cable', mbs.GetObjectOutputBody(cables[1][-1], exu.OutputVariableType.Position,
          localPosition=[Le, 0, 0]), ', largest difference to ObjectANCFCable2D:', differenceCable)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(2) a mass point as a user body: the mass integral is m*I, as Python and as symbolic function
m = 2.
def UFmassPoint(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return m*np.eye(3)

def UFmassIntegralPoint(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return m*np.eye(3)

def UFmassIntegralPointSymbolic(mbs, t, itemNumber, qRef, qTotal, q_t, parameters):
    return esym.Matrix(m*np.eye(3))

def SolveMassPoint(kind):
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    node = mbs.AddNode(NodePoint(referenceCoordinates=[0, 0, 0], initialVelocities=[1, 2, 0]))
    if kind == 'MassPoint':
        body = mbs.AddObject(MassPoint(nodeNumber=node, physicsMass=m))
    else:
        body = mbs.AddObject(ObjectUserBody(nodeNumbers=[node], massMatrixUserFunction=UFmassPoint,
                                            massIntegralUserFunction=UFmassIntegralPoint))
        if kind == 'symbolic':
            mbs.variables['keep'] = CreateSymbolicUserFunction(mbs, UFmassIntegralPointSymbolic, 'massIntegralUserFunction', body)
            mbs.SetObjectParameter(body, 'massIntegralUserFunction', mbs.variables['keep'])
    mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=body)), loadVector=gravity))
    mbs.Assemble()
    mbs.SolveDynamic(simulationSettings)
    return np.array(mbs.GetNodeOutput(node, exu.OutputVariableType.Position))

points = [SolveMassPoint(kind) for kind in ['MassPoint', 'python', 'symbolic']]
tEnd = simulationSettings.timeIntegration.endTime
exact = np.array([1*tEnd, 2*tEnd + 0.5*gravity[1]*tEnd**2, 0])
differencePoint = max(np.max(np.abs(points[1] - points[0])), np.max(np.abs(points[2] - points[0])))
exu.Print('mass point under gravity: user body', points[1], ', exact', exact, ', difference to MassPoint:', differencePoint)
u += np.sum(points[2])

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(3) without massIntegralUserFunction, a mass-proportional load on the user body is refused
SC = exu.SystemContainer()
mbs = SC.AddSystem()
node = mbs.AddNode(NodePoint())
body = mbs.AddObject(ObjectUserBody(nodeNumbers=[node], massMatrixUserFunction=UFmassPoint))
mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=body)), loadVector=gravity))
mbs.Assemble()
refused = 0
try:
    mbs.SolveDynamic(simulationSettings)
except ValueError as e:
    refused = int('massIntegralUserFunction' in str(e))
exu.Print('a mass-proportional load without massIntegralUserFunction is refused:', refused == 1)

u += int(differenceCable > 1e-8) + int(differencePoint > 1e-12) + int(np.max(np.abs(points[1] - exact)) > 1e-10) + (1 - refused)
exu.Print('solution of userMassIntegralTest=', u)

exu.sys['testResult'] = u
