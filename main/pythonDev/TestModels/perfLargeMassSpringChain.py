#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Performance test with a LARGE number of coordinates, as the counterpart to the
#           small-system tests in the performance suite. perfRigidPendulum and the two
#           spring-damper tests run a handful of coordinates for ~1e6 steps, so they measure
#           per-step overhead; their system vectors are 3-20 elements long and no vectorized
#           linear algebra can show up in them (revision plan step 23, issue #2397).
#
#           This model instead uses a chain of nMasses point masses coupled by coordinate
#           spring-dampers, integrated explicitly: 3*nMasses ODE2 coordinates, no linear solver,
#           and comparatively few time steps. The system vectors are therefore thousands of
#           elements long, which is the regime where AVX2 and multithreading can matter.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-12
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *

useGraphics = False #without test
#the following is used to always get the same results, independent of the test suite
try:
    from modelUnitTests import exudynTestGlobals #for globally storing test results
    useGraphics = exudynTestGlobals.useGraphics
except:
    class ExudynTestGlobals:
        pass
    exudynTestGlobals = ExudynTestGlobals()
    exudynTestGlobals.useGraphics = False

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
nMasses = 2000      #=> 3*nMasses = 6000 ODE2 coordinates
mass = 1.6          #mass in kg
spring = 4000       #stiffness of spring-damper in N/m
damper = 8          #damping constant in N/(m/s)
L = 0.5             #distance between masses

nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))
groundMarker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))

lastMarker = groundMarker
lastNode = None
for i in range(nMasses):
    #small initial displacement, varying along the chain, so the motion is not uniform
    u0 = 0.01*(1. + (i % 7))
    n = mbs.AddNode(Point(referenceCoordinates=[L*(i+1), 0, 0],
                          initialCoordinates=[u0, 0, 0],
                          initialVelocities=[0, 0, 0]))
    mbs.AddObject(MassPoint(physicsMass=mass, nodeNumber=n))

    nodeMarker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n, coordinate=0))
    mbs.AddObject(CoordinateSpringDamper(markerNumbers=[lastMarker, nodeMarker],
                                         stiffness=spring, damping=damper))
    lastMarker = nodeMarker
    lastNode = n

mbs.Assemble()

tEnd = 0.5          #end time of simulation
h = 5e-5            #step size => 10000 steps

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.writeSolutionToFile = False
simulationSettings.solutionSettings.sensorsWritePeriod = 2e7 #no sensor output
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.displayComputationTime = False

simulationSettings.timeIntegration.explicitIntegration.useLieGroupIntegration = False

#EigenSparse is ESSENTIAL here, not a tuning detail: with the default dense linear solver this
#model costs O(N^2) per step even under explicit integration - measured 2026-09-12, per-step time
#quadruples on every doubling of nMasses (250/500/1000/2000 -> 2.5/10.1/42/168 ms) and nMasses=2000
#runs ~280x slower than with EigenSparse (32.6 s against 0.115 s for 200 steps). See issue #2398.
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

#NOTE explicitIntegration.computeMassMatrixInversePerBody is the flag intended for exactly this
#case - it inverts the mass matrix per body instead of solving a system - and it is deliberately
#NOT set here. Two reasons. It is only correct when bodies do not share nodes, which holds for
#this chain but is a precondition a later edit could silently break (it is wrong for a beam or an
#FEM body). And measured on this model it does not remove the cost by itself: with the dense
#solver it changes nothing (8.57 s against 8.43 s at nMasses=1000, identical within noise, for
#ExplicitEuler, RK44 and DOPRI5 alike); it is worth a further ~10-15% only once EigenSparse has
#removed the dominant term. See issue #2400.

mbs.SolveDynamic(simulationSettings,
                 solverType=exu.DynamicSolverType.ExplicitEuler)

#displacement of the last mass in the chain
u = mbs.GetNodeOutput(lastNode, exu.OutputVariableType.Displacement)
result = abs(u[0])

exu.Print('result perfLargeMassSpringChain=', result)

exudynTestGlobals.testError = 0     #filled by the performance suite against its reference
exudynTestGlobals.testResult = result
