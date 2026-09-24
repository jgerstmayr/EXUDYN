#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Performance test with a LARGE number of coordinates, as the counterpart to the
#           small-system tests in the performance suite. perfRigidPendulum and the two
#           spring-damper tests run a handful of coordinates for ~1e6 steps, so they measure
#           per-step overhead; their system vectors are 3-20 elements long and no vectorized
#           linear algebra can show up in them (revision2026 step R2.10, issue #2397).
#
#           This model instead builds a chain of nBodies bodies coupled by spring-dampers and
#           integrates it over three sizes, so the summary shows how the cost scales rather than
#           one point on the curve (revision2026 step R5.15, issue #2460). In the default rigid
#           body mode every body carries 7 coordinates and the connectors are RigidBodySpringDamper
#           elements, which puts the weight on the OBJECT computation - rotation parameters,
#           local-to-global transformations, connector jacobians - instead of on the linear solver.
#           The two smaller sizes are additionally integrated implicitly, which exercises the
#           jacobian and the sparse factorization.
#
#           useRigidBodies=False falls back to the original mass point chain, which is pure
#           per-step vector work with no rotations at all.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-12
# Modified: 2026-09-16 (sizes, rigid body mode, implicit runs; issue #2460)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
import numpy as np

import testRunnerTools

testIsActive = exu.sys.get('testIsActive', True)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
useRigidBodies = True   #False: the original mass point chain, 3 coordinates per body

mass = 1.6          #mass in kg
spring = 4000       #stiffness of spring-damper in N/m
damper = 8          #damping constant in N/(m/s)
L = 0.5             #distance between masses
sideLength = 0.2    #edge length of the rigid body cubes

#The runs of this model. The step size is the same for all explicit runs and for all implicit ones,
#so that only the SIZE differs between them and the per-step cost can be compared directly across
#the three sizes; the number of steps is then chosen per run to land near 1.5 s of solver time
#(measured 2026-09-16, Windows cp313). The largest size runs explicitly only: an implicit step on
#120000 coordinates costs far more than the target run time.
#h=5e-4 is well inside the explicit stability limit of this chain - the stiffest mode is the
#rotational one, omega = sqrt(0.1*spring/Jxx) ~ 193 rad/s, so h*omega ~ 0.1.
hExplicit = 5e-4
hImplicit = 5e-3
runList = [
    {'nBodies':  1000, 'implicit': False, 'numberOfSteps': 900},
    {'nBodies':  1000, 'implicit': True,  'numberOfSteps': 450},
    {'nBodies':  5000, 'implicit': False, 'numberOfSteps': 175},
    {'nBodies':  5000, 'implicit': True,  'numberOfSteps':  65},
    {'nBodies': 20000, 'implicit': False, 'numberOfSteps':  30},
    ]

result = 0
for run in runList:
    nBodies = run['nBodies']

    SC = exu.SystemContainer()
    mbs = SC.AddSystem()

    oGround = mbs.CreateGround(referencePosition=[0,0,0])
    lastBody = oGround
    lastNode = None
    for i in range(nBodies):
        #small initial displacement, varying along the chain, so the motion is not uniform
        u0 = 0.01*(1. + (i % 7))
        if useRigidBodies:
            b = mbs.CreateRigidBody(inertia=InertiaCuboid(density=mass/sideLength**3,
                                                          sideLengths=[sideLength]*3),
                                    nodeType=exu.NodeType.RotationRxyz,
                                    referencePosition=[L*(i+1)+u0, 0, 0],
                                    gravity=[0,0,0])
            mbs.CreateRigidBodySpringDamper(bodyNumbers=[lastBody, b],
                                            localPosition0=[0.5*L,0,0],
                                            localPosition1=[-0.5*L,0,0],
                                            stiffness=np.diag([spring]*3+[0.1*spring]*3),
                                            damping=np.diag([damper]*3+[0.1*damper]*3))
            lastBody = b
            lastNode = mbs.GetObject(b)['nodeNumber']
        else:
            n = mbs.AddNode(Point(referenceCoordinates=[L*(i+1), 0, 0],
                                  initialCoordinates=[u0, 0, 0],
                                  initialVelocities=[0, 0, 0]))
            mbs.AddObject(MassPoint(physicsMass=mass, nodeNumber=n))

            nodeMarker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n, coordinate=0))
            if i == 0:
                lastMarker = mbs.AddMarker(MarkerNodeCoordinate(
                    nodeNumber=mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])),
                    coordinate=0))
            mbs.AddObject(CoordinateSpringDamper(markerNumbers=[lastMarker, nodeMarker],
                                                 stiffness=spring, damping=damper))
            lastMarker = nodeMarker
            lastNode = n

    mbs.Assemble()

    h = hImplicit if run['implicit'] else hExplicit
    tEnd = run['numberOfSteps']*h

    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = run['numberOfSteps']
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.solutionSettings.writeSolutionToFile = False
    simulationSettings.timeIntegration.verboseMode = 1
    simulationSettings.displayComputationTime = False

    simulationSettings.timeIntegration.explicitIntegration.useLieGroupIntegration = False

    #EigenSparse is ESSENTIAL here, not a tuning detail: with the default dense linear solver this
    #model costs O(N^2) per step even under explicit integration - measured 2026-09-12, per-step time
    #quadruples on every doubling of nBodies (250/500/1000/2000 -> 2.5/10.1/42/168 ms) and nBodies=2000
    #runs ~280x slower than with EigenSparse (32.6 s against 0.115 s for 200 steps). See issue #2398.
    simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse

    #explicitIntegration.computeMassMatrixInversePerBody inverts the mass matrix per body instead
    #of solving a system, and this chain is exactly what the flag is made for: no two bodies share
    #a node, which is the precondition (it would be wrong for a beam or an FEM body). With the flag
    #off, the explicit runs largely measure the Eigen solver rather than the object computation this
    #model is here to measure - see issue #2400.
    simulationSettings.timeIntegration.explicitIntegration.computeMassMatrixInversePerBody = True

    if run['implicit']:
        simulationSettings.timeIntegration.newton.useModifiedNewton = True
        mbs.SolveDynamic(simulationSettings,
                         solverType=exu.DynamicSolverType.TrapezoidalIndex2)
    else:
        mbs.SolveDynamic(simulationSettings,
                         solverType=exu.DynamicSolverType.ExplicitEuler)

    #displacement of the last body in the chain
    u = mbs.GetNodeOutput(lastNode, exu.OutputVariableType.Displacement)
    result = abs(u[0])

    runName = ('perfLargeMassSpringChain:' + ('rigid' if useRigidBodies else 'mass')
               + '-n' + str(nBodies) + ('-implicit' if run['implicit'] else '-explicit'))
    exu.Print('result ' + runName + '=', result)
    testRunnerTools.AddTiming(runName, mbs, result)

exu.sys['testResult'] = result
