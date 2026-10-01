#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Performance test of the connector interface (#2745) against the legacy path of the
#           connectors: chains of bodies joined by connectors of the three marker kinds - position
#           (ObjectConnectorSpringDamper on body markers at offset points of rigid bodies,
#           ObjectConnectorGravity on mass points), coordinate (ObjectConnectorCoordinateSpringDamper
#           on Mass1D) and rigid (ObjectConnectorRigidBodySpringDamper) - each solved twice, with
#           exu.experimental.connectorInterfaceLegacy = 1 and = 0, so that the summary of the
#           performance run shows the two solver times side by side. The two runs of a pair compute
#           the same result to round-off and share their reference value - except the implicit rigid
#           pair, whose Jacobians differ (numerical on the legacy path, by AD on the new one) and whose
#           results agree to the Newton tolerance. The legacy runs go when the
#           legacy path goes (#2745).
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import numpy as np

import testRunnerTools

testIsActive = exu.sys.get('testIsActive', True)

inertia = InertiaCuboid(density=1000, sideLengths=[0.1, 0.05, 0.05])


def BuildChain(mbs, connector, nBodies, explicit):
    """a chain of nBodies bodies joined by the given connector; returns the last node"""
    oGround = mbs.AddObject(ObjectGround())
    if connector == 'coordinate':
        nGround = mbs.AddNode(NodePointGround())
        lastMarker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
    elif connector == 'rigid':
        lastMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
    else:
        lastMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround))

    for i in range(nBodies):
        if connector in ['spring', 'rigid']:   #rigid bodies; the explicit solver takes no Euler parameters
            if explicit:
                n = mbs.AddNode(NodeRigidBodyRotVecLG(referenceCoordinates=[0.2*(i+1), 0, 0, 0, 0, 0],
                                                      initialVelocities=[0, 0.1*np.sin(i), 0.05, 0.1, 0.2, 0.3]))
            else:
                n = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0.2*(i+1), 0, 0] + list(eulerParameters0),
                                                initialVelocities=[0, 0.1*np.sin(i), 0.05] + list(
                                                    AngularVelocity2EulerParameters_t([0.1, 0.2, 0.3], eulerParameters0))))
            b = mbs.AddObject(ObjectRigidBody(nodeNumber=n, physicsMass=inertia.Mass(), physicsInertia=inertia.GetInertia6D()))
            MarkerType = MarkerBodyRigid if connector == 'rigid' else MarkerBodyPosition
            m0 = mbs.AddMarker(MarkerType(bodyNumber=b, localPosition=[-0.05, 0.01, 0]))
            m1 = mbs.AddMarker(MarkerType(bodyNumber=b, localPosition=[0.05, 0.01, 0]))
        elif connector == 'gravity':
            n = mbs.AddNode(NodePoint(referenceCoordinates=[0.2*(i+1), 0, 0], initialVelocities=[0, 0.1*np.sin(i), 0.05]))
            mbs.AddObject(MassPoint(nodeNumber=n, physicsMass=1))
            m0 = m1 = mbs.AddMarker(MarkerNodePosition(nodeNumber=n))
        else:
            n = mbs.AddNode(Node1D(referenceCoordinates=[0], initialVelocities=[0.1*np.sin(i)]))
            mbs.AddObject(Mass1D(nodeNumber=n, physicsMass=1))
            m0 = m1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n, coordinate=0))

        if connector == 'spring':
            mbs.AddObject(SpringDamper(markerNumbers=[lastMarker, m0], referenceLength=0.1, stiffness=1000, damping=1))
        elif connector == 'gravity':
            mbs.AddObject(ObjectConnectorGravity(markerNumbers=[lastMarker, m0], gravitationalConstant=1e-2,
                                                 mass0=1, mass1=1, minDistanceRegularization=0.01))
        elif connector == 'coordinate':
            mbs.AddObject(CoordinateSpringDamper(markerNumbers=[lastMarker, m0], stiffness=1000, damping=1))
        else:
            mbs.AddObject(RigidBodySpringDamper(markerNumbers=[lastMarker, m0], stiffness=np.diag([1000]*3+[10]*3),
                                                damping=np.diag([1]*3+[0.01]*3), offset=[0.1, 0, 0, 0, 0, 0]))
        lastMarker = m1
    return n


#The runs: the connector, the size, explicit (RK44, h=1e-4) or implicit (generalized-alpha, h=1e-3), and the steps,
#chosen for about 0.3 s of solver time per run (measured 2026-10-01, Windows cp313)
runList = [
    {'connector': 'spring',     'nBodies': 200,  'explicit': False, 'numberOfSteps': 100},
    {'connector': 'spring',     'nBodies': 200,  'explicit': True,  'numberOfSteps': 400},
    {'connector': 'gravity',    'nBodies': 200,  'explicit': False, 'numberOfSteps': 800},
    {'connector': 'coordinate', 'nBodies': 1000, 'explicit': True,  'numberOfSteps': 1200},
    {'connector': 'rigid',      'nBodies': 100,  'explicit': False, 'numberOfSteps': 100},  #legacy: numerical Jacobian; new: by AD
    {'connector': 'rigid',      'nBodies': 100,  'explicit': True,  'numberOfSteps': 900},
    ]

result = 0
for run in runList:
    for legacy in [1, 0]:
        exu.experimental.connectorInterfaceLegacy = legacy
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        lastNode = BuildChain(mbs, run['connector'], run['nBodies'], run['explicit'])
        mbs.Assemble()

        h = 1e-4 if run['explicit'] else 1e-3
        simulationSettings = exu.SimulationSettings()
        simulationSettings.timeIntegration.numberOfSteps = run['numberOfSteps']
        simulationSettings.timeIntegration.endTime = run['numberOfSteps']*h
        simulationSettings.solutionSettings.writeSolutionToFile = False
        simulationSettings.timeIntegration.verboseMode = 1
        simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
        mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.RK44 if run['explicit']
                         else exu.DynamicSolverType.GeneralizedAlpha)

        result = float(np.abs(mbs.systemData.GetODE2Coordinates()).sum())
        runName = ('perfConnectorInterface:' + run['connector'] + '-n' + str(run['nBodies'])
                   + ('-explicit' if run['explicit'] else '-implicit') + ('-legacy' if legacy else ''))
        exu.Print('result ' + runName + '=', result)
        testRunnerTools.AddTiming(runName, mbs, result)

exu.experimental.connectorInterfaceLegacy = 0
exu.sys['testResult'] = result
