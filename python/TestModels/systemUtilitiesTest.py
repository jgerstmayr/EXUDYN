#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  The quantities of a whole system, exudyn.systemUtilities (#2997), on a pendulum of a rigid body on a revolute
#           joint with a mass point on a spring and a distance constraint, under gravity and a constant force:
#           (1) SystemEnergy of all items, of chosen items, its contributions per item and a sensor of one component;
#           (2) a load with a user function, which has no potential: listed in .unavailable, or raising;
#           (3) SystemConstraintDrift after an index 2 solution: the largest drift, the same as from
#           mbs.ComputeConstraintResiduals, the contribution of each constraint and node, and its sensor;
#           (4) ComputeSystemGraph, the graph of the items without drawing, if networkx is installed.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-16
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface, rigidBodyUtilities and systemUtilities
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround = mbs.CreateGround()
oBody = mbs.CreateRigidBody(referencePosition=[0.5, 0, 0], inertia=InertiaCuboid(1000, [1, 0.1, 0.1]),
                            gravity=[0, -9.81, 0], initialVelocity=[0, 0.5, 0], initialAngularVelocity=[0, 0, 1])
oJoint = mbs.CreateRevoluteJoint(bodyNumbers=[oGround, oBody], position=[0, 0, 0], axis=[0, 0, 1])
oMass = mbs.CreateMassPoint(referencePosition=[1.5, 0, 0], physicsMass=2, gravity=[0, -9.81, 0])
mbs.CreateSpringDamper(bodyNumbers=[oBody, oMass], localPosition0=[0.5, 0, 0], stiffness=1000, damping=0)
oDistance = mbs.CreateDistanceConstraint(bodyNumbers=[oGround, oMass], localPosition0=[0, 0, 0], distance=1.5)
lForce = mbs.CreateForce(bodyNumber=oBody, localPosition=[0.5, 0, 0], loadVector=[0, 2, 0])
mbs.Assemble()

checks = []

#(1) the energies: all items, chosen items, per item
energy = SystemEnergy(mbs)
values = energy.Compute()
exu.Print('(1) SystemEnergy', energy.names, values, 'of', len(energy.items), 'items')
contributions = energy.ComputeItems()
checks += [abs(sum(v[3] for (item, v) in contributions) - values[3]) < 1e-14, abs(values[3] - sum(values[:3])) < 1e-14,
           len(energy.unavailable) == 0]
energyBody = SystemEnergy(mbs, items=[oBody, lForce])
exu.Print('    of the body and the force:', energyBody.Compute(), 'items', [str(item) for item in energyBody.items])
checks += [len(energyBody.items) == 2] #the gravity loads, mass-proportional, are not chosen
sTotal = energy.AddSensor(components=['total'])
mbs.Assemble()

#(2) a load without a potential: listed, or raising with skipUnavailable=False
lUser = mbs.AddLoad(LoadForceVector(markerNumber=mbs.AddMarker(MarkerBodyPosition(bodyNumber=oMass)),
                                    loadVectorUserFunction=lambda mbs, t, load: [0, 0, 0]))
mbs.Assemble()
energy.Refresh()
exu.Print('(2) unavailable:', energy.unavailable)
refused = ''
try:
    SystemEnergy(mbs, skipUnavailable=False)
except NotImplementedError as error:
    refused = str(error)
exu.Print('    refused:', refused)
checks += [len(energy.unavailable) == 1 and int(energy.unavailable[0][0]) == int(lUser), 'not available' in refused]

#(3) the drift of an index 2 solution
drift = SystemConstraintDrift(mbs)
sDrift = drift.AddSensor()
mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 500
simulationSettings.timeIntegration.endTime = 1
simulationSettings.timeIntegration.verboseMode = 0
simulationSettings.solution.file.write = False
simulationSettings.solution.sensors.writePeriod = 0.01
mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2, index2CheckConstraints=False)

[positionDrift, velocityResidual] = drift.Compute()
residuals = mbs.ComputeConstraintResiduals()[0]
exu.Print('(3) drift', positionDrift, 'velocity residual', velocityResidual, 'items', [str(item) for item in drift.items])
checks += [abs(positionDrift - max(abs(residuals))) < 1e-15, velocityResidual < 1e-8, 1e-12 < positionDrift < 1e-3,
           len(drift.items) == 3] #the joint, the distance constraint, the Euler parameters of the body's node
storedDrift = mbs.GetSensorStoredData(sDrift)
storedTotal = mbs.GetSensorStoredData(sTotal)
exu.Print('    sensors: largest drift', max(storedDrift[:, 1]), ', total energy', storedTotal[0, 1], '->', storedTotal[-1, 1])
checks += [storedDrift.shape[1] == 3, storedTotal.shape[1] == 2, abs(storedDrift[-1, 1] - positionDrift) < 1e-15]

#(4) the graph, without drawing
try:
    import networkx as nx
    G = ComputeSystemGraph(mbs)
    exu.Print('(4) graph:', G.number_of_nodes(), 'nodes,', G.number_of_edges(), 'edges,', nx.number_connected_components(G),
              'connected part')
    checks += [G.number_of_nodes() == mbs.systemData.NumberOfNodes() + mbs.systemData.NumberOfObjects() +
               mbs.systemData.NumberOfMarkers() + mbs.systemData.NumberOfLoads() + mbs.systemData.NumberOfSensors(),
               nx.number_connected_components(G) == 1 + 2] #the two sensors of the whole system have no edge
except ImportError:
    exu.Print('(4) graph: networkx is not installed; skipped')

exu.Print('checks:', checks)
u = sum(checks) - len(checks) + values[3] + positionDrift
exu.Print('solution of systemUtilitiesTest=', u)

exu.sys['testResult'] = u
