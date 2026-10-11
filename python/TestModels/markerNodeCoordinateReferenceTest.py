#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  MarkerNodeCoordinate with includeReferenceCoordinate=True (#990): the marker gives the coordinate including
#           its reference value, so a constraint or a spring acts on absolute positions instead of displacements:
#           a mass point with reference position 2 is held at the absolute position 2.5 by
#           CreateCoordinateConstraint(includeReferenceCoordinates=[False, True], offset=2.5), and a second one
#           hangs on a CoordinateSpringDamper between a ground node at x=1 and its own absolute position.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-11
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#a mass point at x=2, held at the ABSOLUTE position x=2.5; the displacement is 0.5
mass0 = mbs.CreateMassPoint(referencePosition=[2, 0, 0], mass=1)
mbs.CreateCoordinateConstraint(itemNumbers=[None, mass0], coordinates=[None, 0], offset=2.5,
                               includeReferenceCoordinates=[False, True])
mbs.CreateCoordinateConstraint(itemNumbers=[None, mass0], coordinates=[None, 1])
mbs.CreateCoordinateConstraint(itemNumbers=[None, mass0], coordinates=[None, 2])

#a spring between a ground node at x=1 and a mass point at x=3, both with their reference values: the spring of
#length offset=2 is unstressed in the reference configuration, so the force F stretches it by F/k
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[1, 0, 0]))
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0, includeReferenceCoordinate=True))
node1 = mbs.AddNode(NodePoint(referenceCoordinates=[3, 0, 0]))
mbs.AddObject(MassPoint(nodeNumber=node1, mass=1))
m1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node1, coordinate=0, includeReferenceCoordinate=True))
k = 100
F = 5
mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mGround, m1], stiffness=k, offset=2))
mbs.AddLoad(LoadCoordinate(markerNumber=m1, load=F))
for coordinate in [1, 2]:
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0)),
                                                      mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=node1, coordinate=coordinate))]))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.solution.file.write = False
simulationSettings.staticSolver.verboseMode = 0
mbs.SolveStatic(simulationSettings)

x0 = mbs.GetObjectOutputBody(mass0, exu.OutputVariableType.Position, localPosition=[0, 0, 0])[0]
x1 = mbs.GetNodeOutput(node1, exu.OutputVariableType.Position)[0]
exu.Print('position of the held mass point:', x0, '(2.5)')
exu.Print('position of the mass point on the spring:', x1, '(3 + F/k = 3.05)')

errors = int(abs(x0 - 2.5) > 1e-12) + int(abs(x1 - (3 + F/k)) > 1e-12)
u = errors + x0 + x1
exu.Print('solution of markerNodeCoordinateReferenceTest=', u)

exu.sys['testResult'] = u
