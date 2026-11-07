#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
# 
# Details:  Mini example for class ObjectJournalBearing
# 
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
# 
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics

import numpy as np

#create an environment for mini example
SC = exu.SystemContainer()
mbs = SC.AddSystem()

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))
nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))

#a journal under its weight in a fixed bearing, turning at 100 rad/s: it floats on the film
m, R, c, L = 10., 0.02, 1e-4, 0.01
node = mbs.AddNode(NodeRigidBodyEP(referenceCoordinates=[0,0,0]+eulerParameters0, initialVelocities=[0,0,0]+list(AngularVelocity2EulerParameters_t([0,0,100], eulerParameters0))))
journal = mbs.AddObject(ObjectRigidBody(nodeNumber=node, mass=m, inertia=[1e-3,1e-3,1e-3,0,0,0]))
mbs.AddLoad(LoadMassProportional(markerNumber=mbs.AddMarker(MarkerBodyMass(bodyNumber=journal)), loadVector=[0,-9.81,0]))
mJournal = mbs.AddMarker(MarkerBodyRigid(bodyNumber=journal))
mBearing = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround))
mbs.AddObject(ObjectJournalBearing(markerNumbers=[mJournal, mBearing], journalRadius=R-c, bearingRadius=R,
                                   bearingLength=L, viscosity=0.04, contactStiffness=1e8))

mbs.Assemble()
simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 1000
simulationSettings.timeIntegration.endTime = 0.5
mbs.SolveDynamic(simulationSettings)

#the eccentricity of the journal, below the clearance 1e-4
exu.sys['testResult'] = mbs.GetObjectOutput(mbs.systemData.NumberOfObjects()-1, exu.OutputVariableType.Displacement)[1]

exu.Print("example for ObjectJournalBearing completed, test result =", exu.sys['testResult'])

