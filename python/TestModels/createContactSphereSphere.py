#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test for mass points and sphere-sphere contact
#
# Author:   Johannes Gerstmayr
# Date:     2026-04-06
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
import numpy as np

testIsActive = exu.sys.get('testIsActive', False)

SC = exu.SystemContainer()
mbs = SC.AddSystem()

radius = 0.1
mass = 1
distance = 1.0

oMass1 = mbs.CreateMassPoint(
    referencePosition=[-distance/2,0,0],
    initialVelocity=[1,0,0],
    mass=mass,
    drawSize = 2*radius, 
    color=exu.graphics.color.blue,
)

oMass2 = mbs.CreateMassPoint(
    referencePosition=[distance/2,0,0],
    initialVelocity=[-1,0,0],
    mass=mass,
    drawSize = 2*radius, 
    color=exu.graphics.color.red,
)

mbs.CreateSphereSphereContact(itemNumbers=[oMass1, oMass2],
                              spheresRadii =[radius,radius],
                              contactStiffness=1e4,
                              dynamicFriction=0,
                              impactModel = 2, #to use restitution coefficient
                              restitutionCoefficient=0.2)


mbs.Assemble()

stepSize = 1e-5
tEnd = 1

simulationSettings = exu.SimulationSettings()
simulationSettings.solution.file.write = (not testIsActive)
simulationSettings.solution.file.writePeriod = 0.02
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
simulationSettings.timeIntegration.endTime = tEnd

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1

simulationSettings.show.statistics = True
simulationSettings.timeIntegration.verboseMode = 1

SC.visualizationSettings.view0.window.renderWindowSize=[1600,2000]
SC.visualizationSettings.openGL.multiSampling=4
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.tiling = 16

#start solver:q
mbs.SolveDynamic(simulationSettings)

nMass1 = mbs.GetObject(oMass1)['nodeNumber']
uTotal = mbs.GetNodeOutput(nMass1, exu.OutputVariableType.CoordinatesTotal).sum()
exu.Print('uTotal=',uTotal)

exu.sys['testResult'] = uTotal

mbs.SolutionViewer()
