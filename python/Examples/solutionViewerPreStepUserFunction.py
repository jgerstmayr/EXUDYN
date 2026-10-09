# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A ground object - a brick with its frame drawn - moves along a screw (a rotation about z and a
#           translation along z), prescribed by a PreStepUserFunction that sets its referenceHT as an exu.HT;
#           a pendulum hangs beside it. The SolutionViewer shows the solution with
#           callPreStepUserFunction=True, so the ground moves in the viewer as in the simulation: the
#           function is called with the time of each shown step before it is drawn. Space starts the
#           viewer in 'Continuous run' and stops it in 'Static'.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-10
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics

import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#the moving ground: a brick and its frame, drawn in the frame of the ground object
graphicsGround = [graphics.Brick(size=[0.4, 0.3, 0.2], color=graphics.color.steelblue),
                  graphics.Basis(length=0.5, radius=0.01)]
oGround = mbs.CreateGround(graphicsDataList=graphicsGround)

#a pendulum beside it, so that there is a solution to show
oBody = mbs.CreateRigidBody(inertia=InertiaCuboid(density=1000, sideLengths=[0.1, 0.5, 0.1]),
                            referencePosition=[1, -0.25, 0], gravity=[0, -9.81, 0],
                            graphicsDataList=[graphics.Brick(size=[0.1, 0.5, 0.1], color=graphics.color.orange)])
mbs.CreateSphericalJoint(itemNumbers=[mbs.CreateGround(), oBody], position=[1, 0, 0])

def GroundHT(t):
    """the screw motion: one turn about z in 4 s, rising 0.25 per turn"""
    angle = 2*np.pi*t/4
    return exu.HT(Rxyz=[0, 0, angle], translation=[0, 0, 0.25*angle/(2*np.pi)])

def PreStepUserFunction(mbs, t):
    mbs.SetObjectParameter(oGround, 'referenceHT', GroundHT(t))
    return True

mbs.SetPreStepUserFunction(PreStepUserFunction)
mbs.Assemble()

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 400
simulationSettings.timeIntegration.endTime = 4
simulationSettings.solution.file.writePeriod = 0.02
SC.visualizationSettings.general.autoFitScene = False

mbs.SolveDynamic(simulationSettings)

#the ground moves in the viewer too, because its PreStepUserFunction is called for each shown step
mbs.SolutionViewer(callPreStepUserFunction=True)
