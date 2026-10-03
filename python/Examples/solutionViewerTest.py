# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A chain of 100 rigid bodies, each turned against the previous one, linked by revolute joints and
#           falling under gravity; the frames of the chain are homogeneous transformations (exu.HT): the
#           bodies take theirs as referenceHT, the joint markers as localHT. The solution is shown again
#           with the SolutionViewer.
#
# Author:   Johannes Gerstmayr 
# Date:     2021-07-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics

from math import sin, cos, pi
import numpy as np

SC = exu.SystemContainer()
mbs = SC.AddSystem()


#background
color = [0.1,0.1,0.8,1]
L = 0.4 #length of bodies
d = 0.1 #diameter of bodies

oGround=mbs.AddObject(ObjectGround(referenceHT=exu.HT(translation=[-0.5*L,0,0])))
nBodies = 100
delta = 0.01*pi
g = [0,0,9.81]

#the frames of the chain are homogeneous transformations (exu.HT): the joint frame of each body is the joint
#frame of the previous one, moved along the body by L and turned by Hturn; the body sits halfway in between
Hhalf = exu.HT(translation=[0.5*L,0,0])                    #from a joint to the body center, and on to the next joint
Hturn = exu.HT().SetRotationX(delta) * exu.HT().SetRotationZ(2*delta) #the turn from one body to the next
Hjoint = exu.HT()                                          #the first joint at the origin; its z-axis is the joint axis
bodyLast = oGround
HturnLast = exu.HT()                                       #the ground is not turned against the first body

for i in range(nBodies):
    inertia = InertiaCuboid(density=1000, sideLengths=[L,d,d])
    graphicsBody = graphics.Brick([0,0,0], [0.96*L,d,d], graphics.color.steelblue)
    oRB = mbs.CreateRigidBody(inertia=inertia,
                              referenceHT=Hjoint*Hhalf,
                              gravity=g,
                              graphicsDataList=[graphicsBody])
    nRB= mbs.GetObject(oRB)['nodeNumber']

    #the joint frame in each body is the localHT of its marker: at the end of the previous body, turned to this
    #body, and at the start of this body
    mLast = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bodyLast, localHT=Hhalf*HturnLast))
    mThis = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRB, localHT=Hhalf.Inverse()))
    mbs.AddObject(RevoluteJointZ(markerNumbers=[mLast, mThis],
                                 visualization=VRevoluteJointZ(axisRadius=0.6*d, axisLength=1.2*d)))

    bodyLast = oRB
    HturnLast = Hturn
    Hjoint = Hjoint*Hhalf*Hhalf*Hturn                      #the next joint frame


mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

tEnd = 1
h=0.0005  #use small step size to detext contact switching

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solution.file.writePeriod = 0.005
simulationSettings.solution.sensors.writePeriod = 0.01
#simulationSettings.timeIntegration.realtime.active = True
simulationSettings.timeIntegration.realtime.factor = 0.5
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.8
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
simulationSettings.timeIntegration.newton.useModifiedNewton = True
#simulationSettings.timeIntegration.newton.modifiedNewtonJacUpdatePerStep = True
simulationSettings.linearSolver.solverType = exu.LinearSolverType.EigenSparse
# simulationSettings.parallel.numberOfThreads=4

SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.nodes.drawNodesAsPoint  = False
SC.visualizationSettings.nodes.showBasis = True
SC.visualizationSettings.nodes.basisSize = 0.015
SC.visualizationSettings.connectors.showJointAxes = True

#for snapshot:
SC.visualizationSettings.openGL.multiSampling=4
SC.visualizationSettings.openGL.lineWidth=2
SC.visualizationSettings.view0.window.renderWindowSize = [800,600]
SC.visualizationSettings.view0.scene.drawCoordinateSystem=False
SC.visualizationSettings.view0.scene.drawWorldBasis=True
# SC.visualizationSettings.general.useMultiThreadedRendering = False
SC.visualizationSettings.general.autoFitScene = False #use loaded render state


#test UTF-8 characters:
text = 'Demo UTF-8 text:ΓΔΘΛΞΠΣΦΨΩ\nαβγδεζηθικλμνξοπρστυφχψωϕϵ\n'
text+= 'x₀₁₂₃₄₅₆₇₈₉x⁰¹²³⁴⁵⁶⁷⁸⁹\n∂∫♥√≈∞🙂😒°×·\nüöäÜÖÄßéèáàØ§ÿ~'

SC.visualizationSettings.general.renderWindowString = text
SC.visualizationSettings.view0.window.globalFontSize = 14 #to see special characters
useGraphics = True
if useGraphics:
    simulationSettings.show.computationTime = True
    simulationSettings.show.statistics = True
    SC.renderer.Start()
    SC.renderer.RestoreSavedState()
    #SC.renderer.DoIdleTasks()
else:
    simulationSettings.solution.file.write = False

#mbs.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)
mbs.SolveDynamic(simulationSettings, showHints=True)

if True: #use this to reload the solution and use SolutionViewer
    #sol = LoadSolutionFile(OutputFilePath('coordinatesSolution.txt'))
    
    mbs.SolutionViewer() #can also be entered in IPython ...


u0 = mbs.GetNodeOutput(nRB, exu.OutputVariableType.Displacement)
rot0 = mbs.GetNodeOutput(nRB, exu.OutputVariableType.Rotation)
exu.Print('u0=',u0,', rot0=', rot0)

result = (abs(u0)+abs(rot0)).sum()
exu.Print('solution of addRevoluteJoint=',result)



#%%+++++++++++++++++++++++++++++
if useGraphics:
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!


