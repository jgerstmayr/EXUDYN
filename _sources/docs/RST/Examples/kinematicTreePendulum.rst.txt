
.. _examples-kinematictreependulum:

************************
kinematicTreePendulum.py
************************

You can view and download this file on Github: `kinematicTreePendulum.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/kinematicTreePendulum.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN python utility library
   #
   # Details:  test of MarkerKinematicTreeRigid in combination with loads and joint
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-05-29
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.FEM import *
   
   import numpy as np
   
   from math import pi, sin, cos#, sqrt
   from copy import copy, deepcopy
   from exudyn.robotics import *
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   useGraphics = True
   
   gGround =  graphics.CheckerBoard(point= [0,0,-2], size = 12)
   objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0],
                                             visualization=VObjectGround(graphicsData=[gGround])))
   
   L = 0.5 #length
   w = 0.1 #width of links
   
   gravity3D = [0,-9.81*1,0]
   graphicsBaseList = [graphics.Brick(size=[L*4, 0.8*w, 0.8*w], color=graphics.color.grey)] #rail
   
   newRobot = Robot(gravity=gravity3D,
                 base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
                 tool = RobotTool(HT=HTtranslate([0,0.5*L,0]), visualization=VRobotTool(graphicsData=[
                     graphics.Brick(size=[w, L, w], color=graphics.color.orange)])),
                 referenceConfiguration = []) #referenceConfiguration created with 0s automatically
   
   linksList = []
   
   nChainLinks = 1 #5
   for i in range(nChainLinks):
       Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
       Jlink = Jlink.Translated([0,0.5*L,0])
       preHT = HT0()
       if i > 0:
           preHT = HTtranslateY(L)
   
       link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                        jointType='Rz', preHT=preHT, 
                        #PDcontrol=(pControl*0, dControl*0),
                        visualization=VRobotLink(linkColor=graphics.color.blue))
       newRobot.AddLink(link)
       linksList += [copy(link)]
   
   #newRobot.referenceConfiguration[0] = 0.5*0
   # for i in range(nChainLinks):
   #     newRobot.referenceConfiguration[i+1] = (2*pi/360) * 5 
   newRobot.referenceConfiguration[0] = -(2*pi/360) * 90 #-0.5*pi
   # newRobot.referenceConfiguration[2] = (2*pi/360) * 12 #-0.5*pi
       
   
   dKT = newRobot.CreateKinematicTree(mbs)
   oKT = dKT['objectKinematicTree']
   
   sCoords=mbs.AddSensor(SensorBody(bodyNumber=oKT, storeInternal=True,
                       outputVariableType=exu.OutputVariableType.Coordinates))
   
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   
   tEnd = 2000
   h = 1e-2#*0.01
   #tEnd = h
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.writeSolutionToFile=False
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.05
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.95 #SHOULD work with 0.9 as well
   
   SC.visualizationSettings.general.autoFitScene=False
   SC.visualizationSettings.window.renderWindowSize = [1600,1200]
   SC.visualizationSettings.general.drawCoordinateSystem=True
   SC.visualizationSettings.general.drawWorldBasis=True
   SC.visualizationSettings.openGL.multiSampling=4
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.5
   
   if useGraphics:
   
       SC.renderer.Start()
       if 'renderState' in exu.sys: SC.renderer.SetState(exu.sys['renderState']) #load last model view
   
       SC.renderer.DoIdleTasks() #press space to continue
   
   
   
   
   mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.ExplicitMidpoint)
   mbs.PlotSensor(sensorNumbers=sCoords, components=0, labels='Explicit Midpoint', colorCodeOffset=2, closeAll=True)
   
   mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.RK33)
   mbs.PlotSensor(sensorNumbers=sCoords, components=0, labels='Heun', colorCodeOffset=1, newFigure=False)
   
   mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.RK44)
   mbs.PlotSensor(sensorNumbers=sCoords, components=0, labels='Runge Kutta 44', newFigure=False)
   
   
   #mbs.SolveDynamic(simulationSettings)
   
   simulationSettings.timeIntegration.numberOfSteps = int(7/h)
   simulationSettings.timeIntegration.endTime = 7
   mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.ExplicitEuler)
   mbs.PlotSensor(sensorNumbers=sCoords, components=0, yLabel='pendulum angle', labels=['Explicit Euler'], colorCodeOffset=3, newFigure=False)
   
   if useGraphics:
       #SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
       
           
   
   


