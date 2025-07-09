
.. _examples-gyrostability:

****************
gyroStability.py
****************

You can view and download this file on Github: `gyroStability.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/gyroStability.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example showing stable and unstable rotation axes of gyros
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-01-19
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   import numpy as np
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   useGraphics = True
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
   #physical parameters
   g = [0,-9.81*0,0]           #gravity
   L = 0.1                     #length
   w = 0.03                    #width
   r = 0.5*w
   
   #origin of bodies:
   pList =    [[0,0,0], [3*L,0,0], [6*L,0,0], ]        
   #initial angular velocities of bodies:
   angVel = 5
   eps = 2e-4
   omegaList =    [[angVel,eps*angVel,eps*angVel],
                   [eps*angVel,angVel,eps*angVel],
                   [eps*angVel,eps*angVel,angVel]]    
   sAnglVelList= []
   sAngleList  = []
   nodeList = []
   #ground body
   oGround = mbs.AddObject(ObjectGround())
   
   doImplicit = True
   #create several bodies
   for i, p0 in enumerate(pList):
       omega0 = omegaList[i]
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
       iCyl0 = InertiaCylinder(density=1000, length=2*L, outerRadius=r, axis=0, innerRadius=r*0.5)
       iCyl1 = InertiaCylinder(density=1000, length=L, outerRadius=r, axis=1, innerRadius=r*0.5)
       iCylSum = iCyl1.Translated([0,0.5*L,0]) + iCyl0
       com = iCylSum.COM()
       iCylSum = iCylSum.Translated(-com)
   
       #graphics for body
       color = [graphics.color.red, graphics.color.lawngreen, graphics.color.steelblue][i]
       gCyl0 = graphics.SolidOfRevolution(pAxis=[0,0,0]-com, vAxis=[2*L,0,0], 
                                              contour = [[-L,-r],[L,-r],[L,-0.5*r],[-L,-0.5*r],], 
                                              color= color, nTiles= 32, smoothContour=False)
       gCyl1 = graphics.Cylinder(pAxis=[-L,0,0]-com, vAxis=[2*L,0,0],radius=0.5*w, color=color, nTiles=32)
       gCyl2 = graphics.Cylinder(pAxis=[0,0,0]-com, vAxis=[0,L,0],radius=0.5*w, color=color, nTiles=32)
       gCOM = graphics.Basis(origin=[0,0,0],length = 1.25*L)
       
       nodeType = exu.NodeType.RotationRotationVector
       if doImplicit:
           nodeType = exu.NodeType.RotationEulerParameters
           
       result0 = mbs.CreateRigidBody(
           referencePosition = p0,
           initialAngularVelocity = omega0,
           inertia = iCylSum,
           gravity = g,
           nodeType = nodeType,
           graphicsDataList = [gCyl1, gCyl2, gCOM],
           returnDict = True
       )
       n0, b0 = result0['nodeNumber'], result0['bodyNumber']
       sAngVel = mbs.AddSensor(SensorNode(nodeNumber=n0, fileName='solution/gyroAngVel'+str(i)+'.txt',
                                          outputVariableType=exu.OutputVariableType.AngularVelocityLocal))
       sAngle = mbs.AddSensor(SensorNode(nodeNumber=n0, fileName='solution/gyroAngle'+str(i)+'.txt',
                                          outputVariableType=exu.OutputVariableType.Rotation))
       
       nodeList += [n0]
       sAnglVelList += [sAngVel]
       sAngleList += [sAngle]
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #assemble system before solving
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 20 #simulation time
   h = 0.5e-3 #step size
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.timeIntegration.verboseMode = 1
   simulationSettings.solutionSettings.solutionWritePeriod = 0.04 #store every 5 ms
   # simulationSettings.displayComputationTime = True
   
   SC.visualizationSettings.window.renderWindowSize=[1600,1080]
   SC.visualizationSettings.openGL.multiSampling = 4
   SC.visualizationSettings.general.autoFitScene = False
   SC.visualizationSettings.general.graphicsUpdateInterval = 0.02
   SC.visualizationSettings.general.showSolutionInformation = 0
   SC.visualizationSettings.general.showSolverInformation = 0
   # SC.visualizationSettings.general.showSolverTime = 0
   SC.visualizationSettings.general.renderWindowString = 'gyro stability for rotation about axis with \nsmallest (red), middle (green), largest (blue) moment of inertia\ninitial angular velocity = '+str(angVel)+' rad/s, disturbed by '+str(eps*angVel)+' rad/s'
   SC.visualizationSettings.general.textSize = 16
   
   SC.visualizationSettings.nodes.drawNodesAsPoint=False
   SC.visualizationSettings.nodes.showBasis=False
   
   #h=5e-4:
   # omega 0 =  [ 4.99821534  0.17224996 -0.02919272]
   # omega 1 =  [1.14695373 4.853952   0.82419239]
   # omega 2 =  [-0.08742566  0.11224089  4.99987917]
   
   if useGraphics:
       simulationSettings.timeIntegration.simulateInRealtime = True
       SC.renderer.Start()
       if 'renderState' in exu.sys: #reload old view
           SC.renderer.SetState(exu.sys['renderState'])
       
       SC.renderer.DoIdleTasks() #stop before simulating
   
   if doImplicit:
       mbs.SolveDynamic(simulationSettings = simulationSettings,
                        solverType=exu.DynamicSolverType.TrapezoidalIndex2)
   else:
       mbs.SolveDynamic(simulationSettings = simulationSettings,
                        solverType=exu.DynamicSolverType.RK44)
       
   if useGraphics:
       SC.renderer.DoIdleTasks() #stop before closing
       SC.renderer.Stop() #safely close rendering window!
   
   
   for i, n in enumerate(nodeList):
       om = mbs.GetNodeOutput(n,exu.OutputVariableType.AngularVelocityLocal)
       print('omega '+str(i)+' = ',om)
   
   if not useGraphics:
       pass #mbs.SolutionViewer()
   
   if False:
       
       
       for i, omega in enumerate(omegaList):
           if i==1:
               s='unstable'
           else:
               s='stable'
           mbs.PlotSensor([sAnglVelList[i]]*3,[0,1,2], 
                      yLabel='omega init = '+str(omega),colorCodeOffset=0*i*3,
                      #fileName='plots/gyroRotAxis'+str(i)+'-'+s+'-Eps'+str(eps*100)+'percent.png',
                      fontSize=12,
                      newFigure=True, closeAll=(i==0))
   
   


