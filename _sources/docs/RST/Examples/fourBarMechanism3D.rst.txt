
.. _examples-fourbarmechanism3d:

*********************
fourBarMechanism3D.py
*********************

You can view and download this file on Github: `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  A simple 3D four bar mechanism #read full text output!
   #           1) regular case does not work (redundant constraints/overconstrained joints; jacobian singluar)
   #           2) use simulationSettings.linearSolverSettings.ignoreSingularJacobian = True 
   #           3) remove redundant constraints: change flags for GenericJoint at last joint [1,1,0,0,0,0] to obtain well defined mbs
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-08-05
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   import numpy as np
   from math import pi, sin, cos
   
   
   useGraphics = True
   
   casesText = ['redundant constraints', 'redundant constraints with improved solver', 'non-redundant constraints']
   cases = [0,1,2]
   
   for case in cases:
       caseText = casesText[case]
       print('\n\n************************************************')
       print('run four bar mechanism with case:\n  '+caseText)
       print('************************************************')
       
       SC = exu.SystemContainer()
       mbs = SC.AddSystem()
   
   
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
       #physical parameters
       g =     [0.1,-9.81,0] #gravity + disturbance
       L = 1               #length
       w = 0.1             #width
       bodyDim=[L,w,w]     #body dimensions
       # p0 =    [0,0,0]     
       pMid0 = np.array([0,L*0.5,0]) #center of mass, body0
       pMid1 = np.array([L*0.5,L,0]) #center of mass, body1
       pMid2 = np.array([L,L*0.5,0]) #center of mass, body2
   
       #ground body
       graphicsCOM0 = graphics.Basis(origin=[0,0,0], length=4*w)
       oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[graphicsCOM0])))
       markerGround0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
       markerGround1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[L,0,0]))
   
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
       #first link:
       iCube0 = InertiaCuboid(density=5000, sideLengths=bodyDim)
   
       #graphics for body
       graphicsBody0 = graphics.RigidLink(p0=[-0.5*L,0,0],p1=[0.5*L,0,0], 
                                            axis0=[0,0,1], axis1=[0,0,1], radius=[0.5*w,0.5*w], 
                                            thickness = w, width = [1.2*w,1.2*w], color=graphics.color.red)
       graphicsBody1 = graphics.RigidLink(p0=[-0.5*L,0,0],p1=[0.5*L,0,0], 
                                            axis0=[0,0,1], axis1=[0,0,1], radius=[0.5*w,0.5*w], 
                                            thickness = w, width = [1.2*w,1.2*w], color=graphics.color.green)
       graphicsBody2 = graphics.RigidLink(p0=[-0.5*L,0,0],p1=[0.5*L,0,0], 
                                            axis0=[0,0,1], axis1=[0,0,1], radius=[0.5*w,0.5*w], 
                                            thickness = w, width = [1.2*w,1.2*w], color=graphics.color.steelblue)
       dictCube0 = mbs.CreateRigidBody(
                     inertia=iCube0, # includes COM
                     referencePosition=pMid0,
                     referenceRotationMatrix=RotationMatrixZ(0.5*pi),
                     gravity=g,
                     graphicsDataList=[graphicsBody0],
                     returnDict=True)
       [n0, b0] = [dictCube0['nodeNumber'], dictCube0['bodyNumber']]
       
       markerBody0J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[-0.5*L,0,0]))
       markerBody0J1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0.5*L,0,0]))
       
       dictCube1 = mbs.CreateRigidBody(
                     inertia=iCube0, # includes COM
                     referencePosition=pMid1,
                     gravity=g,
                     graphicsDataList=[graphicsBody1],
                     returnDict=True)
       [n1, b1] = [dictCube1['nodeNumber'], dictCube1['bodyNumber']]
       
       markerBody1J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=[-0.5*L,0,0]))
       markerBody1J1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b1, localPosition=[0.5*L,0,0]))
       
       dictCube2 = mbs.CreateRigidBody(
                     inertia=iCube0, # includes COM
                     referencePosition=pMid2,
                     referenceRotationMatrix=RotationMatrixZ(-0.5*pi),
                     gravity=g,
                     graphicsDataList=[graphicsBody2],
                     returnDict=True)
       [n2, b2] = [dictCube2['nodeNumber'], dictCube2['bodyNumber']]
   
       markerBody2J0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2, localPosition=[-0.5*L,0,0]))
       markerBody2J1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=b2, localPosition=[ 0.5*L,0,0]))
   
   
       #revolute joint option 1:
       mbs.AddObject(GenericJoint(markerNumbers=[markerGround0, markerBody0J0], 
                                   constrainedAxes=[1,1,1,1,1,0],
                                   visualization=VObjectJointGeneric(axesRadius=0.2*w, axesLength=1.4*w)))
   
       mbs.AddObject(GenericJoint(markerNumbers=[markerBody0J1, markerBody1J0], 
                                   constrainedAxes=[1,1,1,1,1,0],
                                   visualization=VObjectJointGeneric(axesRadius=0.2*w, axesLength=1.4*w)))
   
       mbs.AddObject(GenericJoint(markerNumbers=[markerBody1J1, markerBody2J0], 
                                   constrainedAxes=[1,1,1,1,1,0],
                                   visualization=VObjectJointGeneric(axesRadius=0.2*w, axesLength=1.4*w)))
   
       constrainedAxes3 = [1,1,1,1,1,0]
       if case == 2:
           constrainedAxes3 = [1,1,0,0,0,0] #only these constraints are needed for closing loop!
           print('use non-redundant constraints for last joint:', constrainedAxes3)
           
       mbs.AddObject(GenericJoint(markerNumbers=[markerBody2J1, markerGround1], 
                                   constrainedAxes=constrainedAxes3,
                                   visualization=VObjectJointGeneric(axesRadius=0.2*w, axesLength=1.4*w)))
   
       #position sensor at tip of body1
       sens1=mbs.AddSensor(SensorBody(bodyNumber=b1, localPosition=[0,0,0.5*L],
                                      fileName='solution/sensorPos.txt',
                                      outputVariableType = exu.OutputVariableType.Position))
   
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++
       #assemble system before solving
       mbs.Assemble()
       if False:
           mbs.systemData.Info() #show detailed information
       if False:
           #from exudyn.utilities import DrawSystemGraph
           mbs.DrawSystemGraph(useItemTypes=True) #draw nice graph of system
   
       simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
       tEnd = 10 #simulation time
       h = 2e-3 #step size
       simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
       simulationSettings.timeIntegration.endTime = tEnd
       simulationSettings.timeIntegration.verboseMode = 1
       #simulationSettings.timeIntegration.simulateInRealtime = True
       #simulationSettings.timeIntegration.realtimeFactor = 4
   
       if case == 1:
           simulationSettings.linearSolverSettings.ignoreSingularJacobian = True #for redundant constraints
   
       simulationSettings.timeIntegration.newton.useModifiedNewton = True
       simulationSettings.solutionSettings.writeSolutionToFile = False
       #simulationSettings.solutionSettings.solutionWritePeriod = 0.005 #store every 5 ms
   
       SC.visualizationSettings.window.renderWindowSize=[1200,1024]
       SC.visualizationSettings.openGL.multiSampling = 4
       SC.visualizationSettings.general.autoFitScene = False
   
       SC.visualizationSettings.nodes.drawNodesAsPoint=False
       SC.visualizationSettings.nodes.showBasis=True
   
       if useGraphics:
           SC.renderer.Start()
           if 'renderState' in exu.sys: #reload old view
               SC.renderer.SetState(exu.sys['renderState'])
           
           SC.renderer.DoIdleTasks() #stop before simulating
   
       try: #solver will raise exception in case 1
           mbs.SolveDynamic(simulationSettings = simulationSettings)
       except:
           pass
                                    
       # mbs.SolveDynamic(simulationSettings = simulationSettings,
       #                  solverType=exu.DynamicSolverType.TrapezoidalIndex2)
       if useGraphics:
           SC.renderer.DoIdleTasks() #stop before closing
           SC.renderer.Stop() #safely close rendering window!
   
       #check redundant constraints and DOF:
       mbs.ComputeSystemDegreeOfFreedom(verbose=useGraphics)
   
   
   if False:
       sol = LoadSolutionFile('coordinatesSolution.txt')
       
       mbs.SolutionViewer(sol)
   
   if False:
       
       mbs.PlotSensor([sens1],[1])
   
   


