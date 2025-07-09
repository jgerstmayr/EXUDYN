
.. _testmodels-reevingsystemspringstest:

***************************
reevingSystemSpringsTest.py
***************************

You can view and download this file on Github: `reevingSystemSpringsTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/reevingSystemSpringsTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  A simple 3D reeving system;
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-06-16
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   useGraphics = True #without test
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
   try: #only if called from test suite
       from modelUnitTests import exudynTestGlobals #for globally storing test results
       useGraphics = exudynTestGlobals.useGraphics
   except:
       class ExudynTestGlobals:
           pass
       exudynTestGlobals = ExudynTestGlobals()
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import numpy as np
   from math import sin, cos, sqrt,pi
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   gGround = graphics.CheckerBoard(point=[8,-8,-2], size=32, nTiles=10)
   oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))
   
   t = 0.1
   r0 = 0.5
   g = [0,-9.81,0]
   
   posList = [[0,0,0], 
              [7,-20,1], 
              [12,0,2],
              ]
   rList = [r0*1.,4*r0,r0,5.9*r0,r0,r0,r0]
   
   # posList = [[0,0,0], 
   #            [5,-20,1], 
   #            [8,0,2],
   #            [12,-10,0], 
   #            [16,0,0] 
   #            ]
   # rList = [r0*0.,6*r0,r0,5.9*r0,r0,r0,r0]
   dirList = [-1,1,-1,1,-1,1,1]
   
   
   n = len(posList)
   nodeList = []
   bodyList = []
   markerList = []
   sheavesRadii = []
   sheavesAxes = exu.Vector3DList()
   Lref = 0
   pLast = [0,0,0]
   
   for i, pos in enumerate(posList):
       r = rList[i]
   
       graphicsRoll = graphics.Cylinder(pAxis=[0,0,-0.5*t], vAxis=[0,0,t], nTiles=64, radius=r, color=graphics.color.dodgerblue, alternatingColor=graphics.color.darkgrey)
       
       inertiaRoll = InertiaCylinder(density=1000,length=t,outerRadius=r, axis=2)
       inertiaRoll = inertiaRoll.Translated([0,-r,0])
   
       dictR = mbs.CreateRigidBody(referencePosition=pos,  
                                   inertia=inertiaRoll,  
                                   gravity=g,  
                                   graphicsDataList=[graphicsRoll, graphics.Basis(inertiaRoll.COM(), length=0.5)],  
                                   returnDict=True)  
       nR = dictR['nodeNumber']  
       bR = dictR['bodyNumber']
   
       mR = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nR))
       nodeList += [nR]
       bodyList += [bR]
       markerList += [mR]
   
       sheavesAxes.Append([0,0,dirList[i]*1])
       sheavesRadii += [r]
   
       if i != 0:
           Lref += NormL2(np.array(pos)-pLast)
       if i > 0 and i < len(posList)-1:
           #note that in this test example, Lref is slightly too long, leading to negative spring forces (compression) if not treated nonlinearly with default settings in ReevingSystemSprings
           Lref += r*pi #0.8*r*pi would always lead to tension
   
       #mR = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bR, localPosition=[0,-r,0]))
       #mbs.AddLoad(Force(markerNumber=mR, loadVector=[0,-inertiaRoll.mass*9.81,0]))
           
       pLast = pos
   
   
   markerGround0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=posList[0]))
   markerGroundMid = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=posList[2]))
   markerGroundLast = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=posList[-1]))
   
   #sMarkerR = mbs.AddSensor(SensorMarker(markerNumber=markerR, outputVariableType=exu.OutputVariableType.Position))
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #add joints:
   oJoint0 = mbs.AddObject(GenericJoint(markerNumbers=[markerGround0, markerList[0]],
                                       constrainedAxes=[1,1,1,1,1,1],
                                       visualization=VGenericJoint(axesRadius=0.5*t, axesLength=1.5*t)))
   oJointLast = mbs.AddObject(GenericJoint(markerNumbers=[markerGroundLast, markerList[-1]],
                                       constrainedAxes=[1,1,1,1,1,1],
                                       visualization=VGenericJoint(axesRadius=0.5*t, axesLength=1.5*t)))
   
   if len(posList) > 3:
       mbs.AddObject(GenericJoint(markerNumbers=[markerGroundMid, markerList[2]],
                                           constrainedAxes=[1,1,1,1,1,1],
                                           visualization=VGenericJoint(axesRadius=0.5*t, axesLength=1.5*t)))
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #add reeving system spring
   stiffness = 1e5 #stiffness per length
   damping = 0.5*stiffness #dampiung per length
   oRS=mbs.AddObject(ReevingSystemSprings(markerNumbers=markerList, hasCoordinateMarkers=False, 
                                      stiffnessPerLength=stiffness, dampingPerLength=damping, 
                                      referenceLength = Lref,
                                      dampingTorsional = 0.01*damping, dampingShear = 0.1*damping,
                                      sheavesAxes=sheavesAxes, sheavesRadii=sheavesRadii,
                                      #regularizationForce = -1, #purely linear system
                                      visualization=VReevingSystemSprings(ropeRadius=0.05, color=graphics.color.dodgerblue)))
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #add sensors 
   if True:
       sPos1 = mbs.AddSensor(SensorNode(nodeNumber=nodeList[1], storeInternal=True,
                                             outputVariableType=exu.OutputVariableType.Position))
       sOmega1 = mbs.AddSensor(SensorNode(nodeNumber=nodeList[1], storeInternal=True,
                                             outputVariableType=exu.OutputVariableType.AngularVelocity))
       sLength= mbs.AddSensor(SensorObject(objectNumber=oRS, storeInternal=True,
                                             outputVariableType=exu.OutputVariableType.Distance))
       sLength_t= mbs.AddSensor(SensorObject(objectNumber=oRS, storeInternal=True,
                                             outputVariableType=exu.OutputVariableType.VelocityLocal))
       sForce= mbs.AddSensor(SensorObject(objectNumber=oRS, storeInternal=True,
                                             outputVariableType=exu.OutputVariableType.ForceLocal))
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #simulate:
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 2
   if useGraphics:
       tEnd = 2 #200
   h=0.01  #use small step size to detext contact switching
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   #simulationSettings.solutionSettings.writeSolutionToFile= True #set False for CPU performance measurement
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.01
   
   simulationSettings.timeIntegration.verboseMode = 1
   
   simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
   SC.visualizationSettings.nodes.show = True
   SC.visualizationSettings.nodes.drawNodesAsPoint  = False
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = 0.2
   
   SC.visualizationSettings.openGL.multiSampling = 4
   
   #SC.visualizationSettings.general.autoFitScene = False #use loaded render state
   # useGraphics = True
   if useGraphics:
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys[ 'renderState' ])
       SC.renderer.DoIdleTasks()
   
   
   mbs.SolveDynamic(simulationSettings, 
                    #solverType=exu.DynamicSolverType.TrapezoidalIndex2 #in this case, drift shows up significantly!
                    )
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
   if True:
       
   
       mbs.PlotSensor(sPos1, components=[0,1,2], labels=['pos X','pos Y','pos Z'], closeAll=True)
       mbs.PlotSensor(sOmega1, components=[0,1,2], labels=['omega X','omega Y','omega Z'])
       mbs.PlotSensor(sLength, components=[0], labels=['length'])
       mbs.PlotSensor(sLength_t, components=[0], labels=['vel'])
       mbs.PlotSensor(sForce, components=[0], labels=['force'])
   
   
   
   
   #compute error for test suite:
   sol2 = mbs.systemData.GetODE2Coordinates(); 
   u = np.linalg.norm(sol2); 
   exu.Print('solution of ReevingSystemSprings=',u)
   
   exudynTestGlobals.testResult = u


