
.. _testmodels-kinematictreeconstrainttest:

******************************
kinematicTreeConstraintTest.py
******************************

You can view and download this file on Github: `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_

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
   
   useGraphics = True
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
   
   from math import pi, sin, cos#, sqrt
   from copy import copy, deepcopy
   from exudyn.rigidBodyUtilities import Skew, Skew2Vec
   from exudyn.robotics import *
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   # useGraphics = False
   
   useMBS = True
   useKinematicTree = True
   addForce = True #add gravity as body / link forces
   addConstraint = True #add constraint at tip of chain
   
   
   gGround =  graphics.CheckerBoard(point= [0,0,-2], size = 12)
   objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0],
                                             visualization=VObjectGround(graphicsData=[gGround])))
   baseMarker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[0,0,0]))
   
   L = 0.5 #length
   w = 0.1 #width of links
   pControl = 20000 #we keep the motion of the prismatic joint fixed
   dControl = pControl*0.02
   
   gravity3D = [0,-9.81*0,0]
   graphicsBaseList = [graphics.Brick(size=[L*4, 0.8*w, 0.8*w], color=graphics.color.grey)] #rail
   
   newRobot = Robot(gravity=gravity3D,
                 base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
                 tool = RobotTool(HT=HTtranslate([0,0.5*L,0]), visualization=VRobotTool(graphicsData=[
                     graphics.Brick(size=[w, L, w], color=graphics.color.orange)])),
                 referenceConfiguration = []) #referenceConfiguration created with 0s automatically
   
   #cart:
   Jlink = InertiaCuboid(density=5000, sideLengths=[L,w,w]) #w.r.t. reference center of mass
   link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                    jointType='Px', preHT=HT0(), 
                    PDcontrol=(pControl, dControl),
                    visualization=VRobotLink(linkColor=graphics.color.lawngreen))
   newRobot.AddLink(link)
   linksList = [copy(link)]
   
   nChainLinks = 4 #5
   for i in range(nChainLinks):
       Jlink = InertiaCuboid(density=1000, sideLengths=[w,L,w]) #w.r.t. reference center of mass
       Jlink = Jlink.Translated([0,0.5*L,0])
       preHT = HT0()
       if i > 0:
           preHT = HTtranslateY(L)
   
       link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                        jointType='Rz', preHT=preHT, 
                        PDcontrol=(pControl*0, dControl*0),
                        visualization=VRobotLink(linkColor=graphics.color.blue))
       newRobot.AddLink(link)
       linksList += [copy(link)]
   
   newRobot.referenceConfiguration[0] = 0.5*0
   # for i in range(nChainLinks):
   #     newRobot.referenceConfiguration[i+1] = (2*pi/360) * 5 
   newRobot.referenceConfiguration[1] = -(2*pi/360) * 90 #-0.5*pi
   # newRobot.referenceConfiguration[2] = (2*pi/360) * 12 #-0.5*pi
       
   # locPos = [0.1,0.2,0.3]
   locPos = [0,0,0]
   nLinks = newRobot.NumberOfLinks()
   
   sMBS = []
   if useMBS:
       #newRobot.gravity=[0,-9.81,0]
       robDict = newRobot.CreateRedundantCoordinateMBS(mbs=mbs, baseMarker=baseMarker, createJointTorqueLoads=False)
       bodies = robDict['bodyList']
   
       sMBS+=[mbs.AddSensor(SensorBody(bodyNumber=bodies[nLinks-1], localPosition=locPos, storeInternal=True,
                                       outputVariableType=exu.OutputVariableType.Position))]
   
       if addForce:
           for i in range(len(bodies)):
               mBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bodies[i], localPosition=linksList[i].COM))
               mbs.AddLoad(Force(markerNumber=mBody, loadVector=[0,-9.81*linksList[i].mass, 0]))
   
       if addConstraint:
           mTip = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bodies[-1], localPosition=[0,L,0]))
           mTipGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber=objectGround, localPosition=[L*nChainLinks,0,0]))
           mbs.AddObject(SphericalJoint(markerNumbers=[mTip, mTipGround], constrainedAxes=[0,1,0]))
   
   sKT = []
   if useKinematicTree:
       #newRobot.gravity=[0,-9.81,0]
       dKT = newRobot.CreateKinematicTree(mbs)
       oKT = dKT['objectKinematicTree']
       
       sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=nLinks-1, localPosition=locPos, storeInternal=True,
                                               outputVariableType=exu.OutputVariableType.Position))]
       
       if addForce:
           for i in range(nLinks):
               mLink = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT, linkNumber=i, localPosition=linksList[i].COM))
               mbs.AddLoad(Force(markerNumber=mLink, loadVector=[0,-9.81*linksList[i].mass, 0]))
   
       if addConstraint:
           mTip = mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber=oKT, linkNumber=nLinks-1, localPosition=[0,L,0]))
           mTipGround = mbs.AddMarker(MarkerBodyRigid(bodyNumber = objectGround, localPosition=[L*nChainLinks,0,0]))
           mbs.AddObject(SphericalJoint(markerNumbers=[mTip, mTipGround], constrainedAxes=[0,1,0]))
       
   #exu.Print(mbs)
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings()
   
   tEnd = 0.5
   h = 4*1e-3
   #tEnd = h
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   # simulationSettings.timeIntegration.numberOfSteps = 1#int(tEnd/h)
   # simulationSettings.timeIntegration.endTime = h*1#tEnd
   simulationSettings.solutionSettings.solutionWritePeriod = 0.01*100
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.001*20
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.solutionSettings.solutionWritePeriod = tEnd/steps
   simulationSettings.timeIntegration.newton.useModifiedNewton=True
   
   # simulationSettings.displayComputationTime = True
   # simulationSettings.linearSolverType=exu.LinearSolverType.EigenSparse
   
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
   
   # mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.ExplicitMidpoint)
   mbs.SolveDynamic(simulationSettings)
   
   if not useGraphics or True:
       #check results for test suite:
       u = 0.
       for i in range(len(sMBS)):
           v = mbs.GetSensorValues(sMBS[i])
           exu.Print('sensor MBS '+str(i)+'=',v)
           u += np.linalg.norm(v)
           v = mbs.GetSensorValues(sKT[i])
           exu.Print('sensor KT '+str(i)+' =',v)
           u += np.linalg.norm(v)
   
   exu.Print("solution of kinematicTreeConstraintTest=", u)
   exudynTestGlobals.testResult = u #1.8135975385993548 
   
       
   if False and useGraphics: #use this to reload the solution and use SolutionViewer
       #sol = LoadSolutionFile('coordinatesSolution.txt')
       
       mbs.SolutionViewer() #can also be entered in IPython ...
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
           
   
   


