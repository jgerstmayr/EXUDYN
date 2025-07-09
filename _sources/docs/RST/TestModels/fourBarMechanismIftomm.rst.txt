
.. _testmodels-fourbarmechanismiftomm:

*************************
fourBarMechanismIftomm.py
*************************

You can view and download this file on Github: `fourBarMechanismIftomm.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/fourBarMechanismIftomm.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  double four-bar mechanism according to IFToMM benchmarks; here, it is non-redundant
   #
   # Author:   Johannes Gerstmayr
   # Date:     2021-05-30
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
   from math import sin, cos, pi
   import numpy as np
   
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
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   g = [0,-9.81,0]     #gravity in m/s^2
   
   mass = 1 #kg
   L = 1    #m
   w = 0.1  #m, just for drawing 
   
   inertiaBar = InertiaRodX(mass, L)
   J = inertiaBar.inertiaTensor[2,2]
   addSensors = True #turn off to obtain max. computational speed ...
   
   v0 = 1
   listRef=[[0,0.5*L,0.5*pi],[0.5*L,L,0],
            [L,0.5*L,0.5*pi],[1.5*L,L,0],
            [2*L,0.5*L,0.5*pi]]   #reference positions and rotations of bars
   listVel=[[0.5*v0,0,-v0/L],[v0,0,0],
            [0.5*v0,0,-v0/L],[v0,0,0],
            [0.5*v0,0,-v0/L]]   #reference positions and rotations of bars
   
   listNodes=[]
   listBodies=[]
   listMarkers0=[]
   listMarkers1=[]
   listSensors=[]
   
   for i in range(len(listRef)):
       graphicsBar = graphics.RigidLink(p0=[-0.5*L,0,0], p1=[0.5*L,0,0], 
                                           axis0=[0,0,1],axis1=[0,0,1],
                                           radius=[0.5*w,0.5*w], thickness=w,
                                           width=[0.5*w*2,0.5*w*2],
                                           color=graphics.color.steelblue)
       
       nRigid=mbs.AddNode(NodeRigidBody2D(referenceCoordinates=listRef[i],
                                          initialVelocities=listVel[i]))
       oRigid=mbs.AddObject(RigidBody2D(nodeNumber=nRigid, physicsMass=inertiaBar.mass, 
                                 physicsInertia=inertiaBar.inertiaTensor[2,2],
                                 visualization=VRigidBody2D(graphicsData=[graphicsBar])))
       
       mMass = mbs.AddMarker(MarkerBodyMass(bodyNumber=oRigid))
       lMass = mbs.AddLoad(LoadMassProportional(markerNumber=mMass, loadVector=g))
       
       m0=mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[-0.5*L,0,0]))
       m1=mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0.5*L,0,0]))
       
       if addSensors: # and i==0:
           f=0 #set to 0 for energy computation, else 1
           sPos = mbs.AddSensor(SensorBody(bodyNumber=oRigid, 
                                           localPosition=[f*0.5*L,0,0],storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Position))
           sVel = mbs.AddSensor(SensorBody(bodyNumber=oRigid, 
                                           localPosition=[f*0.5*L,0,0],storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.Velocity))
           sAng = mbs.AddSensor(SensorBody(bodyNumber=oRigid, 
                                           localPosition=[f*0.5*L,0,0],storeInternal=True,
                                           outputVariableType=exu.OutputVariableType.AngularVelocity))
           listSensors+=[sPos,sVel,sAng]
   
       listNodes+=[nRigid]
       listBodies+=[oRigid]
       listMarkers0+=[m0]
       listMarkers1+=[m1]
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #ground body and marker
   gGround = graphics.CheckerBoard(point=[L,0,-w], size=4)
   oGround = mbs.AddObject(ObjectGround())
   #oGround = mbs.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=[gGround])))
   markerGround0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))
   markerGround1 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[L,0,0]))
   markerGround2 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[2*L,0,0]))
   
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[markerGround0,listMarkers0[0]]))
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[markerGround1,listMarkers0[2]]))
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[markerGround2,listMarkers0[4]]))
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[listMarkers1[0],listMarkers0[1]]))
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[listMarkers1[1],listMarkers0[3]]))
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[listMarkers1[2],listMarkers0[3]]))
   mbs.AddObject(RevoluteJoint2D(markerNumbers=[listMarkers1[3],listMarkers1[4]]))
   
   if addSensors:
       def UFsensors(mbs, t, sensorNumbers, factors, configuration):
           T=0
           for i in range(int(len(sensorNumbers)/3)):
               h = mbs.GetSensorValues(sensorNumbers[i*3+0])[1]
               v = mbs.GetSensorValues(sensorNumbers[i*3+1])
               omega = mbs.GetSensorValues(sensorNumbers[i*3+2])[2]
               
               T += 0.5*NormL2(v)**2 * mass + 0.5*J*omega**2 + h*(-g[1])*mass
           return [T - 3.5*mass*(-g[1]) - 1.5] #1.5 is initial kinetic energy
           
       sUser = mbs.AddSensor(SensorUserFunction(sensorNumbers=listSensors,
                                        sensorUserFunction=UFsensors,
                                        factors=[0]*len(listSensors),storeInternal=True))
                                        #fileName='solution/energyDoubleFourBar.txt'
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++
   #simulate:
   mbs.Assemble()
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   tEnd = 4 #genAlpha=0.8
   h=0.01  #use small step size to detext contact switching
   #h=0.01: #rho=0.95 #CPU-time=0.0776 s on Intel i9
   #max energy error= 0.1140507007
   #h=0.001:
   #pos=10,0.3284112372,0.9445348375,0
   #vel=10,1.422958522,-0.4947565894,0
   #max energy error= 0.001143193966
   #h=0.0005:
   #pos=10,0.3284465114,0.9445225721,0
   #vel=10,1.423028497,-0.494841072,0
   #max energy error= 0.0002858041366
   #h=0.00025:
   #pos=10,0.3284552113,0.9445195467,0
   #vel=10,1.423045728,-0.4948619021,0
   #max energy error= 7.14659441e-05
   #h=0.000125:
   #pos=10,0.3284573768,0.9445187937,0
   #vel=10,1.423050003,-0.4948670827,0
   #max energy error= 1.79516107e-05
   
   simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
   simulationSettings.timeIntegration.endTime = tEnd
   simulationSettings.solutionSettings.writeSolutionToFile= False
   simulationSettings.solutionSettings.sensorsWritePeriod = 0.01
   simulationSettings.timeIntegration.verboseMode = 1
   #simulationSettings.displayComputationTime=True
   
   if False:
       simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
       simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
   simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.95
   simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations=True
   simulationSettings.timeIntegration.newton.useModifiedNewton=True
   # simulationSettings.timeIntegration.simulateInRealtime= True
   # simulationSettings.timeIntegration.realtimeFactor=0.1
   
   #simulationSettings.linearSolverSettings.ignoreSingularJacobian = True #for redundant constraints
   
   SC.visualizationSettings.nodes.show = True
   SC.visualizationSettings.nodes.drawNodesAsPoint  = False
   SC.visualizationSettings.nodes.showBasis = True
   SC.visualizationSettings.nodes.basisSize = w*2
   
   # simulationSettings.timeIntegration.simulateInRealtime=True
   # simulationSettings.timeIntegration.realtimeFactor=0.1
   if True: #record animation frames:
       SC.visualizationSettings.loads.drawSimplified=False
       SC.visualizationSettings.general.graphicsUpdateInterval=0.01
       SC.visualizationSettings.openGL.lineWidth=2
       SC.visualizationSettings.exportImages.saveImageFileName = "animation/frame"
       #SC.visualizationSettings.window.renderWindowSize=[1980,1080]
       SC.visualizationSettings.window.renderWindowSize=[1280,720]
       SC.visualizationSettings.openGL.multiSampling = 4
       #simulationSettings.solutionSettings.recordImagesInterval = 0.01
       
   SC.visualizationSettings.general.autoFitScene = False #use loaded render state
   #useGraphics = True
   if useGraphics:
       SC.renderer.Start()
       if 'renderState' in exu.sys:
           SC.renderer.SetState(exu.sys[ 'renderState' ])
       SC.renderer.DoIdleTasks()
   
   mbs.SolveDynamic(simulationSettings)
   
   
   if useGraphics:
       SC.renderer.DoIdleTasks()
       SC.renderer.Stop() #safely close rendering window!
   
       ##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
       #plot results
       if True:
           
           mbs.PlotSensor(sensorNumbers=[listSensors[0],listSensors[1],], 
                      components=[0,0], closeAll=True)
           mbs.PlotSensor(sensorNumbers=[sUser], title='Energy error')
   
   if addSensors:
       #x=np.loadtxt('solution/energyDoubleFourBar.txt',comments='#', delimiter=',')
       x=mbs.GetSensorStoredData(sUser)
       maxEnergyError = max(abs(x[:,1]))
       exu.Print("max energy error=",maxEnergyError)
       
       p0=mbs.GetSensorStoredData(listSensors[0])[-1,1:] #of last bar
       exu.Print("pos first bar=",list(p0)) #[0.05815092990659491, 0.49660695660597587, 0.0]
       u = maxEnergyError + p0[0]
       
       exu.Print('fourBarMechanismIftomm result:', u)
       exudynTestGlobals.testError = u - (0.1721665271840173) 
       exudynTestGlobals.testResult = u


