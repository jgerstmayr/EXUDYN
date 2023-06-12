
.. _examples-openaigymtriplependulum:

**************************
openAIgymTriplePendulum.py
**************************

You can view and download this file on Github: `openAIgymTriplePendulum.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/openAIgymTriplePendulum.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  This file shows integration with OpenAI gym by testing a triple pendulum example
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-05-18
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
   import exudyn as exu
   from exudyn.utilities import *
   from exudyn.artificialIntelligence import *
   import math
   
   
   class InvertedTriplePendulumEnv(OpenAIGymInterfaceEnv):
           
       #**classFunction: OVERRIDE this function to create multibody system mbs and setup simulationSettings; call Assemble() at the end!
       #                 you may also change SC.visualizationSettings() individually; kwargs may be used for special setup
       def CreateMBS(self, SC, mbs, simulationSettings, **kwargs):
   
           #%%++++++++++++++++++++++++++++++++++++++++++++++
           #this model uses kwargs: thresholdFactor
           thresholdFactor = 3
           if 'thresholdFactor' in kwargs:
               thresholdFactor = kwargs['thresholdFactor']
           
           gravity = 9.81
           self.length = 1.
           width = 0.1*self.length
           masscart = 1.
           massarm = 0.1
           total_mass = massarm + masscart
           armInertia = self.length**2*0.5*massarm
           self.force_mag = 10.0*2 #must be larger for triple pendulum to be more reactive ...
           self.stepUpdateTime = 0.02  # seconds between state updates
           
           background = GraphicsDataCheckerBoard(point= [0,0.5*self.length,-0.5*width], 
                                                 normal= [0,0,1], size=10, size2=6, nTiles=20, nTiles2=12)
               
           oGround=self.mbs.AddObject(ObjectGround(referencePosition= [0,0,0],  #x-pos,y-pos,angle
                                              visualization=VObjectGround(graphicsData= [background])))
           nGround=self.mbs.AddNode(NodePointGround())
           
           gCart = GraphicsDataOrthoCubePoint(size=[0.5*self.length, width, width], 
                                              color=color4dodgerblue)
           self.nCart = self.mbs.AddNode(Rigid2D(referenceCoordinates=[0,0,0]));
           oCart = self.mbs.AddObject(RigidBody2D(physicsMass=masscart, 
                                             physicsInertia=0.1*masscart, #not needed
                                             nodeNumber=self.nCart,
                                             visualization=VObjectRigidBody2D(graphicsData= [gCart])))
           mCartCOM = self.mbs.AddMarker(MarkerNodePosition(nodeNumber=self.nCart))
           
           gArm1 = GraphicsDataOrthoCubePoint(size=[width, self.length, width], color=color4red)
           gArm1joint = GraphicsDataCylinder(pAxis=[0,-0.5*self.length,-0.6*width], vAxis=[0,0,1.2*width], 
                                             radius=0.0625*self.length, color=color4darkgrey)
           self.nArm1 = self.mbs.AddNode(Rigid2D(referenceCoordinates=[0,0.5*self.length,0]));
           oArm1 = self.mbs.AddObject(RigidBody2D(physicsMass=massarm, 
                                             physicsInertia=armInertia, #not included in original paper
                                             nodeNumber=self.nArm1,
                                             visualization=VObjectRigidBody2D(graphicsData= [gArm1, gArm1joint])))
           
           mArm1COM = self.mbs.AddMarker(MarkerNodePosition(nodeNumber=self.nArm1))
           mArm1JointA = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=oArm1, localPosition=[0,-0.5*self.length,0]))
           mArm1JointB = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=oArm1, localPosition=[0, 0.5*self.length,0]))
   
           gArm2 = GraphicsDataOrthoCubePoint(size=[width, self.length, width], color=color4red)
           self.nArm2 = self.mbs.AddNode(Rigid2D(referenceCoordinates=[0,1.5*self.length,0]));
           oArm2 = self.mbs.AddObject(RigidBody2D(physicsMass=massarm, 
                                             physicsInertia=armInertia, #not included in original paper
                                             nodeNumber=self.nArm2,
                                             visualization=VObjectRigidBody2D(graphicsData= [gArm2, gArm1joint])))
           
           mArm2COM = self.mbs.AddMarker(MarkerNodePosition(nodeNumber=self.nArm2))
           mArm2Joint = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=oArm2, localPosition=[0,-0.5*self.length,0]))
           mArm2JointB = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=oArm2, localPosition=[0, 0.5*self.length,0]))
   
           gArm3 = GraphicsDataOrthoCubePoint(size=[width, self.length, width], color=color4red)
           self.nArm3 = self.mbs.AddNode(Rigid2D(referenceCoordinates=[0,2.5*self.length,0]));
           oArm3 = self.mbs.AddObject(RigidBody2D(physicsMass=massarm, 
                                             physicsInertia=armInertia, #not included in original paper
                                             nodeNumber=self.nArm3,
                                             visualization=VObjectRigidBody2D(graphicsData= [gArm3, gArm1joint])))
           
           mArm3COM = self.mbs.AddMarker(MarkerNodePosition(nodeNumber=self.nArm3))
           mArm3Joint = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=oArm3, localPosition=[0,-0.5*self.length,0]))
           
           mCartCoordX = self.mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=self.nCart, coordinate=0))
           mCartCoordY = self.mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=self.nCart, coordinate=1))
           mGroundNode = self.mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
           
           #gravity
           self.mbs.AddLoad(Force(markerNumber=mCartCOM, loadVector=[0,-masscart*gravity,0]))
           self.mbs.AddLoad(Force(markerNumber=mArm1COM, loadVector=[0,-massarm*gravity,0]))
           self.mbs.AddLoad(Force(markerNumber=mArm2COM, loadVector=[0,-massarm*gravity,0]))
           self.mbs.AddLoad(Force(markerNumber=mArm3COM, loadVector=[0,-massarm*gravity,0]))
           
           #control force
           self.lControl = self.mbs.AddLoad(LoadCoordinate(markerNumber=mCartCoordX, load=1.))
           
           #joints and constraints:
           self.mbs.AddObject(RevoluteJoint2D(markerNumbers=[mCartCOM, mArm1JointA]))
           self.mbs.AddObject(RevoluteJoint2D(markerNumbers=[mArm1JointB, mArm2Joint]))
           self.mbs.AddObject(RevoluteJoint2D(markerNumbers=[mArm2JointB, mArm3Joint]))
   
           self.mbs.AddObject(CoordinateConstraint(markerNumbers=[mCartCoordY, mGroundNode]))
           
           
           
           
           #%%++++++++++++++++++++++++
           self.mbs.Assemble() #computes initial vector
           
           self.simulationSettings.timeIntegration.numberOfSteps = 1
           self.simulationSettings.timeIntegration.endTime = 0 #will be overwritten in step
           self.simulationSettings.timeIntegration.verboseMode = 0
           self.simulationSettings.solutionSettings.writeSolutionToFile = False
           #self.simulationSettings.timeIntegration.simulateInRealtime = True
           
           self.simulationSettings.timeIntegration.newton.useModifiedNewton = True
           
           self.SC.visualizationSettings.general.drawWorldBasis=True
           self.SC.visualizationSettings.general.graphicsUpdateInterval = 0.01 #50Hz
           self.SC.visualizationSettings.openGL.multiSampling=4
           
           #self.simulationSettings.solutionSettings.solutionInformation = "Open AI gym"
           
           #+++++++++++++++++++++++++++++++++++++++++++++++++++++
           # Angle at which to fail the episode
           # these parameters are used in subfunctions
           self.theta_threshold_radians = thresholdFactor* 12 * 2 * math.pi / 360
           self.x_threshold = thresholdFactor*2.4
           
           #must return state size
           stateSize = 8 #the number of states (position/velocity that are used by learning algorithm)
           return stateSize
   
       #**classFunction: OVERRIDE this function to set up self.action_space and self.observation_space
       def SetupSpaces(self):
   
           high = np.array(
               [
                   self.x_threshold * 2,
                   np.finfo(np.float32).max,
                   self.theta_threshold_radians * 2,
                   np.finfo(np.float32).max,
                   self.theta_threshold_radians * 2,
                   np.finfo(np.float32).max,
                   self.theta_threshold_radians * 2,
                   np.finfo(np.float32).max,
               ],
               dtype=np.float32,
           )
   
           #+++++++++++++++++++++++++++++++++++++++++++++++++++++
           #see https://github.com/openai/gym/blob/64b4b31d8245f6972b3d37270faf69b74908a67d/gym/core.py#L16
           #for Env:
           self.action_space = spaces.Discrete(2)
           self.observation_space = spaces.Box(-high, high, dtype=np.float32)        
           #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
       #**classFunction: OVERRIDE this function to map the action given by learning algorithm to the multibody system, e.g. as a load parameter
       def MapAction2MBS(self, action):
           force = self.force_mag if action == 1 else -self.force_mag        
           self.mbs.SetLoadParameter(self.lControl, 'load', force)
   
       #**classFunction: OVERRIDE this function to collect output of simulation and map to self.state tuple
       #**output: return bool done which contains information if system state is outside valid range
       def Output2StateAndDone(self):
           
           #+++++++++++++++++++++++++
           #compute some output:
           cartPosX = self.mbs.GetNodeOutput(self.nCart, variableType=exu.OutputVariableType.Coordinates)[0]
           arm1Angle = self.mbs.GetNodeOutput(self.nArm1, variableType=exu.OutputVariableType.Coordinates)[2]
           arm2Angle = self.mbs.GetNodeOutput(self.nArm2, variableType=exu.OutputVariableType.Coordinates)[2]
           arm3Angle = self.mbs.GetNodeOutput(self.nArm3, variableType=exu.OutputVariableType.Coordinates)[2]
           cartPosX_t = self.mbs.GetNodeOutput(self.nCart, variableType=exu.OutputVariableType.Coordinates_t)[0]
           arm1Angle_t = self.mbs.GetNodeOutput(self.nArm1, variableType=exu.OutputVariableType.Coordinates_t)[2]
           arm2Angle_t = self.mbs.GetNodeOutput(self.nArm2, variableType=exu.OutputVariableType.Coordinates_t)[2]
           arm3Angle_t = self.mbs.GetNodeOutput(self.nArm3, variableType=exu.OutputVariableType.Coordinates_t)[2]
   
           #finally write updated state:
           self.state = (cartPosX, cartPosX_t, arm1Angle, arm1Angle_t, arm2Angle, arm2Angle_t, arm3Angle, arm3Angle_t)
           #++++++++++++++++++++++++++++++++++++++++++++++++++
   
           done = bool(
               cartPosX < -self.x_threshold
               or cartPosX > self.x_threshold
               or arm1Angle < -self.theta_threshold_radians
               or arm1Angle > self.theta_threshold_radians
               or arm2Angle < -self.theta_threshold_radians
               or arm2Angle > self.theta_threshold_radians
               or arm3Angle < -self.theta_threshold_radians
               or arm3Angle > self.theta_threshold_radians
           )
           return done
   
       
       #**classFunction: OVERRIDE this function to maps the current state to mbs initial values
       #**output: return [initialValues, initialValues\_t] where initialValues[\_t] are ODE2 vectors of coordinates[\_t] for the mbs
       def State2InitialValues(self):
           #+++++++++++++++++++++++++++++++++++++++++++++
           #set specific initial state:
           (xCart, xCart_t, phiArm1, phiArm1_t, phiArm2, phiArm2_t, phiArm3, phiArm3_t) = self.state
           
           initialValues = np.zeros(12) #model has 4*3 redundant states
           initialValues_t = np.zeros(12)
           
           #build redundant cordinates from self.state
           initialValues[0] = xCart
           initialValues[3+0] = xCart - 0.5*self.length * sin(phiArm1)
           initialValues[3+1] = 0.5*self.length * (cos(phiArm1)-1)
           initialValues[3+2] = phiArm1
   
           initialValues[6+0] = xCart - self.length * sin(phiArm1) - 0.5*self.length * sin(phiArm2)
           initialValues[6+1] = self.length * cos(phiArm1) + 0.5*self.length * cos(phiArm2) - 1.5*self.length
           initialValues[6+2] = phiArm2
   
           initialValues[9+0] = xCart - self.length * sin(phiArm1) - self.length * sin(phiArm2) - 0.5*self.length * sin(phiArm3)
           initialValues[9+1] = self.length * cos(phiArm1) + self.length * cos(phiArm2) + 0.5*self.length * cos(phiArm3) - 2.5*self.length
           initialValues[9+2] = phiArm3
           
           initialValues_t[0] = xCart_t
           initialValues_t[3+0] = xCart_t - phiArm1_t*0.5*self.length * cos(phiArm1)
           initialValues_t[3+1] = -0.5*self.length * sin(phiArm1)  * phiArm1_t
           initialValues_t[3+2] = phiArm1_t
   
           initialValues_t[6+0] = xCart_t - phiArm1_t*self.length * cos(phiArm1) - phiArm2_t*0.5*self.length * cos(phiArm2)
           initialValues_t[6+1] = -self.length * sin(phiArm1)  * phiArm1_t - 0.5*self.length * sin(phiArm2)  * phiArm2_t
           initialValues_t[6+2] = phiArm2_t
           
           initialValues_t[9+0] = xCart_t - phiArm1_t*self.length * cos(phiArm1) - phiArm2_t*self.length * cos(phiArm2) - phiArm3_t*0.5*self.length * cos(phiArm3)
           initialValues_t[9+1] = -self.length * sin(phiArm1)  * phiArm1_t - self.length * sin(phiArm2)  * phiArm2_t - 0.5*self.length * sin(phiArm3)  * phiArm3_t
           initialValues_t[9+2] = phiArm3_t
           
           return [initialValues,initialValues_t]
           
   
   
   
   
   
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++
   if __name__ == '__main__': #this is only executed when file is direct called in Python
       import time
       
   
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++
       #use some learning algorithm:
       #pip install stable_baselines3
       from stable_baselines3 import A2C
   
   
           #create model and do reinforcement learning
       if False: #'scalar' environment:
           env = InvertedTriplePendulumEnv() #(thresholdFactor=2)
           #check if model runs:
           # env.TestModel(numberOfSteps=1000, seed=42)
       
           #main learning task; 1e7 steps take 2-3 hours
           model = A2C('MlpPolicy', 
                       env,
                       device='cpu',  #usually cpu is faster for this size of networks
                       #device='cuda',  #usually cpu is faster for this size of networks
                       verbose=1)
           ts = -time.time()
           model.learn(total_timesteps=2000) 
           #model.learn(total_timesteps=2e7)  #not sufficient ...
           print('*** learning time total =',ts+time.time(),'***')
       
           #save learned model
           model.save("openAIgymTriplePendulum1e7d")
       else:
           #create vectorized environment, which is much faster for time
           #  consuming environments (otherwise learning algo may be the bottleneck)
           #  https://www.programcreek.com/python/example/121472/stable_baselines.common.vec_env.SubprocVecEnv
           import torch #stable-baselines3 is based on pytorch
           n_cores=14 #should be number of real cores (not threads)
           torch.set_num_threads(n_cores) #seems to be ideal to match the size of subprocVecEnv
           
           #test problem with nSteps=400 in time integration
           #1 core: learning time total = 28.73 seconds
           #4 core: learning time total = 8.10
           #8 core: learning time total = 4.48
           #14 core:learning time total = 3.77
           #standard DummyVecEnv version: 15.14 seconds
           print('using',n_cores,'cores')
   
           from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
           vecEnv = SubprocVecEnv([InvertedTriplePendulumEnv for i in range(n_cores)])
           
       
           #main learning task; 1e7 steps take 2-3 hours
           model = A2C('MlpPolicy', 
                       vecEnv, 
                       device='cpu',  #usually cpu is faster for this size of networks
                       #device='cuda',  #optimal with 64 SubprocVecEnv, torch.set_num_threads(1)
                       verbose=1)
           ts = -time.time()
           print('start learning...')
           #model.learn(total_timesteps=50000) 
           model.learn(total_timesteps=7e7)  #not sufficient ...
           print('*** learning time total =',ts+time.time(),'***')
       
           #save learned model
           model.save("openAIgymTriplePendulum1e7d")
   
       if False:
           #%%++++++++++++++++++++++++++++++++++++++++++++++++++
           #only load and test
           model = A2C.load("openAIgymTriplePendulum1e7")
           env = InvertedTriplePendulumEnv(thresholdFactor=15) #larger threshold for testing
           solutionFile='solution/learningCoordinates.txt'
           env.TestModel(numberOfSteps=2500, model=model, solutionFileName=solutionFile, 
                         stopIfDone=False, useRenderer=False, sleepTime=0) #just compute solution file
   
           #++++++++++++++++++++++++++++++++++++++++++++++
           #visualize (and make animations) in exudyn:
           from exudyn.interactive import SolutionViewer
           env.SC.visualizationSettings.general.autoFitScene = False
           solution = LoadSolutionFile(solutionFile)
           SolutionViewer(env.mbs, solution) #loads solution file via name stored in mbs
   
   
   


