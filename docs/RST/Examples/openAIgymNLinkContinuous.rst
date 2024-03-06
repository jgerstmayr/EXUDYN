
.. _examples-openaigymnlinkcontinuous:

***************************
openAIgymNLinkContinuous.py
***************************

You can view and download this file on Github: `openAIgymNLinkContinuous.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/openAIgymNLinkContinuous.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  This file shows integration with OpenAI gym by testing a inverted quadruple pendulum example
   #
   # Author:    Johannes Gerstmayr
   # edited by: Peter Manzl          
   # Date:      2022-05-25
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   # import sys
   # sys.exudynFast = True #this variable is used to signal to load the fast exudyn module
   
   import exudyn as exu
   from exudyn.utilities import *
   from exudyn.robotics import *
   from exudyn.artificialIntelligence import *
   import math
   
   import os
   os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"
   
   ##%% here the number of links can be changed. Note that for n < 3 the actuator 
   # force might need to be increased
   nLinks = 2 #number of inverted links ...
   flagContinuous = True  # to choose for agent between A2C and SAC
   
   
   class InvertedNPendulumEnv(OpenAIGymInterfaceEnv):
           
       #**classFunction: OVERRIDE this function to create multibody system mbs and setup simulationSettings; call Assemble() at the end!
       #                 you may also change SC.visualizationSettings() individually; kwargs may be used for special setup
       def CreateMBS(self, SC, mbs, simulationSettings, **kwargs):
   
           #%%++++++++++++++++++++++++++++++++++++++++++++++
           #this model uses kwargs: thresholdFactor
           
           global nLinks
           global flagContinuous
           self.nLinks = nLinks
           self.flagContinuous = flagContinuous
           self.nTotalLinks = nLinks+1
           # self.steps_beyond_done = False
           thresholdFactor = 1
           
           gravity = 9.81
           self.length = 1.
           width = 0.1*self.length
           masscart = 1.
           massarm = 0.1
           total_mass = massarm + masscart
           armInertia = self.length**2*0.5*massarm
           
           # environment variables  and force magnitudes and are taken from the  
           # paper "Reliability evaluation of reinforcement learning methods for 
           # mechanical systems with increasing complexity", Manzl et al. 
           
           self.force_mag = 40 # 10 #10*2 works for 7e7 steps; must be larger for triple pendulum to be more reactive ...
           if self.nLinks == 1: 
               self.force_mage = 12
           if self.nLinks == 3: 
               self.force_mag = self.force_mag*1.5 
               thresholdFactor = 2
           if self.nLinks == 4: 
               self.force_mag = self.force_mag*3
               thresholdFactor = 5
   
           if 'thresholdFactor' in kwargs:
               thresholdFactor = kwargs['thresholdFactor']
           
           self.stepUpdateTime = 0.02  # seconds between state updates
           
           background = GraphicsDataCheckerBoard(point= [0,0.5*self.length,-0.5*width], 
                                                 normal= [0,0,1], size=10, size2=6, nTiles=20, nTiles2=12)
           
           oGround=self.mbs.AddObject(ObjectGround(referencePosition= [0,0,0],  #x-pos,y-pos,angle
                                              visualization=VObjectGround(graphicsData= [background])))
   
   
           L = self.length
           w = width
           gravity3D = [0.,-gravity,0]
           graphicsBaseList = [GraphicsDataOrthoCubePoint(size=[L*4, 0.8*w, 0.8*w], color=color4grey)] #rail
           
           newRobot = Robot(gravity=gravity3D,
                         base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
                         tool = RobotTool(HT=HTtranslate([0,0.5*L,0]), visualization=VRobotTool(graphicsData=[
                             GraphicsDataOrthoCubePoint(size=[w, L, w], color=color4orange)])),
                         referenceConfiguration = []) #referenceConfiguration created with 0s automatically
           
           #cart:
           Jlink = RigidBodyInertia(masscart, np.diag([0.1*masscart,0.1*masscart,0.1*masscart]), [0,0,0])
           link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                            jointType='Px', preHT=HT0(), 
                            # PDcontrol=(pControl, dControl),
                            visualization=VRobotLink(linkColor=color4lawngreen))
           newRobot.AddLink(link)
       
           
           
           for i in range(self.nLinks):
               
               Jlink = RigidBodyInertia(massarm, np.diag([armInertia,0.1*armInertia,armInertia]), [0,0.5*L,0]) #only inertia_ZZ is important
               #Jlink = Jlink.Translated([0,0.5*L,0])
               preHT = HT0()
               if i > 0:
                   preHT = HTtranslateY(L)
       
               link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                                jointType='Rz', preHT=preHT, 
                                #PDcontrol=(pControl, dControl),
                                visualization=VRobotLink(linkColor=color4blue))
               newRobot.AddLink(link)
           
           self.Jlink = Jlink
           
           
           sKT = []
           dKT = newRobot.CreateKinematicTree(mbs)
           self.oKT = dKT['objectKinematicTree']
           self.nKT = dKT['nodeGeneric']
           
           # sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=self.nTotalLinks-1, 
           #                                         localPosition=locPos, storeInternal=True,
           #                                         outputVariableType=exu.OutputVariableType.Position))]
   
           #control force
           mCartCoordX = self.mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=self.nKT, coordinate=0))
           self.lControl = self.mbs.AddLoad(LoadCoordinate(markerNumber=mCartCoordX, load=0.))
           
           #%%++++++++++++++++++++++++
           self.mbs.Assemble() #computes initial vector
           
           self.simulationSettings.timeIntegration.numberOfSteps = 1
           self.simulationSettings.timeIntegration.endTime = 0 #will be overwritten in step
           self.simulationSettings.timeIntegration.verboseMode = 0
           self.simulationSettings.solutionSettings.writeSolutionToFile = False
           #self.simulationSettings.timeIntegration.simulateInRealtime = True
           
           self.simulationSettings.timeIntegration.newton.useModifiedNewton = True
           
           self.SC.visualizationSettings.general.drawWorldBasis=True
           self.SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
           self.SC.visualizationSettings.openGL.multiSampling=4
           
           #self.simulationSettings.solutionSettings.solutionInformation = "Open AI gym"
           
           #+++++++++++++++++++++++++++++++++++++++++++++++++++++
           # Angle at which to fail the episode
           # these parameters are used in subfunctions
           self.theta_threshold_radians = thresholdFactor* 18 * 2 * math.pi / 360
           self.x_threshold = thresholdFactor*3.6
           
           #must return state size
           stateSize = (self.nTotalLinks)*2 #the number of states (position/velocity that are used by learning algorithm)
   
           return stateSize
   
       #**classFunction: OVERRIDE this function to set up self.action_space and self.observation_space
       def SetupSpaces(self):
   
           high = np.array(
               [
                   self.x_threshold * 2,
               ] +
               [
                   self.theta_threshold_radians * 2,
               ] * self.nLinks +
               [                
                   np.finfo(np.float32).max,
               ] * self.nTotalLinks
               ,
               dtype=np.float32,
           )
           
           
           #+++++++++++++++++++++++++++++++++++++++++++++++++++++
           #see https://github.com/openai/gym/blob/64b4b31d8245f6972b3d37270faf69b74908a67d/gym/core.py#L16
           #for Env:
           if flagContinuous: 
               self.action_space = spaces.Box(low=np.array([-1 ], dtype=np.float32),
                                              high=np.array([1], dtype=np.float32), dtype=np.float32)
           else: 
               self.action_space = spaces.Discrete(2)
           
           self.observation_space = spaces.Box(-high, high, dtype=np.float32)        
           #+++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
       #**classFunction: this function is overwritten to map the action given by learning algorithm to the multibody system (environment)
       def MapAction2MBS(self, action):
           if flagContinuous: 
               force = action[0] * self.force_mag
               # print(force)
           else:     
               force = self.force_mag if action == 1 else -self.force_mag        
           self.mbs.SetLoadParameter(self.lControl, 'load', force)
   
       #**classFunction: this function is overwrritten to collect output of simulation and map to self.state tuple
       #**output: return bool done which contains information if system state is outside valid range
       def Output2StateAndDone(self):
           
           #+++++++++++++++++++++++++
           statesVector =  self.mbs.GetNodeOutput(self.nKT, variableType=exu.OutputVariableType.Coordinates)
           statesVector_t =  self.mbs.GetNodeOutput(self.nKT, variableType=exu.OutputVariableType.Coordinates_t)
           self.state = tuple(list(statesVector) + list(statesVector_t)) #sorting different from previous implementation
           cartPosX = statesVector[0]
   
           done = bool(
               cartPosX < -self.x_threshold
               or cartPosX > self.x_threshold
               or max(statesVector[1:self.nTotalLinks]) > self.theta_threshold_radians 
               or min(statesVector[1:self.nTotalLinks]) < -self.theta_threshold_radians 
               )
           # print("cartPosX: ", cartPosX, ", done: ", done) 
           return done
   
       
       #**classFunction: OVERRIDE this function to maps the current state to mbs initial values
       #**output: return [initialValues, initialValues\_t] where initialValues[\_t] are ODE2 vectors of coordinates[\_t] for the mbs
       def State2InitialValues(self):
           #+++++++++++++++++++++++++++++++++++++++++++++
           initialValues = self.state[0:self.nTotalLinks]
           initialValues_t = self.state[self.nTotalLinks:]
                  
           return [initialValues,initialValues_t]
           
       def getReward(self): 
           reward = 1 - 0.5 * abs(self.state[0])/self.x_threshold - 0.5 * abs(self.state[1]) / self.theta_threshold_radians
           return reward 
   
       #**classFunction: openAI gym interface function which is called to compute one step
       def step(self, action):
           err_msg = f"{action!r} ({type(action)}) invalid"
           assert self.action_space.contains(action), err_msg
           assert self.state is not None, "Call reset before using step method."
           
           #++++++++++++++++++++++++++++++++++++++++++++++++++
           #main steps:
           [initialValues,initialValues_t] = self.State2InitialValues()
           self.mbs.systemData.SetODE2Coordinates(initialValues, exu.ConfigurationType.Initial)
           self.mbs.systemData.SetODE2Coordinates_t(initialValues_t, exu.ConfigurationType.Initial)
   
           self.MapAction2MBS(action)
           
           #this may be time consuming for larger models!
           self.IntegrateStep()
           
           done = self.Output2StateAndDone()
           # print('state:', self.state, 'done: ', done)
           #++++++++++++++++++++++++++++++++++++++++++++++++++
           #compute reward and done
           # chi_x = []
           # chi_phi = [0.1, 0.1]
           x = self.state[0]
           phi = self.state[1:3]
           # self.state [0] + 
           # if done: 
               # print('done at step {}'.format(self.step))
           if not done:
               
               reward = self.getReward()
           elif self.steps_beyond_done is None:
               # Arm1 just fell!
               self.steps_beyond_done = 0
               reward = self.getReward()
           else:
               
               if self.steps_beyond_done == 0:
                   logger.warn(
                       "You are calling 'step()' even though this "
                       "environment has already returned done = True. You "
                       "should always call 'reset()' once you receive 'done = "
                       "True' -- any further steps are undefined behavior."
                   )
               self.steps_beyond_done += 1
               reward = 0.0
   
           return np.array(self.state, dtype=np.float32), reward, done, {}
   
   
   
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++
   if __name__ == '__main__': #this is only executed when file is direct called in Python
       import time
           
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++
       #use some learning algorithm:
       #pip install stable_baselines3
       from stable_baselines3 import A2C, SAC
       
       
       # here the model is loaded (either for vectorized or scalar environmentÂ´using SAC or A2C).     
       def getModel(flagContinuous, myEnv, modelType='SAC'): 
           if flagContinuous : 
               if modelType=='SAC': 
                   model = SAC('MlpPolicy',
                          env=myEnv,
                          learning_rate=8e-4,
                          device='cpu', #usually cpu is faster for this size of networks
                          batch_size=128,
                          verbose=1)
               elif modelType == 'A2C': 
                   model = A2C('MlpPolicy', 
                           myEnv, 
                           device='cpu',  
                           verbose=1)
               else: 
                   print('Please specify the modelType.')
                   raise ValueError
           else: 
               model = A2C('MlpPolicy', 
                       myEnv, 
                       device='cpu',  #usually cpu is faster for this size of networks
                       #device='cuda',  #optimal with 64 SubprocVecEnv, torch.set_num_threads(1)
                       verbose=1)
           return model
   
   
       #create model and do reinforcement learning
       modelName = 'openAIgym{}Link{}'.format(nLinks, 'Continuous'*flagContinuous)
       if False: #'scalar' environment:
           env = InvertedNPendulumEnv()
           #check if model runs:
           #env.SetSolver(exu.DynamicSolverType.ExplicitMidpoint)
           #env.SetSolver(exu.DynamicSolverType.RK44) #very acurate
           #env.TestModel(numberOfSteps=200, seed=42, sleepTime=0.02*0, useRenderer=True)
           model = getModel(flagContinuous, env)
           # env.useRenderer = True
           # env.render()
   
           ts = -time.time()
           model.learn(total_timesteps=20000) 
           
           print('*** learning time total =',ts+time.time(),'***')
       
           #save learned model
           
           model.save("solution/" + modelName)
       else:
           import torch #stable-baselines3 is based on pytorch
           n_cores= max(1,int(os.cpu_count()/2-1)) #should be number of real cores (not threads)
           torch.set_num_threads(n_cores) #seems to be ideal to match the size of subprocVecEnv
           
           print('using',n_cores,'cores')
   
           from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
           vecEnv = SubprocVecEnv([InvertedNPendulumEnv for i in range(n_cores)])
           
       
           #main learning task;  with 20 cores 800 000 steps take in the continous 
           # case approximatly 18 minutes (SAC), discrete (A2C) takes 2 minutes. 
           model = getModel(flagContinuous, vecEnv, modelType='SAC')
   
           ts = -time.time()
           print('start learning of agent with {}'.format(str(model.policy).split('(')[0]))
           # model.learn(total_timesteps=50000) 
           model.learn(total_timesteps=int(1_000_000))
           print('*** learning time total =',ts+time.time(),'***')
       
           #save learned model
           model.save("solution/" + modelName)
   
       if True:
           #%%++++++++++++++++++++++++++++++++++++++++++++++++++
           #only load and test
           if False: 
               if flagContinuous and modelType == 'A2C': 
                   model = SAC.load("solution/" + modelName)
               else: 
                   model = A2C.load("solution/" + modelName)
           
           env = InvertedNPendulumEnv(thresholdFactor=5) #larger threshold for testing
           solutionFile='solution/learningCoordinates.txt'
           env.TestModel(numberOfSteps=1000, model=model, solutionFileName=solutionFile, 
                         stopIfDone=False, useRenderer=False, sleepTime=0) #just compute solution file
   
           #++++++++++++++++++++++++++++++++++++++++++++++
           #visualize (and make animations) in exudyn:
           from exudyn.interactive import SolutionViewer
           env.SC.visualizationSettings.general.autoFitScene = False
           solution = LoadSolutionFile(solutionFile)
           SolutionViewer(env.mbs, solution) #loads solution file via name stored in mbs
   
   
   


