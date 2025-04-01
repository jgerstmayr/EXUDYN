
.. _examples-openaigymnlinkadvanced:

*************************
openAIgymNLinkAdvanced.py
*************************

You can view and download this file on Github: `openAIgymNLinkAdvanced.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/openAIgymNLinkAdvanced.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  This file shows integration with OpenAI gym by testing a inverted quintuple pendulum example
   #
   # Author:    Johannes Gerstmayr
   # edited by: Peter Manzl          
   # Date:      2022-05-25
   #
   # Update:    2024-08-09: works now with 4 and 5 links using SAC; added truncated time and disturbance force to avoid training of many steps in straight configuration (and forgetting everything else...)
   # Notes:     requires pip install stable-baselines3[extra]
   # 
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   # import sys
   # sys.exudynFast = True #this variable is used to signal to load the fast exudyn module
   
   import exudyn as exu
   # from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   from exudyn.utilities import ObjectGround, VObjectGround, RigidBodyInertia, HTtranslate, HTtranslateY, HT0,\
                               MarkerNodeCoordinate, LoadCoordinate, LoadSolutionFile, MarkerKinematicTreeRigid, Force
   import exudyn.graphics as graphics #only import if it does not conflict
   from exudyn.robotics import Robot, RobotLink, VRobotLink, RobotBase, VRobotBase, RobotTool, VRobotTool
   from exudyn.artificialIntelligence import OpenAIGymInterfaceEnv, spaces, logger
   #from exudyn.artificialIntelligence import *
   import math
   import numpy as np
   
   import os
   import sys
   os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE"
   
   import stable_baselines3
   useOldGym = tuple(map(int, stable_baselines3.__version__.split('.'))) <= tuple(map(int, '1.8.0'.split('.')))
   
   ##%% here the number of links can be changed. Note that for n < 3 the actuator 
   # force might need to be increased
   nLinks = 3 #number of inverted links ...
   flagContinuous = True  # to choose for agent between A2C and SAC
   
   #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #if you like to visualize the training progress in tensorboard:
   # pip install tensorboard
   # open anaconda prompt (where your according python/environment runs)
   # go to directory: 
   #   solution/ 
   # call this in anaconda prompt:
   #   tensorboard --logdir=./tensorboard_log/
   # open web browser to show progress (reward, loss, ...):
   #   http://localhost:6006/
   
   
   #add logger for reward:
   from stable_baselines3.common.callbacks import BaseCallback
   from stable_baselines3.common.logger import Logger
   
   best_model_text = 'invpend_best_model'
   
   hasTensorboard = False
   try:
       import tensorboard
       hasTensorboard = True
       if __name__ == '__main__': 
           print('output written to tensorboard, start tensorboard to see progress!')
   except:
       pass
   
   #derive class from logger to log additional information
   #this is needed for vectorized environments, where reward is not logged automatically
   class RewardLoggingCallback(BaseCallback):
       def __init__(self, verbose=0, log_dir = 'solution/tensorboard_log', bestModelName=best_model_text):
           super(RewardLoggingCallback, self).__init__(verbose)
           self.log_dir = log_dir
           self.save_path = os.path.join(log_dir, bestModelName).replace('tensorboard_log/','')
           self.bestRewardSum = 0
           self.bestRewardSumPrint = 0
   
       #log mean values at rollout end
       def _on_rollout_end(self) -> None:
           rewardSum = -1
           if 'infos' in self.locals:
               info = self.locals['infos'][-1]
               # print('infos:', info)
               if 'rewardMean' in info:
                   self.logger.record("rollout/rewardMean", info['rewardMean'])
               if 'episodeLen' in info:
                   self.logger.record("rollout/episodeLen", info['episodeLen'])
               if 'rewardSum' in info:
                   self.logger.record("rollout/rewardSum", info['rewardSum'])
                   rewardSum = info['rewardSum']
           
   
           # New best model, you could save the agent here
           if rewardSum > self.bestRewardSum:
               self.bestRewardSum = rewardSum
               # Example for saving best model
               if self.verbose > 0 and rewardSum>100 and rewardSum > 1.2*self.bestRewardSumPrint:
                   self.bestRewardSumPrint = rewardSum
                   print("Saving new best model with reward sum "+str(rewardSum)+" to "+self.save_path)
               self.model.save(self.save_path)
   
   
       #log (possibly) every step 
       def _on_step(self) -> bool:
           #extract local variables to find reward
           if 'infos' in self.locals:
               info = self.locals['infos'][-1]
   
               if 'reward' in info:
                   self.logger.record("train/reward", info['reward'])
               #for SAC / A2C in non-vectorized envs, per episode:
               if 'episode' in info and 'r' in info['episode']:
                   self.logger.record("episode/reward", info['episode']['r'])
           return True
   #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   
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
           
           #added for continuous disturbance (reset)
           self.testModel = False #flag to show that model is evaluated
           self.lastReset = 0 #time of last reset
   
           # self.steps_beyond_done = False
           thresholdFactor = 1
           
           gravity = 9.81
           self.length = 1.
           width = 0.1*self.length
           masscart = 1.
           massarm = 0.1
           total_mass = massarm + masscart
           armInertia = self.length**2*massarm/12
   
           self.randomDisturbanceForce = 0.05
           self.maxTimeTruncate = 120 #seconds at which training is truncated
   
           self.randomInitializationValue = 0.05 #default is 0.05, which may be too large for many links
           # environment variables  and force magnitudes and are taken from the  
           # paper "Reliability evaluation of reinforcement learning methods for 
           # mechanical systems with increasing complexity", Manzl et al. 
           
           self.force_mag = 40 # 10 #10*2 works for 7e7 steps; must be larger for triple pendulum to be more reactive ...
           if self.nLinks == 1: 
               self.force_mag = 12
           if self.nLinks == 3: 
               self.force_mag = self.force_mag*1.5 
               thresholdFactor = 2
           if self.nLinks >= 4: 
               self.force_mag = self.force_mag*2.5
               thresholdFactor = 2.5
           if self.nLinks == 6: 
               self.force_mag = 200
               thresholdFactor = 4
   
           if self.flagContinuous: 
               self.force_mag *= 2 #continuous controller can have larger max value
   
           if 'thresholdFactor' in kwargs:
               thresholdFactor = kwargs['thresholdFactor']
   
           self.stepUpdateTime = 0.04  # step size for RL-method; PROBLEM: small steps: no learning progress; large steps: cannot control fast enough
           nSubSteps = 4 #for increased accuracy, definitely needed for nLinks >= 4
           if self.nLinks >= 4: 
               # self.stepUpdateTime = 0.04  # step size for RL-method
               # nSubSteps = 4
               self.randomInitializationValue = 0.04 #default is 0.05, which may be too large for 4 links
           if self.nLinks >= 5: 
               self.randomInitializationValue = 0.02
               self.randomDisturbanceForce = 0.02  # step size for RL-method
               # self.stepUpdateTime = 0.03  # step size for RL-method
               # nSubSteps = 3
           self.randomInitializationValue0 = self.randomInitializationValue
           
           background = graphics.CheckerBoard(point= [0,0.5*self.length,-2.5*width], 
                                                 normal= [0,0,1], size=16, size2=12, nTiles=20, nTiles2=12)
           
           oGround=self.mbs.AddObject(ObjectGround(referencePosition= [0,0,0],  #x-pos,y-pos,angle
                                              visualization=VObjectGround(graphicsData= [background])))
   
   
           #build kinematic tree with Robot class
           L = self.length
           w = width
           gravity3D = [0.,-gravity,0]
           graphicsBaseList = [graphics.Brick(size=[L*max(4,1+2*self.nLinks), 0.6*w, 0.6*w], color=graphics.color.grey)] #rail
           graphicsListCart = [graphics.Brick(size=[L, 1.2*w, 0.8*w], color=graphics.color.orange)] #rail
           
           #graphicsListTool = [graphics.Brick(size=[w, L, w], color=graphics.color.orange)]
           graphicsListLink1 = [graphics.Brick(centerPoint=[0,0.5*L,0], size=[w, L, w], 
                                               color=graphics.color.blue),
                               graphics.Cylinder([0,0,-0.6*w],[0,0,1.2*w],radius=0.6*width,nTiles=32, 
                                                 color=graphics.color.grey)]
           color2 = graphics.color.red
           graphicsListLink2 = [graphics.Brick(centerPoint=[0,0.5*L,0], size=[w, L, w], 
                                               color=color2),
                               graphics.Cylinder([0,0,-0.6*w],[0,0,1.2*w],radius=0.6*width,nTiles=32, 
                                                 color=graphics.color.grey)]
           graphicsListLinks=[graphicsListLink1,graphicsListLink2]
   
           newRobot = Robot(gravity=gravity3D,
                         base = RobotBase(visualization=VRobotBase(graphicsData=graphicsBaseList)),
                         tool = RobotTool(HT=HTtranslate([0,0.5*L,0]), 
                                          #visualization=VRobotTool(graphicsData=graphicsListTool)
                                          ),
                         referenceConfiguration = []) #referenceConfiguration created with 0s automatically
           
           pControl = 140
           dControl = 0.02*pControl
           #cart:
           Jlink = RigidBodyInertia(masscart, np.diag([0.1*masscart,0.1*masscart,0.1*masscart]), [0,0,0], inertiaTensorAtCOM=True)
           link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                            jointType='Px', preHT=HT0(), 
                            #PDcontrol=(pControl, dControl),
                            visualization=VRobotLink(linkColor=graphics.color.green,
                                                     showMBSjoint=False,
                                                     graphicsData = graphicsListCart))
           newRobot.AddLink(link)
       
           
           
           for i in range(self.nLinks):
               
               Jlink = RigidBodyInertia(massarm, np.diag([armInertia,0.1*armInertia,armInertia]), [0,0.5*L,0],
                                           inertiaTensorAtCOM=True) #only inertia_ZZ is important
               #Jlink = Jlink.Translated([0,0.5*L,0])
               preHT = HT0()
               if i > 0:
                   preHT = HTtranslateY(L)
       
               # if i==self.nLinks-1:
               #     pControl=0
               #     dControl=0
   
               link = RobotLink(Jlink.Mass(), Jlink.COM(), Jlink.InertiaCOM(), 
                                jointType='Rz', preHT=preHT, 
                                #PDcontrol=(0*pControl, 0.01*dControl),
                                visualization=VRobotLink(linkColor=graphics.color.blue,
                                                         showMBSjoint=False,
                                                         graphicsData = graphicsListLinks[i%2]))
               newRobot.AddLink(link)
           
           self.Jlink = Jlink
           
           
           sKT = []
           dKT = newRobot.CreateKinematicTree(mbs)
           self.oKT = dKT['objectKinematicTree']
           self.nKT = dKT['nodeGeneric']
           
           #print(mbs.GetObject(self.oKT))
           # sKT+=[mbs.AddSensor(SensorKinematicTree(objectNumber=oKT, linkNumber=self.nTotalLinks-1, 
           #                                         localPosition=locPos, storeInternal=True,
           #                                         outputVariableType=exu.OutputVariableType.Position))]
   
           #control force
           mCartCoordX = self.mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=self.nKT, coordinate=0))
           self.lControl = self.mbs.AddLoad(LoadCoordinate(markerNumber=mCartCoordX, load=0.))
   
           #marker on tip
           mTip = self.mbs.AddMarker(MarkerKinematicTreeRigid(objectNumber = self.oKT, 
                                                       linkNumber = self.nLinks, 
                                                       localPosition=[0,L,0]))
           self.lTip = self.mbs.AddLoad(Force(markerNumber=mTip, loadVector=[0,0,0]))
   
           #%%++++++++++++++++++++++++
           self.mbs.Assemble() #computes initial vector
           
           self.simulationSettings.timeIntegration.numberOfSteps = nSubSteps #this is the number of solver steps per RL-step
           self.simulationSettings.timeIntegration.endTime = 0 #will be overwritten in step
           self.simulationSettings.timeIntegration.verboseMode = 0
           self.simulationSettings.solutionSettings.writeSolutionToFile = False #set True only for postprocessing
           #self.simulationSettings.timeIntegration.simulateInRealtime = True
           self.simulationSettings.solutionSettings.solutionInformation = 'inverted '+str(self.nLinks)+'-link pendulum'
           
           self.simulationSettings.timeIntegration.newton.useModifiedNewton = True
           
           self.SC.visualizationSettings.general.drawWorldBasis=True
           self.SC.visualizationSettings.general.drawCoordinateSystem = False
           self.SC.visualizationSettings.general.graphicsUpdateInterval = 0.01
           self.SC.visualizationSettings.openGL.multiSampling=4
           self.SC.visualizationSettings.bodies.kinematicTree.showCOMframes = False
           SC.visualizationSettings.bodies.kinematicTree.showJointFrames = False
           #+++++++++++++++++++++++++++++++++++++++++++++++++++++
           # Angle at which to fail the episode
           # these parameters are used in subfunctions
           self.theta_threshold_radians = thresholdFactor* 18 * 2 * math.pi / 360
           self.x_threshold = thresholdFactor*3.6
           
           #must return state size
           stateSize = (self.nTotalLinks)*2 #the number of states (position/velocity that are used by learning algorithm)
   
           #to track mean reward:
           self.rewardCnt = 0
           self.rewardMean = 0
   
           return stateSize
   
       #**classFunction: OVERRIDE this function to set up self.action_space and self.observation_space
       def SetupSpaces(self):
   
           #space is 2 times larger than space at which we get done
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
               #action is -1 ... +1, then scaled with self.force_mag
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
           else:     
               force = self.force_mag if action == 1 else -self.force_mag        
           self.mbs.SetLoadParameter(self.lControl, 'load', force)
   
           time = self.simulationSettings.timeIntegration.endTime
   
           self.mbs.SetLoadParameter(self.lTip, 'loadVector', [0,0,0])
   
           
           # if time >= 4 and time < 4.04:
           #     tipForce = 0.05
           #     self.mbs.SetLoadParameter(self.lTip, 'loadVector', [tipForce,0,0])
           if time > 4 and np.random.randint(25*4) < 1: #randomly every 100 action steps
               tipForce = self.randomDisturbanceForce*(np.random.rand()*2-1) #+/-0.05 N or smaller
               if self.testModel:
                   print('time=',time,', tip force=',tipForce)
               self.mbs.SetLoadParameter(self.lTip, 'loadVector', [tipForce,0,0])
   
           #self.mbs.SetLoadParameter(self.lTip, 'loadVector', [5,0,0])
   
   
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
                  
           #set initial values into mbs immediately
           self.mbs.systemData.SetODE2Coordinates(initialValues, exu.ConfigurationType.Initial)
           self.mbs.systemData.SetODE2Coordinates_t(initialValues_t, exu.ConfigurationType.Initial)
   
           #this function is only called at reset(); so, we can use it to reset the mean reward:
           self.rewardCnt = 0
           self.rewardMean = 0
   
           return [initialValues,initialValues_t]
       
       #**classFunction: this is the objective which the RL method tries to maximize (average expected reward)
       def getReward(self): 
           #reward = 1 - 0.5 * abs(self.state[0])/self.x_threshold - 0.5 * abs(self.state[1]) / self.theta_threshold_radians
   
           #avoid larger motion of base
           reward = 1 - 0.25 * (abs(self.state[0])+(0.5*self.state[0])**2)/self.x_threshold
   
           #more emphasis on cart for low number of links (for more links, this is less prioritized)
           if self.nLinks == 1: 
               reward -= 2 * (abs(self.state[0]))/self.x_threshold
   
   
           #add a penalty for each link deviation
           for i in range(self.nLinks):
               reward -=  0.5 * abs(self.state[i+1]) / (self.theta_threshold_radians*self.nLinks)
           
           #more emphasis on last link:
           if self.nLinks >= 3:
               reward -=  0.5 * abs(self.state[self.nTotalLinks-1]) / (self.theta_threshold_radians)
   
           #penalize velocities of top link with higher number of links
           if self.nLinks >= 4:
               reward -= 0.25 * abs(self.state[-1])
   
           
           if reward < 0: reward = 0
   
           return reward 
   
       # #after some time, we could reset state similar to a disturbance
       # def ResetState(self):
       #     if not self.testModel:
       #         randSize = (self.stateSize)
   
       #         self.state = np.random.uniform(low=-self.randomInitializationValue, 
       #                                 high=self.randomInitializationValue, size=randSize)
       #         initialValues = self.state[0:self.nTotalLinks]
       #         initialValues_t = self.state[self.nTotalLinks:]
                   
       #         #set initial values into mbs immediately
       #         self.mbs.systemData.SetODE2Coordinates(initialValues, exu.ConfigurationType.Initial)
       #         self.mbs.systemData.SetODE2Coordinates_t(initialValues_t, exu.ConfigurationType.Initial)
       #         self.mbs.systemData.SetODE2Coordinates(initialValues, exu.ConfigurationType.Current)
       #         self.mbs.systemData.SetODE2Coordinates_t(initialValues_t, exu.ConfigurationType.Current)
   
   
       #**classFunction: openAI gym interface function which is called to compute one step
       def step(self, action):
           err_msg = f"{action!r} ({type(action)}) invalid"
           assert self.action_space.contains(action), err_msg
           assert self.state is not None, "Call reset before using step method."
           
           #++++++++++++++++++++++++++++++++++++++++++++++++++
           #main steps:
   
           self.MapAction2MBS(action)
           
           #this may be time consuming for larger models!
           self.IntegrateStep()
           
           done = self.Output2StateAndDone()
           # print('state:', self.state, 'done: ', done)
           #++++++++++++++++++++++++++++++++++++++++++++++++++
           #compute reward and done
   
           if not done:
               reward = self.getReward()
           elif self.steps_beyond_done is None:
               # system just fell down
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
   
           self.rewardCnt += 1
           self.rewardMean += reward
   
           info = {'reward': reward} #put reward into info for logger
   
           #for many links: start with low value and gradually increase randomInitializationValue
           if self.nLinks >= 6:
               iFact = self.randomInitializationValue / self.randomInitializationValue0
               iFactDesired = max(iFact,min(self.rewardCnt/200+0.5,4))
               self.randomInitializationValue = self.randomInitializationValue0 * iFactDesired
   
           #compute mean values per episode:
           if self.rewardCnt != 0: #per epsiode
               info['rewardMean'] = self.rewardMean / self.rewardCnt
               info['rewardSum'] = self.rewardMean
               info['episodeLen'] = self.rewardCnt
   
           terminated, truncated = done, False # since stable-baselines3 > 1.8.0 implementations terminated and truncated 
           if self.simulationSettings.timeIntegration.endTime > self.maxTimeTruncate: truncated = True
   
           if useOldGym:
               return np.array(self.state, dtype=np.float32), reward, terminated, info
           else:
               return np.array(self.state, dtype=np.float32), reward, terminated, truncated, info
   
   
   
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++
   if __name__ == '__main__': #this is only executed when file is direct called in Python
       import time
           
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++
       #use some learning algorithm:
       #pip install stable_baselines3
       from stable_baselines3 import A2C, SAC
       
       tensorboard_log = None #no logging
       rewardCallback = None
       verbose = 0 #turn off to just view in tensorboard
       if hasTensorboard: #only us if tensorboard is available
           tensorboard_log = "solution/tensorboard_log/" #dir
           rewardCallback = RewardLoggingCallback(verbose=1, bestModelName=best_model_text+str(nLinks))
       else:
           verbose = 0 #turn on without tensorboard
       
       # here the model is loaded (either for vectorized or scalar environmentÂ´using SAC or A2C).     
       def getModel(flagContinuous, myEnv, modelType='SAC', learning_starts = 100,
                       learning_rate=8e-4, batch_size=128, use_sde = True, tau = 0.005, gamma = 0.99): 
   
           if flagContinuous : 
               if modelType=='SAC': 
                   model = SAC('MlpPolicy',
                          env = myEnv,
                          learning_rate = learning_rate, #8e-4; default: 3e-4
                          device = 'cpu', #usually cpu is faster for this size of networks
                          batch_size = batch_size,     #128; default: 256
                          use_sde = use_sde,     #False; default: False
                          learning_starts = learning_starts, #default: 100
                          tensorboard_log = tensorboard_log,
                          verbose = verbose)
               elif modelType == 'A2C': 
                   model = A2C('MlpPolicy', 
                           myEnv, 
                           device='cpu',  
                           tensorboard_log=tensorboard_log,
                           verbose=verbose)
               else: 
                   print('Please specify the modelType.')
                   raise ValueError
           else: 
               model = A2C('MlpPolicy', 
                       myEnv, 
                       learning_rate=0.0007*0.5,
                       device='cpu',  #usually cpu is faster for this size of networks
                       #device='cuda',  #optimal with 64 SubprocVecEnv, torch.set_num_threads(1)
                       tensorboard_log=tensorboard_log,
                       verbose=verbose)
           return model
   
   
       modelType = 'SAC' #use A2C or SAC
       #create model and do reinforcement learning
       modelName = 'openAIgym{}Link{}'.format(nLinks, 'Continuous'*flagContinuous)
       if True: #set to False, if only evaluated
           if False: #'scalar' environment, slower:
               env = InvertedNPendulumEnv()
   
               #first, check if model runs:
               if False:
                   env.TestModel(numberOfSteps=2000, seed=42, sleepTime=0.02, useRenderer=True)
                   sys.exit()
   
               model = getModel(flagContinuous, env, modelType=modelType) 
   
               ts = -time.time()
               model.learn(total_timesteps=int(250e0), #min 250k steps required to start having success to stabilize double pendulum
                           # progress_bar=True, #requires tqdm and rich package; set True to only see progress and set log_interval very high
                           progress_bar=True,
                           log_interval=100, #logs per episode; influences local output and tensorboard
                           callback = rewardCallback,
                           )
               
               print('*** learning time total =',ts+time.time(),'***')
           
               #save learned model
               
               model.save("solution/" + modelName)
           else: #parallel; faster #set verbose=0 in getModel()!
               import torch #stable-baselines3 is based on pytorch
               n_cores= 2*max(1,int(os.cpu_count()/2)) #n_cores should be number of real cores (not threads)
               n_cores = 24 #override if you like to manually select
               torch.set_num_threads(n_cores) #seems to be ideal to match the size of subprocVecEnv
               print('using',n_cores,'cores')
   
               modelCnt = ''
   
               #perform some variations, to check if performance increases (or just repeat case)
               variations = [
                   {},
                   #{'batch_size':256},
                   #{'tau':0.01},
                   #{}, 
               ]
   
               i = -1
               print('performing '+str(len(variations))+' variations')
               for d in variations:
                   i += 1
                   modelCnt = '_Trial'+str(i)
                   rewardCallback.save_path = rewardCallback.save_path.replace('_Trial'+str(i-1),'')+modelCnt
                   rewardCallback.bestRewardSum = 0
                   rewardCallback.bestRewardSumPrint = 0
   
                   from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
                   vecEnv = SubprocVecEnv([InvertedNPendulumEnv for i in range(n_cores)])
                   
               
                   #main learning task;  with 20 cores 800 000 steps take in the continuous 
                   # case approximatly 18 minutes (SAC), discrete (A2C) takes 2 minutes. 
                   model = getModel(flagContinuous, vecEnv, modelType=modelType, **d)
   
                   ts = -time.time()
                   print('start learning of agent with {}'.format(str(model.policy).split('(')[0])+', using options: '+str(d))
                   print('name='+modelName+modelCnt)
   
                   #max number of steps; NOTE: this does not always work, run several cases!
                   nSteps = 400_000
                   log_interval=100
                   if nLinks == 2: nSteps = 800_000
                   if nLinks == 3: nSteps = 1_000_000
                   if nLinks == 4: nSteps = 5_000_000
                   if nLinks == 5: nSteps = 40_000_000
                   
                   if nSteps <= 1000000: log_interval=25
                   
                   model.learn(total_timesteps=int(nSteps), #A2C starts working above 250k; SAC similar
                               progress_bar=True, #requires tqdm and rich package; set True to only see progress and set log_interval very high (100_000_000)
                               log_interval=log_interval, #logs per episode; influences local output and tensorboard
                               callback = rewardCallback,
                               )
                   print('*** learning time total =',ts+time.time(),'***')
               
                   #save learned model
                   model.save("solution/" + modelName+modelCnt)
   
       if True:
           #%%++++++++++++++++++++++++++++++++++++++++++++++++++
           #only load and test
           if True: 
               #here you have to select the name of the model you like to test:
               if flagContinuous and modelType == 'A2C': 
                   model = A2C.load('solution/tensorboard_log/invpend_best_model'+str(nLinks)+'_Trial0')
               else: 
                   model = SAC.load('solution/tensorboard_log/invpend_best_model'+str(nLinks)+'_Trial0')
           
   
           env = InvertedNPendulumEnv(thresholdFactor=5) #larger threshold for testing
           solutionFile='solution/learningCoordinates.txt'
   
   
           #reduce disturbances for evaluation (for 1-link, you may also increase force by 10)
           # env.randomInitializationValue *= 0.5
           # env.randomDisturbanceForce *= 0.5
           if nLinks == 4: 
               env.randomInitializationValue = 0.02 #smaller for evaluation
           if nLinks == 5: 
               env.randomInitializationValue = 0.01 #smaller for evaluation
               env.randomDisturbanceForce = 0.01 #smaller for evaluation
   
           #env.stepUpdateTime = 0.01
           env.testModel = True #to avoid reset
           print('rand init=',env.randomInitializationValue)
   
           env.TestModel(numberOfSteps=500*1, model=model, solutionFileName=solutionFile, 
                         stopIfDone=False, useRenderer=False, sleepTime=0) #just compute solution file
           #++++++++++++++++++++++++++++++++++++++++++++++
           #visualize (and make animations) in exudyn:
           from exudyn.interactive import SolutionViewer
           env.SC.visualizationSettings.general.autoFitScene = False
           env.SC.visualizationSettings.window.renderWindowSize = [1600,1200]
           env.SC.visualizationSettings.openGL.shadow = 0.25
           env.SC.visualizationSettings.openGL.light0position = [3,8,4,0]
           
           solution = LoadSolutionFile(solutionFile)
           
           SolutionViewer(env.mbs, solution, timeout=0.005, rowIncrement=5) #loads solution file via name stored in mbs
   


