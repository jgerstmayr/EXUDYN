#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  This file shows reinfocement learning for spotModel.py (has to be in same folder)
#
# Author:   Johannes Gerstmayr, Janik Thentie
# 
# Date:      2025-03-10
#
# Notes:     requires pip install stable-baselines3[extra]
# 
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import sys
import os
os.environ["KMP_DUPLICATE_LIB_OK"]="TRUE" #this avoids OMP problems

import exudyn as exu
from exudyn.utilities import *
from exudyn.robotics import *
from exudyn.artificialIntelligence import OpenAIGymInterfaceEnv, spaces, logger

from exudyn.utilities import ObjectGround, VObjectGround, RigidBodyInertia, HTtranslate, HTtranslateY, HT0,\
                            MarkerNodeCoordinate, LoadCoordinate, LoadSolutionFile

import stable_baselines3


print('stable baselines version=',stable_baselines3.__version__)

from stable_baselines3 import PPO, A2C, SAC
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import BaseCallback
import numpy as np
import gym

import RL_Spot

dtypeNumpy = np.float64

# call this in anaconda prompt:
#   tensorboard --logdir=./tensorboard_log/
# open web browser to show progress (reward, loss, ...):
#   http://localhost:6006/

hasTensorboard = False
try:
    import tensorboard
    hasTensorboard = True
    print('output written to tensorboard, start tensorboard to see progress!')
except:
    pass

class RewardLoggingCallback(BaseCallback):
    def __init__(self, verbose=0, log_dir = 'solution', 
                 bestModelName='best_model', 
                 nEnvs=1, #should be number of vector environments (1 in case of single env)
                 ):
        super(RewardLoggingCallback, self).__init__(verbose)
        self.log_dir = log_dir
        self.save_path = os.path.join(log_dir, bestModelName)
        self.bestRewardSum = 0
        self.bestRewardSumPrint = 0
        self.rolloutsSinceLastSaved = 0
        self.stepsLastSaved = 0
        self.nSteps = 0
        self.nEnvs = nEnvs
        
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
            #print('info:',info)
        
        if (rewardSum > 5
            and self.nSteps > self.stepsLastSaved+20000
            #and self.rolloutsSinceLastSaved>=10
            ):
            try:
                self.model.save(self.save_path+'_temp')
                print('save temp model; rewardSum=', round(rewardSum,2), ', steps=',self.nSteps)
            except:
                print('save failed (other thread writing?)')
            self.stepsLastSaved = self.nSteps
            self.rolloutsSinceLastSaved = 0
        else:
            self.rolloutsSinceLastSaved += 1
            
        # New best model, you could save the agent here
        if rewardSum > self.bestRewardSum:
            self.bestRewardSum = rewardSum
            # Example for saving best model
            if rewardSum>50: #other models make no sense, they are too bad
                if self.verbose > 0 and rewardSum > 1.1*self.bestRewardSumPrint:
                    self.bestRewardSumPrint = rewardSum
                    print("Save best model; rewardSum="+str(round(rewardSum,2))+" to "+self.save_path+', steps=',self.nSteps)
                    try:
                        self.model.save(self.save_path)
                    except:
                        print('save failed (other thread writing?)')

            
    #log (possibly) every step 
    def _on_step(self) -> bool:
        #extract local variables to find reward
        self.nSteps+=self.nEnvs
        if 'infos' in self.locals:
            info = self.locals['infos'][-1]

            if 'reward' in info:
                self.logger.record("train/reward", info['reward'])
            #for SAC / A2C in non-vectorized envs, per episode:
            if 'episode' in info and 'r' in info['episode']:
                self.logger.record("episode/reward", info['episode']['r'])
        return True


#%%#########################
addShoulderContact = False

class SpotEnv(OpenAIGymInterfaceEnv): 
    # def __init__(self):
        
        
    def CreateMBS(self, SC, mbs, simulationSettings, **kwargs):
        global addShoulderContact
        #%%++++++++++++++++++++++++++++++++++++++++++++++             
        self.doPlanar = False
        self.useLegContactState = True
        self.maxDistTarget = 5 #will be written in reset
        self.target_position = np.array([4, 0*(1-self.doPlanar), 0])

        self.rendererRunning= None
        self.useRenderer = False #turn this on if needed       
        

        #%%++++++++++++++++++++++++
        self.cntCalls = 0
        
        self.mbs, self.SC, self.oKT, self.nKT = RL_Spot.GetModel(addShoulderContact=addShoulderContact)
        self.legsInit = self.mbs.variables['legsInit']
        self.legMarkers = self.mbs.variables['legMarkers']
        self.legRadius = self.mbs.variables['legRadius']
        self.legMarkers = self.mbs.variables['legMarkers']
        self.zContact = self.mbs.variables['zContact']+0.05

        # print('model loaded!', flush=True)
        
        self.targetGraphics = self.mbs.CreateGround(referencePosition=[self.target_position[0], self.target_position[1], -0.5],
                                                    graphicsDataList=[exu.graphics.Sphere(radius=0.1, nTiles=32, color=[1,0,0,0.2])])
        self.mbs.Assemble() #computes initial vector
        
        self.simulationSettings.timeIntegration.numberOfSteps = 100 #this is the number of solver steps per RL-step
        self.simulationSettings.timeIntegration.endTime = 0 #will be overwritten in step
        # self.simulationSettings.timeIntegration.verboseMode = 1
        self.simulationSettings.solutionSettings.writeSolutionToFile = False #set True only for postprocessing
        #self.simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
        
        self.simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False
        self.simulationSettings.timeIntegration.explicitIntegration.computeMassMatrixInversePerBody = True
        
        
        self.SC.visualizationSettings.contact.showSpheres = False
        self.SC.visualizationSettings.general.drawWorldBasis = True
        self.SC.visualizationSettings.general.drawCoordinateSystem = False
        self.SC.visualizationSettings.general.graphicsUpdateInterval = 0.2
        self.SC.visualizationSettings.openGL.multiSampling = 4        
        self.SC.visualizationSettings.bodies.kinematicTree.showJointFrames = False
        self.SC.visualizationSettings.openGL.shadow = 0.25
        
        self.state = None
        self.done = False
        
        #self.stepUpdateTime = 1 #sek step size for RL-method TOO LARGE!
        self.stepUpdateTime = 0.02 #0.02 works; sek step size for RL-method
        self.episodeMaxLen = 300   #250 works; max number of steps before time (e.g. timeout or avoid long episodes where nothing happens)
        
        #to track mean reward:
        self.rewardCnt = 0
        self.rewardMean = 0
        
        #must return state size
        self.nTotalLinks = 18
        self.nActuatedJoints = 12
        self.nLegStates = 4*(self.useLegContactState)
        stateSize = (self.nTotalLinks)*2 + self.nLegStates #the number of states (position/velocity that are used by learning algorithm)

        self.maxAngle = 25 * np.pi/180. #original: 72
        self.maxVel = 720 * np.pi/180  #original 1*np.pi; translation and rotation

        self.previousSetCoordinates = np.zeros(self.nActuatedJoints)
        self.useIncrementalSetValues = True #use this to have "velocity" control

        #avoid randomization of start config
        self.randomInitializationValue = 0 #single float or list
        #this may be harder:
        if not self.doPlanar and True:
             self.randomInitializationValue = [3,2,0, 0,0,0.1*np.pi]+[0]*(self.nTotalLinks*2-6)+[0]*self.nLegStates

        #to track mean reward:
        self.rewardCnt = 0
        self.rewardMean = 0        
        
        
        # print('create finished!')
        return stateSize
    
    def PreInitializeSolver(self):
        self.SetSolver(solverType=exu.DynamicSolverType.ExplicitEuler)

    def SetupSpaces(self):       
        
        maxAngleSet = self.maxAngle
        if self.useIncrementalSetValues:
            maxAngleSet = self.maxAngle/5 #10
        
        self.low = np.array([-5]*6 + [-self.maxAngle*5]*self.nActuatedJoints + [-self.maxVel]*self.nTotalLinks+[-100]*self.nLegStates, dtype=dtypeNumpy)  # Minimale Winkel + Geschindigkeiten
        self.high = np.array([5]*6 + [self.maxAngle*5]*self.nActuatedJoints + [self.maxVel]*self.nTotalLinks+[1]*self.nLegStates, dtype=dtypeNumpy)   # Maximale Winkel + Geschindigkeiten      
        
        self.action_space = self.action_space = spaces.Box(low=np.array([-maxAngleSet]*self.nActuatedJoints, dtype=dtypeNumpy),
                                       high=np.array([maxAngleSet]*self.nActuatedJoints, dtype=dtypeNumpy), dtype=dtypeNumpy)
    
        self.observation_space = spaces.Box(low= self.low, high = self.high, dtype=dtypeNumpy)
        #+++++++++++++++++++++++++++++++++++++++++++++++++++++

        
    def MapAction2MBS(self, action):   
        modAction = np.array(action)
        
        if self.doPlanar:
            modAction[0] = 0
            modAction[3] = 0
            modAction[6] = 0
            modAction[9] = 0
            modAction[0:3] = 0.5*(modAction[0:3]+modAction[3:6])
            modAction[6:9] = 0.5*(modAction[6:9]+modAction[9:12])
            modAction[3:6] = modAction[0:3]
            modAction[9:12] = modAction[6:9]

        if self.useIncrementalSetValues:
            #print(np.round(modAction,3))
            modAction = self.previousSetCoordinates + modAction
            minSideAngle = 0.1*self.maxAngle #sign depends on side
            maxSideAngle = 0.4*self.maxAngle #sign depends on side
            minAngle1 = self.legsInit[1]-self.maxAngle
            maxAngle1 = self.legsInit[1]+self.maxAngle
            minAngle2 = self.legsInit[2]-self.maxAngle
            maxAngle2 = self.legsInit[2]+self.maxAngle
            modAction = np.clip(modAction, 
                                [-minSideAngle,minAngle1,minAngle2]+
                                [-maxSideAngle,minAngle1,minAngle2]+
                                [-minSideAngle,minAngle1,minAngle2]+
                                [-maxSideAngle,minAngle1,minAngle2],
                                [maxSideAngle,maxAngle1,maxAngle2]+
                                [minSideAngle,maxAngle1,maxAngle2]+
                                [maxSideAngle,maxAngle1,maxAngle2]+
                                [minSideAngle,maxAngle1,maxAngle2],
                                )
            self.previousSetCoordinates = modAction

        setJoint = list(self.state[:6]) + list(modAction)
              
        self.mbs.SetObjectParameter(self.oKT, 'jointPositionOffsetVector', setJoint)
        
        
    def Output2StateAndDone(self):        
        #+++++++++++++++++++++++++
        statesVector =  self.mbs.GetNodeOutput(self.nKT, variableType=exu.OutputVariableType.Coordinates)
        statesVector_t =  self.mbs.GetNodeOutput(self.nKT, variableType=exu.OutputVariableType.Coordinates_t)

        #add position of leg contact relative to ground -> allows RL to see if foot has contact
        legStates = [0]*self.nLegStates
        for i in range(self.nLegStates):
            legStates[i] = self.mbs.GetMarkerOutput(markerNumber=self.legMarkers[i], 
                                               variableType=exu.OutputVariableType.Position)[2]-self.zContact
            if legStates[i] > 0: 
                legStates[i] = 0
            else:
                legStates[i] *= 100

        #TODO: add destination to state
        #      add randomization for destination

        self.state = np.array(list(statesVector) + list(statesVector_t) + legStates, dtype=dtypeNumpy)
        
        #done definieren
        isInObservationSpace = np.all(self.state >= self.low) and np.all(self.state <= self.high)
        # if not isInObservationSpace:
        #     print('out of observation space', np.round(self.state,3))
            
        done  = (statesVector[2] < -0.4 #-0.4
                 or abs(statesVector[3]) > 30*np.pi/180 #body points sidewards
                 or abs(statesVector[4]) > 30*np.pi/180 #body points upwards
                 or not isInObservationSpace )#werte definieren; floor is at -1

        #timeout
        time = self.mbs.systemData.GetTime()
        if time >= self.episodeMaxLen*self.stepUpdateTime:
            done = True

        #timeout if it does not walk
        distance_to_target = np.linalg.norm(np.array(self.state[:2]) - np.array(self.target_position[:2]))  # Zielzustand
        if ((time > 1.5 and self.maxDistTarget-distance_to_target < 0.5*time)
            ):
            done = True
            # print('timeout, dist=',round(self.maxDistTarget-distance_to_target,2),
            #       ', rew=',round(self.getReward(),2), ', t=',round(time,1))

        #check if solution diverged
        if np.isnan(self.state).any() or np.isinf(self.state).any() or done:
            self.state = np.zeros(2*self.nTotalLinks+self.nLegStates, dtype=dtypeNumpy)
            #print('state is nan')
            done = True

        
        # if done: 2
        #     print('**DONE**')
        #     print('state=',np.round(self.state,3))

        return done
    
    def State2InitialValues(self):
        # #+++++++++++++++++++++++++++++++++++++++++++++
        # #to be randomized aufter Strukture is running !!!        
        # return np.zeros(18*2)  # 6 FHG + 12 Gelenkwinkel

        #+++++++++++++++++++++++++++++++++++++++++++++
        initialValues = self.state[0:self.nTotalLinks]
        initialValues_t = self.state[self.nTotalLinks:2*self.nTotalLinks]
               
        #set initial values into mbs immediately
        self.mbs.systemData.SetODE2Coordinates(initialValues, exu.ConfigurationType.Initial)
        self.mbs.systemData.SetODE2Coordinates_t(initialValues_t, exu.ConfigurationType.Initial)


        return [initialValues,initialValues_t]

    def reset(self, *, seed= None,return_info = False,options = None):
        RV = super().reset(seed=seed, return_info=return_info, options=options)
                
        #this function is only called at reset(); so, we can use it to reset the mean reward:
        self.rewardCnt = 0
        self.rewardMean = 0
        
        #reset incremental set values
        self.previousSetCoordinates = np.zeros(self.nActuatedJoints)
        self.previousSetCoordinates += np.array(self.legsInit*4)
        self.state[:self.nActuatedJoints] += self.previousSetCoordinates
        
        self.initialPosition = self.state[0:2]
        self.maxDistTarget = np.linalg.norm(self.initialPosition - self.target_position[:2])

        self.state = np.array(self.state, dtype=dtypeNumpy)
        #print('RESET')
        
        return RV
        
    def step(self, action):
        err_msg = f"{action!r} ({type(action)}) invalid"
        assert self.action_space.contains(action), err_msg
        assert self.state is not None, "Call reset before using step method."
        #print('step')
        
        self.MapAction2MBS(action)
        self.IntegrateStep() #this is NEEDED for integration!
        
        done = self.Output2StateAndDone()
        # print('state:', np.round(self.state[:6],3), 'done: ', done)
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

        #compute mean values per episode:
        if self.rewardCnt != 0: #per epsiode
            info['rewardMean'] = self.rewardMean / self.rewardCnt
            info['rewardSum'] = self.rewardMean
            info['episodeLen'] = self.rewardCnt

        # print('reward=', reward, ', done=',done)

        terminated, truncated = done, False # since stable-baselines3 > 1.8.0 implementations terminated and truncated 
        return np.array(self.state, dtype=dtypeNumpy), reward, terminated, truncated, info
        

    def getReward(self):
        # Berechne eine Belohnung basierend auf dem Zustand und der Aktion
        reward = 0
        pos = self.state[:2]
        vel = self.state[self.nTotalLinks:self.nTotalLinks+2]
        vecTarget = np.array(self.target_position[:2]) - np.array(pos)
        distance_to_target = np.linalg.norm(vecTarget)  # Zielzustand
        distReward = 1-(distance_to_target/(self.maxDistTarget+0.5) ) # rel. Fehler 0 - 1

        #reward moving legs
        # for i in range(4):
        #     reward += np.linalg.norm(0.5*vel[self.nTotalLinks+6+3*i+1:self.nTotalLinks+6+3*i+3])

        
        l = np.linalg.norm(vecTarget)
        if l != 0: vecTarget /= l

        #add reward for correct velocity
        velTarget = np.dot(vel,vecTarget)
        addBodyReward = False
        breakDistance = 1  #starts to break
        stopDistance = 0.1 #stops
        maxVel = 1.5 #1 m/s
        if distance_to_target > stopDistance:
            if distance_to_target < breakDistance:
                maxVel *= distance_to_target/breakDistance+0.1
                addBodyReward = True
            reward += 0.5*velTarget
            if velTarget > maxVel: #avoid robot running too fast
                reward -= 0.25*(velTarget-maxVel)
        else:
            maxVel *= distance_to_target/stopDistance+0.05
            reward += (1-distance_to_target/stopDistance
                       + 0.5*(velTarget * distance_to_target/stopDistance)
                       )
            if velTarget > maxVel: #avoid robot running too fast
                reward -= 0.25*(velTarget-maxVel)
            addBodyReward = True

        if addBodyReward: #don't pitch and roll to much; stay up; final orientation
            angleOffset = 5 * np.pi/180 #this angle does not lead to decrease of reward
            angleXOff = max(abs(self.state[3])-angleOffset,0)
            angleYOff = max(abs(self.state[4])-angleOffset,0)
            angleZOff = abs(self.state[5])
            zOff = max(abs(self.state[2])-0.1,0)
            reward -= 0.5*(angleXOff + angleYOff + angleZOff + zOff) 


        #reward = min(max(reward, 0),1) #not needed, maybe less efficient
        
        # self.cntCalls += 1
        # if self.cntCalls %10 == 0:
        #     print('reward=',round(reward,2), ',pos=',np.round(pos,3), ',vel=',
        #           np.round(velTarget,3))
        
        return reward


#%%+++++++++++++++++++++++++++++++++++++++++++++
if __name__ == '__main__': #this is only executed when file is direct called in Python
    import time
        
    #%%++++++++++++++++++++++++++++++++++++++++++++++++++
    #use some learning algorithm:
    #pip install stable_baselines3
    from stable_baselines3 import A2C, SAC    
    
    modelName = 'Spot_RL'
    
    tensorboard_log = None #no logging
    rewardCallback = None
    verbose = 0 #turn off to just view in tensorboard
    log_interval = 10
    
    import torch #stable-baselines3 is based on pytorch
    torch.set_num_threads(1) #1 seems to be ideal
    n_cores= os.cpu_count() #n_cores should be number of threads!
    n_cores = 20
    doParallel = True

    if hasTensorboard: #only us if tensorboard is available
        tensorboard_log = "solution/tensorboard_log/" #dir
        nEnvs = n_cores if doParallel else 1
        rewardCallback = RewardLoggingCallback(verbose=1, log_dir=tensorboard_log, 
                                               bestModelName=modelName+'_best',
                                               nEnvs=nEnvs)
    else:
        verbose = 1 #turn on without tensorboard
    
    # here the model is loaded (either for vectorized or scalar environment´using SAC or A2C).     
    def getModel(myEnv, modelType='A2C'):
        
        if modelType == 'A2C':
            model = A2C('MlpPolicy', 
                        myEnv, 
                        device='cpu',                                        
                        tensorboard_log=tensorboard_log,
                        verbose=verbose)
            torch.set_num_threads(4) #seems to be better than 1
        elif modelType == 'SAC':
            model = SAC('MlpPolicy',
                  env=myEnv,
                  learning_rate=3e-4, #3e-4 works
                  device='cpu', #usually cpu is faster for this size of networks
                  batch_size=128, #default:256
                  #target_entropy=0.1,  #default: 'auto'
                  ent_coef = 'auto_500.',     #default: 'auto'
                  tensorboard_log=tensorboard_log,
                  verbose=verbose)
        elif modelType == 'PPO':
            model = PPO('MlpPolicy',
                  env=myEnv,
                  learning_rate=3e-4, #3e-4 works
                  device='cpu', #usually cpu is faster for this size of networks
                  batch_size=64,
                  tensorboard_log=tensorboard_log,
                  verbose=verbose)
        else:
            raise ValueError('getModel: no available model: '+modelType)
            
        return model

    if True: #train
        
        if False:
            env = SpotEnv()
            env.TestModel(numberOfSteps=2000, seed=42, sleepTime=0.0, useRenderer=True)
            sys.exit()
    
    
        # modelType = 'PPO'    #40 steps/second
        # modelType = 'A2C'    #40 steps/second
        modelType = 'SAC'  #30 steps/second / 28 threads:425 steps/second
        if modelType == 'PPO': log_interval = 1
        if modelType == 'A2C': log_interval = 100
        
        if not doParallel:
            env = SpotEnv()
            showDuringLearning = True
            #torch.set_num_threads(1) #seems to be best for serial
    
            if showDuringLearning:
                exu.StartRenderer() #do this to see what is done during learning
            model = getModel(env,modelType=modelType)
            print('start learning of agent with algorithm: '+modelType)
        
            ts = -time.time()
            model.learn(total_timesteps=int(1e5), 
                        #progress_bar=True, #requires tqdm and rich package; set True to only see progress and set log_interval very high
                        log_interval=log_interval, #logs per episode; influences local output and tensorboard
                        callback = rewardCallback,
                        )
        
            if showDuringLearning:
                exu.StopRenderer() #do this to see what is done during learning
        
        
        else: #parallel; faster #set verbose=0 in getModel()!
            
            print('Train in parallel, using',n_cores,'cores')
    
            from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
            vecEnv = SubprocVecEnv([SpotEnv for i in range(n_cores)])
            
        
            model = getModel(vecEnv,modelType=modelType)
    
            ts = -time.time()
    
            model.learn(total_timesteps=int(20e6), #A2C starts working above 250k; SAC similar
                        progress_bar=True, #requires tqdm and rich package; set True to only see progress and set log_interval very high (100_000_000)
                        log_interval=log_interval, #logs per episode; influences local output and tensorboard
                        callback = rewardCallback,
                        )
        
            #save learned model
            model.save("solution/" + modelName)
        
        
        
        print('*** learning time total =',ts+time.time(),'***')
        #save learned model
        model.save("solution/" + modelName)
    
    if True: #set True to see what has been learned
        #%%++++++++++++++++++++++++++++++++++++++++++++++++++
        #only load and test
        if True: 
            model = SAC.load("solution/tensorboard_log/Spot_RL_best")
            #model = SAC.load("solution/tensorboard_log/Spot_RL_best_temp")
            #model = SAC.load("solution/" + modelName)
        
        addShoulderContact = True
        env = SpotEnv() #larger threshold for testing
        solutionFile='solution/learningCoordinates.txt'
        env.testModel = True
        env.TestModel(numberOfSteps=250, model=model, solutionFileName=solutionFile, 
                      stopIfDone=False, useRenderer=True, sleepTime=0.02) #just compute solution file
    
        #++++++++++++++++++++++++++++++++++++++++++++++
        #visualize (and make animations) in exudyn:
        #SC = exu.SystemContainer() #don't do that, it will be different from mbs!!!
        
        if False:
            pass
            #%%
            from exudyn.interactive import SolutionViewer
            env.SC.visualizationSettings.general.autoFitScene = False
            solution = LoadSolutionFile(solutionFile)
            
            SolutionViewer(env.mbs, solution, timeout=0.005, rowIncrement=2) #loads solution file via name stored in mbs


