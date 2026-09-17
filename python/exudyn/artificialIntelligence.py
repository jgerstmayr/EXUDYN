#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python artificial intelligence library
#
# Details:  This library collects interfaces and functionality for artificial intelligence
#           This library is under construction (2022-05);
#           To make use of this libraries, you need to install openAI gym with 'pip install gym';
#           For standard machine learning algorithms, install e.g. stable\_baselines3 using 'pip install stable\_baselines3'
#
# Author:   Johannes Gerstmayr
# Date:     2022-05-21 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#to be able to use this library, please:
#pip install gym
##optional: pip install pygame

import exudyn as exu
from exudyn.advancedUtilities import IsReal, IsInteger
from typing import Optional #, Union

import numpy as np

# for older versions the reset and step function behaves slightly differently
# for step see https://gymnasium.farama.org/tutorials/gymnasium_basics/handling_time_limits/
import stable_baselines3
#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'useOldGym', 'OpenAIGymInterfaceEnv',
    ]

useOldGym = False
try:
    useOldGym = tuple(map(int, stable_baselines3.__version__.split('.')))[:3] <= tuple(map(int, '1.8.0'.split('.')))
except:
    pass #if it fails, useOldGym = False

if useOldGym:
    #old version #will be removed in future versions!
    from gym import logger, spaces, Env
else:
    from gymnasium import logger, spaces, Env # noqa # pylint: disable=unused-import


class OpenAIGymInterfaceEnv(Env):
    """interface class to set up Exudyn model which can be used as model in open AI gym;
    see specific class functions which contain 'OVERRIDE' to integrate your model;
    in general, set up a model with CreateMBS(), map state to initial values, initial values to state and action to mbs;
    """
    metadata = {'render_modes': ['human'], 'render_fps': 50}
    def __init__(self, **kwargs):
        """internal function to initialize model; store self.mbs and self.simulationSettings; special arguments **kwargs are passed to CreateMBS
        """
        #some general gym initialization
        self.state = None

        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #some settings, which may be changed after creation of environment:
        self.rendererRunning=None
        self.useRenderer = False #turn this on if needed
        
        #this is the high/low range of randomizer for state initialization
        #it may also be defined as list, having the length according to the number of states
        self.randomInitializationValue = 0.05 
        #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
        #mbs initialization        
        self.SC = exu.SystemContainer()
        self.mbs = self.SC.AddSystem()
        self.simulationSettings = exu.SimulationSettings() #takes currently set values or default values
        
        #add name to settings and show in animation:
        self.simulationSettings.solutionSettings.solutionInformation = str(self.__class__).split('.')[1].split("'")[0]
        
        self.stateSize = self.CreateMBS(self.SC, self.mbs, self.simulationSettings, **kwargs)

        self.SetupSpaces() #in future, there may be a more convenient way to do so!

        #THIS NEEDS TO BE CALLED AT THE END:
        self.PreInitializeSolver() #overwrite PreInitializeSolver() to use other solvers
        #++++++++++++++++++++++++++++
        #now system is ready to go!
        
    def CreateMBS(self, SC, mbs, simulationSettings, **kwargs):
        """OVERRIDE this function to create multibody system mbs and setup simulationSettings; call Assemble() at the end!
        you may also change SC.visualizationSettings() individually; kwargs may be used for special setup
        """
        return 0#override this class and return state size!

    def SetupSpaces(self):
        """OVERRIDE this function to set up self.action_space and self.observation_space
        """
        pass #override this class!


    def MapAction2MBS(self, action):
        """OVERRIDE this function to map the action given by learning algorithm to the multibody system, e.g. as a load parameter
        """
        pass #override this class!

    def Output2StateAndDone(self):
        """OVERRIDE this function to collect output of simulation and map to self.state tuple

        Returns:
            return bool done which contains information if system state is outside valid range
        """
        return False#override this class!

    
    def State2InitialValues(self):
        """OVERRIDE this function to maps the current state to mbs initial values

        Returns:
            return [initialValues, initialValues_t] where initialValues[_t] are ODE2 vectors of coordinates[_t] for the mbs
        """
        return [[0],[0]]#override this class and return the two vectors!


    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #HELPER functions
    def TestModel(self, numberOfSteps=500, seed=0, model = None, solutionFileName = None,
                  useRenderer=True, sleepTime=0.01, stopIfDone=False, showTimeSpent=True, **kwargs):
        """test model by running in simulation environment having several options

        Args:
            numberOfSteps: number of steps to test MBS and model (with or without learned model); with renderer, press 'Q' in render window to stop simulation
            seed: seed value for reset function; this value initializes the randomizer; use e.g. time to obtain non-reproducible results
            model: either None to just test the MBS model without learned model, or containing a learned model, e.g., with A2C; use A2C.save(...) and A2C.load(...) for storing and retrieving models
            solutionFileName: if given, the MBS internal states are written to the file with given name, which can be loaded with solution viewer and visualized; solution is written every period given in simulationSettings.solutionSettings.solutionWritePeriod
            useRenderer: if set True, the internal renderer is used and model updates are shown in visualization of Exudyn
            return_info: internal value in reset function
            sleepTime: sleep time between time steps to obtain certain frame rate for visualization
            stopIfDone: if set to True, the simulation will reset as soon as the defined observation limits are reached and done is set True
            showTimeSpent: if True, the total time spent is measured; this helps to check the performance of the model (e.g. how many steps can be computed per second)
        """
        import time

        writeToFile = solutionFileName is not None
        self.simulationSettings.solutionSettings.writeSolutionToFile = writeToFile
        if writeToFile:
            self.simulationSettings.solutionSettings.coordinatesSolutionFileName = solutionFileName

        storeRenderer = self.useRenderer 
        self.useRenderer = useRenderer #set this true to show visualization
        self.flagNan = False
        observation, info = self.reset(seed=seed, return_info=True)
    
        ts = -time.time()
        for _ in range(numberOfSteps):
            if model is not None: #use model to predict action (e.g., controller)
                action, _state = model.predict(observation, deterministic=True)
            else:
                action = self.action_space.sample()

            if np.isnan(self.state).any(): 
                self.flagNan = True
                break # 
            observation, reward, done, info = self.step(action)[:4] #accomodate for old and new version
            self.render()
            if self.mbs.GetRenderEngineStopFlag(): #user presses quit
                break
            if stopIfDone and done:
                observation, info = self.reset(return_info=True)
            if useRenderer and sleepTime!=0:
                time.sleep(sleepTime)        
        if showTimeSpent:
            exu.Print('time spent=',ts+time.time())

        self.close()
        self.useRenderer = storeRenderer #restore

    def SetSolver(self, solverType):
        """use solverType = exudyn.DynamicSolverType.[...] to define solver (choose between implicit and explicit solvers!)
        """
        self.simulationSettings == solverType
        if solverType==exu.DynamicSolverType.TrapezoidalIndex2 or solverType==exu.DynamicSolverType.GeneralizedAlpha:
            useIndex2 = False
            if solverType == exu.DynamicSolverType.TrapezoidalIndex2:
                useIndex2 = True

            #manually override settings for integrator
            self.simulationSettings.timeIntegration.generalizedAlpha.useNewmark = useIndex2
            self.simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = useIndex2
        
            self.dynamicSolver = exu.MainSolverImplicitSecondOrder()
            self.dynamicSolver.InitializeSolver(self.mbs, self.simulationSettings)
            self.dynamicSolver.SolveSteps(self.mbs, self.simulationSettings) #to initialize all data
        else:
            #explicit integration:
            self.simulationSettings.timeIntegration.explicitIntegration.dynamicSolverType = solverType
            self.dynamicSolver = exu.MainSolverExplicit()
            self.dynamicSolver.InitializeSolver(self.mbs, self.simulationSettings)
            self.dynamicSolver.SolveSteps(self.mbs, self.simulationSettings) #to initialize all data


    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #INTERNAL FUNCTIONS!
    def PreInitializeSolver(self):
        """internal function which initializes dynamic solver; adapt in special cases; this function has some overhead and should not be called during reset() or step()
        """
        self.SetSolver(solverType=exu.DynamicSolverType.GeneralizedAlpha)

        # #in derived class, you can also call explicit solver, e.g. using:
        # self.SetSolver(solverType=exu.DynamicSolverType.ExplicitEuler)

    def IntegrateStep(self):
        """internal function which is called to solve for one step
        """
        #exudyn simulation part
        #index 2 solver (may have some drift, but can be restarted easily)

        #progress integration time
        currentTime = self.simulationSettings.timeIntegration.endTime
        self.simulationSettings.timeIntegration.startTime = currentTime
        self.simulationSettings.timeIntegration.endTime = currentTime+self.stepUpdateTime

        self.dynamicSolver.InitializeSolverInitialConditions(self.mbs, self.simulationSettings)
        self.dynamicSolver.SolveSteps(self.mbs, self.simulationSettings)
        currentState = self.mbs.systemData.GetSystemState() #get current values
        self.mbs.systemData.SetSystemState(systemStateList=currentState, 
                                        configuration = exu.ConfigurationType.Initial)
        self.mbs.systemData.SetODE2Coordinates_tt(coordinates = self.mbs.systemData.GetODE2Coordinates_tt(), 
                                                configuration = exu.ConfigurationType.Initial)

    def step(self, action):
        """openAI gym interface function which is called to compute one step
        """
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

        #++++++++++++++++++++++++++++++++++++++++++++++++++
        #compute reward and done
        if not done:
            reward = 1.0
        elif self.steps_beyond_done is None:
            # Arm1 just fell!
            self.steps_beyond_done = 0
            reward = 1.0
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


        if useOldGym: #will be removed in future versions!
            return np.array(self.state, dtype=np.float32), reward, done, {}
        else:
            terminated, truncated = done, False # in implementations terminated and truncated should be distinguished
            return np.array(self.state, dtype=np.float32), reward, terminated, truncated, {}

 

    def reset(
        self,
        *,
        seed: Optional[int] = None,
        return_info: bool = False,
        options: Optional[dict] = None,
    ):
        """openAI gym function which resets the system
        """
        #super().reset(seed=seed)
        randSize = (self.stateSize)
        #randomInitializationValue could also be a vector!
        if not(IsReal(self.randomInitializationValue)) and not(IsInteger(self.randomInitializationValue)): 
            # if the randomInitializationValue is not a float/int/np.floating it should be a 
            # list or np.array to pass different initialization values for different states (thereby the list must have )
            randSize = None
            if type(self.randomInitializationValue) == list:  
                # cast to array to obtain -self.randomInitializationValue
                self.randomInitializationValue = np.array(self.randomInitializationValue)
        self.state = np.random.uniform(low=-self.randomInitializationValue, 
                                   high=self.randomInitializationValue, size=randSize)
        self.steps_beyond_done = None


        #+++++++++++++++++++++++++++++++++++++++++++++
        #set state into initial values:
        self.State2InitialValues()

        self.simulationSettings.timeIntegration.endTime = 0
        self.dynamicSolver.InitializeSolver(self.mbs, self.simulationSettings) #needed to update initial conditions

        #+++++++++++++++++++++++++++++++++++++++++++++


        if not return_info and useOldGym:
            #will be removed in future!
            return np.array(self.state, dtype=np.float32)
        else:
            return np.array(self.state, dtype=np.float32), {}
    
     

    def render(self, mode="human"):
        """openAI gym interface function to render the system
        """
        if self.rendererRunning is None and self.useRenderer:
            self.SC.renderer.Start()
            self.rendererRunning = True

    def close(self):
        """openAI gym interface function to close system after learning or simulation
        """
        self.dynamicSolver.FinalizeSolver(self.mbs, self.simulationSettings)
        if self.rendererRunning:
            # SC.renderer.DoIdleTasks()
            self.SC.renderer.Stop() #safely close rendering window!

