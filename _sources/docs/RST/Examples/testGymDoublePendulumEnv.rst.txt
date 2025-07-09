
.. _examples-testgymdoublependulumenv:

***************************
testGymDoublePendulumEnv.py
***************************

You can view and download this file on Github: `testGymDoublePendulumEnv.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/testGymDoublePendulumEnv.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  This file serves as an input to testGymDoublePendulum.py
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-05-18
   # Update:   2023-05-20: derive from gym.Env to ensure compatibility with newer stable-baselines3
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   import math
   
   import math
   from typing import Optional, Union
   
   import numpy as np
   
   import gym
   from gym import logger, spaces, Env
   from gym.error import DependencyNotInstalled
   
   import stable_baselines3
   useOldGym = tuple(map(int, stable_baselines3.__version__.split('.'))) <= tuple(map(int, '1.8.0'.split('.')))
   
   class DoublePendulumEnv(Env):
       
       #metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}
       metadata = {"render_modes": ["human"], "render_fps": 50}
   
       def __init__(self, thresholdFactor = 1.):
       
           self.SC = exu.SystemContainer()
           self.mbs = self.SC.AddSystem()
           
           
           #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
           #+++++++++++++++++++++++++++++++++++++++++++++++++++++
           self.gravity = 9.81
           self.length = 1.
           self.masscart = 1.
           self.massarm = 0.1
           self.total_mass = self.massarm + self.masscart
           self.armInertia = self.length**2*0.5*self.massarm
           self.force_mag = 10.0
           self.stepUpdateTime = 0.02  # seconds between state updates
           
           # Angle at which to fail the episode
           self.theta_threshold_radians = thresholdFactor* 12 * 2 * math.pi / 360
           self.x_threshold = thresholdFactor*2.4
   
           high = np.array(
               [
                   self.x_threshold * 2,
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
           self.state = None
           self.rendererRunning=None
           self.useRenderer = False #turn this on if needed
           
           background = graphics.CheckerBoard(point= [0,0,0], normal= [0,0,1], size=4)
               
           oGround=self.mbs.AddObject(ObjectGround(referencePosition= [0,0,0],  #x-pos,y-pos,angle
                                              visualization=VObjectGround(graphicsData= [background])))
           nGround=self.mbs.AddNode(NodePointGround())
           
           gCart = graphics.Brick(size=[0.5*self.length, 0.1*self.length, 0.1*self.length], 
                                              color=graphics.color.dodgerblue)
           self.nCart = self.mbs.AddNode(Rigid2D(referenceCoordinates=[0,0,0]));
           oCart = self.mbs.AddObject(RigidBody2D(physicsMass=self.masscart, 
                                             physicsInertia=0.1*self.masscart, #not needed
                                             nodeNumber=self.nCart,
                                             visualization=VObjectRigidBody2D(graphicsData= [gCart])))
           mCartCOM = self.mbs.AddMarker(MarkerNodePosition(nodeNumber=self.nCart))
           
           gArm1 = graphics.Brick(size=[0.1*self.length, self.length, 0.1*self.length], color=graphics.color.red)
           self.nArm1 = self.mbs.AddNode(Rigid2D(referenceCoordinates=[0,0.5*self.length,0]));
           oArm1 = self.mbs.AddObject(RigidBody2D(physicsMass=self.massarm, 
                                             physicsInertia=self.armInertia, #not included in original paper
                                             nodeNumber=self.nArm1,
                                             visualization=VObjectRigidBody2D(graphicsData= [gArm1])))
           
           mArm1COM = self.mbs.AddMarker(MarkerNodePosition(nodeNumber=self.nArm1))
           mArm1JointA = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=oArm1, localPosition=[0,-0.5*self.length,0]))
           mArm1JointB = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=oArm1, localPosition=[0, 0.5*self.length,0]))
   
           gArm2 = graphics.Brick(size=[0.1*self.length, self.length, 0.1*self.length], color=graphics.color.red)
           self.nArm2 = self.mbs.AddNode(Rigid2D(referenceCoordinates=[0,1.5*self.length,0]));
           oArm2 = self.mbs.AddObject(RigidBody2D(physicsMass=self.massarm, 
                                             physicsInertia=self.armInertia, #not included in original paper
                                             nodeNumber=self.nArm2,
                                             visualization=VObjectRigidBody2D(graphicsData= [gArm2])))
           
           mArm2COM = self.mbs.AddMarker(MarkerNodePosition(nodeNumber=self.nArm2))
           mArm2Joint = self.mbs.AddMarker(MarkerBodyPosition(bodyNumber=oArm2, localPosition=[0,-0.5*self.length,0]))
           
           mCartCoordX = self.mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=self.nCart, coordinate=0))
           mCartCoordY = self.mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=self.nCart, coordinate=1))
           mGroundNode = self.mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
           
           #gravity
           self.mbs.AddLoad(Force(markerNumber=mCartCOM, loadVector=[0,-self.masscart*self.gravity,0]))
           self.mbs.AddLoad(Force(markerNumber=mArm1COM, loadVector=[0,-self.massarm*self.gravity,0]))
           self.mbs.AddLoad(Force(markerNumber=mArm2COM, loadVector=[0,-self.massarm*self.gravity,0]))
           
           #control force
           self.lControl = self.mbs.AddLoad(LoadCoordinate(markerNumber=mCartCoordX, load=1.))
           
           #joints and constraints:
           self.mbs.AddObject(RevoluteJoint2D(markerNumbers=[mCartCOM, mArm1JointA]))
           self.mbs.AddObject(RevoluteJoint2D(markerNumbers=[mArm1JointB, mArm2Joint]))
           self.mbs.AddObject(CoordinateConstraint(markerNumbers=[mCartCoordY, mGroundNode]))
           
           
           
           
           #%%++++++++++++++++++++++++
           self.mbs.Assemble() #computes initial vector
           
           self.simulationSettings = exu.SimulationSettings() #takes currently set values or default values
           
           
           self.simulationSettings.timeIntegration.numberOfSteps = 1
           self.simulationSettings.timeIntegration.endTime = 0 #will be overwritten in step
           self.simulationSettings.timeIntegration.verboseMode = 0
           self.simulationSettings.solutionSettings.writeSolutionToFile = False
           #self.simulationSettings.timeIntegration.simulateInRealtime = True
           
           self.simulationSettings.timeIntegration.newton.useModifiedNewton = True
           
           self.SC.visualizationSettings.general.drawWorldBasis=True
           self.SC.visualizationSettings.general.graphicsUpdateInterval = 0.01 #50Hz
           
           self.simulationSettings.solutionSettings.solutionInformation = "Open AI gym"
   
           self.dynamicSolver = exudyn.MainSolverImplicitSecondOrder()
           self.dynamicSolver.InitializeSolver(self.mbs, self.simulationSettings)
           self.dynamicSolver.SolveSteps(self.mbs, self.simulationSettings) #to initialize all data
   
       def integrateStep(self, force):
           #exudyn simulation part
           #index 2 solver
           self.mbs.SetLoadParameter(self.lControl, 'load', force)
   
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
           err_msg = f"{action!r} ({type(action)}) invalid"
           assert self.action_space.contains(action), err_msg
           assert self.state is not None, "Call reset before using step method."
           self.setState2InitialValues()
               
           force = self.force_mag if action == 1 else -self.force_mag
           
           #++++++++++++++++++++++++++++++++++++++++++++++++++
           #++++++++++++++++++++++++++++++++++++++++++++++++++
           self.integrateStep(force)
           #+++++++++++++++++++++++++
           #compute some output:
           cartPosX = self.mbs.GetNodeOutput(self.nCart, variableType=exu.OutputVariableType.Coordinates)[0]
           arm1Angle = self.mbs.GetNodeOutput(self.nArm1, variableType=exu.OutputVariableType.Coordinates)[2]
           arm2Angle = self.mbs.GetNodeOutput(self.nArm2, variableType=exu.OutputVariableType.Coordinates)[2]
           cartPosX_t = self.mbs.GetNodeOutput(self.nCart, variableType=exu.OutputVariableType.Coordinates_t)[0]
           arm1Angle_t = self.mbs.GetNodeOutput(self.nArm1, variableType=exu.OutputVariableType.Coordinates_t)[2]
           arm2Angle_t = self.mbs.GetNodeOutput(self.nArm2, variableType=exu.OutputVariableType.Coordinates_t)[2]
   
           #finally write updated state:
           self.state = (cartPosX, cartPosX_t, arm1Angle, arm1Angle_t, arm2Angle, arm2Angle_t)
           #++++++++++++++++++++++++++++++++++++++++++++++++++
           #++++++++++++++++++++++++++++++++++++++++++++++++++
   
           done = bool(
               cartPosX < -self.x_threshold
               or cartPosX > self.x_threshold
               or arm1Angle < -self.theta_threshold_radians
               or arm1Angle > self.theta_threshold_radians
               or arm2Angle < -self.theta_threshold_radians
               or arm2Angle > self.theta_threshold_radians
           )
   
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
   
           info = {}
           terminated, truncated = done, False # since stable-baselines3 > 1.8.0 implementations terminated and truncated 
           if useOldGym:
               return np.array(self.state, dtype=np.float32), reward, terminated, info
           else:
               return np.array(self.state, dtype=np.float32), reward, terminated, truncated, info
   
       def setState2InitialValues(self):
           #+++++++++++++++++++++++++++++++++++++++++++++
           #set specific initial state:
           (xCart, xCart_t, phiArm1, phiArm1_t, phiArm2, phiArm2_t) = self.state
           
           initialValues = np.zeros(9) #model has 3*3 = 9  redundant states
           initialValues_t = np.zeros(9)
           
           #build redundant cordinates from self.state
           initialValues[0] = xCart
           initialValues[3+0] = xCart - 0.5*self.length * sin(phiArm1)
           initialValues[3+1] = 0.5*self.length * (cos(phiArm1)-1)
           initialValues[3+2] = phiArm1
           initialValues[6+0] = xCart - 1.*self.length * sin(phiArm1) - 0.5*self.length * sin(phiArm2)
           initialValues[6+1] = self.length * cos(phiArm1) + 0.5*self.length * cos(phiArm2) - 1.5*self.length
           initialValues[6+2] = phiArm2
           
           initialValues_t[0] = xCart_t
           initialValues_t[3+0] = xCart_t - phiArm1_t*0.5*self.length * cos(phiArm1)
           initialValues_t[3+1] = -0.5*self.length * sin(phiArm1)  * phiArm1_t
           initialValues_t[3+2] = phiArm1_t
           initialValues_t[6+0] = xCart_t - phiArm1_t*1.*self.length * cos(phiArm1) - phiArm2_t*0.5*self.length * cos(phiArm2)
           initialValues_t[6+1] = -self.length * sin(phiArm1)  * phiArm1_t - 0.5*self.length * sin(phiArm2)  * phiArm2_t
           initialValues_t[6+2] = phiArm2_t
           
           self.mbs.systemData.SetODE2Coordinates(initialValues, exu.ConfigurationType.Initial)
           self.mbs.systemData.SetODE2Coordinates_t(initialValues_t, exu.ConfigurationType.Initial)
   
   
       def reset(
           self,
           *,
           seed: Optional[int] = None,
           return_info: bool = False,
           options: Optional[dict] = None,
       ):
           #super().reset(seed=seed)
           self.state = np.random.uniform(low=-0.05, high=0.05, size=(6,))
           self.steps_beyond_done = None
   
   
           #+++++++++++++++++++++++++++++++++++++++++++++
           #set state into initial values:
           self.setState2InitialValues()
   
           self.simulationSettings.timeIntegration.endTime = 0
           self.dynamicSolver.InitializeSolver(self.mbs, self.simulationSettings) #needed to update initial conditions
   
           #+++++++++++++++++++++++++++++++++++++++++++++
   
           if not return_info and useOldGym:
               return np.array(self.state, dtype=np.float32)
           else:
               return np.array(self.state, dtype=np.float32), {}
   
       def render(self, mode="human"):
           if self.rendererRunning==None and self.useRenderer:
               SC.renderer.Start()
               self.rendererRunning = True
   
       def close(self):
           self.dynamicSolver.FinalizeSolver(self.mbs, self.simulationSettings)
           if self.rendererRunning==True:
               # SC.renderer.DoIdleTasks()
               SC.renderer.Stop() #safely close rendering window!
   
   
   
   # #+++++++++++++++++++++++++++++++++++++++++++++
   # #reset:
   # self.mbs.systemData.SetODE2Coordinates(initialValues, exu.ConfigurationType.Initial)
   # self.mbs.systemData.SetODE2Coordinates_t(initialValues, exu.ConfigurationType.Initial)
   # self.mbs.systemData.SetODE2Coordinates_tt(initialValues, exu.ConfigurationType.Initial)
   
   


