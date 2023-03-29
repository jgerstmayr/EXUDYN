
.. _examples-testgymdoublependulum:

************************
testGymDoublePendulum.py
************************

You can view and download this file on Github: `testGymDoublePendulum.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/testGymDoublePendulum.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  This file shows integration with OpenAI gym by testing a double pendulum example
   #           Needs input file testGymDoublePendulumEnv.py which defines the model in the gym environment
   #
   # Author:   Johannes Gerstmayr
   # Date:     2022-05-18
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #pip install gym
   #pip instal pygame
   import time
   from math import sin, cos
   from testGymDoublePendulumEnv import DoublePendulumEnv
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++
   if False: #test the model by just integrating in Exudyn and apply force
   
       env = DoublePendulumEnv()
       env.useRenderer = True #set this true to show visualization
       observation, info = env.reset(seed=42, return_info=True)
   
       for i in range(10000):
           force = 0.1*(cos(i/50))
           env.integrateStep(force)
           env.render()
           time.sleep(0.01)
   
       env.close()
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++
   if False: #testing the model with some random input
       import gym
       env = DoublePendulumEnv(5)
       env.useRenderer = True #set this true to show visualization
       observation, info = env.reset(seed=42, return_info=True)
   
       ts = -time.time()
       for _ in range(1000):
           action = env.action_space.sample()
           observation, reward, done, info = env.step(action)
           env.render()
           if done:
               observation, info = env.reset(return_info=True)
       env.close()
       
       print('time spent=',ts+time.time())
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++
   #reinforment learning algorithm
   #pip install gym[spaces]
   #pip install pyglet
   
   if True: #do some reinforcement learning with exudyn model
       import gym
       
       from stable_baselines3 import A2C
   
       doLearning = True
       if doLearning:
           env = DoublePendulumEnv(1)
           env.useRenderer = False
           #env = gym.make('CartPole-v1')
           
           ts = -time.time()
           model = A2C('MlpPolicy', env, verbose=1)
           model.learn(total_timesteps=10000000)
           print('time spent=',ts+time.time())
   
       #%%++++++++++++++++++++++++ 
       env = DoublePendulumEnv(10) #allow larger threshold for testing
       env.useRenderer = True
       obs = env.reset()
       for i in range(5000):
           action, _state = model.predict(obs, deterministic=True)
           obs, reward, done, info = env.step(action)
           env.render()
           time.sleep(0.01) 
           if done:
             obs = env.reset()
           if env.mbs.GetRenderEngineStopFlag(): #stop if user press Q
               break
   
       env.close()
         
         
         

