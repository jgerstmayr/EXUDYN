
.. _examples-testgymcartpole:

******************
testGymCartpole.py
******************

You can view and download this file on Github: `testGymCartpole.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/testGymCartpole.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  This file shows integration with OpenAI gym by testing a cart-pole example
   #           Needs input file testGymCartpoleEnv.py which defines the model in the gym environment
   #           Works well with Python3.8!
   #
   # Author:   Johannes Gerstmayr, Grzegorz Orzechowski
   # Date:     2022-05-17
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++
   #conda create -n venvGym python=3.10 numpy matplotlib spyder-kernels=2.4 ipykernel -y
   #pip install gym[spaces]
   #pip install stable-baselines3==1.7.0
   #pip install exudyn
   
   import time
   from math import sin, cos
   from testGymCartpoleEnv import CartPoleEnv
   
   if True: #test the model by just integrating in Exudyn and apply force
   
       env = CartPoleEnv()
       env.useRenderer = False #set this true to show visualization
       observation, info = env.reset(seed=42, return_info=True)
       ts = -time.time()
   
       for i in range(10000):
           force = 0.1*(cos(i/50))
           env.integrateStep(force)
           # action = env.action_space.sample()
           # observation, reward, done, info = env.step(action)
           # if done:
               # observation, info = env.reset(return_info=True)
           # env.render()
           # time.sleep(0.01)
       ts = ts+time.time()
       print('measured max. step FPS:', int(10000/ts))
       env.close()
   
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++
   #reinforment learning algorithm
   
   if True: #do some reinforcement learning with exudyn model
       import gym
       
       env = CartPoleEnv(thresholdFactor=5,forceFactor=2)
   
       env.useRenderer = False
       
       from stable_baselines3 import A2C
       model = A2C('MlpPolicy', env,
                   device='cpu',  #usually cpu is faster for this size of networks
                   verbose=1)
       ts = -time.time()
       model.learn(total_timesteps=10000)
       print('time spent=',ts+time.time())
   
       model.save('solution/cartpoleLearn')
       
       #%%+++++++++++++++++++++++++++++++++++++++
       env = CartPoleEnv(10)#test with larger threshold
       env.useRenderer = True
       obs = env.reset()
       for i in range(100):
           action, _state = model.predict(obs, deterministic=True)
           obs, reward, done, info = env.step(action)
           env.render()
           if done:
             obs = env.reset()
           time.sleep(0.05) #to see results ...
           
       env.close()
   
   


