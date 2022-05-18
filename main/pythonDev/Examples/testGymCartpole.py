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
#pip install gym
#pip instal pygame
import time
from math import sin, cos
from testGymCartpoleEnv import CartPoleEnv

if False: #test the model by just integrating in Exudyn and apply force

    env = CartPoleEnv()
    env.useRenderer = False #set this true to show visualization
    observation, info = env.reset(seed=42, return_info=True)
    ts = -time.time()

    for i in range(10000):
        force = 0.1*(cos(i/50))
        env.integrateStep(force)
        env.render()
        # time.sleep(0.01)

    print('time spent=',ts+time.time())
    env.close()


if False: #testing the model with some random input
    import gym
    env = CartPoleEnv()
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


#+++++++++++++++++++++++++++++++++++++++++++++++++
#reinforment learning algorithm
#alternatives see: https://www.gymlibrary.ml/environments/mujoco/
#pip install gym[spaces]
#pip install pyglet
#pip install stable-baselines3

if True: #do some reinforcement learning with exudyn model
    import gym
    
    from stable_baselines3 import A2C
    
    env = CartPoleEnv(2)
    env.useRenderer = False
    #env = gym.make('CartPole-v1')
    
    ts = -time.time()
    model = A2C('MlpPolicy', env, verbose=1)
    model.learn(total_timesteps=50000)
    print('time spent=',ts+time.time())
    
    env = CartPoleEnv(4)#test with larger threshold
    env.useRenderer = True
    obs = env.reset()
    for i in range(40*250):
        action, _state = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
          obs = env.reset()
    env.close()
      
      
      