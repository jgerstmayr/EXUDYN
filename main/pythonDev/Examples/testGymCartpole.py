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
#pip install pygame
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
    
    env = CartPoleEnv(2)

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
    env = CartPoleEnv(20)#test with larger threshold
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


#tests with cuda and multiprocessing:      
if False: 
    import gym
    
    import torch #stable-baselines3 is based on pytorch
    torch.set_num_threads(1) #CPU usage will increase with larger number, but no speedup!
    #torch.cuda.is_available() #check if cuda is available - if not, need to install nvidia drivers and cuda for pytorch
    #torch.cuda.get_device_capability() #shows cuda capability
    
    env = CartPoleEnv(2)

    env.useRenderer = False
    
    from stable_baselines3 import A2C, PPO
    if True:
        model = A2C('MlpPolicy', env,
                    #device='cuda', #will be 3x slower for A2C !
                    device='cpu', 
                    verbose=1)
    else: #faster per step, but no speedup with multi-core or cuda!
        model = PPO('MlpPolicy', env,
                    #device='cuda', 
                    device='cpu', 
                    #batch_size=512*2, #changes performance, but no real speedup
                    #n_cpu_tf_sess=14,
                    verbose=1)
    ts = -time.time()
    model.learn(total_timesteps=10000)
    print('time spent=',ts+time.time())

    
    model.save('solution/cartpoleLearn')


    
    #%%+++++++++++++++++++++++++++++++++++++++
    env = CartPoleEnv(20)#test with larger threshold
    env.useRenderer = True
    obs = env.reset()
    for i in range(40*250):
        action, _state = model.predict(obs, deterministic=True)
        obs, reward, done, info = env.step(action)
        env.render()
        if done:
          obs = env.reset()
        time.sleep(0.01) #to see results ...
        
    env.close()
      
      