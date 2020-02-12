#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test suite for Rotation Vector Update formulas
#           Refs.:  Holzinger S., Gerstmayr J.: Time integration of rigid bodies modelled with three rotation parameters, Multibody System Dynamics (2020)
#
# Author:   Stefan Holzinger
# Date:     2020-06-02
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import numpy as np
from timeIntegrationOfRotationVectorFormulas import *
from numpy import linalg as LA
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
plt.close("all") 

globalErrorBound = 1e-14

###############################################################################
#
# Test compose rotation vector
def TestComposeRotationVector():
    #
    #pi = 3.141592653589793
    pi = np.pi
    
    #+++++++++++++++++++++++++++++++++ TEST 1 +++++++++++++++++++++++++++++++++
    v0    = np.array([1.0, 2.0, 3.0])
    Omega = np.array([0.1, 0.2, 0.3])
    
    # matlab results:
    vMatlab   = np.array([1.100000000000000, 2.200000000000000, 3.300000000000000])
    nMAtlab   = np.array([0.267261241912424, 0.534522483824849, 0.801783725737273])
    phiMatlab = 4.115823125451335
    
    # python results
    vPython   = ComposeRotationVectors(v0,Omega) 
    nPython   = ComputeRotationAxisFromRotationVector(vPython)
    phiPython = LA.norm(vPython)
    
    # compute deviation
    deviationOfRotatioVectorComponents = vMatlab - vPython
    deviationOfRotatioAxisComponents   = nMAtlab - nPython
    deviationOfRotatioAngle            = phiMatlab - phiPython
    
    # print deviations
    print(' ')
    print('RESULTS: TEST 1')
    print('  vMatlab - vPython = ' + str(deviationOfRotatioVectorComponents))
    print('  nMAtlab - nPython = ' + str(deviationOfRotatioAxisComponents))
    print('  phiMatlab - phiPython = ' + str(deviationOfRotatioAngle))
    
    # check if tests were successfull
    sumOfDeviations = LA.norm(deviationOfRotatioVectorComponents) + LA.norm(deviationOfRotatioAxisComponents) + LA.norm(deviationOfRotatioAngle)
    if sumOfDeviations < globalErrorBound:
        print('  TEST 1 was SUCCESSFULL')
    else:
        print('  TEST 1 FAILED!')
    print(' ')
    
    
    
    #+++++++++++++++++++++++++++++++++ TEST 2 +++++++++++++++++++++++++++++++++
    v0    = np.array([1.0, 1.0, 1.0])
    n0    = ComputeRotationAxisFromRotationVector(v0)
    v0    = pi*n0
    Omega = v0
    
    # matlab results:
    vMatlab   = np.array([0.0, 0.0, 0.0])
    nMAtlab   = np.array([0.0, 0.0, 0.0])
    phiMatlab = 0.0
    
    # python results
    vPython   = ComposeRotationVectors(v0,Omega) 
    nPython   = ComputeRotationAxisFromRotationVector(vPython)
    phiPython = LA.norm(vPython)
    
    # compute deviation
    deviationOfRotatioVectorComponents = vMatlab - vPython
    deviationOfRotatioAxisComponents   = nMAtlab - nPython
    deviationOfRotatioAngle            = phiMatlab - phiPython
    
    # print deviations
    print(' ')
    print('RESULTS: TEST 2')
    print('  vMatlab - vPython = ' + str(deviationOfRotatioVectorComponents))
    print('  nMAtlab - nPython = ' + str(deviationOfRotatioAxisComponents))
    print('  phiMatlab - phiPython = ' + str(deviationOfRotatioAngle))
    
    # check if tests were successfull
    sumOfDeviations = LA.norm(deviationOfRotatioVectorComponents) + LA.norm(deviationOfRotatioAxisComponents) + LA.norm(deviationOfRotatioAngle)
    if sumOfDeviations < globalErrorBound:
        print('  TEST 2 was SUCCESSFULL')
    else:
        print('  TEST 2 FAILED!')
    print(' ')
    
    
    
    #+++++++++++++++++++++++++++++++++ TEST 3 +++++++++++++++++++++++++++++++++
    v0    = np.array([0.814723686393179, 0.905791937075619, 0.126986816293506])
    Omega = np.array([0.913375856139019, 0.632359246225410, 0.097540404999410])
    
    # matlab results:
    vMatlab   = np.array([1.723793103824437, 1.555433894116284, 0.047932757694710])
    nMAtlab   = np.array([0.742274430881640, 0.669778064413679, 0.020640099069599])
    phiMatlab = 2.322312384891114
    
    # python results
    vPython   = ComposeRotationVectors(v0,Omega) 
    nPython   = ComputeRotationAxisFromRotationVector(vPython)
    phiPython = LA.norm(vPython)
    
    # compute deviation
    deviationOfRotatioVectorComponents = vMatlab - vPython
    deviationOfRotatioAxisComponents   = nMAtlab - nPython
    deviationOfRotatioAngle            = phiMatlab - phiPython
    
    # print deviations
    print(' ')
    print('RESULTS: TEST 3')
    print('  vMatlab - vPython = ' + str(deviationOfRotatioVectorComponents))
    print('  nMAtlab - nPython = ' + str(deviationOfRotatioAxisComponents))
    print('  phiMatlab - phiPython = ' + str(deviationOfRotatioAngle))
    
    # check if tests were successfull
    sumOfDeviations = LA.norm(deviationOfRotatioVectorComponents) + LA.norm(deviationOfRotatioAxisComponents) + LA.norm(deviationOfRotatioAngle)
    if sumOfDeviations < globalErrorBound:
        print('  TEST 3 was SUCCESSFULL')
    else:
        print('  TEST 3 FAILED!')
    print(' ')
    
    
    
    #+++++++++++++++++++++++++++++++++ TEST 4 +++++++++++++++++++++++++++++++++
    v0    = np.array([0.0, 0.0, 0.0])
    Omega = np.array([0.278498218867048, 0.546881519204984, 0.957506835434298])
    
    # matlab results:
    vMatlab   = np.array([0.278498218867048, 0.546881519204984, 0.957506835434298])
    nMAtlab   = np.array([0.244875830334816, 0.480857890778889, 0.841909446789566])
    phiMatlab = 1.137303826540416
    
    # python results
    vPython   = ComposeRotationVectors(v0,Omega) 
    nPython   = ComputeRotationAxisFromRotationVector(vPython)
    phiPython = LA.norm(vPython)
    
    # compute deviation
    deviationOfRotatioVectorComponents = vMatlab - vPython
    deviationOfRotatioAxisComponents   = nMAtlab - nPython
    deviationOfRotatioAngle            = phiMatlab - phiPython
    
    # print deviations
    print(' ')
    print('RESULTS: TEST 4')
    print('  vMatlab - vPython = ' + str(deviationOfRotatioVectorComponents))
    print('  nMAtlab - nPython = ' + str(deviationOfRotatioAxisComponents))
    print('  phiMatlab - phiPython = ' + str(deviationOfRotatioAngle))
    
    # check if tests were successfull
    sumOfDeviations = LA.norm(deviationOfRotatioVectorComponents) + LA.norm(deviationOfRotatioAxisComponents) + LA.norm(deviationOfRotatioAngle)
    if sumOfDeviations < globalErrorBound:
        print('  TEST 4 was SUCCESSFULL')
    else:
        print('  TEST 4 FAILED!')
    print(' ')
    
    
    
    #+++++++++++++++++++++++++++++++++ TEST 5 +++++++++++++++++++++++++++++++++
    v0    = np.array([0.957166948242946, 0.485375648722841, 0.800280468888800])
    Omega = np.array([0.0, 0.0, 0.0])
    
    # matlab results:
    vMatlab   = np.array([0.957166948242946, 0.485375648722841, 0.800280468888800])
    nMAtlab   = np.array([0.714979548359578, 0.362563357150639, 0.597789308602281])
    phiMatlab = 1.338733325224524
    
    # python results
    vPython   = ComposeRotationVectors(v0,Omega) 
    nPython   = ComputeRotationAxisFromRotationVector(vPython)
    phiPython = LA.norm(vPython)
    
    # compute deviation
    deviationOfRotationVectorComponents = vMatlab - vPython
    deviationOfRotationAxisComponents   = nMAtlab - nPython
    deviationOfRotationAngle            = phiMatlab - phiPython
    
    # print deviations
    print(' ')
    print('RESULTS: TEST 5')
    print('  vMatlab - vPython = ' + str(deviationOfRotationVectorComponents))
    print('  nMatlab - nPython = ' + str(deviationOfRotationAxisComponents))
    print('  phiMatlab - phiPython = ' + str(deviationOfRotationAngle))
    
    # check if tests were successfull
    sumOfDeviations = LA.norm(deviationOfRotationVectorComponents) + LA.norm(deviationOfRotationAxisComponents) + LA.norm(deviationOfRotationAngle)
    if sumOfDeviations < globalErrorBound:
        print('  TEST 5 was SUCCESSFULL')
    else:
        print('  TEST 5 FAILED!')
    print(' ')





###############################################################################
# Euler's EOM of a rigid body in a local frame
def EulersEOM(v, w):
    InertiaTensor = np.diag([5.2988, 1.1775, 4.3568])
    Fode1 = -np.dot(Skew(w), np.dot(InertiaTensor,w))
    w_t = np.linalg.solve(InertiaTensor,Fode1)
    return w_t
#
# test compute step
#
def TestComputeStep():
    v0 = np.array([1, 2, 3])
    w0 = 2*v0
    
    #+++++++++++++++++++++++++++++++++++ Test 1 +++++++++++++++++++++++++++++++
    h = 1.0e-3
    stepResults = ComputeStepWithRK4(EulersEOM, v0, w0, h)
    
    # matlab result
    vMatlab = np.array([1.002014565846977, 2.003988840119048, 3.006000763507420])
    wMatlab = np.array([1.985608117600764, 3.990428541317109, 6.007531319827856])
    
    # compute single step with python
    vPython = stepResults[0]
    wPython = stepResults[1]
    
    # compute deviation
    deviationOfRotatioVectorComponents = vMatlab - vPython
    deviationOfAngularVelocityComponents = wMatlab - wPython
    
    # print deviations
    print(' ')
    print('RESULTS: TEST 6 (RK4)')
    print('  vMatlab - vPython = ' + str(deviationOfRotatioVectorComponents))
    print('  wMAtlab - wPython = ' + str(deviationOfAngularVelocityComponents))
    
    # check if tests were successfull
    sumOfDeviations = LA.norm(deviationOfRotatioVectorComponents) + LA.norm(deviationOfAngularVelocityComponents)
    if sumOfDeviations < globalErrorBound:
        print('  TEST 6 was SUCCESSFULL')
    else:
        print('  TEST 6 FAILED!')
    print(' ')
    
    
    #+++++++++++++++++++++++++++++++++++ Test 1 +++++++++++++++++++++++++++++++
    h = 1.0e-4
    stepResults = ComputeStepWithRK1(EulersEOM, v0, w0, h)
    
    # matlab result
    vMatlab = np.array([1.000200290826077, 2.000399777080365, 3.000600015346463])
    wMatlab = np.array([1.998559990941345, 3.999040000000000, 6.000756757253030])
    
    # compute single step with python
    vPython = stepResults[0]
    wPython = stepResults[1]
    
    # compute deviation
    deviationOfRotatioVectorComponents = vMatlab - vPython
    deviationOfAngularVelocityComponents = wMatlab - wPython
    
    # print deviations
    print(' ')
    print('RESULTS: TEST 7 (RK1)')
    print('  vMatlab - vPython = ' + str(deviationOfRotatioVectorComponents))
    print('  wMAtlab - wPython = ' + str(deviationOfAngularVelocityComponents))
   
    # check if tests were successfull
    sumOfDeviations = LA.norm(deviationOfRotatioVectorComponents) + LA.norm(deviationOfAngularVelocityComponents)
    if sumOfDeviations < globalErrorBound:
        print('  TEST 7 was SUCCESSFULL')
    else:
        print('  TEST 7 FAILED!')
    print(' ')
    

###############################################################################
#
#
#
def TestTSO3Inv():
    # arbitrary test angular velocity
    wTest = np.array([1.0, 1.0, 1.0])
    
    #+++++++++++++++++++++++++++++++++ TEST 1 +++++++++++++++++++++++++++++++++
    Omega = np.array([0.0, 0.0, 0.0])
    
    # matlab results:
    Omega_t_Matlab = np.array([1.0, 1.0, 1.0])
    
    # python results
    Omega_t_Python = np.dot(TSO3Inv(Omega), wTest)
    
    # compute deviation
    deviationOfOmega_t = Omega_t_Matlab - Omega_t_Python
    
    # print deviations
    print(' ')
    print('RESULTS: TEST 8')
    print('  Omega_t_Matlab = ' + str(Omega_t_Matlab))
    print('  Omega_t_Python = ' + str(Omega_t_Python))
    print('  Omega_t_Matlab - Omega_t_Python = ' + str(deviationOfOmega_t))
    
    # check if tests were successfull
    sumOfDeviations = LA.norm(deviationOfOmega_t)
    if sumOfDeviations < globalErrorBound:
        print('  TEST 8 was SUCCESSFULL')
    else:
        print('  TEST 8 FAILED!')
    print(' ')
    
    
    #+++++++++++++++++++++++++++++++++ TEST 2 +++++++++++++++++++++++++++++++++
    Omega = np.array([1.0, 2.0, 3.0])
    
    # matlab results:
    Omega_t_Matlab = np.array([-0.402160845072268, 1.774459788731933, 0.951080422536134])
    
    # python results)
    Omega_t_Python = np.dot(TSO3Inv(Omega), wTest)
    
    # compute deviation
    deviationOfOmega_t = Omega_t_Matlab - Omega_t_Python
    
    # print deviations
    print(' ')
    print('RESULTS: TEST 9')
    print('  Omega_t_Matlab = ' + str(Omega_t_Matlab))
    print('  Omega_t_Python = ' + str(Omega_t_Python))
    print('  Omega_t_Matlab - Omega_t_Python = ' + str(deviationOfOmega_t))
    
    # check if tests were successfull
    sumOfDeviations = LA.norm(deviationOfOmega_t)
    if sumOfDeviations < globalErrorBound:
        print('  TEST 9 was SUCCESSFULL')
    else:
        print('  TEST 9 FAILED!')
    print(' ')
   
    
    #+++++++++++++++++++++++++++++++++ TEST 3 +++++++++++++++++++++++++++++++++
    Omega = np.array([0.141886338627215, 0.421761282626275, 0.915735525189067])
    
    # matlab results:
    Omega_t_Matlab = np.array([0.682902413227995, 1.351928852030877, 0.887043643491090])
    
    # python results)
    Omega_t_Python = np.dot(TSO3Inv(Omega), wTest)
    
    # compute deviation
    deviationOfOmega_t = Omega_t_Matlab - Omega_t_Python
    
    # print deviations
    print(' ')
    print('RESULTS: TEST 10')
    print('  Omega_t_Matlab = ' + str(Omega_t_Matlab))
    print('  Omega_t_Python = ' + str(Omega_t_Python))
    print('  Omega_t_Matlab - Omega_t_Python = ' + str(deviationOfOmega_t))
    
    # check if tests were successfull
    sumOfDeviations = LA.norm(deviationOfOmega_t)
    if sumOfDeviations < globalErrorBound:
        print('  TEST 10 was SUCCESSFULL')
    else:
        print('  TEST 10 FAILED!')
    print(' ')

    
###############################################################################
#
# run unit tests
TestComposeRotationVector()
TestComputeStep()
TestTSO3Inv()

   