#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is the test suite which shall serve as a general driver to run:
#   - model unit tests
#   - test examples (e.g. from example folder)
#   - internal EXUDYN unit tests
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import sys, platform
workingReleasePath = 'C:\\DATA\\cpp\\EXUDYN_git\\main\\bin\\EXUDYN'
#workingReleasePath = '../bin/WorkingRelease'
if platform.architecture()[0] == '64bit':
    workingReleasePath += '64bitsPython'
else:
    workingReleasePath += '32bitsPython'
if sys.version_info.major == 3 and sys.version_info.minor == 7:
    workingReleasePath += '37'
elif sys.version_info.major == 3 and sys.version_info.minor == 6:
    workingReleasePath += '36'

if sys.version_info.major != 3 or sys.version_info.minor < 6 or sys.version_info.minor > 7:
    raise ImportError("EXUDYN only supports python 3.6 or python 3.7")

#only if not installed:
#sys.path.append(workingReleasePath) #for exudyn, itemInterface and from exudyn.utilities import *

#sys.path.append('TestModels')            #for modelUnitTest as this example may be used also as a unit test

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include right exudyn module now:
import numpy as np
import exudyn as exu
from modelUnitTests import RunAllModelUnitTests, TestInterface, ExudynTestStructure, exudynTestGlobals
import time

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#parse command line arguments:
# -quiet
writeToConsole = True  #do not output to console / shell
#copyLog = False         #copy log to final TestSuiteLogs
# if sys.version_info.major == 3 and sys.version_info.minor == 7:
#     copyLog = True #for P3.7 tests always copy log to WorkingRelease
if len(sys.argv) > 1:
    for i in range(len(sys.argv)-1):
        #print("arg", i+1, "=", sys.argv[i+1])
        if sys.argv[i+1] == '-quiet':
            writeToConsole = False
        # elif sys.argv[i+1] == '-copylog': #not needed any more
        #     copyLog = True
        else:
            print("ERROR in runTestSuite: unknown command line argument '"+sys.argv[i+1]+"'")

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#choose which tests to run:
runUnitTests = True
runTestExamples = True
runMiniExamples = True
printTestResults = False #print list, which can be imported for new reference values
if platform.architecture()[0] == '64bit':
    testTolerance = 5e-11 #larger tolerance, because reference values are computed with 32bit version (WHY?)
else:
    testTolerance = 3e-14

#++++++++++++++++++++++++++++++++++++++++++++++++
#additional options for old trapezoidal solver, NOT active any more!
# exudynTestGlobals.useCorrectedAccGenAlpha = True                        #use corrected reference values in test suite Examples
# exu.CorrectOldImplicitSolver(exudynTestGlobals.useCorrectedAccGenAlpha) #add algorithmic acceleration correction to old solver

# #switch tests between old/new dynamic solver
# exudynTestGlobals.useNewGenAlphaSolver = True
# if exudynTestGlobals.useNewGenAlphaSolver: 
#     exu.sys['experimentalNewSolver']='' #solver checks only if variable exists
# else:
#     if 'experimentalNewSolver' in exu.sys: #delete if exists already
#         del exu.sys['experimentalNewSolver']
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#current date and time
def NumTo2digits(n):
    if n < 10:
        return '0'+str(n)
    return str(n)
    
import datetime # for current date
now=datetime.datetime.now()
dateStr = str(now.year) + '-' + NumTo2digits(now.month) + '-' + NumTo2digits(now.day) + ' ' + NumTo2digits(now.hour) + ':' + NumTo2digits(now.minute) + ':' + NumTo2digits(now.second)
#date and time of exudyn library:
import os #for retrieving file information
from datetime import datetime #datetime contains .fromtimestamp(...)
import exudyn.exudynCPP as exuCPP #this is the cpp file, 

exu.Print("exudyn path=",exuCPP.__file__)
fileInfo=os.stat(exuCPP.__file__)
exuDate = datetime.fromtimestamp(fileInfo.st_mtime) 
exuDateStr = str(exuDate.year) + '-' + NumTo2digits(exuDate.month) + '-' + NumTo2digits(exuDate.day) + ' ' + NumTo2digits(exuDate.hour) + ':' + NumTo2digits(exuDate.minute) + ':' + NumTo2digits(exuDate.second)

#previous, with WorkingRelease:
# fileInfo=os.stat(workingReleasePath+'\\exudyn\\exudynCPP.pyd')
# exuDate = datetime.fromtimestamp(fileInfo.st_mtime) #mtime=modified time; ctime=created time, but contains time of first creation (2019...)
# exuDateStr = str(exuDate.year) + '-' + NumTo2digits(exuDate.month) + '-' + NumTo2digits(exuDate.day) + ' ' + NumTo2digits(exuDate.hour) + ':' + NumTo2digits(exuDate.minute) + ':' + NumTo2digits(exuDate.second)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

platformString = platform.architecture()[0]#'32bit'
platformString += 'P'+str(sys.version_info.major) +'.'+ str(sys.version_info.minor)

logFileName = '../TestSuiteLogs/testSuiteLog_V'+exu.GetVersionString()+'_'+platformString+'.txt'
exu.SetWriteToFile(filename=logFileName, flagWriteToFile=True, flagAppend=False) #write all testSuite logs to files
#exu.SetWriteToFile(filename='testOutput.log', flagWriteToFile=True, flagAppend=False)

exu.Print('\n+++++++++++++++++++++++++++++++++++++++++++')
exu.Print('+++++        EXUDYN TEST SUITE        +++++')
exu.Print('+++++++++++++++++++++++++++++++++++++++++++')
exu.Print('EXUDYN version      = '+exu.GetVersionString())
exu.Print('EXUDYN build date   = '+exuDateStr)
exu.Print('platform            = '+platform.architecture()[0])
exu.Print('Windows binary path = ',workingReleasePath)
exu.Print('python version      = '+str(sys.version_info.major)+'.'+str(sys.version_info.minor)+'.'+str(sys.version_info.micro))
exu.Print('test tolerance      = ',testTolerance)
exu.Print('testsuite date (now)= '+dateStr)
exu.Print('+++++++++++++++++++++++++++++++++++++++++++')
exu.SetWriteToConsole(writeToConsole) #stop output from now on

testFileList = [
                'ANCFcontactCircleTest.py',
                'ANCFcontactFrictionTest.py',
                'ANCFmovingRigidBodyTest.py',
                'ACNFslidingAndALEjointTest.py',
                'carRollingDiscTest.py',
                'compareAbaqusAnsysRotorEigenfrequencies.py',
                'compareFullModifiedNewton.py',
                'computeODE2EigenvaluesTest.py',
                'driveTrainTest.py',
                'explicitLieGroupIntegratorPythonTest.py',
                'explicitLieGroupIntegratorTest.py',
                'fourBarMechanismTest.py', 
                'genericJointUserFunctionTest.py',
                'genericODE2test.py',
                'geneticOptimizationTest.py',
                'heavyTop.py',
                'manualExplicitIntegrator.py',
                'mecanumWheelRollingDiscTest.py',
                'objectFFRFreducedOrderAccelerations.py',
                'objectFFRFreducedOrderTest.py',
                'objectFFRFTest.py',
                'objectFFRFTest2.py',
                'objectGenericODE2Test.py',
                'PARTS_ATEs_moving.py',
                'pendulumFriction.py',
                'rigidBodyCOMtest.py',
                'rollingCoinTest.py',
                'rollingCoinPenaltyTest.py',
                'scissorPrismaticRevolute2D.py',
                'serialRobotTest.py',
                'sliderCrank3Dbenchmark.py',
                'sliderCrankFloatingTest.py',
                'solverExplicitODE1ODE2test.py',
                'sparseMatrixSpringDamperTest.py',
                'sphericalJointTest.py',
                'springDamperUserFunctionTest.py',
                'stiffFlyballGovernor.py',
                'superElementRigidJointTest.py',
                'connectorRigidBodySpringDamperTest.py',
                ]


#testFileList = ['Examples/fourBarMechanism.py']
totalTests = len(testFileList)
testsFailed = [] #list of numbers containing the test numbers of failed tests
exudynTestGlobals.useGraphics = False
exudynTestGlobals.performTests = True

timeStart= -time.time()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#small (old) unit tests
testInterface = TestInterface(exudyn = exu, systemContainer = SC, useGraphics=False)
                              # useCorrectedAccGenAlpha = exudynTestGlobals.useCorrectedAccGenAlpha,
                              # useNewGenAlphaSolver = exudynTestGlobals.useNewGenAlphaSolver)
rv = False
if runUnitTests:
    exu.Print('\n***********************')
    exu.Print('  RUN MODEL UNIT TESTS ')
    exu.Print('***********************\n')
    rv = RunAllModelUnitTests(mbs, testInterface)
SC.Reset()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#run general test examples
examplesTestSolList={}
examplesTestErrorList={}
invalidResult = 1234567890123456 #should not happen occasionally
if runTestExamples:
    from runTestSuiteRefSol import TestExamplesReferenceSolution
    examplesTestRefSol = TestExamplesReferenceSolution()
    
    testExamplesCnt = 0
    for file in testFileList:
        name = file #.split('.')[0] #without '.py'
        exu.Print('\n\n******************************************')
        exu.Print('  START EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '"):')
        exu.Print('******************************************')
        SC.Reset()
        exudynTestGlobals.testError = -1 #default value !=-1, if there is an error in the calculation
        exudynTestGlobals.testResult = invalidResult #strange default value to see if there is a missing testResult
        try:
            exec(open(file).read(), globals())
        except Exception as e:
            exu.Print('EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") raised exception:\n'+str(e))
            print('EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") raised exception:\n'+str(e))

        examplesTestErrorList[name] = exudynTestGlobals.testError
        examplesTestSolList[name] = exudynTestGlobals.testResult

        #compute error from reference solution
        if examplesTestRefSol[name] != invalidResult:
            exudynTestGlobals.testError = exudynTestGlobals.testResult - examplesTestRefSol[name]
        if abs(exudynTestGlobals.testError) < testTolerance:
            exu.Print('******************************************')
            exu.Print('  EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") FINISHED SUCCESSFUL')
            exu.Print('******************************************')
        else:
            exu.Print('******************************************')
            exu.Print('  EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") FAILED')
            exu.Print('  ERROR = ' + str(exudynTestGlobals.testError))
            exu.Print('******************************************')
            testsFailed = testsFailed + [testExamplesCnt]
        
        testExamplesCnt += 1

    #create new reference values set for runTestSuiteRefSol.py:
    if printTestResults: #print reference solution list:
        for key,value in examplesTestSolList.items(): print("'"+key+"':"+str(value)+",")

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#test mini examples which are generated with objects
from MiniExamples.miniExamplesFileList import miniExamplesFileList
miniExamplesFailed = []
if runMiniExamples:
    from runTestSuiteRefSol import MiniExamplesReferenceSolution

    miniExamplesRefSol = MiniExamplesReferenceSolution()
    testExamplesCnt = 0
    miniExamplesTestSolList={}
    miniExamplesTestErrorList={}

    for file in miniExamplesFileList:
        name = file
        exu.Print('\n\n******************************************')
        exu.Print('  START MINI EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '"):')
        SC.Reset()
        testError = -1
        fileDir = 'MiniExamples/'+file
        exec(open(fileDir).read(), globals())
        exudynTestGlobals.testError = exudynTestGlobals.testResult-miniExamplesRefSol[name]
        if abs(exudynTestGlobals.testError) < testTolerance:
            exu.Print('  MINI EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") FINISHED SUCCESSFUL')
            exu.Print('  RESULT = ' + str(exudynTestGlobals.testResult))
            exu.Print('  ERROR  = ' + str(exudynTestGlobals.testError))
        else:
            exu.Print('******************************************')
            exu.Print('  MINI EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") FAILED')
            exu.Print('  RESULT = ' + str(exudynTestGlobals.testResult))
            exu.Print('  ERROR  = ' + str(exudynTestGlobals.testError))
            exu.Print('******************************************')
            miniExamplesFailed += [testExamplesCnt]
        miniExamplesTestSolList[name] = exudynTestGlobals.testResult #this list contains reference solutions, can be used for miniExamplesRefSol
        miniExamplesTestErrorList[name] = exudynTestGlobals.testError #this list contains errors
        testExamplesCnt+=1

    if printTestResults: #print reference solution list:
        for key,value in miniExamplesTestSolList.items(): print("'"+key+"':"+str(value)+",")
    

timeStart += time.time()
        
        
exu.Print('\n')
exu.SetWriteToConsole(True) #final output always written
exu.SetWriteToFile(filename=logFileName, flagWriteToFile=True, flagAppend=True) #write also to file (needed?)

exu.Print('******************************************')
exu.Print('TEST SUITE RESULTS SUMMARY:')
exu.Print('******************************************')

#++++++++++++++++++++++++++++++++++
exu.Print('time elapsed =',round(timeStart,3),'seconds') 
#10+5 tests:   2019-12-10: 2.4 seconds on Surface Pro
#10+5 tests:   2019-12-13: 3.0,2.7 seconds on Surface Pro
#10+6 tests:   2019-12-16: 3.8, 3.7 seconds on i9
#10+7 tests:   2019-12-16: 4.49 seconds on i9
#10+7 tests:   2019-12-17: 3.94 / 3.87 seconds on i9
#10+8 tests:   2019-12-18: 5.96 / 6.06 seconds on i9
#10+11tests:   2020-01-6:  6.96 seconds on i9
#10+11tests:   2020-01-24: 8.30 seconds on Surface Pro
#10+12tests:   2020-02-03: 7.10 seconds on i9
#10+14tests:   2020-02-19: 7.60 seconds on Surface Pro
#10+15+8tests: 2020-02-19: 7.729 seconds on i9
#10+19+8tests: 2020-04-22: 7.754 seconds on i9
#10+21+8tests: 2020-05-17: 9.949 seconds on i9
#10+28+12tests: 2020-05-17: 15.667 seconds on i9
#10+29+12tests: 2020-09-10: 17.001 seconds on Surface Pro
#10+36+13tests: 2021-01-04: 23.54 seconds on i9

if runUnitTests:
    if rv == True:
        exu.Print('ALL UNIT TESTS SUCCESSFUL')
    else:
        exu.Print('UNIT TESTS FAILED: see above section UNIT TESTS for detailed information')
else:
    exu.Print('UNIT TESTS SKIPPED')
    
if runTestExamples:
    if len(testsFailed) == 0:
        exu.Print('ALL ' + str(totalTests) + ' EXAMPLE TESTS SUCCESSFUL')
    else:
        exu.Print(str(len(testsFailed)) + ' EXAMPLE TEST(S) OUT OF '+ str(totalTests) + ' FAILED: ')
        for i in testsFailed:
            exu.Print('  EXAMPLE ' + str(i) + ' (' + testFileList[i] + ') FAILED')
else:
    exu.Print('EXAMPLE TESTS SKIPPED')
    
if runMiniExamples:
    if len(miniExamplesFailed) == 0:
        exu.Print('ALL ' + str(len(miniExamplesFileList)) + ' MINI EXAMPLE TESTS SUCCESSFUL')
    else:
        exu.Print(str(len(miniExamplesFailed)) + ' MINI EXAMPLE TEST(S) OUT OF '+ str(len(miniExamplesFileList)) + ' FAILED: ')
    exu.Print('******************************************\n')
else:
    exu.Print('MINI EXAMPLE TESTS SKIPPED')

    
exu.SetWriteToFile(filename='', flagWriteToFile=False, flagAppend=False) #stop writing to file, close file


#delete in future: (logs anyway written in testsuitelogs)
# if copyLog:
#     file=open(logFileName,'r') 
#     strLog = file.read()
#     file.close()
    
#     workingReleaseLog = workingReleasePath+"/testSuiteLog.txt"
#     file = open(workingReleaseLog,'w')
#     file.write(strLog)
#     file.close()
