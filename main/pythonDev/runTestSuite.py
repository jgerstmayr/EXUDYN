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
import sys
sys.path.append('../bin/WorkingRelease') #for exudyn, itemInterface and exudynUtilities
sys.path.append('TestModels')            #for modelUnitTest as this example may be used also as a unit test

import exudyn as exu
from modelUnitTests import RunAllModelUnitTests, TestInterface, ExudynTestStructure, exudynTestGlobals
import time

runUnitTests = True
runTestExamples = True
runMiniExamples = True

print('EXUDYN version='+exu.__version__)

SC = exu.SystemContainer()
mbs = SC.AddSystem()
exu.SetWriteToFile(filename='testOutput.log', flagWriteToFile=True, flagAppend=False)

#testFileList = ['Examples/fourBarMechanism.py', 
#                'Examples/sparseMatrixSpringDamperTest.py',
#                'Examples/springDamperUserFunctionTest.py',
#                'Examples/ANCF_contact_circle_test.py',
#                'Examples/sliderCrankFloating.py']

testFileList = ['TestModels/fourBarMechanismTest.py', 
                'TestModels/sparseMatrixSpringDamperTest.py',
                'TestModels/springDamperUserFunctionTest.py',
                'TestModels/ANCFcontactCircleTest.py',
                'TestModels/sliderCrankFloatingTest.py',
                'TestModels/ANCFmovingRigidBodyTest.py',
                'TestModels/ANCFcontactFrictionTest.py',
                'TestModels/ACNFslidingAndALEjointTest.py',
                'TestModels/scissorPrismaticRevolute2D.py',
                'TestModels/manualExplicitIntegrator.py',
                'TestModels/PARTS_ATEs_moving.py',
                'TestModels/sliderCrank3Dbenchmark.py',
                'TestModels/explicitLieGroupIntegratorTest.py',
                'TestModels/sphericalJointTest.py',
                'TestModels/heavyTop.py']



#testFileList = ['Examples/fourBarMechanism.py']
totalTests = len(testFileList)
testsFailed = [] #list of numbers containing the test numbers of failed tests
exudynTestGlobals.useGraphics = False
exudynTestGlobals.performTests = True

timeStart= -time.time()

testInterface = TestInterface(exudyn = exu, systemContainer = SC, useGraphics=False)
rv = False
if runUnitTests:
    rv = RunAllModelUnitTests(mbs, testInterface)
SC.Reset()

#test manual examples
if runTestExamples:
    cnt = 0
    for file in testFileList:
        print('\n\n******************************************')
        print('  START EXAMPLE ' + str(cnt) + ' ("' + file + '"):')
        print('******************************************')
        SC.Reset()
        exudynTestGlobals.testError = -1 #default value !=-1, if there is an error in the calculation
        exec(open(file).read(), globals())
        if abs(exudynTestGlobals.testError) < 3e-14:
            print('******************************************')
            print('  EXAMPLE ' + str(cnt) + ' ("' + file + '") FINISHED SUCCESSFUL')
            print('******************************************')
        else:
            print('******************************************')
            print('  EXAMPLE ' + str(cnt) + ' ("' + file + '") FAILED')
            print('  ERROR = ' + str(exudynTestGlobals.testError))
            print('******************************************')
            testsFailed = testsFailed + [cnt]
        
        cnt += 1

#test mini examples which are generated with objects
from MiniExamples.miniExamplesFileList import miniExamplesFileList
miniExamplesFailed = []

if runMiniExamples:
    cnt = 0
    for file in miniExamplesFileList:
        print('\n\n******************************************')
        print('  START MINI EXAMPLE ' + str(cnt) + ' ("' + file + '"):')
        SC.Reset()
        testError = -1
        fileDir = 'MiniExamples/'+file
        exec(open(fileDir).read(), globals())
        if abs(testError) < 3e-14:
            print('  MINI EXAMPLE ' + str(cnt) + ' ("' + file + '") FINISHED SUCCESSFUL')
        else:
            print('******************************************')
            print('  MINI EXAMPLE ' + str(cnt) + ' ("' + file + '") FAILED')
            print('  ERROR = ' + str(exudynTestGlobals.testError))
            print('******************************************')
            miniExamplesFailed += [cnt]
    

timeStart += time.time()
        
        
print('\n\n******************************************')
print('TEST SUITE RESULTS SUMMARY:')
print('******************************************')

#++++++++++++++++++++++++++++++++++
print('time elapsed =',round(timeStart,3),'seconds') 
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


if rv == True:
    print('ALL UNIT TESTS SUCCESSFUL')
else:
    if runUnitTests:
        print('UNIT TESTS FAILED: see above section UNIT TESTS for detailed information')
    
if len(testsFailed) == 0:
    if runTestExamples:
        print('ALL ' + str(totalTests) + ' EXAMPLE TESTS SUCCESSFUL')
else:
    print(str(len(testsFailed)) + ' EXAMPLE TEST(S) OUT OF '+ str(totalTests) + ' FAILED: ')
    for i in testsFailed:
        print('  EXAMPLE ' + str(i) + ' (' + testFileList[i] + ') FAILED')
        
if len(miniExamplesFailed) == 0:
    print('ALL ' + str(len(miniExamplesFileList)) + ' MINI EXAMPLE TESTS SUCCESSFUL')
else:
    print(str(len(testsFailed)) + ' MINI EXAMPLE TEST(S) OUT OF '+ str(len(miniExamplesFileList)) + ' FAILED: ')

    
    