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

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import sys, platform

if sys.version_info.major != 3 or sys.version_info.minor < 6:# or sys.version_info.minor > 12:
    raise ImportError("EXUDYN only supports python versions >= 3.6")
isMacOS = (sys.platform == 'darwin')
isWindows = (sys.platform == 'win32')
isARM = False
if platform.processor().find('arm') != -1:
    isARM = True


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include right exudyn module now:
import numpy as np
import exudyn as exu
from modelUnitTests import RunAllModelUnitTests, TestInterface, ExudynTestStructure, exudynTestGlobals
import time

try:
    import matplotlib 
    matplotlib.use('Agg') #do not show figures... in test examples
except:
    exu.Print('import matplotlib failed ... using standard plot engine')

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#parse command line arguments:
# -quiet
writeToConsole = True  #do not output to console / shell
outputLocal = False
quietMode = False
#copyLog = False         #copy log to final TestSuiteLogs
# if sys.version_info.major == 3 and sys.version_info.minor == 7:
#     copyLog = True #for P3.7 tests always copy log to WorkingRelease
if len(sys.argv) > 1:
    for i in range(len(sys.argv)-1):
        #print("arg", i+1, "=", sys.argv[i+1])
        if sys.argv[i+1] == '-quiet':
            quietMode = True
        elif sys.argv[i+1] == '-local':
            outputLocal = True
        # elif sys.argv[i+1] == '-copylog': #not needed any more
        #     copyLog = True
        else:
            print("ERROR in runTestSuite: unknown command line argument '"+sys.argv[i+1]+"'")

if quietMode:
    writeToConsole = False

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#choose which tests to run:
runUnitTests = False #skipped at least since V1.6
runTestExamples = True
runMiniExamples = True
runCppUnitTests = True

printTestResults = False #print list, which can be imported for new reference values
if platform.architecture()[0] == '32bit' and isWindows:
    testTolerance = 2e-12 #2022-03-17: use 2e-12 instead of 2e-13 to complete all tests; larger tolerance, because reference values are computed with 64bit version (WHY?)
elif isMacOS or not isWindows:
    testTolerance = 3e-11 #use larger tolerance value due to different compilation (heavy top gives error > 2.2e-11) on linux error > 2.5e-11
else:
    testTolerance = 5e-14 #on windows 64bit

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
    return '0'*(n<10)+str(n)
    # if n < 10:
    #     return '0'+str(n)
    # return str(n)
    
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


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

platformString = platform.architecture()[0]#'32bit'
platformString += 'P'+str(sys.version_info.major) +'.'+ str(sys.version_info.minor)

if isARM:
    processorString = 'ARM'
else:
    processorString = 'x86'

if isMacOS:
    platformString += 'macOS'
    platformString += '-'+processorString
elif not isWindows:
    platformString += sys.platform #usually linux
    if isARM:
        platformString += '-'+processorString

pythonVersion = str(sys.version_info.major)+'.'+str(sys.version_info.minor)+'.'+str(sys.version_info.micro)
pythonVersionMain = str(sys.version_info.major)+'.'+str(sys.version_info.minor)

# localFileName = 'Test for EXUDYN V'+exu.GetVersionString()+' (built:'+exuDateStr+'),'\
#          +sys.platform+'-'+processorString+'-'+platform.architecture()[0]+',Python'\
#          +pythonVersion+',date:'+dateStr+': '
platformString = sys.platform+'-'+processorString+'-'+platform.architecture()[0]+'-P'+pythonVersionMain
localFileName = 'testSuiteLog_V'+exu.GetVersionString()+'_'+platformString

#logFileName = '../TestSuiteLogs/testSuiteLog_V'+exu.GetVersionString()+'_'+platformString+'.txt'
logFileName = '../TestSuiteLogs/'+localFileName+'.txt'
exu.SetWriteToFile(filename=logFileName, flagWriteToFile=True, flagAppend=False) #write all testSuite logs to files



exu.Print('\n+++++++++++++++++++++++++++++++++++++++++++')
exu.Print('+++++        EXUDYN TEST SUITE        +++++')
exu.Print('+++++++++++++++++++++++++++++++++++++++++++')
exu.Print('EXUDYN version      = '+exu.GetVersionString())
exu.Print('EXUDYN build date   = '+exuDateStr)
exu.Print('architecture        = '+platform.architecture()[0])
exu.Print('processor           = '+processorString)
exu.Print('platform            = '+sys.platform)
exu.Print('Python version      = '+pythonVersion)
exu.Print('NumPy version       = '+np.__version__)
exu.Print('test tolerance      =',testTolerance)
exu.Print('testsuite date (now)= '+dateStr)
exu.Print('+++++++++++++++++++++++++++++++++++++++++++')

exu.SetWriteToConsole(writeToConsole) #stop output from now on

#testFileList = ['Examples/fourBarMechanism.py']
testsFailed = [] #list of numbers containing the test numbers of failed tests
exudynTestGlobals.useGraphics = False
exudynTestGlobals.performTests = True

timeStart= -time.time()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#small (old) unit tests
testInterface = TestInterface(exudyn = exu, systemContainer = SC, useGraphics=False)
                              # useCorrectedAccGenAlpha = exudynTestGlobals.useCorrectedAccGenAlpha,
                              # useNewGenAlphaSolver = exudynTestGlobals.useNewGenAlphaSolver)
rvModelUnitTests = True
unitTestsFailed = []
if runUnitTests:
    exu.Print('\n***********************')
    exu.Print('  RUN MODEL UNIT TESTS ')
    exu.Print('***********************\n')
    [rvModelUnitTests, unitTestsFailed] = RunAllModelUnitTests(mbs, testInterface)
SC.Reset()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#run general test examples
examplesTestSolList={}
examplesTestErrorList={}
invalidResult = 1234567890123456 #should not happen occasionally
if runTestExamples:
    from runTestSuiteRefSol import TestExamplesReferenceSolution
    examplesTestRefSol = TestExamplesReferenceSolution()
    
    testFileList=[] #automatically create list from reference solution ...
    for key in examplesTestRefSol.keys():
        testFileList+=[key]
    totalTests = len(testFileList)
    
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
            print('EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") raised exception:\n'+str(e), flush=True)
        finally:
            examplesTestErrorList[name] = exudynTestGlobals.testError
            examplesTestSolList[name] = exudynTestGlobals.testResult
            
            testTolFact = 1 #special factor for some examples which make problems, e.g., due to sparse eigenvalue solver
            if file == 'serialRobotTest.py':
                testTolFact = 100
                if platform.architecture()[0] != '64bit':
                    testTolFact = 1e7 #32 bits makes problems (error=1e-7)
    
            if platform.architecture()[0] != '64bit':
                if file == 'ACNFslidingAndALEjointTest.py':
                    testTolFact = 50
    
    
            #compute error from reference solution
            if examplesTestRefSol[name] != invalidResult:
                exudynTestGlobals.testError = exudynTestGlobals.testResult - examplesTestRefSol[name]
                exu.Print("refsol=",examplesTestRefSol[name])
                exu.Print("tol=", testTolerance*testTolFact)
    
            if abs(exudynTestGlobals.testError) < testTolerance*testTolFact:
                exu.Print('******************************************')
                exu.Print('  EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") FINISHED SUCCESSFUL')
                exu.Print('  RESULT = ' + str(exudynTestGlobals.testResult))
                exu.Print('  ERROR = ' + str(exudynTestGlobals.testError))
                exu.Print('******************************************')
            else:
                exu.Print('******************************************')
                exu.Print('  EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") *FAILED*')
                exu.Print('  RESULT = ' + str(exudynTestGlobals.testResult))
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
        try:
            exec(open(fileDir).read(), globals())
        except Exception as e:
            exu.Print('MINI EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") raised exception:\n'+str(e))
            print('MINI EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") raised exception:\n'+str(e), flush=True)
        finally:
            exudynTestGlobals.testError = exudynTestGlobals.testResult-miniExamplesRefSol[name]
            if abs(exudynTestGlobals.testError) < testTolerance:
                exu.Print('  MINI EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") FINISHED SUCCESSFUL')
                exu.Print('  RESULT = ' + str(exudynTestGlobals.testResult))
                exu.Print('  ERROR  = ' + str(exudynTestGlobals.testError))
            else:
                exu.Print('******************************************')
                exu.Print('  MINI EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") *FAILED*')
                exu.Print('  RESULT = ' + str(exudynTestGlobals.testResult))
                exu.Print('  ERROR  = ' + str(exudynTestGlobals.testError))
                exu.Print('******************************************')
                miniExamplesFailed += [testExamplesCnt]
            miniExamplesTestSolList[name] = exudynTestGlobals.testResult #this list contains reference solutions, can be used for miniExamplesRefSol
            miniExamplesTestErrorList[name] = exudynTestGlobals.testError #this list contains errors
            testExamplesCnt+=1

    if printTestResults: #print reference solution list:
        for key,value in miniExamplesTestSolList.items(): print("'"+key+"':"+str(value)+",")
    
if runCppUnitTests:
    if hasattr(exu.solver, 'RunCppUnitTests'):
        exu.Print('\n******************************************')
        exu.Print('RUN CPP UNIT TESTS:')
        exu.Print('******************************************')
        numberOfCppUnitTestsFailed = exu.special.RunCppUnitTests()
    else:
        runCppUnitTests = False #will display that they were skipped 
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

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
totalFails = 0
if runUnitTests:
    if rvModelUnitTests:
        exu.Print('ALL UNIT TESTS SUCCESSFUL')
    else:
        exu.Print('UNIT TESTS FAILED: '+str(unitTestsFailed))
    # localFileName += '-unittests'+str(len(unitTestsFailed))
    totalFails+=len(unitTestsFailed)
else:
    exu.Print('UNIT TESTS SKIPPED')
    
if runTestExamples:
    if len(testsFailed) == 0:
        exu.Print('ALL ' + str(totalTests) + ' TestModel TESTS SUCCESSFUL')
    else:
        exu.Print(str(len(testsFailed)) + ' TestModel TEST(S) OUT OF '+ str(totalTests) + ' FAILED: ')
        for i in testsFailed:
            exu.Print('  TestModel ' + str(i) + ' (' + testFileList[i] + ') FAILED')
    # localFileName += '-models'+str(len(testsFailed))
    totalFails+=len(testsFailed)
else:
    exu.Print(', EXAMPLE TESTS SKIPPED')
    
if runMiniExamples:
    if len(miniExamplesFailed) == 0:
        exu.Print('ALL ' + str(len(miniExamplesFileList)) + ' MINI EXAMPLE TESTS SUCCESSFUL')
    else:
        exu.Print(str(len(miniExamplesFailed)) + ' MINI EXAMPLE TEST(S) OUT OF '+ str(len(miniExamplesFileList)) + ' FAILED: ')
        for i in miniExamplesFailed:
            exu.Print('  MINI EXAMPLE ' + str(i) + ' (' + miniExamplesFileList[i] + ') FAILED')
        
    exu.Print('******************************************\n')
    # localFileName += '-mini'+str(len(miniExamplesFailed))
    totalFails+=len(miniExamplesFailed)
else:
    exu.Print('MINI EXAMPLE TESTS SKIPPED')

if runCppUnitTests:
    if numberOfCppUnitTestsFailed == 0:
        exu.Print('ALL CPP UNIT TESTS SUCCESSFUL')
    else:
        exu.Print(str(numberOfCppUnitTestsFailed) + ' CPP UNIT TESTS FAILED: see above section for detailed information')
    # localFileName += '-cpp'+str(numberOfCppUnitTestsFailed)
    totalFails+=len(numberOfCppUnitTestsFailed)
else:
    exu.Print('CPP UNIT TESTS SKIPPED')
    # localFileName += '-nocpp'

#create a filename which indicates the number of fails
localFileName = localFileName+'-'+'F'+str(totalFails).zfill(2)+'.txt'
# exu.Print('\n'+localFileName)

exu.SetWriteToFile(filename='', flagWriteToFile=False, flagAppend=False) #stop writing to file, close file

#write summary for github actions
if outputLocal:
    # testSummaryFileName = 'test-exudyn.txt'
    allText = ''
    with open(logFileName, 'r') as f:
        allText = f.read()
        
    with open(localFileName, 'w') as f:
        f.write(allText)


