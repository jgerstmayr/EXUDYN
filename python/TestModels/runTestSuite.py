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

#put all local variables into special class, to avoid unintentional overwriting
class TSScope:
    pass

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
import testRunnerTools
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
useExitCode = False     #--exit-code: return non-zero on reproducible failures, for CI
overwriteLog = False    #--overwrite-log: replace an existing log instead of diverting to tmp
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
        elif sys.argv[i+1] == '--exit-code':
            useExitCode = True
        elif sys.argv[i+1] == '--overwrite-log':
            overwriteLog = True
        # elif sys.argv[i+1] == '-copylog': #not needed any more
        #     copyLog = True
        else:
            print("ERROR in runTestSuite: unknown command line argument '"+sys.argv[i+1]+"'")

if quietMode:
    writeToConsole = False

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#choose which tests to run:
TSScope.runUnitTests = False #skipped at least since V1.6
TSScope.runTestExamples = True
TSScope.runMiniExamples = True
TSScope.runCppUnitTests = True

#root for everything the models write; each model gets its own subdirectory below it (#2418)
TSScope.solutionDirectory = 'solution'

TSScope.printTestResults = False #print list, which can be imported for new reference values
if platform.architecture()[0] == '32bit' and isWindows:
    TSScope.testTolerance = 2e-12 #2022-03-17: use 2e-12 instead of 2e-13 to complete all tests; larger tolerance, because reference values are computed with 64bit version (WHY?)
elif isMacOS or not isWindows:
    TSScope.testTolerance = 3e-11 #use larger tolerance value due to different compilation (heavy top gives error > 2.2e-11) on linux error > 2.5e-11
else:
    TSScope.testTolerance = 5e-14 #on windows 64bit

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
#do NOT import exudyn.exudynCPP here: __init__.py may have selected exudynCPPfast or
#exudynCPPnoAVX, and naming the default module would load a SECOND C++ binary into the
#process and then report the wrong one as the module under test
exuCPPfile = ''
for exuCPPname in ['exudynCPP', 'exudynCPPfast', 'exudynCPPnoAVX']:
    exuCPPmodule = sys.modules.get('exudyn.'+exuCPPname, None)
    if exuCPPmodule is not None:
        exuCPPfile = exuCPPmodule.__file__
        break

if exuCPPfile == '': #fallback: the package directory, so the date below is still meaningful
    exuCPPfile = exu.__file__

exu.Print("exudyn path=",exuCPPfile)
fileInfo=os.stat(exuCPPfile)
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

# localFileName = 'Test for EXUDYN V'+exu.config.Version()+' (built:'+exuDateStr+'),'\
#          +sys.platform+'-'+processorString+'-'+platform.architecture()[0]+',Python'\
#          +pythonVersion+',date:'+dateStr+': '
platformString = sys.platform+'-'+processorString+'-'+platform.architecture()[0]+'-P'+pythonVersionMain
localFileName = 'testSuiteLog_V'+exu.config.Version()+'_'+platformString

#logFileName = '../TestSuiteLogs/testSuiteLog_V'+exu.config.Version()+'_'+platformString+'.txt'
logFileName = '../TestSuiteLogs/'+localFileName+'.txt'
#never truncate an existing (committed) log by accident: SetWriteToFile below wipes the target
#immediately, before any test runs, so an interrupted run would leave it half-written
logFileName = testRunnerTools.ResolveLogFile(logFileName, allowOverwrite=overwriteLog)
exu.SetWriteToFile(filename=logFileName, flagWriteToFile=True, flagAppend=False) #write all testSuite logs to files



exu.Print('\n+++++++++++++++++++++++++++++++++++++++++++')
exu.Print('+++++        EXUDYN TEST SUITE        +++++')
exu.Print('+++++++++++++++++++++++++++++++++++++++++++')
exu.Print('EXUDYN version      = '+exu.config.Version())
exu.Print('EXUDYN build date   = '+exuDateStr)
exu.Print('architecture        = '+platform.architecture()[0])
exu.Print('processor           = '+processorString)
exu.Print('CPU                 = '+testRunnerTools.CpuInfoString())
exu.Print('platform            = '+sys.platform)
exu.Print('Python version      = '+pythonVersion)
exu.Print('NumPy version       = '+np.__version__)
#test results depend on these; scipy 1.18 vs 1.15 is already a recorded factor
exu.Print(testRunnerTools.PackageVersionReport())
exu.Print('test tolerance      =',TSScope.testTolerance)
exu.Print('testsuite date (now)= '+dateStr)
exu.Print('+++++++++++++++++++++++++++++++++++++++++++')

exu.config.printToConsole = writeToConsole #stop output from now on

#TSScope.testFileList = ['Examples/fourBarMechanism.py']
testsFailed = [] #list of numbers containing the test numbers of failed tests
testsFailedSensitive = [] #subset of testsFailed which are known to be machine-sensitive
exudynTestGlobals.useGraphics = False
exudynTestGlobals.performTests = True

TSScope.timeStart = -time.time()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#small (old) unit tests
testInterface = TestInterface(exudyn = exu, systemContainer = SC, useGraphics=False)
                              # useCorrectedAccGenAlpha = exudynTestGlobals.useCorrectedAccGenAlpha,
                              # useNewGenAlphaSolver = exudynTestGlobals.useNewGenAlphaSolver)
rvModelUnitTests = True
unitTestsFailed = []
if TSScope.runUnitTests:
    exu.Print('\n***********************')
    exu.Print('  RUN MODEL UNIT TESTS ')
    exu.Print('***********************\n')
    [rvModelUnitTests, unitTestsFailed] = RunAllModelUnitTests(mbs, testInterface)
SC.Reset()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#run general test examples
#use TSScope. to avoid that variables in testsuite are overwritten by test models!
TSScope.examplesTestSolList={}
TSScope.examplesTestErrorList={}
TSScope.examplesTestTimeList={}     #per-test runtime, for the overview at the end of the log
TSScope.examplesTestTolList={}      #effective tolerance actually applied, varies per test
TSScope.examplesTestFinalErrorList={} #error vs reference solution, after recomputation
TSScope.examplesFailedNames=set()   #names rather than indices, for the overview
TSScope.invalidResult = 1234567890123456 #should not happen occasionally
if TSScope.runTestExamples:
    from runTestSuiteRefSol import (TestExamplesReferenceSolution, TestExamplesToleranceFactors,
                                    SensitiveTests, UnresolvedOnLinux,
                                    NotTestModels, DeliberatelyNotRun,
                                    PerformanceTestsReferenceSolution)
    TSScope.examplesTestRefSol = TestExamplesReferenceSolution()
    TSScope.testTolFactors = TestExamplesToleranceFactors()
    TSScope.sensitiveTests = SensitiveTests()
    #known Windows/Linux differences are excluded from the exit code ON LINUX ONLY: the
    #reference values are the Windows ones, so Windows must still pass them (revision2026 phase R10)
    TSScope.unresolvedTests = set()
    if not isWindows and not isMacOS:
        TSScope.unresolvedTests = UnresolvedOnLinux()
    #the tests whose failure must not set the exit code, whatever the reason
    TSScope.excludedFromExitCode = TSScope.sensitiveTests | TSScope.unresolvedTests
    
    #the reference lists ARE the run manifest, so a model missing from them is never executed.
    #Check that against the folder before running anything, and report it in the log where the
    #next reader will see it (revision2026 step R5.9).
    TSScope.coverageText, TSScope.coverageFailed = testRunnerTools.CheckTestCoverage(
        modelsDir='.',
        #MiniExamples are deliberately absent: they live in MiniExamples/, not here, and have
        #their own generated manifest. raytracerNOGLFWtest.py is added back because
        #TestExamplesReferenceSolution() pops it on macOS only - the file still exists there.
        refSolNames=(set(TSScope.examplesTestRefSol.keys())
                     | set(PerformanceTestsReferenceSolution().keys())
                     | set(['raytracerNOGLFWtest.py'])),
        notTestModels=NotTestModels(),
        deliberatelyNotRun=DeliberatelyNotRun())
    exu.Print(TSScope.coverageText)

    TSScope.testFileList=[] #automatically create list from reference solution ...
    for key in TSScope.examplesTestRefSol.keys():
        TSScope.testFileList+=[key]
    TSScope.totalTests = len(TSScope.testFileList)
    
    TSScope.testExamplesCnt = 0
    for TSScope.file in TSScope.testFileList:
        import platform #if platform is overwritten
        
        TSScope.name = TSScope.file #.split('.')[0] #without '.py'
        exu.Print('\n\n******************************************')
        exu.Print('  START TESTMODEL ' + str(TSScope.testExamplesCnt) + ' ("' + TSScope.file + '"):')
        exu.Print('******************************************')
        SC.Reset() #??needed
        #every model writes into its own directory, so two models cannot collide on a file name
        #such as solution/coordinatesSolution.txt - the prerequisite for running the suite in
        #parallel (#2418). Models keep their own relative file names; only the root moves.
        exu.config.outputDirectory = TSScope.solutionDirectory + '/' + TSScope.file[:-3]
        exudynTestGlobals.testError = -1 #default value !=-1, if there is an error in the calculation
        exudynTestGlobals.testResult = TSScope.invalidResult #strange default value to see if there is a missing testResult
        TSScope.testTimeStart = time.perf_counter()
        try:
            exec(open(TSScope.file, encoding='utf8').read(), globals())
        except Exception as e:
            exu.Print('TESTMODEL ' + str(TSScope.testExamplesCnt) + ' ("' + TSScope.file + '") raised exception:\n'+str(e))
            print('TESTMODEL ' + str(TSScope.testExamplesCnt) + ' ("' + TSScope.file + '") raised exception:\n'+str(e), flush=True)
        finally:
            TSScope.examplesTestTimeList[TSScope.name] = time.perf_counter() - TSScope.testTimeStart
            TSScope.examplesTestErrorList[TSScope.name] = exudynTestGlobals.testError
            TSScope.examplesTestSolList[TSScope.name] = exudynTestGlobals.testResult
            
            #special factor for some examples which make problems, e.g., due to sparse
            #eigenvalue solver; maintained as data in runTestSuiteRefSol.py
            TSScope.testTolFact = TSScope.testTolFactors.get(TSScope.file, 1)

            if platform.architecture()[0] != '64bit': #32 bits makes problems
                if TSScope.file == 'serialRobotTest.py':
                    TSScope.testTolFact = 1e7 #error=1e-7
                elif TSScope.file == 'ACNFslidingAndALEjointTest.py':
                    TSScope.testTolFact = 50
    
    
            #compute error from reference solution
            if TSScope.examplesTestRefSol[TSScope.name] != TSScope.invalidResult:
                exudynTestGlobals.testError = exudynTestGlobals.testResult - TSScope.examplesTestRefSol[TSScope.name]
                exu.Print("refsol=",TSScope.examplesTestRefSol[TSScope.name])
                exu.Print("tol=", TSScope.testTolerance*TSScope.testTolFact)
    
            TSScope.examplesTestTolList[TSScope.name] = TSScope.testTolerance*TSScope.testTolFact
            #NOTE: examplesTestErrorList above is captured BEFORE the error is recomputed from
            #the reference solution, so for most models it holds the default -1 rather than the
            #comparison error. Keep that dictionary as it was, and record the final error here.
            TSScope.examplesTestFinalErrorList[TSScope.name] = exudynTestGlobals.testError

            if abs(exudynTestGlobals.testError) < TSScope.testTolerance*TSScope.testTolFact:
                exu.Print('******************************************')
                exu.Print('  TESTMODEL ' + str(TSScope.testExamplesCnt) + ' ("' + TSScope.file + '") FINISHED SUCCESSFUL')
                exu.Print('  RESULT = ' + str(exudynTestGlobals.testResult))
                exu.Print('  ERROR = ' + str(exudynTestGlobals.testError))
                exu.Print('******************************************')
            else:
                exu.Print('******************************************')
                exu.Print('  TESTMODEL ' + str(TSScope.testExamplesCnt) + ' ("' + TSScope.file + '") *FAILED*')
                exu.Print('  RESULT = ' + str(exudynTestGlobals.testResult))
                exu.Print('  ERROR = ' + str(exudynTestGlobals.testError))
                if TSScope.file in TSScope.sensitiveTests:
                    exu.Print('  NOTE: this test is marked SENSITIVE (chaotic or unseeded);')
                    exu.Print('        it is reported but does not affect the exit code')
                elif TSScope.file in TSScope.unresolvedTests:
                    exu.Print('  NOTE: known unresolved Windows/Linux difference (revision plan')
                    exu.Print('        revision2026 phase R10); reported but does not affect the exit code')
                exu.Print('******************************************')
                testsFailed = testsFailed + [TSScope.testExamplesCnt]
                TSScope.examplesFailedNames.add(TSScope.name)
                if TSScope.file in TSScope.excludedFromExitCode:
                    testsFailedSensitive = testsFailedSensitive + [TSScope.testExamplesCnt]

            TSScope.testExamplesCnt += 1

    #create new reference values set for runTestSuiteRefSol.py:
    if TSScope.printTestResults: #print reference solution list:
        for key,value in TSScope.examplesTestSolList.items(): print("'"+key+"':"+str(value)+",")

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#test mini examples which are generated with objects
from MiniExamples.miniExamplesFileList import miniExamplesFileList
miniExamplesFailed = []
if TSScope.runMiniExamples:
    from runTestSuiteRefSol import MiniExamplesReferenceSolution

    miniExamplesRefSol = MiniExamplesReferenceSolution()
    testExamplesCnt = 0
    miniExamplesTestSolList={}
    miniExamplesTestErrorList={}
    miniExamplesTestTimeList={}     #for the overview at the end of the log
    miniExamplesTestTolList={}
    miniExamplesFailedNames=set()

    for file in miniExamplesFileList:
        name = file
        exu.Print('\n\n******************************************')
        exu.Print('  START MINI EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '"):')
        SC.Reset()
        testError = -1
        exu.config.outputDirectory = TSScope.solutionDirectory + '/MiniExamples/' + file[:-3] #(#2418)
        fileDir = 'MiniExamples/'+file
        miniTimeStart = time.perf_counter()
        try:
            exec(open(fileDir, encoding='utf8').read(), globals())
        except Exception as e:
            exu.Print('MINI EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") raised exception:\n'+str(e))
            print('MINI EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") raised exception:\n'+str(e), flush=True)
        finally:
            exudynTestGlobals.testError = exudynTestGlobals.testResult-miniExamplesRefSol[name]
            if abs(exudynTestGlobals.testError) < TSScope.testTolerance:
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
                miniExamplesFailedNames.add(name)
            miniExamplesTestSolList[name] = exudynTestGlobals.testResult #this list contains reference solutions, can be used for miniExamplesRefSol
            miniExamplesTestErrorList[name] = exudynTestGlobals.testError #this list contains errors
            miniExamplesTestTimeList[name] = time.perf_counter() - miniTimeStart
            miniExamplesTestTolList[name] = TSScope.testTolerance
            testExamplesCnt+=1

    if TSScope.printTestResults: #print reference solution list:
        for key,value in miniExamplesTestSolList.items(): print("'"+key+"':"+str(value)+",")
    
if TSScope.runCppUnitTests:
    if hasattr(exu.solver, 'RunCppUnitTests'):
        exu.Print('\n******************************************')
        exu.Print('RUN CPP UNIT TESTS:')
        exu.Print('******************************************')
        numberOfCppUnitTestsFailed = exu.special.RunCppUnitTests()
    else:
        TSScope.runCppUnitTests = False #will display that they were skipped 
TSScope.timeStart += time.time()
exu.config.outputDirectory = '' #the setting is global and would otherwise outlive the run (#2418)
        
        
exu.Print('\n')
exu.config.printToConsole = True #final output always written
exu.SetWriteToFile(filename=logFileName, flagWriteToFile=True, flagAppend=True) #write also to file (needed?)

exu.Print('******************************************')
exu.Print('TEST SUITE RESULTS SUMMARY:')
exu.Print('******************************************')

#++++++++++++++++++++++++++++++++++
exu.Print('time elapsed =',round(TSScope.timeStart,3),'seconds') 
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
if TSScope.runUnitTests:
    if rvModelUnitTests:
        exu.Print('ALL UNIT TESTS SUCCESSFUL')
    else:
        exu.Print('UNIT TESTS FAILED: '+str(unitTestsFailed))
    # localFileName += '-unittests'+str(len(unitTestsFailed))
    totalFails+=len(unitTestsFailed)
else:
    exu.Print('UNIT TESTS SKIPPED')
    
if TSScope.runTestExamples:
    if len(testsFailed) == 0:
        exu.Print('ALL ' + str(TSScope.totalTests) + ' TestModel TESTS SUCCESSFUL')
    else:
        exu.Print(str(len(testsFailed)) + ' TestModel TEST(S) OUT OF '+ str(TSScope.totalTests) + ' FAILED: ')
        for i in testsFailed:
            exu.Print('  TestModel ' + str(i) + ' (' + TSScope.testFileList[i] + ') FAILED')
    # localFileName += '-models'+str(len(testsFailed))
    totalFails+=len(testsFailed)
else:
    exu.Print(', EXAMPLE TESTS SKIPPED')
    
if TSScope.runMiniExamples:
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

if TSScope.runCppUnitTests:
    if numberOfCppUnitTestsFailed == 0:
        exu.Print('ALL CPP UNIT TESTS SUCCESSFUL')
    else:
        exu.Print(str(numberOfCppUnitTestsFailed) + ' CPP UNIT TESTS FAILED: see above section for detailed information')
    # localFileName += '-cpp'+str(numberOfCppUnitTestsFailed)
    totalFails+=len(numberOfCppUnitTestsFailed)
else:
    exu.Print('CPP UNIT TESTS SKIPPED')
    # localFileName += '-nocpp'

#per-test overview at the end of the log: value, error, effective tolerance and runtime, one
#fixed-width line each. Comparing these across machines is how SensitiveTests() has to be
#populated (runTestSuiteRefSol.py), which is impractical while the numbers only appear in prose.
if TSScope.runTestExamples:
    exu.Print(testRunnerTools.FormatTestOverview(
        'TESTMODEL OVERVIEW',
        names=TSScope.testFileList,
        results=TSScope.examplesTestSolList,
        errors=TSScope.examplesTestFinalErrorList,
        tolerances=TSScope.examplesTestTolList,
        times=TSScope.examplesTestTimeList,
        failedNames=TSScope.examplesFailedNames,
        sensitiveNames=TSScope.sensitiveTests,
        unresolvedNames=TSScope.unresolvedTests))

if TSScope.runMiniExamples:
    exu.Print(testRunnerTools.FormatTestOverview(
        'MINI EXAMPLE OVERVIEW',
        names=miniExamplesFileList,
        results=miniExamplesTestSolList,
        errors=miniExamplesTestErrorList,
        tolerances=miniExamplesTestTolList,
        times=miniExamplesTestTimeList,
        failedNames=miniExamplesFailedNames))

#NOTE: the number of fails used to be appended to the file name as '-F<NN>'. It was dropped
#2026-09-09: the exit code (--exit-code) now carries that information, and the varying name
#made every run with a different failure count leave a NEW file instead of replacing the
#previous one, which slowly filled the log directories.
localFileName = localFileName+'.txt'

exu.SetWriteToFile(filename='', flagWriteToFile=False, flagAppend=False) #stop writing to file, close file

#write summary for github actions
if outputLocal:
    # testSummaryFileName = 'test-exudyn.txt'
    allText = ''
    with open(logFileName, 'r') as f:
        allText = f.read()
        
    with open(localFileName, 'w') as f:
        f.write(allText)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#exit code for automated runs; only requested explicitly, so interactive and Spyder use
#is unchanged. Failures of tests listed in SensitiveTests() are reported above but do not
#set the exit code: those models are chaotic or use an unseeded sparse eigenvalue solver,
#so they differ between machines and would make a scheduled run fail at random.
if useExitCode:
    reproducibleFails = totalFails - len(testsFailedSensitive)
    #a coverage gap is a failure of the suite itself, not of a test: the list no longer
    #describes the folder, so a passing run no longer means what it says
    if TSScope.runTestExamples and TSScope.coverageFailed:
        print('FAILED: test coverage - see the TEST COVERAGE section of the log', flush=True)
        sys.exit(1)
    if len(testsFailedSensitive) != 0:
        print('note: ' + str(len(testsFailedSensitive)) +
              ' known-difference test(s) failed (sensitive or unresolved-on-Linux);'
              ' excluded from the exit code', flush=True)
    if reproducibleFails > 0:
        print('FAILED: ' + str(reproducibleFails) + ' reproducible test(s)', flush=True)
        sys.exit(1)
    print('PASSED: no reproducible test failed', flush=True)
    sys.exit(0)


