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
#--fast-module: run the whole suite against exudynCPPfast instead of the default module, which
#otherwise ships untested. This has to happen HERE, before exudyn is
#imported below - once the C++ module is loaded the choice is made. The environment variable is
#what the model WORKERS see: --parallel and pytest run every model in its own interpreter, and a
#child inherits the variable but not sys.exudynFast.
#NOTE '--fast' is something else entirely: the pull-request subset.
useFastModule = '--fast-module' in sys.argv
if useFastModule:
    import os
    os.environ['EXUDYN_MODULE'] = 'fast'
    sys.exudynFast = True

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include right exudyn module now:
import numpy as np
import testRunnerTools

#the suite drives python/TestModels/ and runs IN it: every model is written relative to that
#directory and the log goes to ../logs/ next to it. Where the suite was STARTED from does not
#matter (#2512, #2513)
testRunnerTools.WorkInModelsDirectory(testRunnerTools.testModelsDir)

import exudyn as exu

if useFastModule: #asking is not getting; stop rather than write a log that claims the wrong module
    testRunnerTools.RequireFastModule('--fast-module')
import time

try:
    import matplotlib 
    matplotlib.use('Agg') #do not show figures... in test examples
except:
    exu.Print('import matplotlib failed ... using standard plot engine')

#NO WINDOW, IN EITHER PATH (#2632): the two worker bootstraps in
#testRunnerTools.py have called this, but the SERIAL path of this
#runner did not - the models' own 'if useGraphics:' was the only thing keeping windows shut, and
#a model that drops that branch opens one on the screen of whoever runs the suite. The suite
#never wants a window: it sets useGraphics=False for every model a few lines below.
exu.special.userInterface.SuppressAll(True)

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
#copyLog = False         #copy log to final logs/testmodels
# if sys.version_info.major == 3 and sys.version_info.minor == 7:
#     copyLog = True #for P3.7 tests always copy log to WorkingRelease
#--fast: the pull-request subset - without the models that take
#noticeably longer and without those needing an optional package. Both lists are data in
#runTestSuiteRefSol.py, shared with the pytest markers.
TSScope.fastSubset = False

#--parallel: run the models in separate interpreters; serial by default,
#so that the gating run stays exactly what it has always been
TSScope.parallel = False
TSScope.numberOfProcesses = 0 #0: chosen by testRunnerTools.RunModelsInParallel

if len(sys.argv) > 1:
    for i in range(len(sys.argv)-1):
        #print("arg", i+1, "=", sys.argv[i+1])
        if sys.argv[i+1] == '-quiet':
            quietMode = True
        elif sys.argv[i+1] == '-local':
            outputLocal = True
        elif sys.argv[i+1] == '--exit-code':
            useExitCode = True
        elif sys.argv[i+1] == '--fast':
            TSScope.fastSubset = True
        elif sys.argv[i+1] == '--fast-module':
            pass #already acted upon, before the exudyn import; listed so it is not "unknown"
        elif sys.argv[i+1].startswith('--parallel'):
            #--parallel runs every model in its own interpreter, the number of workers after '='
            #; possible since each model writes into its own directory
            TSScope.parallel = True
            if '=' in sys.argv[i+1]:
                TSScope.numberOfProcesses = int(sys.argv[i+1].split('=')[1])
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
TSScope.runTestExamples = True
TSScope.runMiniExamples = True
TSScope.runCppUnitTests = True

#root for everything the models write; each model gets its own subdirectory below it (#2418)
TSScope.solutionDirectory = 'solution'

TSScope.printTestResults = False #print list, which can be imported for new reference values
#the platform-dependent base tolerance lives in testRunnerTools, so that the suite and the
#pytest collector cannot drift apart
TSScope.testTolerance = testRunnerTools.BaseTolerance()

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
(exuCPPname, exuCPPmodule) = testRunnerTools.LoadedCppModule() #never names the module itself (#2466)
exuCPPfile = exuCPPmodule.__file__ if exuCPPmodule is not None else ''

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
#exu.config.Version() is the SAME string for both modules, so without this marker the fast run
#would collide with the default log and be diverted to tmp/ with a misleading message about
#another machine. Derived from what was loaded, not from what was asked;
#and from WHICH MODULE it is, not from whether it has AVX2 - the fast module carries no vector
#extensions on macOS or in a --no-avx2 build, and its log still has to be told apart (#2496).
if not testRunnerTools.ModuleIsRegular():
    platformString += '_fast'
localFileName = 'testSuiteLog_V'+exu.config.Version()+'_'+platformString

logFileName = '../logs/testmodels/'+localFileName+'.txt'
#the directory the run STARTED with, usually from EXUDYN_OUTPUTDIRECTORY. The models below
#overwrite exudyn.config.outputDirectory one by one, and it has to be put BACK to this - not
#to the empty string - or the summary is written to a different file than the rest of the
#log, because the C++ writer resolves the name against it at every open (#2500)
initialOutputDirectory = exu.config.outputDirectory

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

TSScope.timeStart = -time.time()

#the ten 'model unit tests' were functions of python/testing/modelUnitTests.py, run from here
#through a TestInterface object - and NOT run: runUnitTests was False "at least since V1.6".
#They are ordinary test models (#2632), so they are in the
#list below with everything else, and they run.
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
                                    SensitiveTests, UnresolvedOnLinux, UnresolvedOnMacOS,
                                    DeliberatelyNotRun)
    TSScope.examplesTestRefSol = TestExamplesReferenceSolution()
    #the values above belong to the BASELINE module; a module with vector extensions gets the
    #second set on top of them, which holds only the models that actually move (#2470)
    if testRunnerTools.ModuleUsesAVX2():
        from runTestSuiteRefSol import AVX2ReferenceSolutionUpdate
        TSScope.avx2Update = AVX2ReferenceSolutionUpdate()
        TSScope.examplesTestRefSol.update({name: value for name, value
                                           in TSScope.avx2Update.items()
                                           if name in TSScope.examplesTestRefSol})
        exu.Print('module has vector extensions: ' + str(len(TSScope.avx2Update))
                  + ' AVX2 reference values applied')
    TSScope.testTolFactors = TestExamplesToleranceFactors()
    TSScope.sensitiveTests = SensitiveTests()
    #known platform differences are excluded from the exit code ON THAT PLATFORM ONLY: the
    #reference values are the Windows ones, so Windows must still pass them
    TSScope.unresolvedTests = set()
    if isMacOS:
        TSScope.unresolvedTests = UnresolvedOnMacOS()
    elif not isWindows:
        TSScope.unresolvedTests = UnresolvedOnLinux()
    #the tests whose failure must not set the exit code, whatever the reason
    TSScope.excludedFromExitCode = TSScope.sensitiveTests | TSScope.unresolvedTests
    
    #the reference lists ARE the run manifest, so a model missing from them is never executed.
    #Check that against the folder before running anything, and report it in the log where the
    #next reader will see it.
    #since revision2026 step R3.9 this directory holds test models and nothing else, so the
    #check is simply 'every .py is either referenced or explicitly excluded' (#2513).
    #raytracerNOGLFWtest.py is added back because TestExamplesReferenceSolution() pops it on
    #macOS only - the file still exists there.
    TSScope.coverageText, TSScope.coverageFailed = testRunnerTools.CheckTestCoverage(
        modelsDir='.',
        refSolNames=set(TSScope.examplesTestRefSol.keys()) | set(['raytracerNOGLFWtest.py']),
        deliberatelyNotRun=DeliberatelyNotRun())
    exu.Print(TSScope.coverageText)

    TSScope.testFileList=[] #automatically create list from reference solution ...
    for key in TSScope.examplesTestRefSol.keys():
        TSScope.testFileList+=[key]

    #a few models mean something different outside the regular module, so they are not run there at
    #all - one of them would even overwrite its own tracked input file (#2470)
    if not testRunnerTools.ModuleIsRegular():
        from runTestSuiteRefSol import NotJudgedOutsideRegularModule
        for name, reason in NotJudgedOutsideRegularModule().items():
            if name in TSScope.testFileList:
                TSScope.testFileList.remove(name)
                exu.Print('not the regular module: ' + name + ' skipped - ' + reason)

    if TSScope.fastSubset: #revision2026 step R5.2
        from runTestSuiteRefSol import SlowTests, OptionalPackageTests
        skipped = set(SlowTests()) | set(OptionalPackageTests())
        TSScope.testFileList = [f for f in TSScope.testFileList if f not in skipped]
        exu.Print('--fast: ' + str(len(skipped)) + ' slow or optional-package models are skipped: '
                  + ', '.join(sorted(skipped)))

    TSScope.totalTests = len(TSScope.testFileList)
    
    #in parallel mode every model runs in its own interpreter FIRST, and the loop below then
    #reports the results in the order of testFileList, so log and exit code do not depend on the
    #order in which the models finished
    TSScope.parallelResults = {}
    if TSScope.parallel:
        exu.Print('running ' + str(TSScope.totalTests) + ' test models in parallel')
        print('running ' + str(TSScope.totalTests) + ' test models in parallel', flush=True)
        TSScope.parallelResults = testRunnerTools.RunModelsInParallel(
            TSScope.testFileList, TSScope.solutionDirectory, TSScope.invalidResult,
            numberOfProcesses=TSScope.numberOfProcesses, printProgress=writeToConsole)

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
        TSScope.testError = -1 #default value !=-1, if there is an error in the calculation
        TSScope.testResult = TSScope.invalidResult #strange default value to see if there is a missing testResult
        #the channel a model uses (#2632): exu.sys instead of an
        #import of this module. It is cleared for every model, because exu.sys lives as long as
        #the interpreter and a value left over from the previous model would be read as this
        #model's result
        exu.sys['testIsActive'] = True
        exu.sys['testResult'] = TSScope.invalidResult
        exu.sys.pop('testTolerance', None)
        TSScope.testTimeStart = time.perf_counter()
        try:
            if TSScope.parallel:
                #already run in its own interpreter; reproduce its output in the log here
                TSScope.modelRun = TSScope.parallelResults[TSScope.file]
                exu.Print(TSScope.modelRun['output'])
                TSScope.testResult = TSScope.modelRun['result']
                #the model ran in another interpreter, so its tolerance comes back with it
                if TSScope.modelRun.get('tolerance', 0.) > 0.:
                    exu.sys['testTolerance'] = TSScope.modelRun['tolerance']
                if TSScope.modelRun['failed']:
                    exu.Print('TESTMODEL ' + str(TSScope.testExamplesCnt) + ' ("' + TSScope.file
                              + '") terminated with an error, see its output above')
            else:
                exec(open(TSScope.file, encoding='utf8').read(), globals())
        except Exception as e:
            exu.Print('TESTMODEL ' + str(TSScope.testExamplesCnt) + ' ("' + TSScope.file + '") raised exception:\n'+str(e))
            print('TESTMODEL ' + str(TSScope.testExamplesCnt) + ' ("' + TSScope.file + '") raised exception:\n'+str(e), flush=True)
        finally:
            TSScope.examplesTestTimeList[TSScope.name] = (TSScope.parallelResults[TSScope.file]['seconds']
                                                          if TSScope.parallel else
                                                          time.perf_counter() - TSScope.testTimeStart)
            #a converted model writes exu.sys['testResult']; one that has not been converted
            #yet still writes into the suite's own variable
            if exu.sys.get('testResult', TSScope.invalidResult) != TSScope.invalidResult:
                TSScope.testResult = exu.sys['testResult']
            exu.sys['testIsActive'] = False

            TSScope.examplesTestErrorList[TSScope.name] = TSScope.testError
            TSScope.examplesTestSolList[TSScope.name] = TSScope.testResult
            
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
                TSScope.testError = TSScope.testResult - TSScope.examplesTestRefSol[TSScope.name]
                exu.Print("refsol=",TSScope.examplesTestRefSol[TSScope.name])
                exu.Print("tol=", TSScope.testTolerance*TSScope.testTolFact)
    
            #a model may state an absolute tolerance of its own;
            #it replaces multiplying the result by a factor to make it fit, which hid the
            #tolerance inside the number the test compares
            TSScope.testModelTolerance = TSScope.testTolerance*TSScope.testTolFact
            if 'testTolerance' in exu.sys:
                TSScope.testModelTolerance = float(exu.sys['testTolerance'])
                exu.Print('tolerance of this model =', TSScope.testModelTolerance)

            TSScope.examplesTestTolList[TSScope.name] = TSScope.testModelTolerance
            #NOTE: examplesTestErrorList above is captured BEFORE the error is recomputed from
            #the reference solution, so for most models it holds the default -1 rather than the
            #comparison error. Keep that dictionary as it was, and record the final error here.
            TSScope.examplesTestFinalErrorList[TSScope.name] = TSScope.testError

            if abs(TSScope.testError) < TSScope.testModelTolerance:
                exu.Print('******************************************')
                exu.Print('  TESTMODEL ' + str(TSScope.testExamplesCnt) + ' ("' + TSScope.file + '") FINISHED SUCCESSFUL')
                exu.Print('  RESULT = ' + str(TSScope.testResult))
                exu.Print('  ERROR = ' + str(TSScope.testError))
                exu.Print('******************************************')
            else:
                exu.Print('******************************************')
                exu.Print('  TESTMODEL ' + str(TSScope.testExamplesCnt) + ' ("' + TSScope.file + '") *FAILED*')
                exu.Print('  RESULT = ' + str(TSScope.testResult))
                exu.Print('  ERROR = ' + str(TSScope.testError))
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
#python/ is on sys.path since WorkInModelsDirectory(), so the generated list one level up
#is importable while the working directory stays TestModels/ (#2513)
from MiniExamples.miniExamplesFileList import miniExamplesFileList
miniExamplesFailed = []
miniExamplesFailedKnown = []        #of those, the ones a platform list names
if TSScope.runMiniExamples:
    from runTestSuiteRefSol import MiniExamplesReferenceSolution

    miniExamplesRefSol = MiniExamplesReferenceSolution()
    if testRunnerTools.ModuleUsesAVX2(): #the same second reference set as above (#2470)
        from runTestSuiteRefSol import AVX2ReferenceSolutionUpdate
        miniExamplesRefSol.update({name: value for name, value
                                   in AVX2ReferenceSolutionUpdate().items()
                                   if name in miniExamplesRefSol})
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
        exu.sys['testIsActive'] = True          #revision2026b step RG10.6.4
        exu.sys['testResult'] = TSScope.invalidResult
        exu.config.outputDirectory = TSScope.solutionDirectory + '/MiniExamples/' + file[:-3] #(#2418)
        fileDir = '../MiniExamples/'+file
        miniTimeStart = time.perf_counter()
        try:
            exec(open(fileDir, encoding='utf8').read(), globals())
        except Exception as e:
            exu.Print('MINI EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") raised exception:\n'+str(e))
            print('MINI EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") raised exception:\n'+str(e), flush=True)
        finally:
            TSScope.testResult = exu.sys.get('testResult', TSScope.invalidResult)
            exu.sys['testIsActive'] = False
            TSScope.testError = TSScope.testResult-miniExamplesRefSol[name]
            if abs(TSScope.testError) < TSScope.testTolerance:
                exu.Print('  MINI EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") FINISHED SUCCESSFUL')
                exu.Print('  RESULT = ' + str(TSScope.testResult))
                exu.Print('  ERROR  = ' + str(TSScope.testError))
            else:
                exu.Print('******************************************')
                exu.Print('  MINI EXAMPLE ' + str(testExamplesCnt) + ' ("' + file + '") *FAILED*')
                exu.Print('  RESULT = ' + str(TSScope.testResult))
                exu.Print('  ERROR  = ' + str(TSScope.testError))
                exu.Print('******************************************')
                miniExamplesFailed += [testExamplesCnt]
                miniExamplesFailedNames.add(name)
                #a mini example is a model like any other and can be a known platform difference:
                #ObjectConnectorRigidBodySpringDamper.py is one on macOS (#2379)
                if name in TSScope.excludedFromExitCode:
                    miniExamplesFailedKnown += [testExamplesCnt]
            miniExamplesTestSolList[name] = TSScope.testResult #this list contains reference solutions, can be used for miniExamplesRefSol
            miniExamplesTestErrorList[name] = TSScope.testError #this list contains errors
            miniExamplesTestTimeList[name] = time.perf_counter() - miniTimeStart
            miniExamplesTestTolList[name] = TSScope.testTolerance
            testExamplesCnt+=1

    if TSScope.printTestResults: #print reference solution list:
        for key,value in miniExamplesTestSolList.items(): print("'"+key+"':"+str(value)+",")
    
if TSScope.runCppUnitTests:
    #the binding is on exu.special, not exu.solver - checking the wrong module made the tests LOOK
    #skipped even in a build that has them (#2458). It exists only when the module was compiled
    #with PERFORM_UNIT_TESTS, which is the performUnitTests build switch.
    if hasattr(exu.special, 'RunCppUnitTests'):
        exu.Print('\n******************************************')
        exu.Print('RUN CPP UNIT TESTS:')
        exu.Print('******************************************')
        numberOfCppUnitTestsFailed = exu.special.RunCppUnitTests()
    else:
        TSScope.runCppUnitTests = False #will display that they were skipped 
TSScope.timeStart += time.time()
exu.config.outputDirectory = initialOutputDirectory #back to where the log is written (#2500);
#cleared for good after the log file is closed, so that it does not outlive the run (#2418)
        
        
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
    totalFails+=numberOfCppUnitTestsFailed #RunCppUnitTests returns a COUNT, not a list (#2458)
else:
    exu.Print('CPP UNIT TESTS SKIPPED: this build has no lest unit tests; rebuild with the '
              'performUnitTests switch')
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
exu.config.outputDirectory = '' #global setting, must not outlive the run (#2418)

#write summary for github actions
if outputLocal:
    # testSummaryFileName = 'test-exudyn.txt'
    allText = ''
    #the same resolution the C++ writer applied when it opened the file (#2500)
    from exudyn.basicUtilities import OutputFilePath
    with open(OutputFilePath(logFileName, 'runTestSuite'), 'r') as f:
        allText = f.read()
        
    with open(localFileName, 'w') as f:
        f.write(allText)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#exit code for automated runs; only requested explicitly, so interactive and Spyder use
#is unchanged. Failures of tests listed in SensitiveTests() are reported above but do not
#set the exit code: those models are chaotic or use an unseeded sparse eigenvalue solver,
#so they differ between machines and would make a scheduled run fail at random.
if useExitCode:
    reproducibleFails = totalFails - len(testsFailedSensitive) - len(miniExamplesFailedKnown)
    #a coverage gap is a failure of the suite itself, not of a test: the list no longer
    #describes the folder, so a passing run no longer means what it says
    if TSScope.runTestExamples and TSScope.coverageFailed:
        print('FAILED: test coverage - see the TEST COVERAGE section of the log', flush=True)
        sys.exit(1)
    knownDifferences = len(testsFailedSensitive) + len(miniExamplesFailedKnown)
    if knownDifferences != 0:
        print('note: ' + str(knownDifferences) +
              ' known-difference test(s) failed (sensitive, or unresolved on this platform);'
              ' excluded from the exit code', flush=True)
    if reproducibleFails > 0:
        print('FAILED: ' + str(reproducibleFails) + ' reproducible test(s)', flush=True)
        sys.exit(1)
    print('PASSED: no reproducible test failed', flush=True)
    sys.exit(0)


