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

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the user settings of ~/.exudyn/config.json are IGNORED here, and this has to happen
#before exudyn is imported. A maintainer who stores a setting must not thereby change
#what a test computes; a child process inherits the variable, so the workers of
#--parallel and of pytest are covered too
import os
os.environ['EXUDYN_NO_USER_SETTINGS'] = '1'

import multiprocessing #for determining if on laptop or workstation

if sys.version_info.major != 3 or sys.version_info.minor < 6:# or sys.version_info.minor > 9:
    raise ImportError("EXUDYN only supports python versions >= 3.6")
isMacOS = (sys.platform == 'darwin')
isWindows = (sys.platform == 'win32')

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#include right exudyn module now:
import numpy as np
import testRunnerTools

#the performance suite drives python/PerformanceModels/ and runs IN it; the log goes to
#../logs/performance/ next to it. Where it was STARTED from does not matter
#(#2512, #2513)
testRunnerTools.WorkInModelsDirectory(testRunnerTools.performanceModelsDir)

#--fast-module measures exudynCPPfast; without it, the regular module. Which module is measured is
#asked for, and not decided by the Python version, so the release procedure can run both
#deliberately; see docs/dev/WORKFLOW.md. Must happen before 'import exudyn' below (#2495).
useFastModule = '--fast-module' in sys.argv
if useFastModule:
    import os
    os.environ['EXUDYN_MODULE'] = 'fast'
    sys.exudynFast = True
else:
    sys.exudynFast = False

import exudyn as exu

if useFastModule: #asking is not getting - a declined request would mismeasure the wrong module
    testRunnerTools.RequireFastModule('--fast-module')

(exuCPPname, exuCPP) = testRunnerTools.LoadedCppModule() #never names the module itself (#2466)

import time

psutilExists = False
try:
    import psutil #for cpu_percent
    psutilExists = True
except:
    print('*** WARNING: no psutils installed! Will NOT show CPU load during performance tests! ***')

SC = exu.SystemContainer()
mbs = SC.AddSystem()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#parse command line arguments:
# -quiet
writeToConsole = True  #do not output to console / shell
overwriteLog = False   #--overwrite-log: replace an existing log instead of diverting to tmp
useExitCode = False    #--exit-code: exit non-zero when a performance test failed (#2504)
#copyLog = False         #copy log to final logs/performance
# if sys.version_info.major == 3 and sys.version_info.minor == 7:
#     copyLog = True #for P3.7 tests always copy log to WorkingRelease
if len(sys.argv) > 1:
    for i in range(len(sys.argv)-1):
        #print("arg", i+1, "=", sys.argv[i+1])
        if sys.argv[i+1] == '-quiet':
            writeToConsole = False
        elif sys.argv[i+1] == '--overwrite-log':
            overwriteLog = True
        elif sys.argv[i+1] == '--exit-code':
            useExitCode = True
        elif sys.argv[i+1] == '--fast-module':
            pass #already acted upon, before the exudyn import; listed so it is not "unknown"
        # elif sys.argv[i+1] == '-copylog': #not needed any more
        #     copyLog = True
        else:
            print("ERROR in runPerformanceTests: unknown command line argument '"+sys.argv[i+1]+"'")

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

fileInfo=os.stat(exuCPP.__file__)
exuDate = datetime.fromtimestamp(fileInfo.st_mtime) 
exuDateStr = str(exuDate.year) + '-' + NumTo2digits(exuDate.month) + '-' + NumTo2digits(exuDate.day) + ' ' + NumTo2digits(exuDate.hour) + ':' + NumTo2digits(exuDate.minute) + ':' + NumTo2digits(exuDate.second)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
testTolerance = 1e-10

platformString = platform.architecture()[0]#'32bit'
platformString += 'P'+str(sys.version_info.major) +'.'+ str(sys.version_info.minor)
if isMacOS:
    platformString += 'MacOSX'
elif not isWindows: #add linux, to distinguish linux tests from windows tests!
    platformString += sys.platform

#exu.config.Version() is the same string for both modules, so without this marker a --fast-module
#run would collide with the regular log - and comparing the two is the whole point of measuring
#them. Derived from what was loaded, not from what was asked, and from
#WHICH MODULE it is rather than from its instruction set (#2496).
if not testRunnerTools.ModuleIsRegular():
    platformString += '_fast'

#performance logs are collected per machine, because timings from a mobile CPU are not
#comparable with a workstation. Set EXUDYN_MACHINE_ID once per machine (e.g. 'i7-1370P') and
#its logs land in that subfolder.
subFolder = ''
machineId = os.environ.get('EXUDYN_MACHINE_ID', '').strip()
if machineId != '':
    #keep the subfolder name usable as a path
    machineId = ''.join([c if (c.isalnum() or c in '-_.') else '_' for c in machineId])
    subFolder = machineId + '/'
elif multiprocessing.cpu_count() == 20:
    #LEGACY fallback, kept so existing behaviour does not change silently; any 20-core machine
    #lands here, which is why EXUDYN_MACHINE_ID exists. Remove once it is set on all machines.
    subFolder = 'i7-1370P/'

if subFolder != '' and not os.path.exists('../logs/performance/'+subFolder):
    os.makedirs('../logs/performance/'+subFolder, exist_ok=True)

logFileName = '../logs/performance/'+subFolder+'performanceLog_V'+exu.config.Version()+'_'+platformString+'.txt'
#never truncate an existing (committed) log by accident; see testRunnerTools.ResolveLogFile
logFileName = testRunnerTools.ResolveLogFile(logFileName, allowOverwrite=overwriteLog)
exu.SetWriteToFile(filename=logFileName, flagWriteToFile=True, flagAppend=False) #write all testSuite logs to files

exu.config.printToConsole = writeToConsole #stop output from now on

exu.Print('\n+++++++++++++++++++++++++++++++++++++++++++')
exu.Print('+++++    EXUDYN PERFORMANCE TESTS     +++++')
exu.Print('+++++++++++++++++++++++++++++++++++++++++++')
exu.Print('EXUDYN version      = '+exu.config.Version())
exu.Print('EXUDYN build date   = '+exuDateStr)
exu.Print('platform            = '+platform.architecture()[0])
exu.Print('system              = '+sys.platform)

#Surface book 2 = 'Intel64 Family 6 Model 142 Stepping 10, GenuineIntel'
exu.Print('processor           = '+platform.processor()) 

exu.Print('python version      = '+str(sys.version_info.major)+'.'+str(sys.version_info.minor)+'.'+str(sys.version_info.micro))
#timings depend on these; record them so runs can be compared across machines and dates
exu.Print(testRunnerTools.PackageVersionReport())
exu.Print('test tolerance      = ',testTolerance)
exu.Print('test date (now)     = '+dateStr)
if psutilExists:
    exu.Print('CPU usage (%/thread)= '+str(psutil.cpu_percent(interval=1, percpu=True)))
    if psutil.sensors_battery()!=None:
        exu.Print('Power plugged       = '+str(psutil.sensors_battery().power_plugged))
exu.Print('+++++++++++++++++++++++++++++++++++++++++++')


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Tests are grouped by what they actually measure, because the two groups answer different
#questions and mixing them hides both (#2397):
#
#  'small'  few coordinates, ~1e6 steps -> measures PER-STEP OVERHEAD. The system vectors are
#           3-20 elements long, so vectorized linear algebra (AVX2) cannot show up here at all.
#  'large'  many coordinates, comparatively few steps -> measures the cost of the work per step,
#           where long vectors, AVX2 and multithreading are visible.
#
#A change that speeds up one group and not the other is the normal case, not an anomaly; the
#summary at the end therefore reports the groups separately as well as the total.
testGroups = {
    'small': [
                'perfRigidPendulum.py',
                'perfSpringDamperExplicit.py',
                'perfSpringDamperUserFunction.py',
             ],
    'large': [
                'generalContactSpheresPerf.py',
                'perf3DRigidBodies.py',
                'perfObjectFFRFreducedOrder.py',
                'perfLargeMassSpringChain.py',
                'perfConnectorInterface.py',
             ],
    }

testFileList = []
testFileGroup = {}      #file name -> group, for the summary
for groupName in ['small', 'large']:
    for fileName in testGroups[groupName]:
        testFileList.append(fileName)
        testFileGroup[fileName] = groupName


totalTests = len(testFileList)
testsFailed = [] #list of numbers containing the test numbers of failed tests
#the channel a model uses is exu.sys, as for the test models


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#run general test examples
examplesTestSolList={}
examplesTestErrorList={}
invalidResult = 1234567890123456 #should not happen occasionally
totalTime = 0
testTimings = {}    #file name -> CPU time, for the grouped summary at the end
allRuns = []        #every single simulation run of every model (#2460)

from runTestSuiteRefSol import PerformanceTestsReferenceSolution
performanceTestRefSol = PerformanceTestsReferenceSolution()

#the reference list IS the run manifest here too, so check it against the folder before
#running anything - the same check runTestSuite.py makes over TestModels/, made possible for
#the performance models by giving them a directory of their own.
#PerformanceTestsReferenceSolution() also holds values for the SINGLE RUNS of a model that
#solves several sizes or thread counts ('...:nt8'); those are not file names (issue #2460).
coverageText, coverageFailed = testRunnerTools.CheckTestCoverage(
    modelsDir='.',
    refSolNames=set(k for k in performanceTestRefSol.keys() if k.endswith('.py')),
    deliberatelyNotRun={})
exu.Print(coverageText)

testExamplesCnt = 0
for file in testFileList:
    name = file #.split('.')[0] #without '.py'
    exu.Print('\n\n****************************************************')
    exu.Print('  START PERFORMANCE TEST ' + str(testExamplesCnt) + ' ("' + file + '"):')
    exu.Print('****************************************************')
    SC.Reset()
    exu.sys['testIsActive'] = True
    exu.sys['testTimings'] = [] #filled by the model, one dict per simulation run
    exu.sys.pop('testTolerance', None) #a model may state one of its own
    testError = -1
    exu.sys['testResult'] = invalidResult #a value that says 'the model set none'
    timeStart= -time.time()
    try:
        exec(open(file).read(), globals())
    except Exception as e:
        exu.Print('PERFORMANCE TEST ' + str(testExamplesCnt) + ' ("' + file + '") raised exception:\n'+str(e))
        print('PERFORMANCE TEST ' + str(testExamplesCnt) + ' ("' + file + '") raised exception:\n'+str(e))

    timeStart += time.time()
    totalTime += timeStart

    testResult = exu.sys.get('testResult', invalidResult)
    modelTolerance = float(exu.sys.get('testTolerance', testTolerance))
    exu.sys['testIsActive'] = False

    examplesTestErrorList[name] = testError
    examplesTestSolList[name] = testResult
    
    #compute error from reference solution
    testError = testResult - performanceTestRefSol[name]
    if abs(testError) < modelTolerance:
        exu.Print('****************************************************')
        exu.Print('  PERFORMANCE TEST ' + str(testExamplesCnt) + ' ("' + file + '") FINISHED SUCCESSFUL')
    else:
        exu.Print('****************************************************')
        exu.Print('  PERFORMANCE TEST ' + str(testExamplesCnt) + ' ("' + file + '") *FAILED*')
        testsFailed = testsFailed + [testExamplesCnt]

    exu.Print('  RESULT   = ' + str(testResult))
    exu.Print('  ERROR    = ' + str(testError))
    exu.Print('  CPU TIME = ' + str(timeStart))
    exu.Print('****************************************************')

    testTimings[name] = timeStart
    allRuns += exu.sys['testTimings']
    testExamplesCnt += 1

exu.Print('\n')
exu.config.printToConsole = True #final output always written
exu.SetWriteToFile(filename=logFileName, flagWriteToFile=True, flagAppend=True) #write also to file (needed?)

if psutilExists:
    exu.Print('CPU usage (%/thread)= '+str(psutil.cpu_percent(interval=1, percpu=True)))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the single simulation runs (#2460). A model may solve several
#sizes or thread counts, and each of those is a measurement of its own: the time here is the
#SOLVER time (solver.timer.total), without model build, assembly and Python overhead, so it is
#the number to compare between machines and between builds. The wall-clock table above still
#shows what the file as a whole costs.
if len(allRuns) != 0:
    exu.Print('')
    exu.Print('+++++ SINGLE RUNS (solver time) +++++')
    exu.Print('%-48s %10s %24s %12s' % ('run', 'solver[s]', 'result', 'error'))
    runsFailed = []
    for run in allRuns:
        reference = performanceTestRefSol.get(run['name'], None)
        if reference is None:
            errorString = 'NO REFERENCE'
            runsFailed += [run['name']]
        else:
            error = run['result'] - reference
            errorString = '%12.3e' % error
            if not abs(error) < testTolerance:
                errorString += ' *FAILED*'
                runsFailed += [run['name']]
        exu.Print('%-48s %10.3f %24.16g %12s' % (run['name'], run['time'], run['result'],
                                                 errorString))
    exu.Print('')
    if len(runsFailed) == 0:
        exu.Print('ALL ' + str(len(allRuns)) + ' SINGLE RUNS SUCCESSFUL')
    else:
        exu.Print(str(len(runsFailed)) + ' SINGLE RUN(S) OUT OF ' + str(len(allRuns)) + ' FAILED: '
                  + ', '.join(runsFailed))
        testsFailed += runsFailed   #a failed run fails the suite, like a failed file
    exu.Print('')

exu.Print('****************************************************')
if len(testsFailed) == 0:
    exu.Print('ALL ' + str(totalTests) + ' PERFORMANCE TESTS SUCCESSFUL')
else:
    exu.Print(str(len(testsFailed)) + ' PERFORMANCE TEST(S) FAILED, OUT OF '+ str(totalTests)
              + ' test files and ' + str(len(allRuns)) + ' single runs: ')
    for i in testsFailed:
        #a file is reported by its index, a single run of a file by its name (issue #2460)
        exu.Print('  PERFORMANCE TEST ' + (str(i) + ' (' + testFileList[i] + ')'
                                           if isinstance(i, int) else str(i)) + ' FAILED')

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#per-test timings, grouped. The group subtotals are the point: 'small' measures per-step
#overhead and 'large' measures the work per step, so a change that moves only one of them is
#telling you which of the two it affected. A single total cannot show that.
exu.Print('')
exu.Print('+++++ PERFORMANCE TEST TIMINGS +++++')
exu.Print('%-40s %8s %12s %9s' % ('name', 'group', 'time[s]', '% of total'))
for groupName in ['small', 'large']:
    groupTime = 0
    for fileName in testGroups[groupName]:
        t = testTimings.get(fileName, float('nan'))
        groupTime += t if t == t else 0     #skip a test that did not run
        percent = 100*t/totalTime if totalTime > 0 else 0
        exu.Print('%-40s %8s %12.3f %8.1f%%' % (fileName, groupName, t, percent))
    percent = 100*groupTime/totalTime if totalTime > 0 else 0
    exu.Print('%-40s %8s %12.3f %8.1f%%' % ('  --> subtotal ' + groupName, '', groupTime, percent))
exu.Print('%-40s %8s %12.3f %8.1f%%' % ('  ==> TOTAL', '', totalTime, 100.0))
exu.Print('')

exu.Print('TOTAL PERFORMANCE TEST TIME = ' + str(totalTime) + ' seconds')
#exu.Print('Reference value (i9)        = 88.12 seconds (32bit) / 74.11 seconds (regular) / 57.30 seconds (exudynFast)')
exu.Print('Reference value (i9, 2023-12, Windows)= 48 - 51 seconds (regular) / 39.5 seconds  (exudynFast)')
exu.Print('Reference value (i9, 2023-12, Linux  )= 42 - 44 seconds (regular) / 34 - 36 seconds (exudynFast)')
exu.Print('****************************************************')

    
exu.SetWriteToFile(filename='', flagWriteToFile=False, flagAppend=False) #stop writing to file, close file

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the exit code (#2504). Until this existed the runner ALWAYS returned 0,
#so a failed performance test was invisible to anything that called it. There is no known-failure
#list here: unlike the examples, every performance test passes today, and one that does not is a
#result worth going red for.
if useExitCode:
    #a coverage failure is a real failure: a performance model in no reference list is never
    #measured and nobody would notice
    sys.exit(1 if (len(testsFailed) != 0 or coverageFailed) else 0)


