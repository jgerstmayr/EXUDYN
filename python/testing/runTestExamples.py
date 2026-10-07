#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is the automated run of examples:
#   - start examples with defined timeout
#   - no graphics
#
# Author:   Johannes Gerstmayr
# Date:     2024-05-11
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

from os import listdir
from os.path import isfile, join

if sys.version_info.major != 3 or sys.version_info.minor < 6:# or sys.version_info.minor > 12:
    raise ImportError("EXUDYN only supports python versions >= 3.6")
isMacOS = (sys.platform == 'darwin')
isWindows = (sys.platform == 'win32')
isARM = False
if platform.processor().find('arm') != -1:
    isARM = True

if __name__ == '__main__': #include to avoid potential problems with multiprocessing!

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #include right exudyn module now:
    import numpy as np
    import testRunnerTools

    #the examples reach sideways with '../Examples/testData/...', so the working directory has
    #to be a SIBLING of Examples/ - python/TestModels/, exactly as before the runners moved out
    #of it (#2512, #2513)
    testRunnerTools.WorkInModelsDirectory(testRunnerTools.testModelsDir)

    import exudyn as exu
    import time
    
    
    
    try:
        import matplotlib 
        matplotlib.use('Agg') #do not show figures... in test examples
        import matplotlib.pyplot as plt
    except:
        exu.Print('import matplotlib failed ... using standard plot engine')
    
    def CloseAll():
        try:
            plt.close('all')
        except:
            pass
    
    
    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #parse command line arguments:
    # -quiet
    writeToConsole = True #do not output to console / shell
    quietMode = True
    writeFileNames = False
    overwriteLog = False    #--overwrite-log: replace an existing log instead of diverting to tmp
    useExitCode = False     #--exit-code: exit non-zero on an UNEXPECTED failure (#2504)
    #the examples are an API check, not a numerical one, so they run in parallel processes with a
    #short timeout; --serial restores the old in-process run
    runParallel = True
    numberOfProcesses = 0   #0: testRunnerTools picks it from the number of cores
    exampleTimeout = 60     #seconds per example; a timeout after the solver was reached is a pass
    solverTimeout = 1       #seconds per solver call inside an example
    #copyLog = False         #copy log to final logs/examples
    # if sys.version_info.major == 3 and sys.version_info.minor == 7:
    #     copyLog = True #for P3.7 tests always copy log to WorkingRelease
    if len(sys.argv) > 1:
        for i in range(len(sys.argv)-1):
            #print("arg", i+1, "=", sys.argv[i+1])
            if sys.argv[i+1] == '-quiet':
                quietMode = True
            elif sys.argv[i+1] == '--overwrite-log':
                overwriteLog = True
            elif sys.argv[i+1] == '--exit-code':
                useExitCode = True
            elif sys.argv[i+1] == '--serial':
                runParallel = False
            elif sys.argv[i+1].startswith('--parallel'):
                #--parallel, or --parallel=N to fix the number of interpreters
                runParallel = True
                if '=' in sys.argv[i+1]:
                    numberOfProcesses = int(sys.argv[i+1].split('=')[1])
            elif sys.argv[i+1].startswith('--timeout='):
                exampleTimeout = float(sys.argv[i+1].split('=')[1])
            else:
                print("ERROR in runTestExamples: unknown command line argument '"+sys.argv[i+1]+"'")
    
    if quietMode:
        print('*** write to console deactivated ***')
        writeToConsole = False
    
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #choose which tests to run:
    
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
    
    try:
        import exudyn.exudynCPP as exuCPP #this is the cpp file, 
        
        exu.Print("exudyn path=",exuCPP.__file__)
        fileInfo=os.stat(exuCPP.__file__)
        exuDate = datetime.fromtimestamp(fileInfo.st_mtime) 
        exuDateStr = str(exuDate.year) + '-' + NumTo2digits(exuDate.month) + '-' + NumTo2digits(exuDate.day) + ' ' + NumTo2digits(exuDate.hour) + ':' + NumTo2digits(exuDate.minute) + ':' + NumTo2digits(exuDate.second)
    except:
        exuDateStr = 'unknown'
    
    
    
    #get all filenames in directory
    def GetFileNames(dirPath, fileEnding=''):
        fileNames = [f for f in listdir(dirPath) if isfile(join(dirPath, f))]
        if fileEnding != '':
            fileList = []
            for file in fileNames:
                if file.endswith(fileEnding):
                    fileList += [file]
            fileNames = fileList
            
        return fileNames
    
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
    localFileName = 'testExamplesLog_V'+exu.config.Version()+'_'+platformString
    
    logFileName = '../logs/examples/'+localFileName+'.txt'
    #never truncate an existing (committed) log by accident; see testRunnerTools.ResolveLogFile
    logFileName = testRunnerTools.ResolveLogFile(logFileName, allowOverwrite=overwriteLog)
    exu.SetWriteToFile(filename=logFileName, flagWriteToFile=True, flagAppend=False) #write all testSuite logs to files
    
    
    
    exu.Print('\n+++++++++++++++++++++++++++++++++++++++++++')
    exu.Print('+++++      EXUDYN TEST EXAMPLES       +++++')
    exu.Print('+++++++++++++++++++++++++++++++++++++++++++')
    exu.Print('EXUDYN version      = '+exu.config.Version())
    exu.Print('EXUDYN build date   = '+exuDateStr)
    exu.Print('architecture        = '+platform.architecture()[0])
    exu.Print('processor           = '+processorString)
    exu.Print('CPU                 = '+testRunnerTools.CpuInfoString())
    exu.Print('platform            = '+sys.platform)
    exu.Print('python version      = '+pythonVersion)
    #results depend on these; record them so runs can be compared across machines
    exu.Print(testRunnerTools.PackageVersionReport())
    exu.Print('test date (now)     = '+dateStr)
    exu.Print('+++++++++++++++++++++++++++++++++++++++++++')
    
    exu.config.printToConsole = writeToConsole #stop output from now on
    
    #testFileList = ['Examples/fourBarMechanism.py']
    testsFailed = [] #list of numbers containing the test numbers of failed tests
    
    
    
    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #run general test examples
    examplesTestSolList={}
    examplesTestErrorList={}
    
    #a few examples write with a plain path into 'solution/', as a user would; everything an
    #example writes THROUGH exudyn goes into its own directory
    os.makedirs('solution', exist_ok=True)

    timeStart= -time.time()
    dirPath = '../Examples/'
    listExamples = GetFileNames(dirPath, '.py')
    #and the examples in its subfolders, named with the subfolder: 'FurtherExamples/spotModel.py' (#2757)
    for subfolder in sorted(f for f in listdir(dirPath) if os.path.isdir(join(dirPath, f)) and f[0] not in '_.'):
        for root, dirs, files in os.walk(join(dirPath, subfolder)):
            dirs[:] = sorted(d for d in dirs if d[0] not in '_.')
            listExamples += [os.path.relpath(join(root, f), dirPath).replace(os.sep, '/')
                             for f in sorted(files) if f.endswith('.py')]
    # listExamples = [dirPath+'xExudynConfigSpecial.py']
    
    totalExamples = len(listExamples)   #including the ones that cannot be run, see below
    examplesFailed = []
    
    #the examples that cannot run at all are decided once, from the same rules the worker uses
    skipReasons = {}
    for exampleFileName in listExamples:
        with open(dirPath+exampleFileName, 'r', encoding='utf-8') as file:
            fileString = file.read()
        reason = testRunnerTools.ExampleSkipReason(exampleFileName, fileString)
        if reason != '':
            skipReasons[exampleFileName] = reason

    runList = [f for f in listExamples if f not in skipReasons]
    nSkipped = len(skipReasons)
    for exampleFileName in sorted(skipReasons):
        exu.Print('  ... "'+exampleFileName+'" skipped: '+skipReasons[exampleFileName])

    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #An example is an API CHECK: it has done its job once it has built its model, assembled it and
    #reached the solver, and the solver inside it stops after solverTimeout seconds anyway. The
    #process timeout is therefore short ON PURPOSE, and a timeout after the solver was reached
    #counts as a pass - only a timeout BEFORE it means the example hung while building
    #. Each example runs in its own interpreter, which is what allows
    #them to run in parallel and keeps a crashing example from taking the runner with it.
    exampleTimings = {}
    if runParallel:
        print('running '+str(len(runList))+' examples in parallel, timeout '
              +str(exampleTimeout)+' s', flush=True)
        exu.Print('running '+str(len(runList))+' examples in parallel, timeout '
                  +str(exampleTimeout)+' s, solver timeout '+str(solverTimeout)+' s')
        results = testRunnerTools.RunExamplesInParallel(
            runList, dirPath, '../logs/tmp/exampleOutput',
            numberOfProcesses=numberOfProcesses, timeout=exampleTimeout,
            solverTimeout=solverTimeout, quietMode=quietMode,
            compactProgress=quietMode) #a line rewritten in place when quiet (#2895)

        for testExamplesCnt, exampleFileName in enumerate(runList):
            r = results[exampleFileName]
            exu.Print('\n\n******************************************')
            exu.Print('EXAMPLE ' + str(testExamplesCnt) + ' ("' + exampleFileName + '"):')
            exu.Print('******************************************')
            exu.Print(r['output'])
            exampleTimings[exampleFileName] = r['seconds']
            if r['timedOut'] and not r['failed']:
                exu.Print('  ... timeout after '+str(exampleTimeout)
                          +' s while solving - counted as PASSED')
            elif r['failed']:
                exStr = ('*FAILED*: EXAMPLE ' + str(testExamplesCnt) + ' ("' + exampleFileName
                         + '") ' + ('hung before reaching the solver'
                                    if r['timedOut'] else 'terminated with an error'))
                exu.Print(exStr)
                examplesFailed += [str(testExamplesCnt)+' : '+exampleFileName]
    else:
        #the in-process run, kept for debugging a single example with the debugger attached
        exu.special.solver.timeout = solverTimeout
        for testExamplesCnt, exampleFileName in enumerate(runList):
            CloseAll() #plots
            exu.Print('\n\n******************************************')
            s = 'EXAMPLE ' + str(testExamplesCnt) + ' ("' + exampleFileName + '"):'
            exu.Print(s)
            if not writeToConsole:
                if quietMode and not writeFileNames:
                    testRunnerTools.ProgressLine(testExamplesCnt + 1, len(runList), 'example ' + exampleFileName)
                else:
                    print(s,flush=True)
            exu.Print('******************************************')

            with open(dirPath+exampleFileName, 'r', encoding='utf-8') as file:
                fileString = file.read()

            timeExample = -time.time()
            try:
                exec(testRunnerTools.PrepareExampleSource(fileString, quietMode), globals())
            except SystemExit: #AnimateModes ends the script with sys.exit(), see PrepareExampleSource
                print('(sys.exit)',flush=True, end='')
            except Exception as e:
                exStr = ('*FAILED*: EXAMPLE ' + str(testExamplesCnt) + ' ("' + exampleFileName
                         + '") raised exception:\n'+str(e))
                exu.Print(exStr)
                print(exStr, flush=True)
                examplesFailed += [str(testExamplesCnt)+' : '+exampleFileName]
            exampleTimings[exampleFileName] = timeExample + time.time()

    timeStart += time.time()
            
            
    exu.Print('\n')
    exu.config.printToConsole = True #final output always written
    exu.SetWriteToFile(filename=logFileName, flagWriteToFile=True, flagAppend=True) #write also to file (needed?)
    
    exu.Print('******************************************')
    exu.Print('\nTEST EXAMPLES SUMMARY:')
    
    #++++++++++++++++++++++++++++++++++
    exu.Print('time elapsed =',round(timeStart,3),'seconds') 

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if len(examplesFailed) == 0:
        exu.Print('ALL ' + str(totalExamples) + ' Examples SUCCESSFUL')
    else:
        exu.Print(str(len(examplesFailed)) + ' Examples OUT OF '+ str(totalExamples) + ' FAILED: ')
        for ef in examplesFailed:
            exu.Print('  Example ' + ef + ' FAILED')

    #the examples that cost the most; with a short timeout these are the ones that need it
    if len(exampleTimings) != 0:
        exu.Print('')
        exu.Print('slowest examples:')
        for exampleFileName in sorted(exampleTimings, key=exampleTimings.get, reverse=True)[:10]:
            exu.Print('  %-50s %6.2f s' % (exampleFileName, exampleTimings[exampleFileName]))
        exu.Print('')

    exu.Print('Skipped '+str(nSkipped)+' examples')
    exu.Print('******************************************')

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #the exit code (#2504). Until this existed the runner ALWAYS
    #returned 0, so nothing calling it - CI, a shell script, the exudev driver - could see a
    #failure without parsing the log. Known failures are excluded, the way the test suite excludes
    #UnresolvedOnLinux(), because an exit code that is red on every run says nothing.
    knownFailures = testRunnerTools.KnownExampleFailures()
    failedNames = [entry.split(' : ')[-1] for entry in examplesFailed]
    unexpectedFailures = [name for name in failedNames if name not in knownFailures]
    deadExclusions = [name for name in knownFailures if name not in failedNames]

    if len(failedNames) != len(unexpectedFailures):
        exu.Print('')
        exu.Print('known failures, excluded from the exit code:')
        for name in failedNames:
            if name in knownFailures:
                exu.Print('  ' + name + ' - ' + knownFailures[name])

    if len(deadExclusions) != 0:
        exu.Print('')
        exu.Print('dead exclusion(s) - listed in KnownExampleFailures() but they PASSED; remove them:')
        for name in deadExclusions:
            exu.Print('  ' + name)

    if len(unexpectedFailures) == 0:
        exu.Print('')
        exu.Print('PASSED: no unexpected example failed')
    else:
        exu.Print('')
        exu.Print('FAILED: ' + str(len(unexpectedFailures)) + ' unexpected example failure(s)')

exu.SetWriteToFile(filename='', flagWriteToFile=False, flagAppend=False) #stop writing to file, close file

if __name__ == '__main__':
    if useExitCode:
        sys.exit(1 if len(unexpectedFailures) != 0 else 0)

