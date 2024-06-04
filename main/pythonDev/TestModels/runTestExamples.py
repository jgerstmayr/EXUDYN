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
    #copyLog = False         #copy log to final TestSuiteLogs
    # if sys.version_info.major == 3 and sys.version_info.minor == 7:
    #     copyLog = True #for P3.7 tests always copy log to WorkingRelease
    if len(sys.argv) > 1:
        for i in range(len(sys.argv)-1):
            #print("arg", i+1, "=", sys.argv[i+1])
            if sys.argv[i+1] == '-quiet':
                quietMode = True
            else:
                print("ERROR in runTestExamples: unknown command line argument '"+sys.argv[i+1]+"'")
    
    if quietMode:
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
    
    # localFileName = 'Test for EXUDYN V'+exu.GetVersionString()+' (built:'+exuDateStr+'),'\
    #          +sys.platform+'-'+processorString+'-'+platform.architecture()[0]+',Python'\
    #          +pythonVersion+',date:'+dateStr+': '
    platformString = sys.platform+'-'+processorString+'-'+platform.architecture()[0]+'-P'+pythonVersionMain
    localFileName = 'testExamplesLog_V'+exu.GetVersionString()+'_'+platformString
    
    #logFileName = '../TestSuiteLogs/testSuiteLog_V'+exu.GetVersionString()+'_'+platformString+'.txt'
    logFileName = '../TestExamplesLogs/'+localFileName+'.txt'
    exu.SetWriteToFile(filename=logFileName, flagWriteToFile=True, flagAppend=False) #write all testSuite logs to files
    
    
    
    exu.Print('\n+++++++++++++++++++++++++++++++++++++++++++')
    exu.Print('+++++      EXUDYN TEST EXAMPLES       +++++')
    exu.Print('+++++++++++++++++++++++++++++++++++++++++++')
    exu.Print('EXUDYN version      = '+exu.GetVersionString())
    exu.Print('EXUDYN build date   = '+exuDateStr)
    exu.Print('architecture        = '+platform.architecture()[0])
    exu.Print('processor           = '+processorString)
    exu.Print('platform            = '+sys.platform)
    exu.Print('python version      = '+pythonVersion)
    exu.Print('test date (now)     = '+dateStr)
    exu.Print('+++++++++++++++++++++++++++++++++++++++++++')
    
    exu.SetWriteToConsole(writeToConsole) #stop output from now on
    
    #testFileList = ['Examples/fourBarMechanism.py']
    testsFailed = [] #list of numbers containing the test numbers of failed tests
    
    
    
    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #run general test examples
    examplesTestSolList={}
    examplesTestErrorList={}
    
    nSkipped = 0
    

    exu.special.solver.timeout = 1

    timeStart= -time.time()
    dirPath = '../Examples/'
    listExamples = GetFileNames(dirPath, '.py')
    
    totalExamples = len(listExamples)
    examplesFailed = []
    fastMode = False #skip verbose and slow examples
    
    testExamplesCnt = 0
    for exampleFileName in listExamples:
        CloseAll() #plots
        name = exampleFileName #.split('.')[0] #without '.py'
        exu.Print('\n\n******************************************')
        s = 'EXAMPLE ' + str(testExamplesCnt) + ' ("' + exampleFileName + '"):'
        #print('******************\n'+s+'\n****************\n')
        exu.Print(s)
        if not writeToConsole:
            if quietMode and not writeFileNames: 
                print(' Ex'+str(testExamplesCnt),end='',sep='',flush=True)
            else:
                print(s,flush=True)
        exu.Print('******************************************')

        thisCnt = testExamplesCnt
        testExamplesCnt += 1

        # if thisCnt < 41 or thisCnt > 41:
        #     print('skip',thisCnt)
        #     continue

        
        
        with open(dirPath+exampleFileName, 'r', encoding='utf-8') as file:
            fileString = file.read()

        #examples that cannot be tested:
        if (
            'nMassOscillatorEigenmodes' in exampleFileName
            or 'multiprocessingTest' in exampleFileName    #uses multiprocessing directly, incompatible with exec(...)
            or 'netgenSTLtest' in exampleFileName          #gives netgen specific error, not clear why this occurs with exec(...) but not when directly executing the file.
            #or 'dispyParameterVariationExample' in exampleFileName #hangs, dispy not available; but runs in regular mode ...
            #or 'GeneticOptimization' in fileString
            #or 'ParameterVariation' in fileString
            or 'stable_baselines3' in fileString
            or 'massSpringFrictionInteractive' in exampleFileName #cannot work, interactive
            or 'nMassOscillatorInteractive' in exampleFileName    #cannot work, interactive
            or 'performanceMultiThreadingNG' in exampleFileName
            or 'rospy' in fileString                #not possible without ros installed
            or 'TCPIP' in fileString                #not possible without MATLAB client
            #or 'testData/' in fileString           #numpy file
            #or 'GeneralContact' in fileString
            #or 'mbs.SolveStatic' in fileString
            ):
            s = '  ... "'+exampleFileName+'" skipped'
            exu.Print(s)
            if not writeToConsole and not writeFileNames: 
                if quietMode: 
                    print('not',end='',sep='')
                else:
                    print(s)
            nSkipped += 1
            continue

        if fastMode:
            if ('humanRobotInteraction' in exampleFileName #lots of warnings
                # or 'ngsolve' in fileString
                #or 'particlesSilo' in exampleFileName
                #or 'NGsolvePistonEngine' in exampleFileName
                or 'massSpringFrictionInteractive' in exampleFileName
                ):
                s = '  ... "'+exampleFileName+'" skipped'
                exu.Print(s)
                if not writeToConsole and not writeFileNames:
                    if quietMode: 
                        print('not',end='',sep='')
                    else:
                        print(s)
                nSkipped += 1
                continue


        if 'mbs.SolveStatic' in fileString:
            exu.special.solver.timeout = 4


        #skip everything after plot sensor
        fPlot = fileString.find('mbs.PlotSensor(')
        if fPlot != -1:
            fileString = fileString[:fPlot]+'pass\n'
        
        fileString = fileString.replace('mbs.SolutionViewer(', 'pass #mbs.SolutionViewer(')
        fileString = fileString.replace('exu.StartRenderer(', 'pass #exu.StartRenderer(')
        fileString = fileString.replace('exu.StopRenderer(', 'pass #exu.StopRenderer(')
        fileString = fileString.replace('SC.WaitForRenderEngineStopFlag()', 'pass')
        fileString = fileString.replace('mbs.WaitForUserToContinue()', 'pass')
        fileString = fileString.replace('useRenderer=True', 'useRenderer=False') #InverseKinematicsNumericalExample.py
        fileString = fileString.replace('useGraphics = True', 'useGraphics = False') 
        
        fileString = fileString.replace('plt.show()', '') 
        fileString = fileString.replace('plt.tight_layout()', '') 
        fileString = fileString.replace('ClearWorkspace()', '') 
        fileString = fileString.replace('(verbose=True)', '(verbose=False)') #ComputeSystemDegreeOfFreedom 

        #massSpringFrictionInteractive.py:
        fileString = fileString.replace('def SimulationUF(mbs, dialog):', 
                                        'def SimulationUF(mbs, dialog):\n    if mbs.systemData.GetTime() > 0.1: dialog.OnQuit()') #InverseKinematicsNumericalExample.py

        fileString = fileString.replace('InteractiveDialog(', 'if False: InteractiveDialog(') 
        
        fileString = fileString.replace('AnimateModes(', 'import sys;sys.exit();AnimateModes(') #InverseKinematicsNumericalExample.py

        fileString = fileString.replace('print(', 'exu.Print(') #may fail ...

        if quietMode:
            fileString = fileString.replace('verbose = True', 'verbose = False') 
            fileString = fileString.replace('showProgress = True', 'showProgress = False') 


        if 'exudyn.processing' in fileString and ('useMultiProcessing = True' in fileString or 
                                                  'useMultiProcessing=True' in fileString):
            if not quietMode: print('replace useMultiProcessing=True')
            fileString = fileString.replace('useMultiProcessing=True','useMultiProcessing=False')
            fileString = fileString.replace('useMultiProcessing = True','useMultiProcessing=False')

        if 'GeneticOptimization' in fileString:
            if not quietMode: print('replace numberOfGenerations and populationSize')
            fileString = fileString.replace('numberOfGenerations','numberOfGenerations=1,#')
            fileString = fileString.replace('populationSize ','populationSize = 10,#')
        
        try:
            exec(fileString, globals())
        except Exception as e:
            exStr = '*FAILED*: EXAMPLE ' + str(thisCnt) + ' ("' + exampleFileName + '") raised exception:\n'+str(e)
            exu.Print(exStr)
            print(exStr, flush=True)
            examplesFailed += [str(thisCnt)+' : '+exampleFileName]
        # finally:
            

    timeStart += time.time()
            
            
    exu.Print('\n')
    exu.SetWriteToConsole(True) #final output always written
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
            exu.Print('  TestModel ' + ef + ' FAILED')

    exu.Print('Skipped '+str(nSkipped)+' examples')
    exu.Print('******************************************')

exu.SetWriteToFile(filename='', flagWriteToFile=False, flagAppend=False) #stop writing to file, close file

