#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test for exudyn.misc.resultsMonitor: reading the four results file types, the
#           incremental reading of a file that is still growing, the column selection, and the
#           return codes of the command line. Everything runs with once=True, so no update loop
#           is entered and no window is opened.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-19
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *

useGraphics = True #without test
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
try: #only if called from test suite
    from modelUnitTests import exudynTestGlobals #for globally storing test results
    useGraphics = exudynTestGlobals.useGraphics
except:
    class ExudynTestGlobals:
        pass
    exudynTestGlobals = ExudynTestGlobals()
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import numpy as np
import matplotlib.pyplot as plt

from exudyn.basicUtilities import OutputFilePath
from exudyn.processing import ParameterVariation, GeneticOptimization
from exudyn.misc.resultsMonitor import (MonitorResults, ResultsFileColumns,
                                        ReadResultsFileHeader, FindResultsFiles, Main)

testSolution = 1

def Check(name, condition, info=''):
    """count one check; a failed check sets the test result to 0 and says which one it was"""
    global testSolution
    if not condition:
        testSolution = 0
        exu.Print('FAILED check: ' + name + (' (' + str(info) + ')' if info != '' else ''))
    return condition

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create the results files: a sensor file and a solution file from a small model, and a
#parameter variation and a genetic optimization file from a trivial objective function
sensorFile = 'solution/resultsMonitorSensor.txt'
solutionFile = 'solution/resultsMonitorSolution.txt'
variationFile = 'solution/resultsMonitorVariation.txt'
geneticFile = 'solution/resultsMonitorGenetic.txt'

SC = exu.SystemContainer()
mbs = SC.AddSystem()
node = mbs.AddNode(NodePoint(referenceCoordinates=[0,0,0]))
mbs.AddObject(MassPoint(physicsMass=1, nodeNumber=node))
marker = mbs.AddMarker(MarkerNodePosition(nodeNumber=node))
mbs.AddLoad(Force(markerNumber=marker, loadVector=[1,0,0]))
mbs.AddSensor(SensorNode(nodeNumber=node, fileName=sensorFile, writeToFile=True,
                         outputVariableType=exu.OutputVariableType.Position))
mbs.Assemble()

simulationSettings = exu.SimulationSettings()
simulationSettings.timeIntegration.numberOfSteps = 50
simulationSettings.timeIntegration.endTime = 1
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.coordinatesSolutionFileName = solutionFile
simulationSettings.displayComputationTime = False
simulationSettings.timeIntegration.verboseMode = 0
mbs.SolveDynamic(simulationSettings)

def ObjectiveFunction(parameterSet):
    return (parameterSet['a']-1.5)**2 + (parameterSet['b']-0.5)**2 + 0.01

ParameterVariation(parameterFunction=ObjectiveFunction, parameters={'a':(0,3,4),'b':(0,1,3)},
                   resultsFile=variationFile, showProgress=False)
GeneticOptimization(objectiveFunction=ObjectiveFunction, parameters={'a':(0,3),'b':(0,1)},
                    numberOfGenerations=3, populationSize=8, elitistRatio=0.1,
                    randomizerInitialization=42, distanceFactor=0.1,
                    resultsFile=geneticFile, showProgress=False)

#the monitor reads files by their name, so the output directory has to be applied here
sensorPath = OutputFilePath(sensorFile)
solutionPath = OutputFilePath(solutionFile)
variationPath = OutputFilePath(variationFile)
geneticPath = OutputFilePath(geneticFile)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#header and column recognition of all four file types
for path, fileType in [[sensorPath,'sensor'], [solutionPath,'solution'],
                       [variationPath,'parameterVariation'], [geneticPath,'geneticOptimization']]:
    header = ReadResultsFileHeader(path)
    Check('file type of ' + os.path.basename(path), header.get('type','') == fileType,
          header.get('type',''))

columns = ResultsFileColumns(sensorPath)
Check('sensor columns', columns == ['time','Position0','Position1','Position2'], columns)
Check('optimization columns', ResultsFileColumns(geneticPath)[0:2] == ['globalIndex','value'])

found = FindResultsFiles([os.path.dirname(sensorPath)])
foundNames = [os.path.basename(info['fileName']) for info in found]
expectedNames = [os.path.basename(path) for path in [sensorPath, solutionPath,
                                                     variationPath, geneticPath]]
Check('all four files are found',                   #the directory may hold more, from a run before
      all(name in foundNames for name in expectedNames), foundNames)
Check('newest file first', all(found[i]['modified'] >= found[i+1]['modified']
                               for i in range(len(found)-1)))

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the drawn data must be the data in the file
monitor = MonitorResults(sensorPath, once=True, showPanel=False, useSettingsFile=False)
Check('sensor file monitored', monitor is not None)
data = np.loadtxt(sensorPath, comments='#', delimiter=',')
Check('one curve per column except time', len(monitor.lineList) == 3, len(monitor.lineList))
Check('x data is time', np.allclose(monitor.lineList[0].get_xdata(), data[:,0]))
Check('y data is column 1', np.allclose(monitor.lineList[0].get_ydata(), data[:,1]))

#selected columns only, and a logarithmic axis shows absolute values
monitor = MonitorResults(sensorPath, xColumns=[0], yColumns=[1], logY=True, once=True,
                         showPanel=False, useSettingsFile=False)
Check('one selected curve', len(monitor.lineList) == 1)
Check('log y uses absolute values', np.allclose(monitor.lineList[0].get_ydata(), abs(data[:,1])))

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the file is read incrementally: appending rows must add exactly those rows, an incomplete last
#line must wait for its newline, and a file that got shorter must be read again from the start
lines = open(sensorPath).read().split('\n')
headerLines = [line for line in lines if line.startswith('#')]
dataLines = [line for line in lines if line != '' and not line.startswith('#')]
growingPath = OutputFilePath('solution/resultsMonitorGrowing.txt')
with open(growingPath,'w') as file:
    file.write('\n'.join(headerLines) + '\n' + '\n'.join(dataLines[0:10]) + '\n')

monitor = MonitorResults(growingPath, once=True, showPanel=False, useSettingsFile=False)
Check('rows of the partial file', monitor.data.NumberOfRows() == 10, monitor.data.NumberOfRows())
with open(growingPath,'a') as file:                     #10 complete rows and one torn row
    file.write('\n'.join(dataLines[10:20]) + '\n' + dataLines[20][0:10])
Check('rows added', monitor.UpdatePlot() == 10)
Check('the torn row is not used yet', monitor.data.NumberOfRows() == 20)
with open(growingPath,'a') as file:                     #complete it and add the rest
    file.write(dataLines[20][10:] + '\n' + '\n'.join(dataLines[21:]) + '\n')
monitor.UpdatePlot()
Check('all rows after completion', monitor.data.NumberOfRows() == len(dataLines),
      monitor.data.NumberOfRows())
Check('drawn points follow the file', len(monitor.lineList[0].get_xdata()) == len(dataLines))
with open(growingPath,'w') as file:                     #the next run overwrites the file
    file.write('\n'.join(headerLines) + '\n' + '\n'.join(dataLines[0:3]) + '\n')
monitor.UpdatePlot()
Check('buffer reset after overwrite', monitor.data.NumberOfRows() == 3,
      monitor.data.NumberOfRows())

#an update that finds no new data must leave the canvas untouched; redrawing an unchanged figure
#once per second is what made the window flicker
monitor.figure.canvas.draw()
Check('no redraw without new data', monitor.UpdatePlot() == 0 and not monitor.figure.stale)
#the margins are recomputed at every draw, so the axis labels survive a smaller window
Check('constrained layout', 'constrained' in
      str(type(monitor.figure.get_layout_engine())).lower(), monitor.figure.get_layout_engine())

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#parameter variation: one curve per parameter, and one curve per variation with colorVariations
monitor = MonitorResults(variationPath, once=True, showPanel=False, useSettingsFile=False)
Check('one plot per varied parameter', len(monitor.lineList) == 2, len(monitor.lineList))
monitor = MonitorResults(variationPath, colorVariations=True, once=True, showPanel=False,
                         useSettingsFile=False)
Check('one curve per variation of b', len(monitor.lineList) == 3, len(monitor.lineList))
monitor = MonitorResults(geneticPath, once=True, showPanel=False, useSettingsFile=False)
Check('genetic optimization monitored', monitor is not None and len(monitor.lineList) == 2)

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the command line reports errors with a return code instead of raising
solutionDirectory = os.path.dirname(sensorPath)
Check('CLI plots a file', Main([sensorPath,'--once','--no-panel','--no-settings']) == 0)
Check('CLI lists the columns', Main([sensorPath,'--list-columns','--no-settings']) == 0)
Check('CLI lists the files', Main(['--list-files','--no-settings',
                                   '--dir',solutionDirectory]) == 0)
Check('CLI takes the newest file', Main(['--last','--once','--no-panel','--no-settings',
                                         '--dir',solutionDirectory]) == 0)
Check('missing file reported', Main(['fileDoesNotExist.txt','--once','--no-settings']) == 1)
Check('column out of range reported',
      Main([sensorPath,'--y-cols','99','--once','--no-settings']) == 1)
notResultsPath = OutputFilePath('solution/resultsMonitorNotAResultsFile.txt')
with open(notResultsPath,'w') as file:
    file.write('some other file\n1,2,3\n')
Check('wrong file type reported', Main([notResultsPath,'--once','--no-settings']) == 1)
Check('old option names still work',
      Main([sensorPath,'-logy','-ycols','1','-update','0.5','--once','--no-settings']) == 0)

plt.close('all') #the figures were never shown, but they must not pile up for the next test model

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
exu.Print('\nsolution of resultsMonitorTest (should be 1)=',testSolution)
exudynTestGlobals.testResult = testSolution
