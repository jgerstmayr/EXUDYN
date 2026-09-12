#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN helper file
#
# Details:  Use this script to continuously visualize results;
#           command line Usage (cmd): python resultsMonitor.py results.txt -updateTime=0.2 -plotMode=lines -logx -logy
#
# Author:   Johannes Gerstmayr 
# Date:     2021-01-14
# Notes:    Parallel processing, which requires multiprocessing library, can lead to considerable speedup (measured speedup factor > 50 on 80 core machine). The progess bar during multiprocessing requires the library tqdm.
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys

doDebug = False

if doDebug:
    print("number of args=",len(sys.argv))
    print("args=",sys.argv)
argList = sys.argv
# for i in sys.argv:
#     print("arg=",i)

import matplotlib.pyplot as plt
import numpy as np
from exudyn.plot import ParseOutputFileHeader
from exudyn.advancedUtilities import PlotLineCode, IsEmptyList
from exudyn.processing import SingleIndex2SubIndices

listMarkerStyles = ['. ', '+', 'x ', 'v ', '^ ', '< ', '> ', '* ', 'd ', 'D', 's', 'X ', 'P', 'o', 'p ', 'h ', 'H ']

x = np.linspace(0, 6*np.pi, 100)
y = np.sin(x)

plt.ion() #interactive mode on



#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
#default values
updatePeriod = 1. #one second update rate
lineColor = 'b'
lineStyle = '-'
logX = False
logY = False
colorVariations = False
sizeXinInches = 5
sizeYinInches = 5
xColumns = [] #which columns to read data for x-axis
yColumns = [] #which columns to read data for y-axis
xLabels = [] #labels per plot
yLabels = [] #labels per plot
addMarker = False #add marker to last point in curve
fileName = '' #must be set
variations = []

#parse command line arguments:
runLoader = True
nArgs = len(argList) #first argument = python file
if nArgs < 2:
    print("ERROR in resultsMonitor: filename missing\nuse option -h for help")
    runLoader = False
else:
    if argList[1] == '-h':
        print('usage for resultsMonitor:')
        print('  python resultsMonitor.py file.txt')
        print('options:')
        print('  -xcols i,j,..: comma-separated columns (NO SPACES!) to be plotted on x-axis')
        print('  -ycols i,j,..: comma-separated columns (NO SPACES!) to be plotted on y-axis')
        print('  -logx: use log scale for x-axis')
        print('  -logy: use log scale for y-axis')
        print('  -colorVariations: for parameter variation file just plot first axis, add different colors for variations of other axes (limited to 28 colors)')
        print('  -variations start,end: comma-separated values (NO SPACES!) for start and end index of colorVariations; -variations 0,2 plots variation 0 and 1')
        print('  -addMarker: add marker (filled red circle) to last point in plot')
        print('  -sizex float: float = x-size of one subplot in inches (default=5)')
        print('  -sizey float: float = y-size of one subplot in inches (default=5)')
        print('  -update float: float = update period in seconds (default: 1)')
        print('  -color char: char = line color code according to pyplot, default=b (blue)')
        print('  -style char: char = line symbol according to pyplot, default="-"')
        print('example: (to be called from windows Anaconda prompt or in linux terminal in the directory where file.txt lies)')
        print('  python resultsMonitor.py file.txt -logy -xcols 0,1 -ycols 2,3 -update 0.2')
        
        runLoader = False
    else:
        fileName = argList[1]
        if doDebug:
            print('fileName=',fileName)
        i = 2
        while i < nArgs:
            if argList[i] == '-xcols':
                strList = argList[i+1].split(',')
                xColumns = []
                for s in strList:
                    xColumns += [int(s)]
                i=i+1
            elif argList[i] == '-ycols':
                strList = argList[i+1].split(',')
                yColumns = []
                for s in strList:
                    yColumns += [int(s)]
                i=i+1
            elif argList[i] == '-logx':
                logX = True
            elif argList[i] == '-logy':
                logY = True
            elif argList[i] == '-colorVariations':
                colorVariations = True                
            elif argList[i] == '-variations':
                strList = argList[i+1].split(',')
                for s in strList:
                    variations += [int(s)]
                i=i+1
            elif argList[i] == '-addMarker':
                addMarker = True
            elif argList[i] == '-sizex':
                sizeXinInches = float(argList[i+1]); i=i+1
            elif argList[i] == '-sizey':
                sizeYinInches = float(argList[i+1]); i=i+1
            elif argList[i] == '-update':
                updatePeriod = float(argList[i+1]); i=i+1
            elif argList[i] == '-color':
                lineColor = argList[i+1]; i=i+1
            elif argList[i] == '-style':
                lineStyle = argList[i+1]; i=i+1
            else:
                print("WARNING: unknown argument '"+argList[i]+"' ignored")
                
            i += 1
            
if runLoader:
    #%%+++++++++++++++++++++++++++++
    #check if file is ready
    fileReady = False
    lines = []
    while not fileReady:
        file = open(fileName, 'r')
        lines = file.readlines()
        file.close()
        if len(lines) >= 1:
            hasComment = False
            hasData = False
            for line in lines:
                if line[0] == '#':
                    hasComment = True
                elif line[0] != '#':
                    hasData = True
            if hasComment and hasData:
                fileReady = True
    
    #%%+++++++++++++++++++++++++++++
    #now we can interpret data

    header = ParseOutputFileHeader(lines)
    #print('header=',header)
    if header['type'] == 'geneticOptimization' or header['type'] == 'parameterVariation':
        lineStyle = '.'
        colValue = -1
        nColumnsProcessed = len(header['columns'])
        for i in range(nColumnsProcessed):
            col = header['columns'][i]
            if col != 'globalIndex' and col != 'computationIndex' and col != 'value':
                xColumns += [i]
                xLabels += [col]
                if header['type'] == 'geneticOptimization':
                    yLabels += ['fitness']
                elif header['type'] == 'parameterVariation':
                    yLabels += ['result']
            elif col == 'value':
                colValue = i #fitness value is always second column in genetic optimization and parameter variation
        yColumns = [colValue]*len(xColumns)
    elif header['type'] == 'sensor' or header['type'] == 'solution':
        if len(xColumns) == 0: #automatically choose all columns
            for i in range(len(header['columns'])-1): #exclude time
                yColumns += [i+1] #exclude time
                xColumns += [0] #automatically choose time
        if len(xColumns) != len(yColumns):
            print('ERROR in resultsLoader: size of xColumns not equal to yColumns! terminating...')
            runLoader = False
        else:
            for i in range(len(xColumns)):
                xLabels += [header['columns'][xColumns[i]]]
                yLabels += [header['columns'][yColumns[i]]]
    else:
        print('ERROR in resultsLoader: unknown file header! terminating...')
        runLoader = False
    
    if len(xColumns)*len(yColumns)*len(xLabels)*len(yLabels) == 0:
        print('ERROR in resultsLoader: no valid columns given! terminating...')
        runLoader = False

    if doDebug:
        print('xColumns=',xColumns)
        print('yColumns=',yColumns)
        print('xLabels=',xLabels)
        print('yLabels=',yLabels)
    
if runLoader:    
    #generate subplots:
    nPlots = len(xColumns)
    nVariations = 1 #regular case
    variableRanges = header['variableRanges']

    maxCols = int(np.sqrt(nPlots)+1)
    if nPlots == 3:
        maxCols = 3
    if nPlots == 4:
        maxCols = 2
    nRows = int(np.ceil(nPlots/maxCols))
    nCols = nPlots
    if nCols > maxCols:
        nCols = maxCols
    # print("nRows=",nRows,", nCols=", nCols)
    if colorVariations:
        # print('variableRanges=',variableRanges)
        for rangeI in variableRanges[1:]:
            nVariations *= rangeI[2] #this is the number of variations
        # print('plot',nVariations,'variations')
        nPlots=nVariations
        if IsEmptyList(variations):
            variations = [0,nPlots]
        else:
            nPlots = variations[1]-variations[0]
        nCols=1 #everything in one plot
        nRows=1
    
    fig = plt.figure('Results monitor')
    fig.dpi = 100 #in terminal, initially set to 200
    fig.tight_layout()
    fig.set_size_inches(nCols*sizeXinInches, nRows*sizeYinInches, forward=True)
    
    axList = []
    lineList = []
    markerList = []
    for i in range(nPlots):
        if not colorVariations or i==0:
            ax = fig.add_subplot(nRows,nCols,i+1)
            
        ax.grid(True, 'major', 'both')
        axList += [ax]
        if logX:
            ax.set_xscale('log')
        if logY:
            ax.set_yscale('log')
        lineColorStyle=lineColor+lineStyle
        if colorVariations:
            lineColorStyle = PlotLineCode(i)
            # print(listMarkerStyles[i%7][0])
            lineColorStyle = lineColorStyle[0]+listMarkerStyles[int(i/7)][0]

        line, = ax.plot(0.1,0.1, lineColorStyle)
        lineList += [line] #empty plot
        
        if addMarker: #red circle at end of line
            markerColorCode = 'ro'
            line, = ax.plot(0.1,0.1, markerColorCode) #red circle
            markerList += [line] #empty plot
            
    
    finished = False #finish when plot window is closed ...
    if len(xColumns) == 0:
        finished = True
    
    #main updating loop, until user closes window:
    firstRun = True
    while not finished:
        #data = np.loadtxt(fileName, delimiter=',') does not work with inconsistent data
        data = np.genfromtxt(fileName,comments='#',delimiter=',',invalid_raise=False)
        if data.ndim == 2: #if only one line, ndim=1 and data[:,xColumns[i]] crashes!
            for i in range(nPlots):
                ax = axList[i]
                # ax.clear() #slow
                # ax.plot(data[:,xColumns[i]], data[:,yColumns[i]], lineColor+lineStyle) 
                if not colorVariations:
                    dataX = data[:,xColumns[i]]
                    dataY = data[:,yColumns[i]]
                else:
                    dataRows = data.shape[0]
                    # print('rows= ',dataRows)
                    ii = variations[0] + i
                    nRanges=[]
                    for kRange in variableRanges:
                        nRanges += [kRange[2]]

                    n0 = nRanges[0]
                    nRest = np.array(nRanges[1:]).prod() #remaining range
                    # print('n ranges=', nRanges)
                    # print('variableRanges=',variableRanges)
                    subIndices = SingleIndex2SubIndices(ii, nRanges[1:]) #i runs in subindices
                    subValues = []
                    for j in range(len(subIndices)):
                        valueStart = variableRanges[j+1][0]
                        valueEnd = variableRanges[j+1][1]
                        value = valueStart
                        if nRanges[j+1] > 1:
                            value += (valueEnd-valueStart)*(subIndices[j]/(nRanges[j+1]-1))
                        subValues += [value]
                    # print('subIndices=', subIndices)
                    jIndex0 = np.arange(n0)*nRest+ii
                    # print(jIndex0)
                    #check if data is available:
                    if dataRows < max(jIndex0):
                        jIndex = []
                        for j in jIndex0:
                            if j < dataRows:
                                jIndex += [j]
                        #print('new indices=',jIndex)
                        jIndex = np.array(jIndex)
                    else:
                        jIndex = jIndex0
                    
                    if len(jIndex) == 0:
                        continue
                    
                    dataX = data[jIndex,xColumns[0]]
                    dataY = data[jIndex,yColumns[0]]
                    
                    # ax.set_label('asdf')
                    # ax.legend()
                    sLabel = 'var'+str(ii)+':'
                    for kRange in range(len(nRanges)-1):
                        #print('kRange=',kRange)
                        colName = header['columns'][kRange+1+2] #offset 1 to take first sub index; offset 2 for globalindex and value
                        sLabel += colName[0:5]+str(round(subValues[kRange],3))+' '
                        #sLabel += colName[0:5]+str(subIndices[kRange])
                        
                    lineList[i].set_label(sLabel)
                    ax.legend()


                if logX:
                    dataX = abs(dataX)
                if logY:
                    dataY = abs(dataY)
                lineList[i].set_data(dataX, dataY) 

                if addMarker:
                    markerList[i].set_data(dataX[-1], dataY[-1])

                if not colorVariations or i==0:
                    ax.set_xlabel(xLabels[i])
                    ax.set_ylabel(yLabels[i])
                    
                #next two commands to zoom all ...:
                ax.relim()
                ax.autoscale_view()
                
            fig.canvas.draw()
            fig.canvas.flush_events()
            #print("dpi=", fig.dpi)
    
        plt.pause(updatePeriod)
        if not plt.fignum_exists(fig.number):
            finished = True
        #firstRun = False
    
    print("Plot window closed by user.")
    
    