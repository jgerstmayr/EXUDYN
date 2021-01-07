#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details: 	Plot utility functions based on matplotlib, including plotting of sensors and FFT.
#
# Author:   Johannes Gerstmayr
# Date:     2020-09-16 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# Notes:	For a list of plot colors useful for matplotlib, see also utilities.PlotLineCode(...)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import matplotlib.pyplot as plt
import numpy as np #for loading
import matplotlib.ticker as ticker
import exudyn #for sensor index

#from exudyn.utilities import PlotLineCode

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: helper for matplotlib in order to easily visualize sensor output
#**input: 
#  mbs: must be a valid MainSystem (mbs)
#  sensorNumbers: consists of one or a list of sensor numbers (type SensorIndex) as returned by the mbs function AddSensor(...)
#  components: consists of one or a list of components according to the component of the sensor to be plotted;
#  *kwargs: additional options, e.g.: 
#        xLabel -> string for text at x-axis (otherwise time is used)
#        yLabel -> string for text at y-axis (otherwise outputvalues are used)
#        fontSize -> default = 16, which is a little bit larger than default (12)
#**output: plots the sensor data
#**example: 
#  s0=mbs.AddSensor(SensorNode(nodeNumber=0))
#  s1=mbs.AddSensor(SensorNode(nodeNumber=1))
#  Plot(mbs, s0, 0)
#  Plot(mbs, sensorNumbers=[s0,s1], components=[0,2], xlabel='time in seconds')
def PlotSensor(mbs, sensorNumbers, components=0, **kwargs):
    #could also be imported from exudyn.utilities import PlotLineCode
    CC = ['k-','g-','b-','r-','c-','m-','y-','k:','g:','b:','r:','c:','m:','y:','k--','g--','b--','r--','c--','m--','y--','k-.','g-.','b-.','r-.','c-.','m-.','y-.']
    
    if isinstance(sensorNumbers,list):
        sensorList = sensorNumbers
    else:
        sensorList = [sensorNumbers]

    if isinstance(components,list):
        componentList = components
    else:
        componentList = [components]

    if len(componentList) == 1 and len(sensorList) != 1:
        componentList = componentList*len(sensorList)
        
    if len(componentList) !=  len(sensorList):
        raise ValueError('ERROR in PlotSensor: number of sensors and number components must be same or number of components is 1')
        
    #increase font size as default is rather small
    if 'fontSize' in kwargs:
        plt.rcParams.update({'font.size': kwargs['fontSize']})
    plt.rcParams.update({'font.size': 16})

    #check if all sensor outputvariables are the same
    checkStr = ''
    allVariablesSame = True
    for i in range(len(sensorList)):
        sensorNumber = sensorList[i]
        component = componentList[i]
        sensorDict = mbs.GetSensor(sensorNumber)
        variableStr = ''
        if 'outputVariableType' in sensorDict:
            variable = sensorDict['outputVariableType']
            variableStr = str(variable).replace('OutputVariableType.','')
        elif sensorDict['sensorType'] == 'Load':
            variableStr = 'Load'
        
        if i == 0:
            checkStr = variableStr
        elif checkStr != variableStr:
            allVariablesSame = False

    yLabel = ''
    if allVariablesSame:
        yLabel = checkStr

    if 'yLabel' in kwargs:
        yLabel = kwargs['yLabel']

    for i in range(len(sensorList)):
        sensorNumber = sensorList[i]
        component = componentList[i]

        if not isinstance(sensorNumber, exudyn.SensorIndex):
            raise ValueError('ERROR in PlotSensor: *args must contain valid sensor numbers')

        #retrieve sensor information:
        sensorDict = mbs.GetSensor(sensorNumber)
        sensorFileName = sensorDict['fileName']
        sensorName = sensorDict['name']
        if 'outputVariableType' in sensorDict:
            variable = sensorDict['outputVariableType']
            variableStr = str(variable).replace('OutputVariableType.','')
        elif sensorDict['sensorType'] == 'Load':
            variableStr = 'Load'

        #now load sensor file:
        data = np.loadtxt(sensorFileName, comments='#', delimiter=',')

        #select color and style for sensor
        col = 'k-.'
        if i < len(CC):
            col = CC[i]
        
        #extract additional paramters
        xLabel = 'time (s)'
        
        if 'xLabel' in kwargs:
            xLabel = kwargs['xLabel']
        if not 'yLabel' in kwargs and not allVariablesSame:
            yLabel += variableStr
            if i < len(sensorList)-1:
                yLabel += ', '
        
        #plot according column (component+1)!:
        sComponent=''
        if len(componentList) != 1:
            sComponent = str(component)
        plt.plot(data[:,0], data[:,component+1], col, label=sensorName+', '+variableStr+sComponent) #numerical solution
        plt.xlabel(xLabel)
        plt.ylabel(yLabel)
        ax=plt.gca() # get current axes
        ax.grid(True, 'major', 'both')
        ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
        ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
        plt.legend() #show labels as legend
        plt.tight_layout()
        plt.show() 
        
    
#**function: plot fft spectrum of signal
#**input: 
#   frequency:  frequency vector (Hz, if time is in SECONDS)   
#   data:       magnitude or phase as returned by ComputeFFT() in exudyn.signal
#   xLabel:     label for x-axis, default=frequency
#   yLabel:     label for y-axis, default=magnitude
#   label:      either empty string ('') or name used in legend
#   freqStart:  starting range for frequency
#   freqEnd:    end of range for frequency; if freqEnd==-1 (default), the total range is plotted
#   logScaleX:  use log scale for x-axis
#   logScaleY:  use log scale for y-axis
#   majorGrid:  if True, plot major grid with solid line 
#   minorGrid:  if True, plot minor grid with dotted line 
#**output: creates plot and returns plot (plt) handle
def PlotFFT(frequency, data, 
               xLabel='frequency', yLabel='magnitude', 
               label = '',
               freqStart = 0, freqEnd = -1, 
               logScaleX = True, logScaleY = True,
               majorGrid = True, minorGrid = True):
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker

    indStart = 0
    indEnd = len(data)
    for i in range(len(frequency)):
        if frequency[i] <= freqStart:
            indStart = i
        if frequency[i] <= freqEnd:
            indEnd = i

    #print("fft ind=", indStart, indEnd)
    if len(label) != 0:
        plt.plot(frequency[indStart:indEnd], data[indStart:indEnd], label=label)
        plt.legend() #show labels as legend
    else:
        plt.plot(frequency[indStart:indEnd], data[indStart:indEnd])
    plt.xlabel(xLabel)
    plt.ylabel(yLabel)
    ax=plt.gca() # get current axes
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
    xScale = 'linear'
    yScale = 'linear'
    if logScaleX:
        plt.xscale('log')
    if logScaleY:
        plt.yscale('log')
    ax.grid(b=True, which='major', color='k', linestyle='-')
    ax.grid(b=True, which='minor', color='k', linestyle=':')
    ax.minorticks_on()

    plt.tight_layout()
    plt.show() 

    return plt
    