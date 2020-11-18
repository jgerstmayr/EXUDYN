# plot utilities for Exudyn
"""
Created on Wed Sept 16 11:00:00 2020

@author: Johannes Gerstmayr

goal: makes plotting a sensor value very easy and short
"""


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
#        xLabel -> string for text at x-axis
#        yLabel -> string for text at y-axis
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
        
    yLabel = ''

    for i in range(len(sensorList)):
        sensorNumber = sensorList[i]
        component = componentList[i]

        if not isinstance(sensorNumber, exudyn.SensorIndex):
            raise ValueError('ERROR in PlotSensor: *args must contain valid sensor numbers')

        #retrieve sensor information:
        sensorFileName = mbs.GetSensor(sensorNumber)['fileName']
        sensorName = mbs.GetSensor(sensorNumber)['name']
        variable = mbs.GetSensor(sensorNumber)['outputVariableType']
        variableStr = str(variable).replace('OutputVariableType.','')

        #now load sensor file:
        data = np.loadtxt(sensorFileName, comments='#', delimiter=',')

        #select color and style for sensor
        col = 'k-.'
        if i < len(CC):
            col = CC[i]
        
        #extract additional paramters
        xLabel = 'time (s)'
        yLabelAdd = variableStr
        if 'xLabel' in kwargs:
            xLabel = kwargs['xLabel']
        if 'yLabel' in kwargs:
            yLabelAdd = kwargs['yLabel']
        if len(yLabel) != 0:
            yLabel += ', '
        yLabel += yLabelAdd
        
        #plot according column (component+1)!:
        plt.plot(data[:,0], data[:,component+1], col, label=sensorName+', '+variableStr) #numerical solution
        plt.xlabel(xLabel)
        plt.ylabel(yLabel)
        ax=plt.gca() # get current axes
        ax.grid(True, 'major', 'both')
        ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
        ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
        plt.legend() #show labels as legend
        plt.tight_layout()
        plt.show() 
        
    
