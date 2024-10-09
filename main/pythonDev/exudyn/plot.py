#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Plot utility functions based on matplotlib, including plotting of sensors and FFT.
#
# Author:   Johannes Gerstmayr
# Date:     2020-09-16 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# Notes:    For a list of plot colors useful for matplotlib, see also advancedUtilities.PlotLineCode(...)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


import numpy as np #for loading
import exudyn #for sensor index
from exudyn.advancedUtilities import PlotLineCode, IsListOrArray, IsEmptyList
import copy
import os

#++++++++++++++++++++++++++++++++
#this structure helps to define default values, that are then always used!
class __PlotSensorDefaults:
    def __init__(self):
        pass
    def __repr__(self):
        return str(self.__dict__) #not very nice, but helps to easily see contents
    
__plotSensorDefaults = __PlotSensorDefaults() #initialize structure

#here we define the modifyable default values (only change with PlotSensorDefaults() function!)
__plotSensorDefaults.xLabel='time (s)'
__plotSensorDefaults.yLabel=None
__plotSensorDefaults.fontSize = 16

__plotSensorDefaults.colors=[]
__plotSensorDefaults.lineStyles=[]
__plotSensorDefaults.lineWidths=[]
__plotSensorDefaults.markerStyles=[]
__plotSensorDefaults.markerSizes=[]
__plotSensorDefaults.markerDensity=0.08

__plotSensorDefaults.majorTicksX = 10
__plotSensorDefaults.majorTicksY = 10
__plotSensorDefaults.sizeInches=[6.4,4.8]

#practical list of marker styles to be used as list:
listMarkerStyles = ['x ', '+ ', '* ', '. ', 'd ', 'D ', 's ', 'X ', 'P ', 'v ', '^ ', '< ', '> ', 'o ', 'p ', 'h ', 'H ']
listMarkerStylesFilled = ['x','+','*','.','d','D','s','X','P','v','^','<','>','o','p','h','H']

#this is the value for a component which indicates to show the norm (e.g. of a vector) instead of the component
componentNorm = -2 #

#**function: parse header of output file (solution file, sensor file, genetic optimization output, ...) given in file.readlines() format
#**output: return dictionary with 'type'=['sensor','solution','geneticOptimization','parameterVariation'], 'variableType' containing variable types, 'variableRanges' containing ranges for parameter variation 
def ParseOutputFileHeader(lines):
    nLines = len(lines)
    parseLines = min(10, nLines) #max 10 lines to parse
    output = {}
    output['type'] = 'unknown'
    #columns = []
    if len(lines) < 1:
        return {} #empty dictionary
    variableTypes = []
    variableRanges = [] #only for parameter variation
    if lines[0].find('EXUDYN genetic optimization results file') != -1:
        output['type'] = 'geneticOptimization'
        for i in range(parseLines): #header is max. 10 lines
            if i+1 < len(lines) and lines[i][0:9] == '#columns:':
                cols = lines[i+1].strip('#').split(',')
                for j in range(len(cols)):
                    variableTypes += [cols[j].strip()]
                break
    if lines[0].find('EXUDYN parameter variation results file') != -1:
        output['type'] = 'parameterVariation'
        for i in range(parseLines): #header is max. 10 lines
            if i+1 < len(lines) and lines[i][0:9] == '#columns:':
                cols = lines[i+1].strip('#').split(',')
                for j in range(len(cols)):
                    variableTypes += [cols[j].strip()]
                break
        for i in range(parseLines): #header is max. 10 lines
            if i+1 < len(lines) and lines[i][0:17] == '#parameter ranges':
                ranges = lines[i+1].strip('#').strip('\n').split(';') #this gives e.g. ['(0.1, 5, 4)', '(2,4.5,2)']
                # print('ranges=', ranges)
                for j in range(len(ranges)):
                    oneRange = ranges[j].strip('(').strip(')').split(',')
                    # print('one range=', oneRange)
                    variableRanges += [[float(oneRange[0]),float(oneRange[1]),int(oneRange[2])]]
                break
    elif lines[0].find('sensor output file') != -1:
        #print("SENSOR")
        output['type'] = 'sensor'
        outputVariableType = ''
        for i in range(parseLines): #header is max. 10 lines
            if lines[i].find('Object number') != -1:
                output['objectNumber'] = int(lines[i].split('=')[1])
            elif lines[i].find('OutputVariableType') != -1:
                outputVariableType = lines[i].split('=')[1].strip() #without spaces
                output['outputVariableType'] = outputVariableType #for PlotSensor
            elif lines[i].find('number of sensor values') != -1:
                output['numberOfSensors'] = int(lines[i].split('=')[1]) 

            if lines[i].find('#measure') != -1:
                if ' ' in lines[i]:
                    output['sensorType'] = lines[i].split(' ')[1] #for PlotSensor
                else:
                    output['sensorType'] = '' #no specific sensor type
                if '=' in lines[i]:
                    output['itemNumber'] = int(lines[i].split('=')[1]) #unused
                else:
                    output['itemNumber'] = -1 #invalid
                
            if lines[i][0] != '#': #break after comment
                break
        variableTypes = ['time']
        for i in range(output['numberOfSensors']):
            variableTypes += [outputVariableType+str(i)] #e.g., Position0, Position1, ...
    elif lines[0].find('solution file') != -1: #coordinates solution file
        output['type'] = 'solution'
        writtenCoordinateTypes = []
        writtenCoordinates = []
        for i in range(parseLines): #header is max. 10 lines
            if lines[i].find('number of written coordinates') != -1:
                line = lines[i]
                writtenCoordinateTypes = line.split('=')[0].split('[')[1].split(']')[0].replace(' ','').split(',')
                writtenCoordinates = line.split('=')[1].split('[')[1].split(']')[0].replace(' ','').split(',')
                variableTypes = ['time']
                #print('writtenCoordinates=',writtenCoordinates)
                for j in range(len(writtenCoordinateTypes)):
                    for k in range(int(writtenCoordinates[j])):
                       variableTypes += [writtenCoordinateTypes[j].strip('n')+'-'+str(k)]
                #variableTypes += [writtenCoordinateTypes[j]]*int(writtenCoordinates[j])
            elif lines[i].find('number of time steps') != -1:
                output['numberOfSteps'] = int(lines[i].split('=')[1])

            if lines[i][0] != '#': #break after comment
                break

    output['columns'] = variableTypes
    output['variableRanges'] = variableRanges
    
    return output

#**function: returns structure with default values for PlotSensor which can be modified once to be set for all later calls of PlotSensor
#**example: 
##change one parameter:
#plot.PlotSensorDefaults().fontSize = 12
##==>now PlotSensor(...) will use fontSize=12
##==>now PlotSensor(..., fontSize=10) will use fontSize=10
##==>BUT PlotSensor(..., fontSize=16) will use fontSize=12, BECAUSE 16 is the original default value!!!
##see which parameters are available:
#print(PlotSensorDefaults())
def PlotSensorDefaults():
    return __plotSensorDefaults #for definition see at top of this file
   

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Helper function for direct and easy visualization of sensor outputs, without need for loading text files, etc.; PlotSensor can be used to simply plot, e.g., the measured x-Position over time in a figure. PlotSensor provides an interface to matplotlib (which needs to be installed). Default values of many function arguments can be changed using the exudyn.plot function PlotSensorDefaults(), see there for usage.
#**input: 
#  mbs: must be a valid MainSystem (mbs)
#  sensorNumbers: consists of one or a list of sensor numbers (type SensorIndex or int) as returned by the mbs function AddSensor(...); sensors need to set writeToFile=True and/or storeInternal=True for PlotSensor to work; alternatively, it may contain FILENAMES (incl. path) to stored sensor or solution files OR a numpy array instead of sensor numbers; the format of data (file or numpy array) must contain per row the time and according solution values in columns; if components is a list and sensorNumbers is a scalar, sensorNumbers is adjusted automatically to the components
#  components: consists of one or a list of components according to the component of the sensor to be plotted at y-axis; if components is a list and sensorNumbers is a scalar, sensorNumbers is adjusted automatically to the components; as always, components are zero-based, meaning 0=X, 1=Y, etc.; for regular sensor files, time will be component=-1; to show the norm (e.g., of a force vector), use component=[plot.componentNorm] for according sensors; norm will consider all values of sensor except time (for 3D force, it will be $\sqrt{f_0^2+f_1^2+f_2^2}$); offsets and factors are mapped on norm (plot value=factor*(norm(values) + offset) ), not on component values
#  componentsX: default componentsX=[] uses time in files; otherwise provide componentsX as list of components (or scalar) representing x components of sensors in plotted curves; DON'T forget to change xLabel accordingly! 
#    Using componentsX=[...] with a list of column indices specifies the respective columns used for the x-coordinates in all sensors; by default, values are plotted against the first column in the files, which is time; according to counting in PlotSensor, this represents componentX=-1; 
#    plotting y over x in a position sensor thus reads: components=[1], componentsX=[0]; 
#    plotting time over x reads: components=[-1], componentsX=[0]; 
#    the default value reads componentsX=[-1,-1,...]
#  xLabel: string for text at x-axis
#  yLabel: string for text at y-axis (default: None==> label is automatically computed from sensor value types)
#  labels: string (for one sensor) or list of strings (according to number of sensors resp. components) representing the labels used in legend; if labels=[], automatically generated legend is used
#  rangeX: default rangeX=[]: computes range automatically; otherwise use rangeX to set range (limits) for x-axis provided as sorted list of two floats, e.g., rangeX=[0,4]
#  rangeY: default rangeY=[]: computes range automatically; otherwise use rangeY to set range (limits) for y-axis provided as sorted list of two floats, e.g., rangeY=[-1,1]
#  figureName: optional name for figure, if newFigure=True
#  fontSize: change general fontsize of axis, labels, etc. (matplotlib default is 12, default in PlotSensor: 16)
#  title: optional string representing plot title 
#  offsets: provide as scalar, list of scalars (per sensor) or list of 2D numpy.arrays (per sensor, having same rows/columns as sensor data; in this case it will also influence x-axis if componentsX is different from -1) to add offset to each sensor output; for an original value fOrig, the new value reads fNew = factor*(fOrig+offset); for offset provided as numpy array (with same time values), the 'time' column is ignored in the offset computation; can be used to compute difference of sensors; if offsets=[], no offset is used
#  factors: provide as scalar or list (per sensor) to add factor to each sensor output; for an original value fOrig, the new value reads fNew = factor*(fOrig+offset); if factor=[], no factor is used
#  majorTicksX: number of major ticks on x-axis; default: 10
#  majorTicksY: number of major ticks on y-axis; default: 10
#  colorCodeOffset: int offset for color code, color codes going from 0 to 27 (see PlotLineCode(...)); automatic line/color codes are used if no colors and lineStyles are used
#  colors: color is automatically selected from colorCodeOffset if colors=[]; otherwise chose from 'b', 'g', 'r', 'c', 'm', 'y', 'k' and many other colors see https://matplotlib.org/stable/gallery/color/named\_colors.html
#  lineStyles: line style is automatically selected from colorCodeOffset if lineStyles=[]; otherwise define for all lines with string or with list of strings, chosing from '-', '--', '-.', ':', or '' 
#  lineWidths: float to define line width by float (default=1); either use single float for all sensors or list of floats with length >= number of sensors
#  markerStyles: if different from [], marker styles are defined as list of marker style strings or single string for one sensor; chose from '.', 'o', 'x', '+' ... check listMarkerStylesFilled and listMarkerStyles in exudyn.plot and see https://matplotlib.org/stable/api/markers\_api.html ; ADD a space to markers to make them empty (transparent), e.g. 'o ' will create an empty circle 
#  markerSizes: float to define marker size by float (default=6); either use single float for all sensors or list of floats with length >= number of sensors 
#  markerDensity: if int, it defines approx. the total number of markers used along each graph; if float, this defines the distance of markers relative to the diagonal of the plot (default=0.08); if None, it adds a marker to every data point if marker style is specified for sensor
#  newFigure: if True, a new matplotlib.pyplot figure is created; otherwise, existing figures are overwritten
#  subPlot: given as list [nx, ny, position] with nx, ny being the number of subplots in x and y direction (nx=cols, ny=rows), and position in [1,..., nx*ny] gives the position in the subplots; use the same structure for first PlotSensor (with newFigure=True) and all subsequent PlotSensor calls with newFigure=False, which creates the according subplots; default=[](no subplots) 
#  sizeInches: given as list [sizeX, sizeY] with the sizes per (sub)plot given in inches; default: [6.4, 4.8]; in case of sub plots, the total size of the figure is computed from nx*sizeInches[0] and ny*sizeInches[1]
#  fileName: if this string is non-empty, figure will be saved to given path and filename (use figName.pdf to safe as PDF or figName.png to save as PNG image); use matplotlib.use('Agg') in order not to open figures if you just want to save them
#  useXYZcomponents: of True, it will use X, Y and Z for sensor components, e.g., measuring Position, Velocity, etc. wherever possible
#  closeAll: if True, close all figures before opening new one (do this only in first PlotSensor command!)
#  [*kwargs]:
#        minorTicksXon: if True, turn minor ticks for x-axis on
#        minorTicksYon: if True, turn minor ticks for y-axis on
#        logScaleX: use log scale for x-axis
#        logScaleY: use log scale for y-axis
#        fileCommentChar: if exists, defines the comment character in files (\#, %, ...)
#        fileDelimiterChar: if exists, defines the character indicating the columns for data (',', ' ', ';', ...)
#**output: [Any, Any, Any, Any]; plots the sensor data; returns [plt, fig, ax, line] in which plt is matplotlib.pyplot, fig is the figure (or None), ax is the axis (or None) and line is the return value of plt.plot (or None) which could be changed hereafter
#**notes: adjust default values by modifying the variables exudyn.plot.plotSensorDefault..., e.g., exudyn.plot.plotSensorDefaultFontSize
#**belongsTo: MainSystem
#**example: 
##assume to have some position-based nodes 0 and 1:
#s0=mbs.AddSensor(SensorNode(nodeNumber=0, fileName='s0.txt',
#                            outputVariableType=exu.OutputVariableType.Position))
#s1=mbs.AddSensor(SensorNode(nodeNumber=1, fileName='s1.txt',
#                            outputVariableType=exu.OutputVariableType.Position))
#mbs.PlotSensor(s0, 0) #plot x-coordinate
##plot x for s0 and z for s1:
#mbs.PlotSensor(sensorNumbers=[s0,s1], components=[0,2], yLabel='this is the position in meter')
#mbs.PlotSensor(sensorNumbers=s0, components=plot.componentNorm) #norm of position
#mbs.PlotSensor(sensorNumbers=s0, components=[0,1,2], factors=1000., title='Answers to the big questions')
#mbs.PlotSensor(sensorNumbers=s0, components=[0,1,2,3], 
#           yLabel='Coordantes with offset 1\nand scaled with $\\frac{1}{1000}$', 
#           factors=1e-3, offsets=1,fontSize=12, closeAll=True)
#
##assume to have body sensor sBody, marker sensor sMarker:
#mbs.PlotSensor(sensorNumbers=[sBody]*3+[sMarker]*3, components=[0,1,2,0,1,2], 
#           colorCodeOffset=3, newFigure=False, fontSize=10, 
#           yLabel='Rotation $\\alpha, \\beta, \\gamma$ and\n Position $x,y,z$',
#           title='compare marker and body sensor')
##assume having file plotSensorNode.txt:
#mbs.PlotSensor(sensorNumbers=[s0]*3+ [filedir+'plotSensorNode.txt']*3, 
#           components=[0,1,2]*2)
##plot y over x:
#mbs.PlotSensor(sensorNumbers=s0, componentsX=[0], components=[1], xLabel='x-Position', yLabel='y-Position')
##for further examples, see also Examples/plotSensorExamples.py
def PlotSensor(mbs, sensorNumbers=[], components=0, xLabel='time (s)', yLabel=None, labels=[], 
               colorCodeOffset=0, newFigure=True, closeAll=False, 
               componentsX=[], title='', figureName='', fontSize=16, 
               colors=[], lineStyles=[], lineWidths=[], markerStyles=[], markerSizes=[], markerDensity=0.08,
               rangeX=[], rangeY=[], majorTicksX=10, majorTicksY=10,
               offsets=[], factors=[], subPlot=[], sizeInches=[6.4,4.8],
               fileName='', useXYZcomponents=True, **kwargs):
    #could also be imported from exudyn.utilities import PlotLineCode
    #CC = ['k-','g-','b-','r-','c-','m-','y-','k:','g:','b:','r:','c:','m:','y:','k--','g--','b--','r--','c--','m--','y--','k-.','g-.','b-.','r-.','c-.','m-.','y-.']
    try:
        import matplotlib
        import matplotlib.pyplot as plt
        import matplotlib.ticker as ticker
    except:
        raise ValueError('ERROR: PlotSensor: matplotlib is not installed; PlotSensor is therefore not available')

    
    for key in kwargs:
        if (key!='minorTicksXon' and key!='minorTicksYon'
            and key!='fileCommentChar' and key!='fileDelimiterChar'
            and key!='logScaleX'  and key!='logScaleY'):
            raise ValueError('PlotSensor: invalid argument: '+key)

    if xLabel == 'time (s)':
        xLabel = __plotSensorDefaults.xLabel
    if yLabel == None:
        yLabel = __plotSensorDefaults.yLabel

    if fontSize == 16:
        fontSize = __plotSensorDefaults.fontSize

    #the following code is not totally safe regarding mutable args, but works with this kind of default args
    if IsEmptyList(colors):
        colors = __plotSensorDefaults.colors
    if IsEmptyList(lineStyles):
        lineStyles = __plotSensorDefaults.lineStyles
    if IsEmptyList(lineWidths):
        lineWidths = __plotSensorDefaults.lineWidths
    if IsEmptyList(markerStyles):
        markerStyles = __plotSensorDefaults.markerStyles
    if IsEmptyList(markerSizes):
        markerSizes = __plotSensorDefaults.markerSizes
    if markerDensity == 0.08:
        markerDensity = __plotSensorDefaults.markerDensity

    if majorTicksX == 10:
        majorTicksX = __plotSensorDefaults.majorTicksX
    if majorTicksY == 10:
        majorTicksY = __plotSensorDefaults.majorTicksY
    if sizeInches == [6.4,4.8]:
        sizeInches = __plotSensorDefaults.sizeInches


            
    if isinstance(sensorNumbers,list):
        sensorList = list(sensorNumbers)
    else:
        sensorList = [sensorNumbers]

    if isinstance(components,list):
        componentList = components
    else:
        componentList = [components]

    if len(componentList) == 1 and len(sensorList) != 1:
        componentList = componentList*len(sensorList)
        
    if len(componentList) != 1 and len(sensorList) == 1:
        sensorList = sensorList*len(componentList)
        
    if len(componentList) !=  len(sensorList):
        raise ValueError('PlotSensor: size of sensorNumbers and size of components must be same or either components or sensorNumbers is scalar, sensorNumbers='+str(sensorNumbers)+', components='+str(components))

    nSensors = len(sensorList)

    componentsXnew = copy.copy(componentsX)
    if componentsXnew != []:
        if not isinstance(components,list):
            componentsXnew = [componentsXnew]*nSensors
        elif len(componentsXnew) != nSensors:
            raise ValueError('PlotSensor: size of componentsX and size of sensors or components must be agree; componentsX='+str(componentsXnew))
    else:
        componentsXnew = [-1]*nSensors

    
    if closeAll:
        plt.close('all')

    logScaleX = False
    if 'logScaleX' in kwargs:
        logScaleX = kwargs['logScaleX']
    logScaleY = False
    if 'logScaleY' in kwargs:
        logScaleY = kwargs['logScaleY']
        
    #increase font size as default is rather small
    plt.rcParams.update({'font.size': fontSize})

    factorOffsetUsed = False
    if IsListOrArray(factors, checkIfNoneEmpty=True):#factors!=[]:
        if type(factors) != list:
            factors = [factors]*nSensors
        if len(factors) != nSensors:
            raise ValueError('PlotSensor: factors must be scalar or have same dimension as sensors')
        factorOffsetUsed = True
    else:
        factors = [1.]*nSensors

    if IsListOrArray(offsets, checkIfNoneEmpty=True):#offsets!=[]:
        if type(offsets) != list:
            offsets = [offsets]*nSensors
        if len(offsets) != nSensors:
            raise ValueError('PlotSensor: offsets must be scalar or have same dimension as sensors')
        factorOffsetUsed = True
    else:
        offsets = [0.]*nSensors


    fig=None
    ax=None
    line=None
    if nSensors:
        if figureName!='':
            if newFigure and plt.fignum_exists(figureName):
                plt.close(figureName)
            fig = plt.figure(figureName)
        elif newFigure:
            fig = plt.figure()
        else:
            if IsEmptyList(plt.get_fignums()):
                print('WARNING: PlotSensor(...,newFigure=False):  no existing figure was found, creating new figure')
                fig = plt.figure()
            else:
                fig = plt.figure(plt.get_fignums()[-1]) #get last (current figure)
    
    # subNx=1
    # subNy=1
    if fig!=None:
        if subPlot!=[]:
            if type(subPlot)!=list or len(subPlot)!=3:
                raise ValueError('PlotSensor: subPlot must have 3 integers [nx, ny, position]')
            [subNx, subNy, subPos] = subPlot
            fig.add_subplot(subNy, subNx, subPos)
            fig.set_size_inches(subNx*sizeInches[0],subNy*sizeInches[1], forward=True)
        else:
            fig.set_size_inches(sizeInches[0],sizeInches[1], forward=True)

    sensorFileNames = [] #for loading of files
    sensorLabels = []    #plot label (legend)
    sensorTypes = []     #for comparison, if all are of the same type
    sensorDicts = []     #to check if stored internally
    
    for i in range(nSensors):
        component = componentList[i]
        sensorNumber = sensorList[i]
        if not (isinstance(sensorNumber, exudyn.SensorIndex) or 
                type(sensorNumber) == int or
                type(sensorNumber) == str or 
                type(sensorNumber) == np.ndarray):
            raise ValueError('PlotSensor: *args must contain valid sensor numbers (SensorIndex or integers) or represent a filename string')

        #retrieve sensor information:
        if type(sensorNumber) == str: #direct path to file name
            sensorDict={}
            sensorDict['fileName'] = sensorNumber #sensorNumber must contain a file name, otherwise will fail
            sensorDict['outputVariableType']=''
            sensorDict['name'] = sensorNumber.split('/')[-1].split('\\')[-1].split('.')[0] #use filename without path and ending
            
            with open(sensorDict['fileName']) as file:
                sensorDict.update(ParseOutputFileHeader(file.readlines()) )
            
            if sensorDict['type'] == 'solution':
                sensorDict['outputVariableType'] = 'Coordinates'
                
        elif type(sensorNumber) == np.ndarray: #data provided as numpy array
            sensorDict={}
            sensorDict['fileName'] = sensorNumber #sensorNumber must contain the data as numpy array
            sensorDict['outputVariableType']=''
            sensorDict['name'] = ''
            sensorDict['numpyArray'] = True #signal that it contains a numpy array
            sensorDict['outputVariableType'] = ''
        else:
            sensorDict = mbs.GetSensor(sensorNumber)
            if not (sensorDict['storeInternal'] or (sensorDict['writeToFile']
                    and len(sensorDict['fileName'])!=0)):
                raise ValueError('PlotSensor: sensor '+str(sensorNumber) +' has neither writeToFile=True nor storeInternal=True or filenName is empty, thus sensor cannot be plotted!')

        sensorDicts += [sensorDict]
        
        #print('sensorDict=',sensorDict)
        sensorName = sensorDict['name']

        variableStr = '' #in case of markers, etc.
        if 'outputVariableType' in sensorDict:
            variable = sensorDict['outputVariableType']
            variableStr = str(variable).replace('OutputVariableType.','')
        elif 'sensorType' in sensorDict:
            if sensorDict['sensorType'] == 'Load':
                loadNumber = sensorDict['loadNumber']
                loadDict = mbs.GetLoad(loadNumber)
                loadType = loadDict['loadType']
                if (loadType == 'ForceVector' or
                    loadType == 'TorqueVector'):
                    variableStr = loadType.replace('Vector','')
                else:
                    variableStr = 'Load'
            else:
                variableStr = sensorDict['sensorType']

        #+++++++++++++++++++++++++++++++++++        
        #create name for component
        sComponent=''
        #if len(componentList) != 1: #changed 2022-01-25: should show up anyway
        varStrNoLocal = variableStr.replace('Local','')
        compXYZ = ['X','Y','Z']
        compXYZ2 = ['XX','YY','ZZ','YZ','XZ','XY'] #for stress, strain,...
        
        if (useXYZcomponents and component < 3 and component >= 0 and 
            (varStrNoLocal == 'Force' or varStrNoLocal == 'Torque' or 
             varStrNoLocal == 'Position' or varStrNoLocal == 'Displacement' or 
             varStrNoLocal == 'Velocity' or varStrNoLocal == 'Acceleration' or
             varStrNoLocal == 'Rotation' or varStrNoLocal == 'AngularVelocity' or
             varStrNoLocal == 'AngularAcceleration' or 
             varStrNoLocal == 'Force' or varStrNoLocal == 'Torque' or 
             varStrNoLocal == 'AngularVelocity' or varStrNoLocal == 'AngularAcceleration')):
            sComponent = compXYZ[component]
        elif (useXYZcomponents and component < 6 and component >= 0 and 
            (varStrNoLocal == 'Strain' or varStrNoLocal == 'Stress')):
            sComponent = compXYZ2[component]
        else:
            sComponent = str(component)

        sensorFileNames += [sensorDict['fileName']]
        sensorLabels += [sensorName+', '+variableStr+sComponent]
        sensorTypes += [variableStr]
    
    if labels != []:
        if nSensors == 1 and type(labels)==str:
            sensorLabels = [labels]
        elif type(labels)==list and len(labels) == nSensors:
            sensorLabels = labels
        else:
            raise ValueError('PlotSensor: labels must be either string for one sensor or list of strings according to number of sensors / components')
            
    
    #+++++++++++++++++++++++++++++++++++++++++++
    #check if all sensor outputvariables are the same => generate ylabel automatically!
    checkStr = ''
    allVariablesSame = True
    for i in range(nSensors):

        if i == 0:
            checkStr = sensorTypes[i]
        elif checkStr != sensorTypes[i]:
            allVariablesSame = False

    if yLabel == None:
        yLabel = ''
        if allVariablesSame:
            yLabel = checkStr
        else:
            for (i, tt) in enumerate(sensorTypes):
                yLabel += tt
                if i < len(sensorTypes)-1:
                    yLabel += ', '
    
    #+++++++++++++++++++++++++++++++++++++++++++
    #finally plot:
    for i in range(nSensors):
        if componentList[i] != componentNorm:
            componentY = componentList[i] + 1
        else:
            componentY = componentNorm
        componentX = componentsXnew[i] + 1
        
        #now load sensor file:
        if ('storeInternal' in sensorDicts[i] and
            sensorDicts[i]['storeInternal']): #preferred way (higher accuracy, faster)
            sensorNumber = sensorList[i]
            data = mbs.GetSensorStoredData(sensorNumber)
        elif 'numpyArray' in sensorDicts[i]:
            sensorNumber = 'data'+str(i) #this string appears in errors and possibly in legend
            data = sensorList[i]
        else:
            fileCommentChar = '#'
            fileDelimiterChar = ','
            if 'fileCommentChar' in kwargs:
                fileCommentChar = kwargs['fileCommentChar']
            if 'fileDelimiterChar' in kwargs:
                fileDelimiterChar = kwargs['fileDelimiterChar']
            
            data = np.loadtxt(sensorFileNames[i], comments=fileCommentChar, delimiter=fileDelimiterChar)

        #select color and style for sensor
        CC = PlotLineCode(i+colorCodeOffset)
        
        color = CC[0]
        lineStyle = CC[1:]
        markerStyle = ''
        markerSize = 6 #default matplotlib for scatter
        lineWidth = 1

        if lineStyles != []:
            if type(lineStyles)==str:
                lineStyle = lineStyles
            elif type(lineStyles)==list and len(lineStyles) >= nSensors:
                lineStyle = lineStyles[i]
            else:
                raise ValueError('PlotSensor: lineStyles must be either string for one sensor or list of matplotlib line style codes with length >= number of sensors / components')
        
        if lineWidths != []:
            if type(lineWidths)==float or type(lineWidths)==int:
                lineWidth = lineWidths
            elif type(lineWidths)==list and len(lineWidths) >= nSensors:
                lineWidth = lineWidths[i]
            else:
                raise ValueError('PlotSensor: lineWidths must be either a single float for all sensors or list of floats with length >= number of sensors / components (default line width=1)')
        
        if colors != []:
            if nSensors == 1 and type(colors)==str:
                color = colors
            elif type(colors)==list and len(colors) >= nSensors:
                color = colors[i]
            else:
                raise ValueError('PlotSensor: colors must be either string for one sensor or list of matplotlib color codes with length >= to number of sensors / components')
        
        markEvery = None
        markerFillStyle = 'full'
        if markerStyles != []:
            if nSensors == 1 and type(markerStyles)==str:
                markerStyle = markerStyles
            elif type(markerStyles)==list and len(markerStyles) >= nSensors:
                markerStyle = markerStyles[i]
                if ' ' in markerStyle:
                    markerFillStyle = 'none'
                    markerStyle = markerStyle.replace(' ','')
            else:
                raise ValueError('PlotSensor: markerStyles must be either string for one sensor or list of matplotlib marker style codes with length >= number of sensors / components')
            if type(markerDensity) == int:
                nd = len(data) #len(data[:,componentY])
                if markerDensity != 0:
                    markEvery = int(1+nd/markerDensity)
                    #print('markEvery=',markEvery, ', nd=',nd)
            else:
                markEvery = markerDensity
        else:
            markEvery = None
        
        if markerSizes != []:
            if type(markerSizes)==float or type(markerSizes)==int:
                markerSize = markerSizes
            elif type(markerSizes)==list and len(markerSizes) == nSensors:
                markerSize = markerSizes[i]
            else:
                raise ValueError('PlotSensor: markerSizes must be either a single float for all sensor or list of marker sizes with length >= number of sensors / components')
                
        #+++++++++++++++++++++++++++++++++++        
        xData = data[:,componentX]
        if componentY != componentNorm:
            yData = data[:,componentY]
        else:
            #compute norm, not including time
            nValues = len(data)
            yData = np.zeros(nValues)
            for rowNorm in range(nValues):
                yData[rowNorm] = np.linalg.norm(data[rowNorm,1:])

        #+++++++++++++++++++++++++++++++++++        
        #add factor and offset if defined:
        if factorOffsetUsed:
            if type(offsets[i]) == float or type(offsets[i]) == int:
                yData = factors[i]*(yData + offsets[i])
            else: #must be numpy array
                if componentY == componentNorm:
                    raise ValueError('PlotSensor: sensor '+str(sensorNumber) +': component plot.componentNorm is only possible with scalar offsets')
                if not isinstance(offsets[i], np.ndarray):
                    raise ValueError('PlotSensor: sensor '+str(sensorNumber) +' offset must be either scalar (float) or numpy array, but received: '+str(type(offsets[i])))
                if offsets[i].shape != data.shape:
                    raise ValueError('PlotSensor: sensor '+str(sensorNumber) +' offset must have same dimensions as sensor data ('+str(data.shape)+') but received: '+str(offsets[i].shape))
                yData = factors[i]*(yData + offsets[i][:,componentY])
                if componentX != 0: #ignored for time
                    xData = xData + offsets[i][:,componentX]

        #+++++++++++++++++++++++++++++++++++        
        #finally plot curve:
 
        plt.plot(xData, yData, color=color, linestyle=lineStyle, linewidth=lineWidth,
                 marker=markerStyle, fillstyle=markerFillStyle, markersize=markerSize, markevery=markEvery, label=sensorLabels[i]) #numerical solution
       
        #plt.plot(data[:,componentX], yData, CC, label=sensorLabels[i]) #numerical solution
        plt.xlabel(xLabel)
        plt.ylabel(yLabel)
        ax=plt.gca() # get current axes
        ax.grid(True, 'major', 'both')
        
        ax.xaxis.set_major_locator(ticker.MaxNLocator(majorTicksX)) 
        ax.yaxis.set_major_locator(ticker.MaxNLocator(majorTicksY)) 

        if logScaleX:
            plt.xscale('log')
        if logScaleY:
            plt.yscale('log')

        if 'minorTicksOn' in kwargs:
            if kwargs['minorTicksOn']:
                ax.minorticks_on()
            else:
                ax.minorticks_off()

        
        if title!='':
            plt.title(title)

        if rangeX!=[]:
            if type(rangeX)!=list or len(rangeX)!=2:
                raise ValueError('PlotSensor: rangeX must be list of length 2 with minimum and maximum x-values')
            plt.xlim(rangeX)
            
        if rangeY!=[]:
            if type(rangeY)!=list or len(rangeY)!=2:
                raise ValueError('PlotSensor: rangeY must be list of length 2 with minimum and maximum y-values')
            plt.ylim(rangeY)


    #do this finally!!!
    if nSensors > 0:
        handle = plt
        if fig!=None:
            handle = fig #better to use fig; plt.tight_layout() gives warning

        
        handle.legend() #show labels as legend
        handle.tight_layout()
        if matplotlib.get_backend() != 'agg': #this is used to avoid showing the figures, if they are just saved
            handle.show() 
        
        if fileName != '':
            try:
                os.makedirs(os.path.dirname(fileName), exist_ok=True)
            except:
                pass #makedirs may fail on some systems, but we keep going

            handle.savefig(fileName)
    
    return [plt, fig, ax, line]
    
#**function: plot fft spectrum of signal
#**input: 
#   frequency:  frequency vector (Hz, if time is in SECONDS)   
#   data:       magnitude or phase as returned by ComputeFFT() in exudyn.signalProcessing
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
    # xScale = 'linear'
    # yScale = 'linear'
    if logScaleX:
        plt.xscale('log')
    if logScaleY:
        plt.yscale('log')
    ax.grid(visible=True, which='major', color='k', linestyle='-')
    ax.grid(visible=True, which='minor', color='k', linestyle=':')
    ax.minorticks_on()

    plt.tight_layout()
    plt.show() 

    return plt

#**function: strip spaces at beginning / end of lines; this may be sometimes necessary when reading solutions from files that are space-separated
#**input:
#  filename: name of file to process
#  outputFilename: name of file to which text without leading/trailing spaces is written
#  fileCommentChar: if not equal '', lines starting with this character will not be processed
#  removeDoubleChars: if not equal '', this double characters (especial multiple spaces) will be removed; '1.0   3.0' will be converted into '1.0 3.0'
#**output: new file written
def FileStripSpaces(filename, outputFilename, fileCommentChar='', removeDoubleChars=''):
    if filename==outputFilename:
        raise ValueError('StripSpaces: filename and outputFilename must be different')
    with open(filename, 'r') as file:
        lines = file.readlines()
        with open(outputFilename, 'w') as outfile:

            for line in lines:
                if fileCommentChar=='' or not (len(line)>=len(fileCommentChar)
                    and line[0:len(fileCommentChar)] == fileCommentChar):

                    line = line.strip('\n').strip(' ')+'\n'
                    found = (removeDoubleChars != '')
                    while found:
                        if (removeDoubleChars+removeDoubleChars) in line:
                            line = line.replace(removeDoubleChars+removeDoubleChars,removeDoubleChars)
                        else: found=False
                outfile.write(line)


#**function: helper function to create data array from outputs defined by sensorNumbers list [+optional positionList which must have, e.g., local arc-length of beam according to sensor numbers]; if time=='', current sensor values will be used; if time!=[], evaluation will be based on loading values from file or sensor internal data and evaluate at that time 
#**input: 
#  mbs: a MainSystem where the sensors are given
#  sensorNumbers: a list of sensor numbers, which shall be evaluated
#  positionList: an optional list of positions per sensor (e.g., axial positions at beam) 
#  time: optional time at which the sensor values are evaluated (currently not implemented)
#**output: returns data as numpy array, containg per row the number or position (positionList) in the first column and all sensor values in the remaining columns
def DataArrayFromSensorList(mbs, sensorNumbers, positionList=[], time=''):
    if time != '':
        raise ValueError("DataArrayFromSensors: time != '' is currently not implemented!")
    n = len(sensorNumbers)
    if positionList != [] and (n != len(positionList)):
        raise ValueError("DataArrayFromSensors: sensorNumbers and positionList must have same dimensions, or positionList must be empty list")
    
    data = []
    
    for i, sensor in enumerate(sensorNumbers):
        values = mbs.GetSensorValues(sensor)
        if type(values) != np.ndarray:
            values = np.array([values])
            
        if i == 0:
            data = np.zeros((n, 1+len(values)))
        if positionList != []:
            data[i,0] = positionList[i]
        else:
            data[i,0] = i
        data[i,1:] = values
    return data


#**function: import image text file as exported from RedrawAndSaveImage() with exportImages.saveImageFormat='TXT'; triangles are converted to lines
#**input: fileName includes directory
#**output: returns dictionary with according structures
def LoadImage(fileName, trianglesAsLines = True, verbose=False):
        
    with open(fileName) as file:
        lines = file.readlines()

    if len(lines) == 0:
        raise ValueError('LoadImage: empty file')

    if lines[0][:-1] != '#Exudyn text image export file':
        print('WARNING: LoadImage found inconsistent file header:',lines[0])
    
    if lines[-1][:-1] != '#END':
        print('WARNING: LoadImage found inconsistent file ending; expected "END"')
    
    i = 1
    listLines = []
    listLineColors = [] #line colors as tuples
    listTriangles = []
    nSegments = 0
    nTriangles = 0
    actColor = (0,0,0,1)
    while i < len(lines)-1: #there will be always 1 extra line (or file end)!
        lineType = lines[i][:-1]
        data = lines[i+1]
        if lineType == '#COLOR':
            actColor = tuple(np.array(data.split(','), dtype=float))
            # print('color', actColor)
        elif lineType == '#LINE':
            splitLine = np.array(data.split(','), dtype=float)
            listLines += [list(splitLine)]
            listLineColors += [actColor] #per line
            nSegments +=int(len(splitLine)/3)-1
            # print('line', listLines)
        elif lineType == '#TRIANGLE':
            nTriangles += 1
            splitLine = np.array(data.split(','), dtype=float)
            linePoints = list(np.array(data.split(','), dtype=float))
            if trianglesAsLines:
                linePoints += linePoints[0:3] #add first point as last point
                listLines += [linePoints]
                listLineColors += [actColor] #per line
            else:
               listTriangles += [(list(splitLine), actColor)]
        else:
            i -= 1 #this may be a comment line; just increment by 1
        
        i += 2 #always increment by 2

    if verbose:
        print('number of lines:', len(listLines))
        print('number of line segments:', nSegments)
        print('number of triangles:', nTriangles)
        
    return {'linePoints':listLines, 'lineColors':listLineColors, 'triangles':listTriangles}

#**function: plot 2D or 3D vector image data as provided by LoadImage(...) using matplotlib
#**input:
#  imageData: dictionary as provided by LoadImage(...) 
#  HT: homogeneous transformation, used to transform coordinates; lines are drawn in (x,y) plane
#  axesEqual: for 2D mode, axis are set equal, otherwise model is distorted
#  plot3D: in this mode, a 3D visualization is used; triangles are only be displayed in this mode!
#  lineWidths: width of lines
#  lineStyles: matplotlib codes for lines
#  triangleEdgeColors: color for triangle edges as tuple of rgb colors or matplotlib color code strings 'black', 'r', ...
#  triangleEdgeWidths: width of triangle edges; set to 0 if edges shall not be shown
#  removeAxes: if True, all axes and background are removed for simpler export
#  orthogonalProjection: if True, projection is orthogonal with no perspective view
#  title: optional string representing plot title 
#  figureName: optional name for figure, if newFigure=True
#  fileName: if this string is non-empty, figure will be saved to given path and filename (use figName.pdf to safe as PDF or figName.png to save as PNG image); use matplotlib.use('Agg') in order not to open figures if you just want to save them
#  fontSize: change general fontsize of axis, labels, etc. (matplotlib default is 12, default in PlotSensor: 16)
#  closeAll: if True, close all figures before opening new one (do this only in first PlotSensor command!)
#  azim, elev: for 3D plots: the initial angles for the 3D view in degrees
def PlotImage(imageData, HT = np.eye(4), axesEqual=True, plot3D=False, lineWidths=1, lineStyles='-', 
              triangleEdgeColors='black', triangleEdgeWidths=0.5, removeAxes = True, orthogonalProjection=True,
              title = '', figureName='', fileName = '', fontSize = 16, closeAll = False,
              azim=0., elev=0.):

    from matplotlib import collections  as mc #plot does not accept colors
    import matplotlib.pyplot as plt
    import matplotlib

    from exudyn.rigidBodyUtilities import HT2rotationMatrix, HT2translation

    linePoints = imageData['linePoints']
    lineColors = imageData['lineColors']
    triangles = imageData['triangles']

    if closeAll:
        plt.close('all')

    plt.rcParams.update({'font.size': fontSize})
    fig = plt.figure()

    if figureName!='':
        if plt.fignum_exists(figureName):
            plt.close(figureName)
        fig = plt.figure(figureName)
    
    #optional transformation
    A = HT2rotationMatrix(HT)
    p0 = HT2translation(HT)

    if not plot3D: #plot in 2D
        ax = fig.gca()
        plotData = []
        colors = []
        
        for i, line3D in enumerate(linePoints):
            # print('line3D:', line3D)
            nPoints = int(len(line3D)/3)
    
            line = (A @ np.array(line3D).reshape((nPoints,3)).T).T.flatten()
            color = lineColors[i]
            x = np.zeros(nPoints)
            y = np.zeros(nPoints)
            z = np.zeros(nPoints)
            for j in range(nPoints):
                x[j] = float(line[j*3+0]+p0[0])
                y[j] = float(line[j*3+1]+p0[0])
                z[j] = float(line[j*3+2]+p0[0])
                
            for j in range(nPoints-1):
                plotData += [[(x[j], y[j]), (x[j+1], y[j+1])]] #for plot
                colors += [color]
    
        if plotData != []:
            collLines = mc.LineCollection(plotData, colors=colors, 
                                   linewidths=lineWidths, linestyles=lineStyles)
            ax.add_collection(collLines)
            if axesEqual:
                ax.set_aspect('equal', 'box') #for 2D only

        if len(triangles) != 0:
            print('WARNING: PlotImage: triangles are ignored; they can only be plotted if plot3D=True')
            
    else: #plot in 3D
        ax = fig.gca(projection='3d')
        plotData = []
        colors = []
        from mpl_toolkits.mplot3d.art3d import Line3DCollection

        for i, line3D in enumerate(linePoints):
            # print('line3D:', line3D)
            nPoints = int(len(line3D)/3)

            line = (A @ np.array(line3D).reshape((nPoints,3)).T).T.flatten()
            color = lineColors[i]
            x = np.zeros(nPoints)
            y = np.zeros(nPoints)
            z = np.zeros(nPoints)
            for j in range(nPoints):
                x[j] = float(line[j*3+0]+p0[0])
                y[j] = float(line[j*3+1]+p0[1])
                z[j] = float(line[j*3+2]+p0[2])
                
            for j in range(nPoints-1):
                plotData += [[(x[j], y[j], z[j]), (x[j+1], y[j+1], z[j])]] #for plot
                colors += [color]

        if plotData != []:
            collLines = Line3DCollection(plotData, colors=colors, 
                                   linewidths=lineWidths, linestyles=lineStyles)
            ax.add_collection3d(collLines)

        ##Poly3DCollection seems not to work!
        if len(triangles) != 0:
            triangle_vertices = []
            colors = []
            #from mpl_toolkits.mplot3d import Axes3D
            from mpl_toolkits.mplot3d.art3d import Poly3DCollection
            pOff = np.array([list(p0),list(p0),list(p0)]).T
            for i, trig in enumerate(triangles):
                trigPoints = (A @ np.array(trig[0]).reshape(3,3).T+pOff).T
                #trigPoints = np.array(trig[0]).reshape(3,3)
                triangle_vertices += [trigPoints]
                colors += [np.array(trig[1][0:3])]
    
            # fig = plt.figure()
            # ax = fig.gca(projection='3d')
            edgeColors = triangleEdgeColors
            if lineWidths == 0:
                edgeColors = 'none'
            collTrigs = Poly3DCollection(triangle_vertices, facecolors=colors, 
                                         edgecolors=edgeColors, linewidths=triangleEdgeWidths)
            collTrigs._facecolors2d = ax._facecolor
            ax.add_collection(collTrigs)

        ax.view_init(elev=0., azim=0.)
        if removeAxes:
            ax.set_axis_off()
        if orthogonalProjection:
            ax.set_proj_type('ortho') #this is better for e.g. xy view

        if axesEqual:
            ax.set_aspect('auto') 
            ax.set_box_aspect([1,1,1])

        ax.autoscale()
    #end 3D plotting

    if title!='':
        plt.title(title)

    plt.autoscale()
    # plt.margins(0.1)

    plt.tight_layout() #not needed
    if matplotlib.get_backend() != 'agg': #this is used to avoid showing the figures, if they are just saved
        plt.show() 
    
    if fileName != '':
        try:
            os.makedirs(os.path.dirname(fileName), exist_ok=True)
        except:
            pass #makedirs may fail on some systems, but we keep going
        plt.savefig(fileName)




