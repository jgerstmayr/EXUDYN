#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Basic support functions for simpler creation of Exudyn models.
#			Advanced functions for loading and animating solutions and for drawing a graph of the mbs system.
#           This library requires numpy (as well as time and copy)
#
# Author:   Johannes Gerstmayr
# Date:     2019-07-26 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++import sys
# Utility functions and structures for Exudyn

import numpy as np #LoadSolutionFile
import time        #AnimateSolution
import copy as copy #to be able to copy e.g. lists
from math import sin, cos, pi

import exudyn
from exudyn.basicUtilities import *
from exudyn.rigidBodyUtilities import *
from exudyn.graphicsDataUtilities import *
from exudyn.itemInterface import *

#for compatibility with older models:
from exudyn.beams import GenerateStraightLineANCFCable2D, GenerateSlidingJoint, GenerateAleSlidingJoint

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: helper functions for matplotlib, returns a list of 28 line codes to be used in plot, e.g. 'r-' for red solid line
#**input: index in range(0:28)
#**output: a color and line style code for matplotlib plot
def PlotLineCode(index):
    CC = ['r-','g-','b-','k-','c-','m-','y-','r:','g:','b:','k:','c:','m:','y:','r--','g--','b--','k--','c--','m--','y--','r-.','g-.','b-.','k-.','c-.','m-.','y-.']
    if index < len(CC):
        return CC[index]
    else:
        return 'k:' #black line

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++       NUMPY BASED UTILITIES     ++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#**function: fill submatrix into given destinationMatrix; all matrices must be numpy arrays 
#**input: 
#  subMatrix: input matrix, which is filled into destinationMatrix
#  destinationMatrix: the subMatrix is entered here
#  destRow: row destination of subMatrix
#  destColumn: column destination of subMatrix
#**notes: may be erased in future!
#**output: destinationMatrix is changed after function call
def FillInSubMatrix(subMatrix, destinationMatrix, destRow, destColumn):
    nRows = subMatrix.shape[0]
    nColumns = subMatrix.shape[1]

    destinationMatrix[destRow:destRow+nRows, destColumn:destColumn+nColumns] = subMatrix

    #for i in range(nRows):
    #    for j in range(nColumns):
    #        destinationMatrix[i+destRow, j+destColumn] = subMatrix[i,j]


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute sin sweep at given time t
#**input: 
#  t: evaluate of sweep at time t
#  t1: end time of sweep frequency range
#  f0: start of frequency interval [f0,f1] in Hz
#  f1: end of frequency interval [f0,f1] in Hz
#**output: evaluation of sin sweep (in range -1..+1)
def SweepSin(t, t1, f0, f1):
    k = (f1-f0)/t1
    return sin(2*pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!

#**function: compute cos sweep at given time t
#**input: 
#  t: evaluate of sweep at time t
#  t1: end time of sweep frequency range
#  f0: start of frequency interval [f0,f1] in Hz
#  f1: end of frequency interval [f0,f1] in Hz
#**output: evaluation of cos sweep (in range -1..+1)
def SweepCos(t, t1, f0, f1):
    k = (f1-f0)/t1
    return cos(2*pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!

#**function: frequency according to given sweep functions SweepSin, SweepCos
#**input: 
#  t: evaluate of frequency at time t
#  t1: end time of sweep frequency range
#  f0: start of frequency interval [f0,f1] in Hz
#  f1: end of frequency interval [f0,f1] in Hz
#**output: frequency in Hz
def FrequencySweep(t, t1, f0, f1):
    return t*(f1-f0)/t1 + f0

#**function: step function with smooth transition from value0 to value1; transition is computed with cos function
#**input:
#  x: argument at which function is evaluated
#  x0: start of step (f(x) = value0)
#  x1: end of step (f(x) = value1)
#  value0: value before smooth step
#  value1: value at end of smooth step
#**output: returns f(x)
def SmoothStep(x, x0, x1, value0, value1): 
    loadValue = value0

    if x > x0:
        if x < x1:
            dx = x1-x0
            loadValue = value0 + (value1-value0) * 0.5*(1-cos((x-x0)/dx*pi)) 
        else:
            loadValue = value1
    return loadValue

#**function: derivative of SmoothStep using same arguments
#**input:
#  x: argument at which function is evaluated
#  x0: start of step (f(x) = value0)
#  x1: end of step (f(x) = value1)
#  value0: value before smooth step
#  value1: value at end of smooth step
#**output: returns d/dx(f(x))
def SmoothStepDerivative(x, x0, x1, value0, value1): 
    loadValue = 0

    if x > x0 and x < x1:
        dx = x1-x0
        loadValue = (value1-value0) * 0.5*(pi/dx*sin((x-x0)/dx*pi)) 
    return loadValue

#**function: get index from value in given data vector (numpy array); usually used to get specific index of time vector; this function is slow (linear search), if sampling rate is non-constant; otherwise set assumeConstantSampleRate=True!
#**input: 
#  data: containing (almost) equidistant values of time
#  value: e.g., time to be found in data
#  tolerance: tolerance, which is accepted (default: tolerance=1e-7)
#  rangeWarning: warn, if index returns out of range; if warning is deactivated, function uses the closest value
#**notes: to obtain the interpolated value of a time-signal array, use GetInterpolatedSignalValue() in exudyn.signalProcessing
#**output: index
def IndexFromValue(data, value, tolerance=1e-7, assumeConstantSampleRate=False, rangeWarning=True):
    index  = -1
    
    if assumeConstantSampleRate and len(data) > 1:
        dt = data[1] - data[0]
        if dt == 0.:
            raise ValueError('IndexFromValue: sample rate is zero!')
        index = int((value-data[0]) / dt)
        if index < 0:
            index = 0
            if rangeWarning:
                print('Warning: IndexFromValue: index returned smaller than 0; using 0 instead')
        elif index >= len(data):
            if rangeWarning:
                print('Warning: IndexFromValue: index returned larger than array length-1; using array max length-1 instead')
            index = len(data)-1
            
    else:
        bestTol = 1e37
        for i in range(len(data)):
            if abs(data[i] - value) < min(bestTol, tolerance):
                index = i
                bestTol = abs(data[i] - value)

    if index == -1:
        raise ValueError("IndexFromValue: value not found with given tolerance")
    return index



#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: set all entries in matrix to zero which are smaller than given treshold; operates directly on matrix
#**input: matrix as np.array, treshold as positive value
#**output: changes matrix
def RoundMatrix(matrix, treshold = 1e-14):
    (rows, cols) = matrix.shape
    for i in range (rows):
        for j in range(cols):
            if abs(matrix[i,j]) < treshold:
                matrix[i,j]=0

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: compute (3 x 3*n) skew matrix from (3*n) vector; used for ObjectFFRF and CMS implementation
#**input: a vector v in np.array format, containing 3*n components
#**output: (3 x 3*n) skew matrix in np.array format
def ComputeSkewMatrix(v):
    if v.ndim == 1:
        n = int(len(v)/3) #number of nodes
        sm = np.zeros((3*n,3))

        for i in range(n):
            off = 3*i
            x=v[off+0]
            y=v[off+1]
            z=v[off+2]
            mLoc = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
            sm[off:off+3,:] = mLoc[:,:]
    
        return sm
    elif v.ndim==2: #dim=2
        (nRows,nCols) = v.shape
        n = int(nRows/3) #number of nodes
        sm = np.zeros((3*n,3*nCols))

        for j in range(nCols):
            for i in range(n):
                off = 3*i
                x=v[off+0,j]
                y=v[off+1,j]
                z=v[off+2,j]
                mLoc = np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
                sm[off:off+3,3*j:3*j+3] = mLoc[:,:]
    else: 
        print("ERROR: wrong dimension in ComputeSkewMatrix(...)")
    return sm

#tests for ComputeSkewMatrix
#x = np.array([1,2,3,4,5,6])
#print(ComputeSkewMatrix(x))
#x = np.array([[1,2],[3,4],[5,6],[1,2],[3,4],[5,6]])
#print(ComputeSkewMatrix(x))


#**function: check if input is list or array with according length; if length==-1, the length is not checked; raises Exception if the check fails
#**input: 
#  vector: a vector in np.array or list format
#  length: desired length of vector; if length=-1, it is ignored
#**output: None
def CheckInputVector(vector, length=-1):
    try:
        v = np.array(vector)
    except:
        raise ValueError("CheckInputVector: expected vector/list of length "+str(length)+
                         ", but received '"+str(vector)+"'")
    if v.ndim != 1:
        raise ValueError("CheckInputVector: expected 1D vector/list of length "+str(length)+
                         ", but received '"+str(vector)+"'")
    if v.shape[0] != length:
        raise ValueError("CheckInputVector: expected vector/list of length "+str(length)+
                         ", but received vector with length "+str(v.shape[0])+", vector='"+str(vector)+"'")
        
#**function: check if input is list or array with according length and positive indices; if length==-1, the length is not checked; raises Exception if the check fails
#**input: 
#  indexArray: a vector in np.array or list format
#  length: desired length of vector; if length=-1, it is ignored
#**output: None
def CheckInputIndexArray(indexArray, length=-1):
    try:
        v = np.array(indexArray)
    except:
        raise ValueError("CheckInputIndexArray: expected vector/list of length "+str(length)+
                         ", but received '"+str(indexArray)+"'")
    if v.ndim != 1:
        raise ValueError("CheckInputIndexArray: expected 1D vector/list of length "+str(length)+
                         ", but received '"+str(indexArray)+"'")
    if v.shape[0] != length:
        raise ValueError("CheckInputIndexArray: expected vector/list of length "+str(length)+
                         ", but received vector with length "+str(v.shape[0])+", indexArray='"+str(indexArray)+"'")
    
    cnt=0
    for i in indexArray:
        if i!=int(i) or i < 0:
            raise ValueError("CheckInputIndexArray: expected array/list of positive indices (int), but received '"+str(i)+
                             "' at position " + str(cnt)+", indexArray='"+str(indexArray)+"'")
        cnt+=1   

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#INSPECTION:
        
#**function: simple function to find object index i within the local or global scope of variables
#**input: i, the integer object number and  globalVariables=globals()
#**example:
# FindObjectIndex(2, locals() )  #usually sufficient
# FindObjectIndex(2, globals() ) #wider search
def FindObjectIndex(i, globalVariables):
    #run through all variables and check if an object index exists
    found = False
    for varname in globalVariables:
        if type(globalVariables[varname]) == exudyn.ObjectIndex:
            if int(globalVariables[varname]) == int(i):
                print("variable '"+varname+"' links to object "+str(i) )
                found = True
    if not found:
        print("no according variable found")

#**function: simple function to find node index i within the local or global scope of variables
#**input: i, the integer node number and  globalVariables=globals()
#**example:
# FindObjectIndex(2, locals() )  #usually sufficient
# FindObjectIndex(2, globals() ) #wider search
def FindNodeIndex(i, globalVariables):
    #run through all variables and check if an object index exists
    found = False
    for varname in globalVariables:
        if type(globalVariables[varname]) == exudyn.NodeIndex:
            if int(globalVariables[varname]) == int(i):
                print("variable '"+varname+"' links to node "+str(i) )
                found = True
    if not found:
        print("no according variable found")


#**function: function to hide all objects in mbs except for those listed in objectNumbers
#**input: 
#  mbs: mbs containing object
#  objectNumbers: integer object number or list of object numbers to be shown; if empty list [], then all objects are shown
#  showOthers: if True, then all other objects are shown again
#**output: changes all colors in mbs, which is NOT reversible
def ShowOnlyObjects(mbs, objectNumbers=[], showOthers=False):
    if not isinstance(objectNumbers,list):
        listObjects = [objectNumbers]
    else:
        listObjects = objectNumbers
    isEmpty = len(listObjects) == 0
    
    for objectIndex in range(mbs.systemData.NumberOfObjects()):
        oDict = mbs.GetObject(objectIndex)
        flag = showOthers
        if objectIndex in listObjects or isEmpty: #if no objects to show,  
            flag = not showOthers
        if 'Vshow' in oDict:
            mbs.SetObjectParameter(objectIndex,'Vshow', flag)
    mbs.SendRedrawSignal()

#**function: highlight a certain item with number itemNumber; set itemNumber to -1 to show again all objects
#**input: 
#  mbs: mbs containing object
#  itemNumbers: integer object/node/etc number to be highlighted
#  itemType: type of items to be highlighted
#  showNumbers: if True, then the numbers of these items are shown
def HighlightItem(SC, mbs, itemNumber, itemType=exudyn.ItemType.Object, showNumbers=True):
    SC.visualizationSettings.interactive.highlightItemIndex = itemNumber
    SC.visualizationSettings.interactive.highlightItemType = itemType
    if showNumbers and itemType == exudyn.ItemType.Node:
        SC.visualizationSettings.nodes.showNumbers = True
        SC.visualizationSettings.nodes.show = True
    else:
        SC.visualizationSettings.nodes.showNumbers = False
    if showNumbers and itemType == exudyn.ItemType.Object:
        SC.visualizationSettings.bodies.showNumbers = True
        SC.visualizationSettings.connectors.showNumbers = True
        SC.visualizationSettings.bodies.show = True
        SC.visualizationSettings.connectors.show = True
    else:
        SC.visualizationSettings.bodies.showNumbers = False
        SC.visualizationSettings.connectors.showNumbers = False
    if showNumbers and itemType == exudyn.ItemType.Marker:
        SC.visualizationSettings.markers.showNumbers = True
        SC.visualizationSettings.markers.show = True
    else:
        SC.visualizationSettings.markers.showNumbers = False
    if showNumbers and itemType == exudyn.ItemType.Load:
        SC.visualizationSettings.loads.showNumbers = True
        SC.visualizationSettings.loads.show = True
    else:
        SC.visualizationSettings.loads.showNumbers = False
    if showNumbers and itemType == exudyn.ItemType.Sensor:
        SC.visualizationSettings.sensors.showNumbers = True
        SC.visualizationSettings.sensors.show = True
    else:
        SC.visualizationSettings.sensors.showNumbers = False

    mbs.SendRedrawSignal()


#**function: Internal SensorUserFunction, used in function AddSensorRecorder
def UFsensorRecord(mbs, t, sensorNumbers, factors, configuration):
    iSensor = sensorNumbers[0]
    val = mbs.GetSensorValues(iSensor, configuration=configuration) #get all values
    if type(val) == float:# or type(x) == nd.float64:
        val = np.array([val]) #for scalar values
    ti = int((t+1e-9)/factors[0]) #add 1e-10 safety factor due to rounding errors when adding time steps (may lead to small errors after 1e7 steps)
    if ti >= 0 and ti < len(mbs.variables['sensorRecord'+str(iSensor)]):
        mbs.variables['sensorRecord'+str(iSensor)][ti,0] = t
        mbs.variables['sensorRecord'+str(iSensor)][ti,1:] = val
        
    return val #return value usually not used further

#**function: Add a SensorUserFunction object in order to record sensor output internally; this avoids creation of files for sensors, which can speedup and simplify evaluation in ParameterVariation and GeneticOptimization; values are stored internally in mbs.variables['sensorRecord'+str(sensorNumber)] where sensorNumber is the mbs sensor number
#**input: 
#  mbs: mbs containing object
#  sensorNumber: integer sensor number to be recorded
#  endTime: end time of simulation, as given in simulationSettings.timeIntegration.endTime 
#  sensorsWritePeriod: as given in simulationSettings.solutionSettings.sensorsWritePeriod
#  sensorOutputSize: size of sensor data: 3 for Displacement, Position, etc. sensors; may be larger for RotationMatrix or Coordinates sensors; check this size by calling mbs.GetSensorValues(sensorNumber)
#**output: adds an according SensorUserFunction sensor to mbs; returns new sensor number; during initialization a new numpy array is allocated in  mbs.variables['sensorRecord'+str(sensorNumber)] and the information is written row-wise: [time, sensorValue1, sensorValue2, ...]
#**notes: This sensor usually just passes through values of an existing sensor, while recording the values to a numpy array row-wise (time in first column, data in remaining columns)
def AddSensorRecorder(mbs, sensorNumber, endTime, sensorsWritePeriod, sensorOutputSize=3):
    nSteps = int(endTime/sensorsWritePeriod)
    mbs.variables['sensorRecord'+str(sensorNumber)] = np.zeros((nSteps+1,1+sensorOutputSize)) #time+3 sensor values

    sUserRecord = mbs.AddSensor(SensorUserFunction(sensorNumbers=[sensorNumber], 
                                                   factors=[sensorsWritePeriod],
                                                   writeToFile=False,
                                                   sensorUserFunction=UFsensorRecord))
    
    return sUserRecord    
    
    
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++   LOAD SOLUTION AND ANIMATION   ++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#++++++++++++++++++++++++++++++++++++++++++++
#**function: read coordinates solution file (exported during static or dynamic simulation with option exu.SimulationSettings().solutionSettings.coordinatesSolutionFileName='...') into dictionary:
#**input: 
#  fileName: string containing directory and filename of stored coordinatesSolutionFile
#  saveMode: if True, it loads lines directly to load inconsistent lines as well; use this for huge files (>2GB); is slower but needs less memory!
#  verbose: if True, some information is written when importing file (use for huge files to track progress)
#  maxRows: maximum number of data rows loaded, if saveMode=True; use this for huge files to reduce loading time; set -1 to load all rows
#**output: dictionary with 'data': the matrix of stored solution vectors, 'columnsExported': a list with integer values showing the exported sizes [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData], 'nColumns': the number of data columns and 'nRows': the number of data rows
def LoadSolutionFile(fileName, safeMode=False, maxRows=-1, verbose=True):

    #check if is binary or ASCII
    isBinary = False
    with open(fileName, 'r') as file:
        data = np.fromfile(file, dtype=np.byte, count=6)
        if data.size==6:
            s = NumpyInt8ArrayToString(data)
            if s=='EXUBIN':
                isBinary=True

    if isBinary:
        return LoadBinarySolutionFile(fileName,maxRows,verbose)
    
    #read HEADER
    with open(fileName) as fileRead:
        # fileRead=open(fileName,'r') 
        fileLines = []
        fileLines += [fileRead.readline()]
        fileLines += [fileRead.readline()]
        fileLines += [fileRead.readline()]
        fileLines += [fileRead.readline()]
        fileLines += [fileRead.readline()]
        # fileRead.close()

    if len(fileLines[4]) == 0:
        raise ValueError('ERROR in LoadSolution: file empty or header missing')
        
    leftStr=fileLines[4].split('=')[0]
    if leftStr[0:30] != '#number of written coordinates': 
        raise ValueError('ERROR in LoadSolution: file header corrupted')

    columnsExported = eval(fileLines[4].split('=')[1]) #load according column information into vector: [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData]
    nColumns = sum(columnsExported)

    #read DATA
    if not safeMode:
        data = np.loadtxt(fileName, comments='#', delimiter=',')
    else:
        #alternative, but needs factor 5 times the memory of data loaded:
        #  data = np.genfromtxt(fileName,comments='#',delimiter=',',invalid_raise=False)

        with open(fileName) as file:
            lines = file.readlines()
        
        if verbose: print('text file loaded ... converting ...')
        
        cntDataRows = 0
        dataRowStart = -1
        cnt = 0
        for line in lines: 
            if line[0]!='#': 
                if dataRowStart == -1:
                    dataRowStart = cnt
                cntDataRows+=1
            cnt+=1
            
        if maxRows != -1 and cntDataRows > maxRows:
            cntDataRows = maxRows
        
        if cntDataRows == 0 or dataRowStart == -1:
            raise ValueError('LoadSolutionFile: no rows found')
        else:
            print('data starts at',dataRowStart, ', found', cntDataRows, 'rows')

        if verbose: print('check columns ...')
        
        cols = len(lines[dataRowStart].split(','))
        if cols != nColumns+1:
            raise ValueError('ERROR in LoadSolution: number of columns in first data row is inconsistent: got ',cols,' columns, but expected ', nColumns+1)
        
        #check last line, which may be incomplete:
        colsLastLine = len(lines[dataRowStart+cntDataRows-1].split(','))
        skipLast = 0
        if colsLastLine != cols:
            print('LoadSolution: WARNING number of columns in last data row is inconsistent; will be skipped')
            skipLast = 1
        
        if verbose: print('file contains ',cntDataRows, ' rows and ', cols, ' columns (incl. time)',sep='')
        
        data = np.zeros((cntDataRows, nColumns+1))
        
        progress = 0
        progressInfo = 5000000
        for i in range(cntDataRows-skipLast):
            if verbose and progress>=progressInfo: #update progress
                print('import data row', i, '/', cntDataRows)
                progress = 0
            progress+=nColumns
            ylist = lines[i+dataRowStart].split(',')
            if len(ylist) == nColumns+1:
                y=np.array(ylist, dtype=float)
                data[i,:] = y[:]
            elif verbose:
                print('  data row', i, 'is inconsistent ... skipped')

    if verbose: print('columns imported =', columnsExported)
    if verbose: print('total columns to be imported =', nColumns, ', array size of file =', np.size(data,1))

    if (nColumns + 1) != np.size(data,1): #one additional column for time!
        raise ValueError('ERROR in LoadSolution: number of columns is inconsistent')

    nRows = np.size(data,0)

    return dict({'data': data, 'columnsExported': columnsExported,'nColumns': nColumns,'nRows': nRows})


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
#helper functions for reading binary files:

#+++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: simple conversion of int8 arrays into strings (not highly efficient, so use only for short strings)
def NumpyInt8ArrayToString(npArray):
    s=''
    for x in npArray:
        s+=chr(x)
    return s

#**function: read single Index from current file position in binary solution file
def BinaryReadIndex(file, intType):
    data = np.fromfile(file, dtype=intType, count=1)
    if data.size != 1: return [0,True] #end of file
    return [data[0], False]

#**function: read single Real from current file position in binary solution file
def BinaryReadReal(file, realType):
    data = np.fromfile(file, dtype=realType, count=1)
    if data.size != 1: return [0,True] #end of file
    return [data[0], False]

#**function: read string from current file position in binary solution file
def BinaryReadString(file, intType):
    dataLength = np.fromfile(file, dtype=intType, count=1)[0]
    data = np.fromfile(file, dtype=np.byte, count=dataLength)
    return [NumpyInt8ArrayToString(data), False]

#**function: read Index array from current file position in binary solution file
def BinaryReadArrayIndex(file, intType):
    dataLength = np.fromfile(file, dtype=intType, count=1)[0]
    data = np.fromfile(file, dtype=intType, count=dataLength)
    return [data, False]

#**function: read Real vector from current file position in binary solution file
#**output: return data as numpy array, or False if no data read
def BinaryReadRealVector(file, intType, realType):
    sizeData = np.fromfile(file, dtype=intType, count=1)
    if sizeData.size != 1: return [[],True] #end of file
    dataLength = sizeData[0]
    data = np.fromfile(file, dtype=realType, count=dataLength)
    if data.size != dataLength: return [[],True] #end of file
    return [data, False]


#++++++++++++++++++++++++++++++++++++++++++++
#**function: read BINARY coordinates solution file (exported during static or dynamic simulation with option exu.SimulationSettings().solutionSettings.coordinatesSolutionFileName='...') into dictionary
#**input: 
#  fileName: string containing directory and filename of stored coordinatesSolutionFile
#  verbose: if True, some information is written when importing file (use for huge files to track progress)
#  maxRows: maximum number of data rows loaded, if saveMode=True; use this for huge files to reduce loading time; set -1 to load all rows
#**output: dictionary with 'data': the matrix of stored solution vectors, 'columnsExported': a list with integer values showing the exported sizes [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData], 'nColumns': the number of data columns and 'nRows': the number of data rows
def LoadBinarySolutionFile(fileName, maxRows=-1, verbose=True):
    print('verbose=',verbose)
    with open(fileName, 'r') as file:
        data = np.fromfile(file, dtype=np.byte, count=6)
        s = NumpyInt8ArrayToString(data)
        if int(verbose)>1: print(s)
        if s!='EXUBIN':
            raise ValueError('LoadBinarySolutionFile: no binary header found!')

        if verbose: print('read binary file')
        fileEnd = False
# 			ExuFile::BinaryWriteHeader(solFile, bfs);
        dataHeader = np.fromfile(file, dtype=np.byte, count=10)
        #dataHeader[0] == '\n'
        indexSize = int(dataHeader[1])
        realSize = int(dataHeader[2])
        pointerSize = int(dataHeader[3])
        bigEndian = int(dataHeader[4])
        
        if indexSize==4:
            intType = np.int32
        elif indexSize==8:
            intType = np.int64
        else:
            raise ValueError('Read binary file: invalid Index type size!')
        
        if realSize==4:
            realType = np.float32
        elif realSize==8:
            realType = np.float64
        else:
            raise ValueError('LoadBinarySolutionFile: invalid Real type size!')
        
        if verbose>1: 
            print('  indexSize=',indexSize)
            print('  realSize=',realSize)
            print('  pointerSize=',pointerSize)
            print('  bigEndian=',bigEndian)

# 		ExuFile::BinaryWrite(EXUstd::exudynVersion, solFile, bfs);
        sVersion, fileEnd=BinaryReadString(file, intType)
        if verbose: print('  version=',sVersion)

# 		ExuFile::BinaryWrite(STDstring("Mode0000"), solFile, bfs); //change this in future to add new features
        sMode, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: print('  mode=',sMode)

# 			STDstring str = "Exudyn " + GetSolverName() + " ";
# 			if (isStatic) { str+="static "; }
# 			str+="solver solution file";
# 			ExuFile::BinaryWrite(str, solFile, bfs);
        sSolver, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: print('  solver=',sSolver)

# 			//solFile << "#simulation started=" << EXUstd::GetDateTimeString() << "\n";
# 			ExuFile::BinaryWrite(EXUstd::GetDateTimeString(), solFile, bfs);
        sTime, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: print('  data/time=',sTime)

# 			//not needed in binary format:
# 			//solFile << "#columns contain: time, ODE2 displacements";
# 			//if (solutionSettings.exportVelocities) { solFile << ", ODE2 velocities"; }
# 			//if (solutionSettings.exportAccelerations) { solFile << ", ODE2 accelerations"; }
# 			//if (nODE1) { solFile << ", ODE1 coordinates"; } //currently not available, but for future solFile structure necessary!
# 			//if (nVel1) { solFile << ", ODE1 velocities"; }
# 			//if (solutionSettings.exportAlgebraicCoordinates) { solFile << ", AE coordinates"; }
# 			//if (solutionSettings.exportDataCoordinates) { solFile << ", ODE2 velocities"; }
# 			//solFile << "\n";

# 			//solFile << "#number of system coordinates [nODE2, nODE1, nAlgebraic, nData] = [" <<
# 			//	nODE2 << "," << nODE1 << "," << nAE << "," << nData << "]\n"; //this will allow to know the system information, independently of coordinates written
# 			ArrayIndex sysCoords({nODE2, nODE1, nAE, nData});
# 			ExuFile::BinaryWrite(sysCoords, solFile, bfs);
        systemSizes, fileEnd=BinaryReadArrayIndex(file, intType)
        if verbose: print('  systemSizes=',systemSizes)


# 			//solFile << "#number of written coordinates [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData] = [" << //these are the exported coordinates line-by-line
# 			//	nODE2 << "," << nVel2 << "," << nAcc2 << "," << nODE1 << "," << nVel1 << "," << nAEexported << "," << nDataExported << "]\n"; //python convert line with v=eval(line.split('=')[1])
# 			ArrayIndex writtenCoords({ nODE2, nVel2, nAcc2, nODE1, nVel1, nAEexported, nDataExported });
# 			ExuFile::BinaryWrite(writtenCoords, solFile, bfs);
        columnsExported, fileEnd=BinaryReadArrayIndex(file, intType)
        if verbose: print('  columnsExported=',columnsExported)
        nColumns = sum(columnsExported) #total size of data per row
        
# 			//solFile << "#total columns exported  (excl. time) = " << totalCoordinates << "\n";
# 			ExuFile::BinaryWrite(totalCoordinates, solFile, bfs);
        totalCoordinates, fileEnd = BinaryReadIndex(file, intType)

# 			Index numberOfSteps;
# 			if (!isStatic) { numberOfSteps = timeint.numberOfSteps; }
# 			else { numberOfSteps = staticSolver.numberOfLoadSteps; }
# 			ExuFile::BinaryWrite(numberOfSteps, solFile, bfs);
        numberOfSteps, fileEnd = BinaryReadIndex(file, intType)
        
# 			//solution information: always export string, even if has zero length:
# 			ExuFile::BinaryWrite(solutionSettings.solutionInformation, solFile, bfs);
        solutionInformation, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: print('  solutionInformation="'+solutionInformation+'"')

# 			//add some checksum ...
# 			ExuFile::BinaryWrite(STDstring("EndOfHeader"), solFile, bfs);
# 			//next byte starts with solution
        EndOfHeader, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: print('  EndOfHeader found: ',EndOfHeader)
        if EndOfHeader!='EndOfHeader':
            raise ValueError('LoadBinarySolutionFile: EndOfHeader not found')


        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #read time steps
# 		if (isBinary) //add size, only in binary mode
# 		{
# 			//including 1 real for time+1 Index for nVectors, but excluding bytes for this Index
# 			Index lineSizeBytes = nVectors * bfs.indexSize + nValues * bfs.realSize + bfs.indexSize + bfs.realSize;
# 			ExuFile::BinaryWrite(lineSizeBytes, solFile, bfs); //size of line, for fast skipping of solution line
# 			ExuFile::BinaryWrite(nVectors, solFile, bfs); //number of vectors could vary if needed
# 		}
        if fileEnd: print('  ==> end of file found during in header')
        fileEnd = False
        data = np.zeros((0,nColumns+1))
        nRows = 0
        line = np.zeros(nColumns+1)
        validEndFound = False
        dataList = [] #list is much faster than hstack !
        
        while not fileEnd:
            #print('read row ',nRows)
            if maxRows != -1 and nRows >= maxRows:
                fileEnd = True
                break

            #includes time and all values for solution according to header    
            sizeData = np.fromfile(file, dtype=intType, count=1)
            if sizeData.size != 1: 
                fileEnd=True
                break
            dataLength = sizeData[0]
            if dataLength  == -1:
                if int(verbose)>1: print('end of file reached')
                validEndFound = True
                break
            if int(verbose)>1: 
                print('  dataLength=',dataLength)
                
            line = np.fromfile(file, dtype=realType, count=dataLength)
            if line.size != dataLength: 
                fileEnd=True
                break
            #line, fileEnd=BinaryReadRealVector(file, intType, realType)
            if int(verbose)>1: 
                print('  read',line.size, 'columns (',nColumns+1,'expected)')
                print('  line=',line)
            if fileEnd: break
            if line.size != nColumns+1:
                raise ValueError('LoadBinarySolutionFile: rows are inconsistent')
            
            nRows += 1

            dataList+=[line]
            #data = np.vstack((data, line)) #slow!

        if verbose: print('  read '+str(nRows)+' rows from file')
        
        data = np.array(dataList)
        nRows = np.size(data,0)

        if not validEndFound:
            print('LoadBinarySolutionFile: WARNING: end of file inconsistent!')
        else:
            if int(verbose)>1: 
                print('  valid end of data found')
                print('LoadBinarySolutionFile finished')
    
        return dict({'data': data, 'columnsExported': columnsExported,'nColumns': nColumns,'nRows': nRows})
        



#++++++++++++++++++++++++++++++++++++++++++++
#**function: recover solution file with last row not completely written (e.g., if crashed, interrupted or no flush file option set)
#**input: 
#  fileName: string containing directory and filename of stored coordinatesSolutionFile
#  newFileName: string containing directory and filename of new coordinatesSolutionFile
#  verbose: 0=no information, 1=basic information, 2=information per row
#**output: writes only consistent rows of file to file with name newFileName
def RecoverSolutionFile(fileName, newFileName, verbose=0):
    #read file header
    fileRead=open(fileName,'r') 
    fileLines = []
    fileLines += [fileRead.readline()]
    fileLines += [fileRead.readline()]
    fileLines += [fileRead.readline()]
    fileLines += [fileRead.readline()]
    fileLines += [fileRead.readline()]
    fileRead.close()
    if len(fileLines[4]) == 0:
        raise ValueError('ERROR in LoadSolution: file empty or header missing')
        
    leftStr=fileLines[4].split('=')[0]
    if leftStr[0:30] != '#number of written coordinates': 
        raise ValueError('ERROR in LoadSolution: file header corrupted')

    columnsExported = eval(fileLines[4].split('=')[1]) #load according column information into vector: [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData]
    nColumns = sum(columnsExported)
    expectedColumns = nColumns+1
    if verbose >= 1:
        print('columns imported =', columnsExported)
        print('total columns to be imported =', expectedColumns, '(incl. time)\n')


    with open(newFileName, 'w') as fileWrite:
        with open(fileName) as file:
            cnt = 0
            cntSolution = 0
            for line in file:
                if line[0] == '#':
                    if len(line) < 1000 and verbose >= 1:
                        print('HEADER:', line, end='')
                    fileWrite.write(line)
                else:
                    #cols = len(line.split(','))
                    cols = line.count(',')+1 #+1 needed, because two columns for one comma
                    if cols == expectedColumns:
                        if verbose >= 2:
                            print('data row ',cntSolution,', #cols=',cols, ', text=',line[0:12],'...', sep='')
                        fileWrite.write(line)
                    else:
                        if verbose >= 1:
                            print('\nWARNING: ignored solution data',cntSolution, '(file line',cnt,'), columns=', cols, '\n')
                    cntSolution += 1
                
                cnt += 1

#++++++++++++++++++++++++++++++++++++++++++++
#**function: recover initial coordinates, time, etc. from given restart file
#**input: 
#  mbs: MainSystem to be operated with
#  simulationSettings: simulationSettings which is updated and shall be used afterwards for SolveDynamic(...) or SolveStatic(...)
#  restartFileName: string containing directory and filename of stored restart file, as given in solutionSettings.restartFileName
#  verbose: False=no information, True=basic information
#**output: modifies simulationSettings and sets according initial conditions in mbs
def InitializeFromRestartFile(mbs, simulationSettings, restartFileName, verbose=True):
    raise ValueError('InitializeFromRestartFile: not fully implemented')

    fileRead=open(fileName,'r') 
    fileLines = fileRead.readlines()
    
    #fileLines = []
    #fileLines += [fileRead.readline()]
    #fileLines += [fileRead.readline()]
    #fileLines += [fileRead.readline()]
    #fileLines += [fileRead.readline()]
    #fileLines += [fileRead.readline()]
    fileRead.close()
    if len(fileLines[4]) == 0:
        raise ValueError('ERROR in InitializeFromRestartFile: file empty or header missing')
        
    leftStr=fileLines[4].split('=')[0]
    if leftStr[0:30] != '#number of written coordinates': 
        raise ValueError('ERROR in InitializeFromRestartFile: file header corrupted')

    columnsExported = eval(fileLines[4].split('=')[1]) #load according column information into vector: [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData]
    nColumns = sum(columnsExported)
    expectedColumns = nColumns+1
    if verbose:
        print('columns available in restart file =', columnsExported)
        print('total columns to be imported =', expectedColumns, '(incl. time)\n')

    if fileLines[-1][0:9]!='#FINISHED':
        raise ValueError('ERROR in InitializeFromRestartFile: last line does not contain "#FINISHED" and is thus expected to be corrupted!')

    #now everything should be ok and we can just read the line with numpy:
    data = np.loadtxt(fileName, comments='#', delimiter=',')
    nRows = np.size(data,0) #should be 1
    if nRows != 1:
        raise ValueError('ERROR in InitializeFromRestartFile: got more than one rows, but expected one')

    row = 0 #first row used 
    rowData = data[row]
    #cols = solution['columnsExported']
    [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData] = columnsExported

    #note that these visualization updates are not threading safe!
    mbs.systemData.SetODE2Coordinates(rowData[1:1+nODE2], configuration)
    if (nVel2): mbs.systemData.SetODE2Coordinates_t(rowData[1+nODE2:1+nODE2+nVel2], configuration)
    if (nAcc2): mbs.systemData.SetODE2Coordinates_tt(rowData[1+nODE2+nVel2:1+nODE2+nVel2+nAcc2], configuration)
    if (nODE1): mbs.systemData.SetODE1Coordinates(rowData[1+nODE2+nVel2+nAcc2:1+nODE2+nVel2+nAcc2+nODE1], configuration)
    if (nVel1): mbs.systemData.SetODE1Coordinates_t(rowData[1+nODE2+nVel2+nAcc2+nODE1:1+nODE2+nVel2+nAcc2+nODE1+nVel1], configuration)
    
    if (nAlgebraic): mbs.systemData.SetAECoordinates(rowData[1+nODE2+nVel2+nAcc2+nODE1+nVel1:1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic], configuration)
    if (nData): mbs.systemData.SetDataCoordinates(rowData[1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic:1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic+nData], configuration)

    if configuration == exudyn.ConfigurationType.Visualization:
        mbs.systemData.SetTime(rowData[0], exudyn.ConfigurationType.Visualization)
        mbs.SendRedrawSignal()
    
    #add integration parameters to simulationSettings ...
    
    if verbose: print('\nInitializeFromRestartFile finished\n')
    
#++++++++++++++++++++++++++++++++++++++++++++
#**function: load selected row of solution dictionary (previously loaded with LoadSolutionFile) into specific state; flag sendRedrawSignal is only used if configuration = exudyn.ConfigurationType.Visualization
def SetSolutionState(mbs, solution, row, configuration=exudyn.ConfigurationType.Current, sendRedrawSignal=True):
    if row < solution['nRows']:
        rowData = solution['data'][row]
        #cols = solution['columnsExported']
        [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData] = solution['columnsExported']

        #note that these visualization updates are not threading safe!
        mbs.systemData.SetODE2Coordinates(rowData[1:1+nODE2], configuration)
        if (nVel2): mbs.systemData.SetODE2Coordinates_t(rowData[1+nODE2:1+nODE2+nVel2], configuration)
        if (nAcc2): mbs.systemData.SetODE2Coordinates_tt(rowData[1+nODE2+nVel2:1+nODE2+nVel2+nAcc2], configuration)
        if (nODE1): mbs.systemData.SetODE1Coordinates(rowData[1+nODE2+nVel2+nAcc2:1+nODE2+nVel2+nAcc2+nODE1], configuration)
        if (nVel1): mbs.systemData.SetODE1Coordinates_t(rowData[1+nODE2+nVel2+nAcc2+nODE1:1+nODE2+nVel2+nAcc2+nODE1+nVel1], configuration)
        
        if (nAlgebraic): mbs.systemData.SetAECoordinates(rowData[1+nODE2+nVel2+nAcc2+nODE1+nVel1:1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic], configuration)
        if (nData): mbs.systemData.SetDataCoordinates(rowData[1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic:1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic+nData], configuration)

        if configuration == exudyn.ConfigurationType.Visualization:
            mbs.systemData.SetTime(rowData[0], exudyn.ConfigurationType.Visualization)
            mbs.SendRedrawSignal()
    else:
        print("ERROR in SetVisualizationState: invalid row (out of range)")

#++++++++++++++++++++++++++++++++++++++++++++
#**function: consecutively load the rows of a solution file and visualize the result
#**input: 
#  mbs: the system used for animation
#  solution: solution dictionary previously loaded with LoadSolutionFile; will be played from first to last row
#  rowIncrement: can be set larger than 1 in order to skip solution frames: e.g. rowIncrement=10 visualizes every 10th row (frame)
#  timeout: in seconds is used between frames in order to limit the speed of animation; e.g. use timeout=0.04 to achieve approximately 25 frames per second
#  createImages: creates consecutively images from the animation, which can be converted into an animation
#  runLoop: if True, the animation is played in a loop until 'q' is pressed in render window
#**output: renders the scene in mbs and changes the visualization state in mbs continuously
def AnimateSolution(mbs, solution, rowIncrement = 1, timeout=0.04, createImages = False, runLoop = False):
    SC = mbs.GetSystemContainer()
    nRows = solution['nRows']
    if nRows == 0:
        print('ERROR in AnimateSolution: solution file is empty')
        return
    if (rowIncrement < 1) or (rowIncrement > nRows):
        print('ERROR in AnimateSolution: rowIncrement must be at least 1 and must not be larger than the number of rows in the solution file')
    oldUpdateInterval = SC.visualizationSettings.general.graphicsUpdateInterval
    SC.visualizationSettings.general.graphicsUpdateInterval = 0.5*min(timeout, 2e-3) #avoid too small values to run multithreading properly
    mbs.SetRenderEngineStopFlag(False) #not to stop right at the beginning

    while runLoop and not mbs.GetRenderEngineStopFlag():
        for i in range(0,nRows,rowIncrement):
            if not(mbs.GetRenderEngineStopFlag()):
                #SetVisualizationState(exudyn, mbs, solution, i) #OLD
                SetSolutionState(mbs, solution, i, exudyn.ConfigurationType.Visualization)
                if createImages:
                    SC.RedrawAndSaveImage() #create images for animation
                #time.sleep(timeout)
                exudyn.DoRendererIdleTasks(timeout)

    SC.visualizationSettings.general.graphicsUpdateInterval = oldUpdateInterval #set values back to original




#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: helper function which draws system graph of a MainSystem (mbs); several options let adjust the appearance of the graph
#**input:
#   mbs: MainSystem to be operated with
#   showLoads: toggle appearance of loads in mbs
#   showSensors: toggle appearance of sensors in mbs
#   useItemNames: if True, object names are shown instead of basic object types (Node, Load, ...)
#   useItemTypes: if True, object type names (MassPoint, JointRevolute, ...) are shown instead of basic object types (Node, Load, ...); Note that Node, Object, is omitted at the beginning of itemName (as compared to theDoc.pdf); item classes become clear from the legend
#**output: [nx, G, items] with nx being networkx, G the graph and item what is returned by nx.draw\_networkx\_labels(...)
def DrawSystemGraph(mbs, showLoads=True, showSensors=True, 
                    useItemNames = False, useItemTypes = False):
    
    try:
        #all imports are part of anaconda (e.g. anaconda 5.2.0, python 3.6.5)
        import numpy as np
        import networkx as nx #for generating graphs and graph arrangement
        import matplotlib.pyplot as plt #for drawing
    except ImportError as e:
        raise ImportError("numpy, networkx and matplotlib required for DrawSystemGraph(...)") from e
    except :
        print("DrawSystemGraph(...): unexpected error during import of numpy, networkx and matplotlib")
        raise
    
    itemColors = {'Node':'red', 'Object':'skyblue', 'Oconnector':'dodgerblue', 'Ojoint':'dodgerblue', 'Ocontact':'dodgerblue', #turqoise, skyblue
                      'Marker': 'orange', 'Load': 'mediumorchid', 
                      'Sensor': 'forestgreen'} #https://matplotlib.org/examples/color/named_colors.html
    
    plt.clf()
    
    itemColorMap=[]     #color per item
    itemNames=[]        #name per item 
    itemTypes=[]        #name per item 
    edgeColorMap=[]
    nodesToItems=[]     #maps mbs-node numbers to item numbers
    markersToItems=[]   #maps mbs-marker numbers to item numbers
    objectsToItems=[]   #maps mbs-object numbers to item numbers
    loadsToItems=[]     #maps mbs-load numbers to item numbers
    sensorsToItems=[]   #maps mbs-sensor numbers to item numbers
    
    objectNodeColor = 'navy' #color for edges between nodes and objects, to be highlighted
            
    G = nx.Graph()
    
    showLegend = False
    addItemNames = False #Object, Node, ... not added but legend added
    if useItemTypes or useItemNames:
        showLegend = True

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    itemType = 'Node'
    n = mbs.systemData.NumberOfNodes()
    for i in range(n):
        item = mbs.GetNode(i)
        itemName=itemType+str(i)
        if item['nodeType'].find('Ground') != -1:
            itemName= 'G'+itemName
    
        if useItemNames:
            itemName=item['name']+str(i)
        elif useItemTypes:
            itemName=item['nodeType']+str(i)
            if addItemNames:
                itemName = 'Node' + itemName
    
    
        G.add_node(itemName) #attributes: size, weight, ...
        nodesToItems += [len(itemColorMap)]
        itemColorMap += [itemColors[itemType]]
        itemNames += [itemName]
        itemTypes += [itemType]
    
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    #add markers without edges
    itemType = 'Marker'
    n = mbs.systemData.NumberOfMarkers()
    for i in range(n):
        item = mbs.GetMarker(i)
        itemName=itemType+str(i)
        if useItemNames:
            itemName=item['name']+str(i)
        elif useItemTypes:
            itemName=item['markerType']+str(i)
            if addItemNames:
                itemName = 'Marker' + itemName
    
        G.add_node(itemName) #attributes: size, weight, ...
        markersToItems += [len(itemColorMap)]
        itemColorMap += [itemColors[itemType]]
        itemNames += [itemName]
        itemTypes += [itemType]
    
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    itemType = 'Object'
    n = mbs.systemData.NumberOfObjects()
    for i in range(n):
        objectType = itemType
        item = mbs.GetObject(i)
        if item['objectType'].find('Connector') != -1:
            objectType = 'Oconnector'
        elif item['objectType'].find('Contact') != -1:
            objectType = 'Ocontact'
        elif item['objectType'].find('Joint') != -1:
            objectType = 'Ojoint'
        itemName=objectType+str(i)
    
        if useItemNames:
            itemName=item['name']+str(i)
        elif useItemTypes:
            itemName=item['objectType']+str(i)
            if addItemNames:
                itemName = 'Object'+itemName
            
        G.add_node(itemName) #attributes: size, weight, ...
        objectsToItems += [len(itemColorMap)]
        itemNames += [itemName]
        itemTypes += [itemType]
        itemColorMap += [itemColors[objectType]]
    
    #    objectColor = ''
        #for objects: add edges to nodes
        nodeNumbers = []
        if 'nodeNumber' in item:
            nodeNumbers += [item['nodeNumber']]
        if 'nodeNumbers' in item:
            nodeNumbers += item['nodeNumbers']
    
        for j in range(len(nodeNumbers)):
            nodeNumbers[j] = int(nodeNumbers[j])
    
        for j in nodeNumbers:
            if j != exudyn.InvalidIndex(): #for RigidBodySpringDamper
                G.add_edge(itemNames[objectsToItems[i]],itemNames[nodesToItems[j]],color=objectNodeColor)
    
        #for connectors, contact, joint: add edges to these objects
        markerNumbers = []
        if 'markerNumbers' in item: #should only be markerNumbers ...
            markerNumbers += item['markerNumbers']
    
        for j in range(len(markerNumbers)):
            markerNumbers[j] = int(markerNumbers[j])
    
        for j in markerNumbers:
            G.add_edge(itemNames[objectsToItems[i]],itemNames[markersToItems[j]], color=itemColors['Oconnector'])
            
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    #now add only edges for markers:
    itemType = 'Marker'
    n = mbs.systemData.NumberOfMarkers()
    for i in range(n):
        objectType = itemType
        item = mbs.GetMarker(i)
    
        #for node markers:
        nodeNumbers = []
        if 'nodeNumber' in item:
            nodeNumbers += [item['nodeNumber']]
       
        for j in range(len(nodeNumbers)):
            nodeNumbers[j] = int(nodeNumbers[j])
    
        for j in nodeNumbers:
            G.add_edge(itemNames[markersToItems[i]],itemNames[nodesToItems[j]],color='orange')
    
        #for object markers:
        objectNumbers = []
        if 'objectNumber' in item: objectNumbers += [item['objectNumber']]
        if 'bodyNumber' in item: objectNumbers += [item['bodyNumber']]
       
        for j in range(len(objectNumbers)):
            objectNumbers[j] = int(objectNumbers[j])
    
        for j in objectNumbers:
            G.add_edge(itemNames[markersToItems[i]],itemNames[objectsToItems[j]],color='orange')
            
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    #add loads
    if showLoads:
        itemType = 'Load'
        n = mbs.systemData.NumberOfLoads()
        for i in range(n):
            item = mbs.GetLoad(i)
            itemName=itemType+str(i)
            if useItemNames:
                itemName=item['name']+str(i)
            elif useItemTypes:
                itemName=item['loadType']+str(i)
                if addItemNames:
                    itemName = 'Load'+itemName
        
            G.add_node(itemName) #attributes: size, weight, ...
            loadsToItems += [len(itemColorMap)]
            itemColorMap += [itemColors[itemType]]
            itemNames += [itemName]
            itemTypes += [itemType]
    
            markerNumbers = [int(item['markerNumber'])]
            
            for j in markerNumbers:
                G.add_edge(itemNames[loadsToItems[i]],itemNames[markersToItems[j]], color=itemColors['Load'])
    
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    #add sensors
    if showSensors:
        itemType = 'Sensor'
        n = mbs.systemData.NumberOfSensors() #only available for Exudyn version >= 1.0.15
        for i in range(n):
            item = mbs.GetSensor(i)
            itemName=itemType+str(i)
            if useItemNames:
                itemName=item['name']+str(i)
            elif useItemTypes:
                itemName=item['sensorType']+str(i)
                if addItemNames:
                    itemName = 'Sensor'+itemName
        
            G.add_node(itemName) #attributes: size, weight, ...
            sensorsToItems += [len(itemColorMap)]
            itemColorMap += [itemColors[itemType]]
            itemNames += [itemName]
            itemTypes += [itemType]
    
            #for object sensors:
            objectNumbers = []
            if 'objectNumber' in item: objectNumbers += [item['objectNumber']]
            if 'bodyNumber' in item: objectNumbers += [item['bodyNumber']]
           
            for j in range(len(objectNumbers)):
                objectNumbers[j] = int(objectNumbers[j])
        
            for j in objectNumbers:
                G.add_edge(itemNames[sensorsToItems[i]],itemNames[objectsToItems[j]],color=itemColors[itemType])

            #for node sensors:
            nodeNumbers = []
            if 'nodeNumber' in item: nodeNumbers += [int(item['nodeNumber'])]
                   
            for j in nodeNumbers:
                G.add_edge(itemNames[sensorsToItems[i]],itemNames[nodesToItems[j]],color=itemColors[itemType])
       
    if showLegend:
        legendColors = {'Node':'red', 'Object':'skyblue', 'Object(Connector)':'dodgerblue', 
                      'Marker': 'orange', 'Load': 'mediumorchid', 
                      'Sensor': 'forestgreen'} 
        #f = plt.figure(1)
        #ax = f.add_subplot(1,1,1)
        for label in legendColors:
            plt.plot([0],[0],linewidth=8,color=legendColors[label],label=label)
        plt.legend(fontsize=10)

    
    #now get out the right sorting of colors ...
    edgeColorMap = []
    edgeWidths = []
    edges=G.edges()
    for item in edges.items(): 
        edgeColorMap += [item[1]['color']] #color is in item[1], which is a dictionary ...
        edgeWidth = 2
        if item[1]['color'] == objectNodeColor: #object-node should be emphasized
            edgeWidth = 4
        edgeWidths += [edgeWidth]
    
    pos = nx.drawing.spring_layout(G, scale=0.5, k=3/sqrt(G.size()), threshold = 1e-5, iterations = 100)
    nx.draw_networkx_nodes(G, pos, node_size=1)
    nx.draw_networkx_edges(G, pos, edge_color=edgeColorMap, width=edgeWidths)#width=2)
    
    #reproduce what draw_networkx_labels does, allowing different colors for nodes
    #check: https://networkx.github.io/documentation/stable/_modules/networkx/drawing/nx_pylab.html
    items = nx.draw_networkx_labels(G, pos, font_size=10, clip_on=False, #clip at plot boundary
                                    bbox=dict(facecolor='skyblue', edgecolor='black', 
                                              boxstyle='round,pad=0.1', lw=10)) #lw is border size (no effect?)

    
    #now assign correct colors:
    for i in range(len(itemNames)):
        currentColor = itemColorMap[i]
        itemType = itemTypes[i]
        boxStyle = 'round,pad=0.2'
        fontSize = 10
        if itemType == 'Object':
            boxStyle = 'round,pad=0.2'
            fontSize = 12
        if itemType == 'Node':  
            boxStyle = 'square,pad=0.1'
            fontSize = 10
        if itemType == 'Marker':  
            boxStyle = 'square,pad=0.1'
            fontSize = 8
    
        #print("color=",currentColor)
        items[itemNames[i]].set_bbox(dict(facecolor=currentColor,  
              edgecolor=currentColor, boxstyle=boxStyle))
        items[itemNames[i]].set_fontsize(fontSize)
    
    plt.axis('off') #do not show frame, because usually some nodes are very close to frame ...
    plt.tight_layout()
    plt.margins(x=0.2, y=0.2) #larger margin, to avoid clipping of long texts
    plt.draw() #force redraw after colors have changed
    
    return [nx, G, items]

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#                              SPECIAL FUNCTIONS FOR EXUDYN
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#TCP/IP functionality
#**class: helper class for CreateTCPIPconnection and for TCPIPsendReceive
class TCPIPdata:
    def __init__(self, sendSize, receiveSize, packerSend, packerReceive, 
                  socketTCP, connection, address, lastReceiveTime):
        self.sendSize = sendSize
        self.receiveSize = receiveSize
        self.packerSend = packerSend
        self.packerReceive = packerReceive
        self.socket = socketTCP
        self.connection = connection
        self.address = address
        self.lastReceiveTime = lastReceiveTime #usually zero; used to make substeps in mbs
        
        
#**function: function which has to be called before simulation to setup TCP/IP socket (server) for 
#  sending and receiving data; can be used to communicate with other Python interpreters
#  or for communication with MATLAB/Simulink
#**input:
#  sendSize: number of double values to be sent to TCPIP client
#  receiveSize: number of double values to be received from TCPIP client
#  IPaddress: string containing IP address of client (e.g., '127.0.0.1')
#  port: port for communication with client
#  bigEndian: if True, it uses bigEndian, otherwise littleEndian is used for byte order
#**output: returns information (TCPIPdata class) on socket; recommended to store this in mbs.sys['TCPIPobject']
#**example:
# mbs.sys['TCPIPobject'] = CreateTCPIPconnection(sendSize=3, receiveSize=2, 
#                                                bigEndian=True, verbose=True)
# sampleTime = 0.01 #sample time in MATLAB! must be same!
# mbs.variables['tLast'] = 0 #in case that exudyn makes finer steps than sample time

# def PreStepUserFunction(mbs, t):
#     if t >= mbs.variables['tLast'] + sampleTime:
#         mbs.variables['tLast'] += sampleTime

#         tcp = mbs.sys['TCPIPobject']
#         y = TCPIPsendReceive(tcp, np.array([t, np.sin(t), np.cos(t)])) #time, torque
#         tau = y[1]
#         print('tau=',tau)
#     return True


# try:
#     mbs.SetPreStepUserFunction(PreStepUserFunction)
    
#     #%%++++++++++++++++++++++++++++++++++++++++++++++++++
#     mbs.Assemble()
#     [...] #start renderer; simulate model
# finally: #use this to always close connection, even in case of errors
#     CloseTCPIPconnection(mbs.sys['TCPIPobject'])
#
# #*****************************************
# #the following settings work between Python and MATLAB-Simulink (client), and gives stable results(with only delay of one step):
# # TCP/IP Client Send:
# #   priority = 2 (in properties)
# #   blocking = false
# #   Transfer Delay on (but off also works)
# # TCP/IP Client Receive:
# #   priority = 1 (in properties)
# #   blocking = true
# #   Sourec Data type = double
# #   data size = number of double in packer
# #   Byte order = BigEndian
# #   timeout = 10
def CreateTCPIPconnection(sendSize, receiveSize, IPaddress='127.0.0.1', port=52421, 
                          bigEndian=False, verbose=False):
    import socket
    import struct
    s = ''
    if bigEndian:
        s = '>' #signals bigEndian format
    packerSend = struct.Struct(s+'d '*sendSize) #'>' for big endian in matlab, I=unsigned int, i=int, d=double
    packerReceive = struct.Struct(s+'d '*receiveSize) #'>' for big endian in matlab, I=unsigned int, i=int, d=double
    if verbose:
        print('setup TCP/IP socket ...')
    socketTCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socketTCP.bind((IPaddress, port))
    socketTCP.listen()
    connection, address = socketTCP.accept()

    if verbose:
        print('TCP/IP connection running!')

    return TCPIPdata(sendSize, receiveSize, packerSend, packerReceive, 
                     socketTCP, connection, address, 0.)

#**function: call this function at every simulation step at which you intend to communicate with
#  other programs via TCPIP; e.g., call this function in preStepUserFunction of a mbs model
#**input:
#  TCPIPobject: the object returned by CreateTCPIPconnection(...)
#  sendData: numpy array containing data (double array) to be sent; must agree with sendSize
#**output: returns array as received from TCPIP
#**example:
#mbs.sys['TCPIPobject']=CreateTCPIPconnection(sendSize=2, receiveSize=1, IPaddress='127.0.0.1')
#y = TCPIPsendReceive(mbs.sys['TCPIPobject'], np.array([1.,2.]))
#print(y)
#
def TCPIPsendReceive(TCPIPobject, sendData):
    #first send data (no other way in MATLAB):
    TCPIPobject.connection.sendall(TCPIPobject.packerSend.pack(*sendData))

    #now receive data:
    data = TCPIPobject.connection.recv(TCPIPobject.packerReceive.size) #data size in bytes
    if not data:
        print('WARNING: TCPIPsendReceive: loss of data') #usually does not happen!
        return np.zeros(TCPIPobject.receiveSize)
    else:
        return TCPIPobject.packerReceive.unpack(data)

#**function: close a previously created TCPIP connection
def CloseTCPIPconnection(TCPIPobject):
    TCPIPobject.connection.close()
    TCPIPobject.socket.close()
    


