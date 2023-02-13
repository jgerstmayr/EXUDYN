#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details: 	Advanced utility functions only depending on numpy or specified exudyn modules;
#           Here, we gather special functions, which are depending on other modules and do not fit into exudyn.utilities as they cannot be imported e.g. in rigidBodyUtilities
#
# Author:   Johannes Gerstmayr
# Date:     2023-01-06 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import numpy as np
from math import sin, cos, pi, sqrt
from enum import Enum

import exudyn

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#GENERAL FUNCTIONS
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#**function: helper functions for matplotlib, returns a list of 28 line codes to be used in plot, e.g. 'r-' for red solid line
#**input: index in range(0:28)
#**output: a color and line style code for matplotlib plot
def PlotLineCode(index):
    CC = ['r-','g-','b-','k-','c-','m-','y-','r:','g:','b:','k:','c:','m:','y:','r--','g--','b--','k--','c--','m--','y--','r-.','g-.','b-.','k-.','c-.','m-.','y-.']
    if index < len(CC):
        return CC[index]
    else:
        return 'k:' #black line


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#INSPECTION, needs numpy and exudyn:
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        
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

#**function: checks, if data is of type list or np.array; used in functions to check input data
#**input:
#  data: any type, preferrably list or numpy.array
#  checkIfNoneEmpty: if True, function only returns True if type is list or array AND if length is non-zero
#**output: returns True/False
def IsListOrArray(data, checkIfNoneEmpty=False):
    if isinstance(data,list) or isinstance(data,np.ndarray):
        if checkIfNoneEmpty and len(data) == 0:
            return False
        return True
    else:
        return False


#**class: internal type which is used for type checking in exudyn Python user functions; used to create unique error messages
class ExpectedType(Enum):
    _None = 0
    Positive = 1
    Unsigned = 2
    Bool = 4
    Int = 8
    Real = 16
    Vector = 32
    Matrix = 64
    RigidBodyInertia = 128
    NodeIndex = 256
    ObjectIndex = 512
    MarkerIndex = 1024
    LoadIndex = 2048
    SensorIndex = 4096

#**function: internal function which is used to raise common errors in case of wrong types; dim is used for vectors and square matrices, cols is used for non-square matrices
def RaiseTypeError(where='', argumentName='', received = None, expectedType = None, dim=None, cols=None):
    errStr = 'ERROR in ' + where + ' in argument ' + argumentName + ': expected type=' + expectedType.name
    if expectedType == ExpectedType.Vector:
        errStr += ' (as list or numpy array)'
    elif expectedType == ExpectedType.Matrix:
        errStr += ' (as list of lists or numpy array)'
        
    if dim != None:
        ncols = cols
        if expectedType == ExpectedType.Matrix and cols == None: 
            ncols = dim  #square Matrix
        if ncols != None:
            errStr += ', expected size = (' + str(dim) + ',' + str(ncols) + ')'
        else:
            errStr += ', expected size = ' + str(dim) 
            
    receivedStr = ', <argument can not be converted to string>'
    try:
        receivedStr  = ', but received "' + str(received) + '"'
    except:
        pass
    errStr += receivedStr

    raise ValueError(errStr)

#**function: return True, if x is int, float, np.double, np.integer or similar types that can be automatically casted to pybind11
def IsValidBool(x):
    if (isinstance(x, bool)
        or isinstance(x, int)
        or isinstance(x, np.integer)
        ):
        return True
    return False

#**function: return True, if x is int, float, np.double, np.integer or similar types that can be automatically casted to pybind11
def IsValidRealInt(x):
    if (isinstance(x, float) 
        or isinstance(x, int)
        or isinstance(x, np.double)
        or isinstance(x, np.integer)
        ):
        return True
    return False

#**function: return True, if x is any python or numpy float type; could also be called IsFloat(), but Real has special meaning in Exudyn
def IsReal(x):
    if isinstance(x, (np.floating, float)): 
        return True
    else:
        return False

#**function: return True, if x is any python or numpy float type
def IsInteger(x):
    if isinstance(x, (np.integer, int)): 
        return True
    else:
        return False

#**function: check if v is a valid vector with floats or ints; if expectedSize!=None, the length is also checked
def IsVector(v, expectedSize=None):
    if type(v) != list and type(v) != np.ndarray:
        return False

    if expectedSize!=None:
        if len(v) != expectedSize:
            return False

    for x in v:
        if not IsValidRealInt(x): return False

    return True

#**function: check if v is a valid vector with floats or ints; if expectedSize!=None, the length is also checked
def IsSquareMatrix(m, expectedSize=None):
    if type(m) != list and type(m) != np.ndarray:
        return False

    if expectedSize!=None:
        if len(m) != expectedSize:
            return False

    for y in m: #works both in list of lists and np.array (over rows)
        if expectedSize!=None and len(y) != expectedSize: 
            return False
        for x in y:
            if not IsValidRealInt(x): return False

    return True

#**function: return True, if x is valid exudyn object index
def IsValidObjectIndex(x):
    if isinstance(x, int) or isinstance(x, np.integer) or isinstance(x, exudyn.ObjectIndex):
        return True
    return False

#**function: return True, if x is valid exudyn node index
def IsValidNodeIndex(x):
    if isinstance(x, int) or isinstance(x, np.integer) or isinstance(x, exudyn.NodeIndex):
        return True
    return False

#**function: return True, if x is valid exudyn marker index
def IsValidMarkerIndex(x):
    if isinstance(x, int) or isinstance(x, np.integer) or isinstance(x, exudyn.MarkerIndex):
        return True
    return False


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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


#DELETE, unused and does not make much sense (slow, not enough information in exception case, etc.)
# #**function: check if input is list or array with according length; if length==-1, the length is not checked; raises Exception if the check fails
# #**input: 
# #  vector: a vector in np.array or list format
# #  length: desired length of vector; if length=-1, it is ignored
# #**output: None
# def CheckInputVector(vector, length=-1):
#     try:
#         v = np.array(vector)
#     except:
#         raise ValueError("CheckInputVector: expected vector/list of length "+str(length)+
#                          ", but received '"+str(vector)+"'")
#     if v.ndim != 1:
#         raise ValueError("CheckInputVector: expected 1D vector/list of length "+str(length)+
#                          ", but received '"+str(vector)+"'")
#     if v.shape[0] != length:
#         raise ValueError("CheckInputVector: expected vector/list of length "+str(length)+
#                          ", but received vector with length "+str(v.shape[0])+", vector='"+str(vector)+"'")
        
# #**function: check if input is list or array with according length and positive indices; if length==-1, the length is not checked; raises Exception if the check fails
# #**input: 
# #  indexArray: a vector in np.array or list format
# #  length: desired length of vector; if length=-1, it is ignored
# #**output: None
# def CheckInputIndexArray(indexArray, length=-1):
#     try:
#         v = np.array(indexArray)
#     except:
#         raise ValueError("CheckInputIndexArray: expected vector/list of length "+str(length)+
#                          ", but received '"+str(indexArray)+"'")
#     if v.ndim != 1:
#         raise ValueError("CheckInputIndexArray: expected 1D vector/list of length "+str(length)+
#                          ", but received '"+str(indexArray)+"'")
#     if v.shape[0] != length:
#         raise ValueError("CheckInputIndexArray: expected vector/list of length "+str(length)+
#                          ", but received vector with length "+str(v.shape[0])+", indexArray='"+str(indexArray)+"'")
    
#     cnt=0
#     for i in indexArray:
#         if i!=int(i) or i < 0:
#             raise ValueError("CheckInputIndexArray: expected array/list of positive indices (int), but received '"+str(i)+
#                              "' at position " + str(cnt)+", indexArray='"+str(indexArray)+"'")
#         cnt+=1   




