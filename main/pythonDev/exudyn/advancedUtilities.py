#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Advanced utility functions only depending on numpy or specified exudyn modules;
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
import copy 

import exudyn
from exudyn.itemInterface import userFunctionArgsDict


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

#Types that can be converted to int; for load/save functions that can be loaded/saved
specialExudynTypes = (exudyn.ObjectIndex, exudyn.NodeIndex, exudyn.LoadIndex, 
               exudyn.MarkerIndex, exudyn.SensorIndex, 
               exudyn.OutputVariableType, exudyn.ConfigurationType, 
               exudyn.ItemType, exudyn.NodeType, exudyn.JointType, exudyn.DynamicSolverType,
               exudyn.CrossSectionType, exudyn.LinearSolverType, exudyn.ContactTypeIndex
               ) #must be tuple!

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
    PInt = 8+1
    UInt = 8+2
    Real = 16
    PReal = 16+1
    UReal = 16+2
    Vector = 32
    IntVector = 32+8
    Matrix = 64
    RigidBodyInertia = 128
    NodeIndex = 256
    ObjectIndex = 512
    MarkerIndex = 1024
    LoadIndex = 2048
    SensorIndex = 4096
    String = 8192

#**function: internal function which is used to raise common errors in case of wrong types; dim is used for vectors and square matrices, cols is used for non-square matrices
def RaiseTypeError(where='', argumentName='', received = None, expectedType = None, dim=None, cols=None):
    t = copy.copy(expectedType)
    
    errStr = 'ERROR in ' + where + ' in argument ' + argumentName + ': '
    
    if type(t) != str:
        errStr += 'expected type=' + t.name
        if t == ExpectedType.Vector or t == ExpectedType.IntVector:
            errStr += ' (as list or numpy array)'
        elif t == ExpectedType.Matrix:
            errStr += ' (as list of lists or numpy array)'
        
        if dim != None:
            ncols = cols
            if t == ExpectedType.Matrix and cols == None: 
                ncols = dim  #square Matrix
            if ncols != None:
                errStr += ', expected size = (' + str(dim) + ',' + str(ncols) + ')'
            else:
                errStr += ', expected size = ' + str(dim) 
    else:
        errStr += expectedType
            
    receivedStr = ', <argument can not be converted to string>'
    try:
        receivedStr  = ', but received "' + str(received) + '"'
    except:
        pass
    errStr += receivedStr

    raise ValueError(errStr)

#**function: return True, if x is None; works also for numpy arrays or structures
def IsNone(x):
    return (x is None)

#**function: return True, if x is not None; works also for numpy arrays or structures
def IsNotNone(x):
    return (x is not None)

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

#**function: return True, if x is valid Real/Int and positive
def IsValidPRealInt(x):
    if IsValidRealInt(x) and x > 0:
        return True
    return False

#**function: return True, if x is valid Real/Int and unsigned (non-negative)
def IsValidURealInt(x):
    if IsValidRealInt(x) and x >= 0:
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
def IsIntVector(v, expectedSize=None):
    if type(v) != list and type(v) != np.ndarray:
        return False

    if expectedSize!=None:
        if len(v) != expectedSize:
            return False

    for x in v:
        if not IsInteger(x): return False

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

#**function: return True, if x is an empty list (or empty list converted from numpy array), otherwise return False
def IsEmptyList(x):
    if isinstance(x, list) or isinstance(x, np.ndarray):
        return len(x) == 0
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


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function:  Function to convert a scipy sparse matrix to a dictionary
def ConvertScipySparseToDict(sparseMatrix):
    from scipy.sparse import csr_matrix
    if not isinstance(sparseMatrix, csr_matrix):
        try:
            sparseMatrix = sparseMatrix.tocsr()
        except:
            raise ValueError(f"ConvertScipySparseToDict: Unsupported sparse matrix type: {type(sparseMatrix)}")
        
    return {'data': sparseMatrix.data,
            'indices': sparseMatrix.indices,
            'indptr': sparseMatrix.indptr,
            'shape': sparseMatrix.shape}

#**function:  Function to convert a dictionary back to a scipy sparse matrix
def ConvertDictToScipySparse(sparseDict):
    from scipy.sparse import csr_matrix
    return csr_matrix((sparseDict['data'], sparseDict['indices'], sparseDict['indptr']),
                      shape=sparseDict['shape'])

#**function: recursively saves a hierarchical dictionary dataDict to a HDF5 file with given fileName; limitations for certain types and Python or symbolic user functions
#**input: 
#  fileName: file name (possibly including path) for HDF5 file, including file ending
#  dataDict: the dictionary containing the hierarchical data to be saved; the data may contain the following data types in hierarchical form: int, bool, float, str (utf-8), list, dict, numpy array, scipy csr\_matrix, Python function
#**output: None
def SaveDictToHDF5(fileName, dataDict):
    try:
        import h5py
        from scipy.sparse import csr_matrix
    except:
        raise ImportError('SaveDictToHDF5 only works if scipy and h5py are installed')        
    
    def IsExudynUserFunction(fDict):
        if isinstance(fDict, dict):
            if 'function' in fDict and 'type' in fDict:
                if fDict['type'] == 'Python':
                    if callable(fDict['function']):
                        return True
                elif fDict['type'] == 'Symbolic':
                    raise ValueError('SaveDictToHDF5: not possible with symbolic user functions: {fDict}')
        return False
    
    def HandleSaveDict(hdf5group, key, item):
        if (IsExudynUserFunction(item)):
            #print('SaveHDF5:dict key=',key,'is user function:',item)
            newItem = {'functionName':item['function'].__name__,
                       'functionVarNames':str(item['function'].__code__.co_varnames),
                       'type':item['type']}
            subgroup = hdf5group.create_group(key)
            subgroup.attrs['datatype'] = 'exuFunction'
            RecursivelySaveDictToHDF5(subgroup, newItem)
        else:
            # Recursively save nested dictionary
            subgroup = hdf5group.create_group(key)
            subgroup.attrs['datatype'] = 'dict'
            RecursivelySaveDictToHDF5(subgroup, item)
            
    def RecursivelySaveDictToHDF5(hdf5group, dataDict):
        for key, item in dataDict.items():
            if isinstance(item, dict):
                HandleSaveDict(hdf5group, key, item)
            elif isinstance(item, csr_matrix):
                # Save sparse matrix by storing its components and setting type attribute
                subgroup = hdf5group.create_group(key)
                subgroup.attrs['datatype'] = 'csr_matrix'
                sparseDict = ConvertScipySparseToDict(item)
                for k, v in sparseDict.items():
                    subgroup.create_dataset(k, data=v)
            elif isinstance(item, list):
                # Save lists by recursively saving each item in the list
                subgroup = hdf5group.create_group(key)
                subgroup.attrs['datatype'] = 'list'
                for idx, subItem in enumerate(item):
                    subKey = f"item_{idx}"
                    if isinstance(subItem, specialExudynTypes):
                        # Save basic types with the corresponding datatype
                        subgroup.create_dataset(subKey, data=int(subItem))
                        subgroup[subKey].attrs['datatype'] = type(subItem).__name__
                    elif isinstance(subItem, (bool, int, float, np.int32, np.int64, np.float32, np.float64)) or subItem is None:
                        # Save basic types with the corresponding datatype
                        if subItem is not None:
                            subgroup.create_dataset(subKey, data=subItem)
                        else:
                            subgroup.create_dataset(subKey, data=0)
                        subgroup[subKey].attrs['datatype'] = type(subItem).__name__
                    elif isinstance(subItem, str):
                        dt = h5py.string_dtype(encoding='utf-8')
                        subgroup.create_dataset(subKey, data=subItem, dtype=dt)
                        subgroup[subKey].attrs['datatype'] = 'str'
                    elif isinstance(subItem, dict):
                        # Handle dictionaries inside lists
                        # dict_group = subgroup.create_group(subKey)
                        # dict_group.attrs['datatype'] = 'dict'
                        # RecursivelySaveDictToHDF5(dict_group, subItem)
                        HandleSaveDict(subgroup, subKey, subItem)
                    elif isinstance(subItem, list):
                        # Handle nested lists recursively
                        nested_group = subgroup.create_group(subKey)
                        nested_group.attrs['datatype'] = 'list'
                        RecursivelySaveDictToHDF5(nested_group, {f"item_{i}": subItem[i] for i in range(len(subItem))})
                    else:
                        raise ValueError(f"SaveDictToHDF5: unsupported type in list: {item} / {subItem}")
                        #RecursivelySaveDictToHDF5(subgroup.create_group(subKey), {'item': subItem})
            elif isinstance(item, np.ndarray): #stored directly
                hdf5group.create_dataset(key, data=item)
                hdf5group[key].attrs['datatype'] = 'ndarray'
            elif isinstance(item, (bool, int, float, np.int32, np.int64, np.float32, np.float64)) or item is None:
                # Save numbers directly with their type in the attributes
                if item is not None:
                    hdf5group.create_dataset(key, data=item)
                else:
                    hdf5group.create_dataset(key, data=0)
                hdf5group[key].attrs['datatype'] = type(item).__name__
            elif isinstance(item, specialExudynTypes):
                # Save numbers directly with their type in the attributes
                hdf5group.create_dataset(key, data=int(item))
                hdf5group[key].attrs['datatype'] = type(item).__name__
            elif isinstance(item, str):
                # Handle strings with datatype attribute
                dt = h5py.string_dtype(encoding='utf-8')
                hdf5group.create_dataset(key, data=item, dtype=dt)
                hdf5group[key].attrs['datatype'] = 'str'
            else:
                raise ValueError(f"SaveDictToHDF5: unsupported type: {item}")

    with h5py.File(fileName, 'w') as h5file:
        RecursivelySaveDictToHDF5(h5file, dataDict)


#**function: recursively loads a hierarchical dictionary from a HDF5 file with given fileName
#**input: 
#  fileName: file name (possibly including path) for HDF5 file, including file ending
#  callerGlobals: optional: if your data contains functions, the callerGlobals must contain, e.g., globals() of the caller, where the Python functions are defined at which the HDF5 function refers to 
#**output: dict which contains loaded data
def LoadDictFromHDF5(fileName, callerGlobals=None):
    try:
        import h5py
        from scipy.sparse import csr_matrix
    except:
        raise ImportError('LoadDictFromHDF5 only works if scipy and h5py are installed')        
    import inspect #for determining globals() of caller
    NoneCast = lambda x: None #returns none
    
    regularTypes = [bool, int, float] + list(specialExudynTypes)
    regularTypeStrings = [item.__name__ for item in regularTypes]
    regularTypeStrings += ['float64']; regularTypes += [float]
    regularTypeStrings += ['float32']; regularTypes += [float]
    regularTypeStrings += ['int64']; regularTypes += [int] #may be data loss!
    regularTypeStrings += ['int32']; regularTypes += [int]
    regularTypeStrings += ['ndarray']; regularTypes += [np.array]
    regularTypeStrings += ['NoneType']; regularTypes += [NoneCast]

    def RecursivelyLoadDictFromHDF5(hdf5group):
        result = {}
        for key, item in hdf5group.items():
            datatype = item.attrs.get('datatype', None)  # Get the 'datatype' attribute
            #print('key:',key)

            if isinstance(item, h5py.Group):
                if datatype == 'csr_matrix':
                    # Reconstruct the sparse matrix
                    sparseDict = {k: item[k][:] for k in item.keys()}
                    result[key] = ConvertDictToScipySparse(sparseDict)
                elif datatype == 'list':
                    # Reconstruct the list
                    listItems = []
                    for i in range(len(item)):
                        subKey = f"item_{i}"
                        if 'datatype' in item[subKey].attrs:
                            subDatatype = item[subKey].attrs['datatype']
                            if subDatatype == 'dict':
                                # Recursively handle a dictionary inside a list
                                listItems.append(RecursivelyLoadDictFromHDF5(item[subKey]))
                            elif subDatatype == 'csr_matrix':
                                # Recursively handle a sparse matrix inside a list
                                sparseDict = {k: item[subKey][k][:] for k in item[subKey].keys()}
                                listItems.append(ConvertDictToScipySparse(sparseDict))
                            elif subDatatype == 'list':
                                # Recursively handle a list inside a list
                                listItems.append(RecursivelyLoadDictFromHDF5(item[subKey]))
                            elif subDatatype in regularTypeStrings:
                                index = regularTypeStrings.index(subDatatype)
                                listItems.append( regularTypes[index](item[subKey][()]) )
                            elif subDatatype == 'str':
                                listItems.append(item[subKey][()].decode('utf-8'))
                            else:
                                raise ValueError(f"Unsupported datatype {subDatatype} in list")
                        else:
                            raise ValueError(f"Missing datatype attribute in list for {subKey} in {key} / {item}")
                    result[key] = listItems
                elif datatype == 'dict':
                    # Recursively load the dictionary
                    result[key] = RecursivelyLoadDictFromHDF5(item)
                elif datatype == 'exuFunction':
                    if callerGlobals is None:
                        raise ValueError('LoadDictFromHDF5: data contains functions: requires callerGlobals to be specified!')
                    # Recursively load the dictionary
                    functionDict = RecursivelyLoadDictFromHDF5(item)
                    if functionDict['type'] != 'Python': 
                        raise ValueError('LoadDictFromHDF5: illegal function type: '+functionDict['type'])
                    name = functionDict['functionName']
                    if name not in callerGlobals:
                        raise ValueError('LoadDictFromHDF5: trying to load function "'+name+'", but did not find it in globals() of function caller. Functions must be available in global scope at which LoadDictFromHDF5 is defined!')
                    func = callerGlobals[name]
                    if functionDict['functionVarNames'] != str(func.__code__.co_varnames):
                        raise ValueError('LoadDictFromHDF5: trying to load function "'+name+'": loaded function and function available in global scope have different argument lists: loaded='+functionDict['functionVarNames'] +', scope='+str(func.__code__.co_varnames))
                    funcDict = {'function': func, 'type': 'Python'}
                    result[key] = funcDict
                else:
                    raise ValueError(f"Unsupported datatype {datatype} for group {key}")
            elif isinstance(item, h5py.Dataset):
                # Reconstruct basic types
                # if datatype == 'int':
                #     result[key] = int(item[()])
                if datatype == 'str':
                    result[key] = item[()].decode('utf-8')  # Strings are stored as byte arrays, so decode them
                elif datatype in regularTypeStrings:
                    index = regularTypeStrings.index(datatype)
                    result[key] = regularTypes[index](item[()])
                else:
                    raise ValueError(f"Unsupported datatype {datatype} for dataset {key}")
        return result

    with h5py.File(fileName, 'r') as h5file:
        return RecursivelyLoadDictFromHDF5(h5file)















#**function: Internal function to convert a Python user function into a dictionary containing the symbolic representation;
#  this function is under development and should be used with care
#**input:
#  mbs: MainSystem, needed currently for interface
#  function: Python function with interface according to desired user function
#  itemIndex: item index, such as ObjectIndex or LoadIndex; -1 indicates MainSystem; if None, itemTypeName must be provided instead
#  itemTypeName: use of type name, such as ObjectConnectorSpringDamper; in this case, itemIndex must be None
#  itemIndex: item index, such as ObjectIndex or LoadIndex; -1 indicates MainSystem
#  userFunctionName: name of user function item, see documentation; this is required, because some items have several user functions, which need to be distinguished
#  verbose: if > 0, according output is printed
#**output: return dictionary with 'functionName', 'argList', and 'returnList'
def ConvertFunctionToSymbolic(mbs, function, userFunctionName, itemIndex=None, itemTypeName=None, verbose=0):
    fnName = function.__name__
    fnArgs = function.__code__.co_varnames
    #fnAnnotations = function.__annotations__ #not necessarily present
    if verbose:
        #print("Annotations:", fnAnnotations)
        print("Function Name:", fnName)
        # print("Docstring:", function.__doc__)
        print("Number of Arguments:", function.__code__.co_argcount)
        print("Argument Names:", fnArgs)


    if itemTypeName != None:
        itemTypeNameCopy = itemTypeName
        if itemIndex != None: raise ValueError('ConvertFunctionToSymbolic: if itemTypeName is provided, itemIndex must be None')
    elif itemIndex == -1: #MainSystem or other function
        itemTypeNameCopy = 'MainSystem'
    else:
        #regular item
        try:
            typeString = itemIndex.GetTypeString()
        except:
            raise ValueError('ConvertFunctionToSymbolic: itemIndex must be a valid exudyn ItemIndex or itemTypeName has to be provided instead')
        
        itemTypeNameCopy = None
        # itemClass = None
        if typeString == 'ObjectIndex':
            itemTypeNameCopy = 'Object'+mbs.GetObject(itemIndex)['objectType']
        elif typeString == 'LoadIndex':
            itemTypeNameCopy = 'Load'+mbs.GetLoad(itemIndex)['loadType']
        else:
            raise ValueError('ConvertFunctionToSymbolic: itemIndex has unsupported type')

    recStored = exudyn.symbolic.GetRecording()
    exudyn.symbolic.SetRecording(True)

    #create args as dict
    fnDict = {}
    argList = []
    argTypeList = []
    
    functionArgs = userFunctionArgsDict[itemTypeNameCopy+','+userFunctionName]
    returnType = functionArgs[2][0]
    nArgs = len(functionArgs[0])
    if nArgs != function.__code__.co_argcount:
        
        sFunctionInterface = 'F('
        sReturn = ' -> '+returnType+': ...'
        sep = ''
        for i in range(nArgs):
            sFunctionInterface+=sep+functionArgs[1][i]+': '+functionArgs[0][i].replace('MainSystem','exudyn.MainSystem').replace('Real','float').replace('Index','int')
            sep = ', '
        sFunctionInterface += ')\n'

        raise ValueError('ConvertFunctionToSymbolic: function "'+fnName+'" does not meet correct number of arguments; '+
                         'user function should read:\n'+sFunctionInterface+sReturn)
    
    for i in range(nArgs):
        varType = functionArgs[0][i]
        arg = fnArgs[i] #functionArgs[1][i] 
        if varType == 'Real':
            var = exudyn.symbolic.Real(arg, 0.)
        elif varType == 'Index':
            var = exudyn.symbolic.Real(arg, 0)
        elif (varType == 'StdVector3D'
              or varType == 'StdVector6D'
              or varType == 'StdVector'
              ):
            l = 0
            if '3D' in varType: l = 3
            if '6D' in varType: l = 6
            var = exudyn.symbolic.Vector(arg,[0.]*l)
        elif (varType == 'NumpyMatrix'
              or varType == 'StdMatrix3D'
              or varType == 'StdMatrix6D'
              ):
            rowsColumns=1 #zero list would not work for matrix ...
            if '3D' in varType: rowsColumns = 3
            if '6D' in varType: rowsColumns = 6
            var = exudyn.symbolic.Matrix(arg,np.zeros((rowsColumns,rowsColumns)).tolist())
        elif varType == 'MainSystem':
            var = mbs
        else:
            raise ValueError("ConvertFunctionToSymbolic: unrecognized type in user function; type probably implemented")
    
        argList += [var] #float, int, MainSystem, Vector3D, etc.
        argTypeList += [varType] #float, int, MainSystem, Vector3D, etc.
        fnDict[arg] = var
        
    if verbose > 1:
        print('\nargList=[mbs]+', argList[1:])
        print('\nfnDict=', fnDict)

    #now we record the function:
    returnValue = function(**fnDict)
    exudyn.symbolic.SetRecording(recStored)


    if type(returnValue) is list:
        #in this case, we create a SymbolicRealVector such that the user also can work with lists ...
        returnValue = exudyn.symbolic.Vector(returnValue) #create vector from list

    if verbose:
        print('return value=', returnValue)

    return {'functionName': fnName, 
            'argList': argList, 
            'argTypeList': argTypeList,
            'returnValue': returnValue,
            'returnType': returnType}


#**function: Helper function to convert a Python user function into a symbolic user function;
#  this function is under development and should be used with care
#**input:
#  mbs: MainSystem, needed currently for interface
#  function: Python function with interface according to desired user function
#  itemIndex: item index, such as ObjectIndex or LoadIndex; -1 indicates MainSystem; if None, itemTypeName must be provided instead
#  itemTypeName: use of type name, such as ObjectConnectorSpringDamper; in this case, itemIndex must be None
#  userFunctionName: name of user function item, see documentation; this is required, because some items have several user functions, which need to be distinguished
#  verbose: if > 0, according output may be printed
#**output: returns symbolic user function; this can be transfered into an item using TransferUserFunction2Item
#**notes: keep the return value alive in a variable (or list), as it contains the expression tree which must exist for the lifetime of the user function
#**example:
# oGround = mbs.AddObject(ObjectGround())
#
# node = mbs.AddNode(NodePoint(referenceCoordinates = [1.05,0,0]))
# oMassPoint = mbs.AddObject(MassPoint(nodeNumber = node, physicsMass=1))
#
# symbolicFunc = CreateSymbolicUserFunction(mbs, function=springForceUserFunction, 
#                                           userFunctionName='springForceUserFunction', 
#                                           itemTypeName='ObjectConnectorSpringDamper')
#
# m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0]))
# m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oMassPoint, localPosition=[0,0,0]))
# co = mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[m0,m1],
#                    referenceLength = 1, stiffness = 100, damping = 1, 
#                    springForceUserFunction=symbolicFunc))
#
# print(symbolicFunc.Evaluate(mbs, 0., 0, 1.1, 0.,  100., 0., 13.) )
def CreateSymbolicUserFunction(mbs, function, userFunctionName, itemIndex=None, itemTypeName=None, verbose=0):
    fnDict = ConvertFunctionToSymbolic(mbs, function, userFunctionName, itemIndex, itemTypeName, verbose)
    symbolicFunc = exudyn.symbolic.UserFunction()
    symbolicFunc.SetUserFunctionFromDict(mbs, fnDict, userFunctionName, itemIndex, str(itemTypeName))
    return symbolicFunc
    



