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
from math import sin, cos, pi
from enum import Enum
import copy 

import exudyn
from exudyn.itemInterface import userFunctionArgsDict


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#GENERAL FUNCTIONS
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#public API of this module; kept complete by tools/checkAll.py (revision plan step 107c)
__all__ = [
    'PlotLineCode', 'specialExudynTypes', 'FindObjectIndex', 'FindNodeIndex', 'IsListOrArray',
    'ExpectedType', 'RaiseTypeError', 'IsNone', 'IsNotNone', 'IsValidBool', 'IsValidRealInt',
    'IsValidInt', 'IsValidPRealInt', 'IsValidURealInt', 'IsReal', 'IsInteger', 'IsVector',
    'IsIntVector', 'IsSquareMatrix', 'IsValidObjectIndex', 'IsValidNodeIndex', 'IsValidMarkerIndex',
    'IsEmptyList', 'FillInSubMatrix', 'SweepSin', 'SweepCos', 'FrequencySweep', 'SmoothStep',
    'SmoothStepDerivative', 'IndexFromValue', 'RoundMatrix', 'ConvertScipySparseToDict',
    'ConvertDictToScipySparse', 'SaveDictToHDF5', 'LoadDictFromHDF5', 'ConvertFunctionToSymbolic',
    'CreateSymbolicUserFunction', 'TCPIPdata', 'CreateTCPIPconnection', 'TCPIPsendReceive',
    'CloseTCPIPconnection',
    ]

def PlotLineCode(index):
    """helper functions for matplotlib, returns a list of 28 line codes to be used in plot, e.g. 'r-' for red solid line

    Args:
        index in range(0:28)

    Returns:
        a color and line style code for matplotlib plot
    """
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
        
def FindObjectIndex(i, globalVariables):
    """simple function to find object index i within the local or global scope of variables

    Args:
        i, the integer object number and  globalVariables=globals()

    Example:
        FindObjectIndex(2, locals() )  #usually sufficient
        FindObjectIndex(2, globals() ) #wider search
    """
    #run through all variables and check if an object index exists
    found = False
    for varname in globalVariables:
        if type(globalVariables[varname]) == exudyn.ObjectIndex:
            if int(globalVariables[varname]) == int(i):
                exudyn.Print("variable '"+varname+"' links to object "+str(i) )
                found = True
    if not found:
        exudyn.Print("no according variable found")

def FindNodeIndex(i, globalVariables):
    """simple function to find node index i within the local or global scope of variables

    Args:
        i, the integer node number and  globalVariables=globals()

    Example:
        FindObjectIndex(2, locals() )  #usually sufficient
        FindObjectIndex(2, globals() ) #wider search
    """
    #run through all variables and check if an object index exists
    found = False
    for varname in globalVariables:
        if type(globalVariables[varname]) == exudyn.NodeIndex:
            if int(globalVariables[varname]) == int(i):
                exudyn.Print("variable '"+varname+"' links to node "+str(i) )
                found = True
    if not found:
        exudyn.Print("no according variable found")

def IsListOrArray(data, checkIfNoneEmpty=False):
    """checks, if data is of type list or np.array; used in functions to check input data

    Args:
        data: any type, preferrably list or numpy.array
        checkIfNoneEmpty: if True, function only returns True if type is list or array AND if length is non-zero

    Returns:
        returns True/False
    """
    if isinstance(data,list) or isinstance(data,np.ndarray):
        if checkIfNoneEmpty and len(data) == 0:
            return False
        return True
    else:
        return False


class ExpectedType(Enum):
    """internal type which is used for type checking in exudyn Python user functions; used to create unique error messages
    """
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

def RaiseTypeError(where='', argumentName='', received = None, expectedType = None, dim=None, cols=None):
    """internal function which is used to raise common errors in case of wrong types; dim is used for vectors and square matrices, cols is used for non-square matrices
    """
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
        receivedStr  = ', but received "' + str(received) + '", type=' + str(type(received))
    except:
        pass
    errStr += receivedStr

    raise ValueError(errStr)

def IsNone(x):
    """return True, if x is None; works also for numpy arrays or structures
    """
    return (x is None)

def IsNotNone(x):
    """return True, if x is not None; works also for numpy arrays or structures
    """
    return (x is not None)

def IsValidBool(x):
    """return True, if x is int, float, np.double, np.integer or similar types that can be automatically casted to pybind11
    """
    if (isinstance(x, bool)
        or isinstance(x, int)
        or isinstance(x, np.integer)
        ):
        return True
    return False

def IsValidRealInt(x):
    """return True, if x is int, float, np.double, np.integer or similar types that can be automatically casted to pybind11
    """
    if (isinstance(x, float) 
        or isinstance(x, int)
        or isinstance(x, np.double)
        or isinstance(x, np.integer)
        ):
        return True
    return False

def IsValidInt(x):
    """return True, if x is int, np.integer or similar types that can be automatically casted to pybind11
    """
    if (isinstance(x, int)
        or isinstance(x, np.integer)
        ):
        return True
    return False

def IsValidPRealInt(x):
    """return True, if x is valid Real/Int and positive
    """
    if IsValidRealInt(x) and x > 0:
        return True
    return False

def IsValidURealInt(x):
    """return True, if x is valid Real/Int and unsigned (non-negative)
    """
    if IsValidRealInt(x) and x >= 0:
        return True
    return False

def IsReal(x):
    """return True, if x is any python or numpy float type; could also be called IsFloat(), but Real has special meaning in Exudyn
    """
    if isinstance(x, (np.floating, float)): 
        return True
    else:
        return False

def IsInteger(x):
    """return True, if x is any python or numpy float type
    """
    if isinstance(x, (np.integer, int)): 
        return True
    else:
        return False

def IsVector(v, expectedSize=None):
    """check if v is a valid vector with floats or ints; if expectedSize!=None, the length is also checked
    """
    if type(v) != list and type(v) != np.ndarray:
        return False

    if expectedSize!=None:
        if len(v) != expectedSize:
            return False

    for x in v:
        if not IsValidRealInt(x): return False

    return True

def IsIntVector(v, expectedSize=None):
    """check if v is a valid vector with floats or ints; if expectedSize!=None, the length is also checked
    """
    if type(v) != list and type(v) != np.ndarray:
        return False

    if expectedSize!=None:
        if len(v) != expectedSize:
            return False

    for x in v:
        if not IsInteger(x): return False

    return True


def IsSquareMatrix(m, expectedSize=None):
    """check if v is a valid vector with floats or ints; if expectedSize!=None, the length is also checked
    """
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

def IsValidObjectIndex(x):
    """return True, if x is valid exudyn object index
    """
    if isinstance(x, int) or isinstance(x, np.integer) or isinstance(x, exudyn.ObjectIndex):
        return True
    return False

def IsValidNodeIndex(x):
    """return True, if x is valid exudyn node index
    """
    if isinstance(x, int) or isinstance(x, np.integer) or isinstance(x, exudyn.NodeIndex):
        return True
    return False

def IsValidMarkerIndex(x):
    """return True, if x is valid exudyn marker index
    """
    if isinstance(x, int) or isinstance(x, np.integer) or isinstance(x, exudyn.MarkerIndex):
        return True
    return False

def IsEmptyList(x):
    """return True, if x is an empty list (or empty list converted from numpy array), otherwise return False
    """
    if isinstance(x, list) or isinstance(x, np.ndarray):
        return len(x) == 0
    return False 


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def FillInSubMatrix(subMatrix, destinationMatrix, destRow, destColumn):
    """fill submatrix into given destinationMatrix; all matrices must be numpy arrays

    Args:
        subMatrix: input matrix, which is filled into destinationMatrix
        destinationMatrix: the subMatrix is entered here
        destRow: row destination of subMatrix
        destColumn: column destination of subMatrix

    Returns:
        destinationMatrix is changed after function call

    Note:
        may be erased in future!
    """
    nRows = subMatrix.shape[0]
    nColumns = subMatrix.shape[1]

    destinationMatrix[destRow:destRow+nRows, destColumn:destColumn+nColumns] = subMatrix

    #for i in range(nRows):
    #    for j in range(nColumns):
    #        destinationMatrix[i+destRow, j+destColumn] = subMatrix[i,j]


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++
def SweepSin(t, t1, f0, f1):
    """compute sin sweep at given time t

    Args:
        t: evaluate of sweep at time t
        t1: end time of sweep frequency range
        f0: start of frequency interval [f0,f1] in Hz
        f1: end of frequency interval [f0,f1] in Hz

    Returns:
        evaluation of sin sweep (in range -1..+1)
    """
    k = (f1-f0)/t1
    return sin(2*pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!

def SweepCos(t, t1, f0, f1):
    """compute cos sweep at given time t

    Args:
        t: evaluate of sweep at time t
        t1: end time of sweep frequency range
        f0: start of frequency interval [f0,f1] in Hz
        f1: end of frequency interval [f0,f1] in Hz

    Returns:
        evaluation of cos sweep (in range -1..+1)
    """
    k = (f1-f0)/t1
    return cos(2*pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!

def FrequencySweep(t, t1, f0, f1):
    """frequency according to given sweep functions SweepSin, SweepCos

    Args:
        t: evaluate of frequency at time t
        t1: end time of sweep frequency range
        f0: start of frequency interval [f0,f1] in Hz
        f1: end of frequency interval [f0,f1] in Hz

    Returns:
        frequency in Hz
    """
    return t*(f1-f0)/t1 + f0

def SmoothStep(x, x0, x1, value0, value1): 
    """step function with smooth transition from value0 to value1; transition is computed with cos function

    Args:
        x: argument at which function is evaluated
        x0: start of step (f(x) = value0)
        x1: end of step (f(x) = value1)
        value0: value before smooth step
        value1: value at end of smooth step

    Returns:
        returns f(x)
    """
    loadValue = value0

    if x > x0:
        if x < x1:
            dx = x1-x0
            loadValue = value0 + (value1-value0) * 0.5*(1-cos((x-x0)/dx*pi)) 
        else:
            loadValue = value1
    return loadValue

def SmoothStepDerivative(x, x0, x1, value0, value1): 
    """derivative of SmoothStep using same arguments

    Args:
        x: argument at which function is evaluated
        x0: start of step (f(x) = value0)
        x1: end of step (f(x) = value1)
        value0: value before smooth step
        value1: value at end of smooth step

    Returns:
        returns d/dx(f(x))
    """
    loadValue = 0

    if x > x0 and x < x1:
        dx = x1-x0
        loadValue = (value1-value0) * 0.5*(pi/dx*sin((x-x0)/dx*pi)) 
    return loadValue

def IndexFromValue(data, value, tolerance=1e-7, assumeConstantSampleRate=False, rangeWarning=True):
    """get index from value in given data vector (numpy array); usually used to get specific index of time vector; this function is slow (linear search), if sampling rate is non-constant; otherwise set assumeConstantSampleRate=True!

    Args:
        data: containing (almost) equidistant values of time
        value: e.g., time to be found in data
        tolerance: tolerance, which is accepted (default: tolerance=1e-7)
        rangeWarning: warn, if index returns out of range; if warning is deactivated, function uses the closest value

    Returns:
        index

    Note:
        to obtain the interpolated value of a time-signal array, use GetInterpolatedSignalValue() in exudyn.signalProcessing
    """
    index  = -1
    
    if assumeConstantSampleRate and len(data) > 1:
        dt = data[1] - data[0]
        if dt == 0.:
            raise ValueError('IndexFromValue: sample rate is zero!')
        index = int((value-data[0]) / dt)
        if index < 0:
            index = 0
            if rangeWarning:
                exudyn.Print('Warning: IndexFromValue: index returned smaller than 0; using 0 instead')
        elif index >= len(data):
            if rangeWarning:
                exudyn.Print('Warning: IndexFromValue: index returned larger than array length-1; using array max length-1 instead')
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
def RoundMatrix(matrix, treshold = 1e-14):
    """set all entries in matrix to zero which are smaller than given treshold; operates directly on matrix

    Args:
        matrix as np.array, treshold as positive value

    Returns:
        changes matrix
    """
    (rows, cols) = matrix.shape
    for i in range (rows):
        for j in range(cols):
            if abs(matrix[i,j]) < treshold:
                matrix[i,j]=0


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def ConvertScipySparseToDict(sparseMatrix):
    """Function to convert a scipy sparse matrix to a dictionary
    """
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

def ConvertDictToScipySparse(sparseDict):
    """Function to convert a dictionary back to a scipy sparse matrix
    """
    from scipy.sparse import csr_matrix
    return csr_matrix((sparseDict['data'], sparseDict['indices'], sparseDict['indptr']),
                      shape=sparseDict['shape'])

def SaveDictToHDF5(fileName, dataDict):
    """recursively saves a hierarchical dictionary dataDict to a HDF5 file with given fileName; limitations for certain types and Python or symbolic user functions

    Args:
        fileName: file name (possibly including path) for HDF5 file, including file ending
        dataDict: the dictionary containing the hierarchical data to be saved; the data may contain the following data types in hierarchical form: int, bool, float, str (utf-8), list, dict, numpy array, scipy csr_matrix, Python function

    Returns:
        None
    """
    try:
        import h5py
        from scipy.sparse import csr_matrix
    except ImportError:
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
                        HandleSaveDict(subgroup, subKey, subItem)
                    elif isinstance(subItem, list):
                        # Handle nested lists recursively
                        nested_group = subgroup.create_group(subKey)
                        nested_group.attrs['datatype'] = 'list'
                        RecursivelySaveDictToHDF5(nested_group, {f"item_{i}": subItem[i] for i in range(len(subItem))})
                    elif isinstance(subItem, np.ndarray):
                        subgroup.create_dataset(subKey, data=subItem)
                        subgroup[subKey].attrs['datatype'] = 'ndarray'
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


def LoadDictFromHDF5(fileName, callerGlobals=None):
    """recursively loads a hierarchical dictionary from a HDF5 file with given fileName

    Args:
        fileName: file name (possibly including path) for HDF5 file, including file ending
        callerGlobals: optional: if your data contains functions, the callerGlobals must contain, e.g., globals() of the caller, where the Python functions are defined at which the HDF5 function refers to

    Returns:
        dict which contains loaded data
    """
    try:
        import h5py
        #from scipy.sparse import csr_matrix
    except:
        raise ImportError('LoadDictFromHDF5 only works if scipy and h5py are installed')        

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
                                # listItems.append(RecursivelyLoadDictFromHDF5(item[subKey]))
                                subListDict = RecursivelyLoadDictFromHDF5(item[subKey]) #stored as dict with 'item_*' keys
                                subList = []
                                for i in range(len(subListDict)):
                                    subList.append(subListDict['item_'+str(i)])
                                listItems.append(subList)
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
                #automatic:
                # elif datatype == 'ndarray':
                #     result[key] = item[:]  # Load the numpy array data
                elif datatype in regularTypeStrings:
                    index = regularTypeStrings.index(datatype)
                    result[key] = regularTypes[index](item[()])
                else:
                    raise ValueError(f"Unsupported datatype {datatype} for dataset {key}")
        return result

    with h5py.File(fileName, 'r') as h5file:
        return RecursivelyLoadDictFromHDF5(h5file)















def ConvertFunctionToSymbolic(mbs, function, userFunctionName, itemIndex=None, itemTypeName=None, verbose=0):
    """Internal function to convert a Python user function into a dictionary containing the symbolic representation;
    this function is under development and should be used with care

    Args:
        mbs: MainSystem, needed currently for interface
        function: Python function with interface according to desired user function
        itemIndex: item index, such as ObjectIndex or LoadIndex; -1 indicates MainSystem; if None, itemTypeName must be provided instead
        itemTypeName: use of type name, such as ObjectConnectorSpringDamper; in this case, itemIndex must be None
        itemIndex: item index, such as ObjectIndex or LoadIndex; -1 indicates MainSystem
        userFunctionName: name of user function item, see documentation; this is required, because some items have several user functions, which need to be distinguished
        verbose: if > 0, according output is printed

    Returns:
        return dictionary with 'functionName', 'argList', and 'returnList'
    """
    fnName = function.__name__
    fnArgs = function.__code__.co_varnames
    #fnAnnotations = function.__annotations__ #not necessarily present
    if verbose:
        exudyn.Print("Function Name:", fnName)
        exudyn.Print("Number of Arguments:", function.__code__.co_argcount)
        exudyn.Print("Argument Names:", fnArgs)


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
        exudyn.Print('\nargList=[mbs]+', argList[1:])
        exudyn.Print('\nfnDict=', fnDict)

    #now we record the function:
    returnValue = function(**fnDict)
    exudyn.symbolic.SetRecording(recStored)


    if type(returnValue) is list:
        #in this case, we create a SymbolicRealVector such that the user also can work with lists ...
        returnValue = exudyn.symbolic.Vector(returnValue) #create vector from list

    if verbose:
        exudyn.Print('return value=', returnValue)

    return {'functionName': fnName, 
            'argList': argList, 
            'argTypeList': argTypeList,
            'returnValue': returnValue,
            'returnType': returnType}


def CreateSymbolicUserFunction(mbs, function, userFunctionName, itemIndex=None, itemTypeName=None, verbose=0):
    """Helper function to convert a Python user function into a symbolic user function;
    this function is under development and should be used with care

    Args:
        mbs: MainSystem, needed currently for interface
        function: Python function with interface according to desired user function
        itemIndex: item index, such as ObjectIndex or LoadIndex; -1 indicates MainSystem; if None, itemTypeName must be provided instead
        itemTypeName: use of type name, such as ObjectConnectorSpringDamper; in this case, itemIndex must be None
        userFunctionName: name of user function item, see documentation; this is required, because some items have several user functions, which need to be distinguished
        verbose: if > 0, according output may be printed

    Returns:
        returns symbolic user function; this can be transfered into an item using TransferUserFunction2Item

    Note:
        keep the return value alive in a variable (or list), as it contains the expression tree which must exist for the lifetime of the user function

    Example:
        oGround = mbs.AddObject(ObjectGround())
        node = mbs.AddNode(NodePoint(referenceCoordinates = [1.05,0,0]))
        oMassPoint = mbs.AddObject(MassPoint(nodeNumber = node, physicsMass=1))
        symbolicFunc = CreateSymbolicUserFunction(mbs, function=springForceUserFunction,
                                                  userFunctionName='springForceUserFunction',
                                                  itemTypeName='ObjectConnectorSpringDamper')
        m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0]))
        m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oMassPoint, localPosition=[0,0,0]))
        co = mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[m0,m1],
                           referenceLength = 1, stiffness = 100, damping = 1,
                           springForceUserFunction=symbolicFunc))
        exudyn.Print(symbolicFunc.Evaluate(mbs, 0., 0, 1.1, 0.,  100., 0., 13.) )
    """
    fnDict = ConvertFunctionToSymbolic(mbs, function, userFunctionName, itemIndex, itemTypeName, verbose)
    symbolicFunc = exudyn.symbolic.UserFunction()
    symbolicFunc.SetUserFunctionFromDict(mbs, fnDict, userFunctionName, itemIndex, str(itemTypeName))
    return symbolicFunc


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#TCP/IP functionality (moved from utilities.py, revision plan step 107b)

#TCP/IP functionality
class TCPIPdata:
    """helper class for CreateTCPIPconnection and for TCPIPsendReceive
    """
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


def CreateTCPIPconnection(sendSize, receiveSize, IPaddress='127.0.0.1', port=52421, 
                          bigEndian=False, verbose=False):
    """function which has to be called before simulation to setup TCP/IP socket (server) for
    sending and receiving data; can be used to communicate with other Python interpreters
    or for communication with MATLAB/Simulink

    Args:
        sendSize: number of double values to be sent to TCPIP client
        receiveSize: number of double values to be received from TCPIP client
        IPaddress: string containing IP address of client (e.g., '127.0.0.1')
        port: port for communication with client
        bigEndian: if True, it uses bigEndian, otherwise littleEndian is used for byte order

    Returns:
        returns information (TCPIPdata class) on socket; recommended to store this in mbs.sys['TCPIPobject']

    Example:
        mbs.sys['TCPIPobject'] = CreateTCPIPconnection(sendSize=3, receiveSize=2,
                                                       bigEndian=True, verbose=True)
        sampleTime = 0.01 #sample time in MATLAB! must be same!
        mbs.variables['tLast'] = 0 #in case that exudyn makes finer steps than sample time
        def PreStepUserFunction(mbs, t):
            if t >= mbs.variables['tLast'] + sampleTime:
                mbs.variables['tLast'] += sampleTime
                tcp = mbs.sys['TCPIPobject']
                y = TCPIPsendReceive(tcp, np.array([t, np.sin(t), np.cos(t)])) #time, torque
                tau = y[1]
                exudyn.Print('tau=',tau)
            return True
        try:
            mbs.SetPreStepUserFunction(PreStepUserFunction)
            #%%++++++++++++++++++++++++++++++++++++++++++++++++++
            mbs.Assemble()
            [...] #start renderer; simulate model
        finally: #use this to always close connection, even in case of errors
            CloseTCPIPconnection(mbs.sys['TCPIPobject'])
        #*****************************************
        #the following settings work between Python and MATLAB-Simulink (client), and gives stable results(with only delay of one step):
        # TCP/IP Client Send:
        #   priority = 2 (in properties)
        #   blocking = false
        #   Transfer Delay on (but off also works)
        # TCP/IP Client Receive:
        #   priority = 1 (in properties)
        #   blocking = true
        #   Sourec Data type = double
        #   data size = number of double in packer
        #   Byte order = BigEndian
        #   timeout = 10
    """
    import socket
    import struct
    s = ''
    if bigEndian:
        s = '>' #signals bigEndian format
    packerSend = struct.Struct(s+'d '*sendSize) #'>' for big endian in matlab, I=unsigned int, i=int, d=double
    packerReceive = struct.Struct(s+'d '*receiveSize) #'>' for big endian in matlab, I=unsigned int, i=int, d=double
    if verbose:
        exudyn.Print('setup TCP/IP socket ...')
    socketTCP = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    socketTCP.bind((IPaddress, port))
    socketTCP.listen()
    connection, address = socketTCP.accept()

    if verbose:
        exudyn.Print('TCP/IP connection running!')

    return TCPIPdata(sendSize, receiveSize, packerSend, packerReceive, 
                     socketTCP, connection, address, 0.)


def TCPIPsendReceive(TCPIPobject, sendData):
    """call this function at every simulation step at which you intend to communicate with
    other programs via TCPIP; e.g., call this function in preStepUserFunction of a mbs model

    Args:
        TCPIPobject: the object returned by CreateTCPIPconnection(...)
        sendData: numpy array containing data (double array) to be sent; must agree with sendSize

    Returns:
        returns array as received from TCPIP

    Example:
        mbs.sys['TCPIPobject']=CreateTCPIPconnection(sendSize=2, receiveSize=1, IPaddress='127.0.0.1')
        y = TCPIPsendReceive(mbs.sys['TCPIPobject'], np.array([1.,2.]))
        exudyn.Print(y)
    """
    #first send data (no other way in MATLAB):
    TCPIPobject.connection.sendall(TCPIPobject.packerSend.pack(*sendData))

    #now receive data:
    data = TCPIPobject.connection.recv(TCPIPobject.packerReceive.size) #data size in bytes
    if not data:
        exudyn.Print('WARNING: TCPIPsendReceive: loss of data') #usually does not happen!
        return np.zeros(TCPIPobject.receiveSize)
    else:
        return TCPIPobject.packerReceive.unpack(data)


def CloseTCPIPconnection(TCPIPobject):
    """close a previously created TCPIP connection
    """
    TCPIPobject.connection.close()
    TCPIPobject.socket.close()
