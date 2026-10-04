#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Basic utility functions and constants; they depend on numpy only, not on exudyn.
#
# Author:   Johannes Gerstmayr
# Date:     2020-03-10 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# Notes:    Additional constants are defined: \\
#           pi = 3.1415926535897932 \\
#           sqrt2 = 2**0.5\\
#           g=9.81\\
#           Two variables 'gaussIntegrationPoints' and 'gaussIntegrationWeights' define integration points and weights for function GaussIntegrate(...)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import math #always available in Python
import os   #for the output file paths
import numpy as np
import exudyn
from exudyn.itemInterface import MarkerBodyRigid, VMarkerBodyRigid, SensorUserFunction
from exudyn.misc.deprecation import Deprecated #the deprecations of the library (#2807)

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'pi', 'sqrt2', 'g', 'ClearWorkspace', 'SmartRound2String', 'Normalize',
    'gaussIntegrationPoints', 'gaussIntegrationWeights', 'GaussIntegrate',
    'lobattoIntegrationPoints', 'lobattoIntegrationWeights', 'LobattoIntegrate', 'GetOtherMarker',
    'GetJointArgs', 'ShowOnlyObjects', 'HighlightItem', 'UFsensorRecord', 'AddSensorRecorder',
    'UIWindowSuppressed', 'OutputFilePath', 'CreateDirectoryForFile', 'LoadSolutionFile',
    'NumpyInt8ArrayToString', 'BinaryReadIndex', 'BinaryReadReal', 'BinaryReadString',
    'BinaryReadArrayIndex', 'BinaryReadRealVector', 'LoadBinarySolutionFile', 'RecoverSolutionFile',
    'SetSolutionState', 'AnimateSolution',
    ]

#define some constants which would require external libraries
#pi = 3.1415926535897932 #define pi in order to avoid importing large libraries; identical to from math import pi
pi = math.pi
sqrt2 = 2.**0.5
g = 9.81 #gravity constant


def ClearWorkspace():
    r"""clear all workspace variables except for system variables with '_' at beginning,
    'func' or 'module' in name; it also deletes all items in exudyn.sys and exudyn.variables,
    EXCEPT from exudyn.sys['renderState'] for pertaining the previous view of the renderer

    Note:
        Use this function with CARE! In Spyder, it is certainly safer to add the preference Run$\ra$'remove all variables before execution'. It is recommended to call ClearWorkspace() at the very beginning of your models, to avoid that variables still exist from previous computations which may destroy repeatability of results

    Example:
        import exudyn as exu
        import exudyn.utilities
        #clear workspace at the very beginning, before loading other modules and potentially destroying unwanted things ...
        ClearWorkspace()       #cleanup
        #now continue with other code
        from exudyn.itemInterface import *
        SC = exu.SystemContainer()
        mbs = SC.AddSystem()
        ...
    """
    #only the caller's globals: the module's own ones (pi, ...) stay, or a later import of it fails (#2757)
    import inspect
    fglobals = inspect.stack()[1][0].f_globals
    gl2 = fglobals.copy() #these are the globals of the caller
    for var in gl2:
        if var[0] == '_': continue
        if 'func' in str(fglobals[var]): continue
        if 'module' in str(fglobals[var]): continue

        del fglobals[var]


    import sys
    if 'exudyn' in sys.modules:
        import exudyn #previously, it may have been loaded under another name (e.g., exu)

        sysCopy = exudyn.sys.copy()
        for (key,value) in sysCopy.items():
            #the renderer link is not in exudyn.sys any more, it is
            #exudyn.special.currentRendererSystemContainer, so clearing
            #the workspace no longer takes the renderer away from the container it is attached to
            if key != 'renderState':
                del exudyn.sys[key]
        variablesCopy = exudyn.variables.copy()
        for (key,value) in variablesCopy.items():
            del exudyn.variables[key]

    if 'matplotlib' in sys.modules: #if already imported, we check if there are open figures (which would be lost otherwise)
        import matplotlib.pyplot as plt
        plt.close('all')

def SmartRound2String(x, prec=3):
    """round to max number of digits; may give more digits if this is shorter; using in general the format() with '.g' option, but keeping decimal point and using exponent where necessary
    """
    s = ("{:.0"+str(prec)+"g}").format(x)
    if abs(x) > 1 and x != int(x) and '.' not in s and 'e' not in s:
        s = s+'.'
    if x == int(x) and len(s) > len(str(x)):
        s = str(x)
    return s
        


def Normalize(v):
    """take a vector and return it normalized to L2-norm 1; a zero vector is returned as zero vector

    Args:
        vector v as list or in numpy format

    Returns:
        list: v multiplied with a scalar such that its L2-norm is 1, or the zero vector; a list, as
        callers append the result to lists of normals
    """
    v = np.array(v, dtype=float)
    norm = np.linalg.norm(v)
    if norm != 0:
        v /= norm
    return v.tolist()

#integration points per integration order (1, 3, ...); for interval [-1,1]
gaussIntegrationPoints=[[0],
                        [-(1. / 3.)**0.5, (1. / 3.)**0.5],
                        [-(3. / 5.)**0.5, 0., (3. / 5.)**0.5],
                        [-(3. / 7. + (120.)**0.5 / 35.)**0.5, -(3. / 7. - (120.)**0.5 / 35.)**0.5, (3. / 7. - (120.)**0.5 / 35.)**0.5, (3. / 7. + (120.)**0.5 / 35.)**0.5],
                        [-0.906179845938664, -0.5384693101056831, 0., 0.5384693101056831, 0.906179845938664],
                        ]

#integration weights per integration order (1, 3, ...); for interval [-1,1]
gaussIntegrationWeights=[[2],
                         [1., 1.],
                         [5. / 9., 8. / 9., 5. / 9.],
                         [1. / 2. - 5. / (3.*(120.)**0.5), 1. / 2. + 5. / (3.*(120.)**0.5), 1. / 2. + 5. / (3.*(120.)**0.5), 1. / 2. - 5. / (3.*(120.)**0.5)],
                         [0.23692688505618914, 0.47862867049936636, 0.5688888888888889, 0.47862867049936636, 0.23692688505618914],
                         ]

def GaussIntegrate(functionOfX, integrationOrder, a, b):
    """compute numerical integration of functionOfX in interval [a,b] using Gaussian integration

    Args:
        functionOfX: scalar, vector or matrix-valued function with scalar argument (X or other variable)
        integrationOrder: odd number in {1,3,5,7,9}; currently maximum order is 9
        a: integration range start
        b: integration range end

    Returns:
        (scalar or vectorized) integral value
    """
    cnt = 0
    value = 0*functionOfX(0) #initialize value with correct shape
    if integrationOrder > 9:
        raise ValueError("GaussIntegrate: maximum implemented integration order is 9!")
    if integrationOrder%2 != 1 or integrationOrder < 1:
        raise ValueError("GaussIntegrate: integration order must be odd (1,3,5,...) and > 0")
    
    points = gaussIntegrationPoints[int(integrationOrder/2)]
    weights = gaussIntegrationWeights[int(integrationOrder/2)]
    
    for p in points:
        x = 0.5*(b - a)*p + 0.5*(b + a)
        value += 0.5*(b - a)*weights[cnt]*functionOfX(x);
        cnt += 1

    return value


#integration points per integration order (1, 3, ...); for interval [-1,1]
lobattoIntegrationPoints=[[-1.,1.],
                          [-1., 0., 1.],
                          [-1., -(1./5.)**0.5, (1./5.)**0.5, 1.]]

#integration weights per integration order (1, 3, ...); for interval [-1,1]
lobattoIntegrationWeights=[[ 1., 1.],
                           [ 1./3., 4./3., 1./3.],
                           [ 1./6., 5./6., 5./6., 1./6.]]

def LobattoIntegrate(functionOfX, integrationOrder, a, b):
    """compute numerical integration of functionOfX in interval [a,b] using Lobatto integration

    Args:
        functionOfX: scalar, vector or matrix-valued function with scalar argument (X or other variable)
        integrationOrder: odd number in {1,3,5}; currently maximum order is 5
        a: integration range start
        b: integration range end

    Returns:
        (scalar or vectorized) integral value
    """
    cnt = 0
    value = 0*functionOfX(0) #initialize value with correct shape
    if integrationOrder > 5:
        raise ValueError("LobattoIntegrate: maximum implemented integration order is 5!")
    if integrationOrder%2 != 1 or integrationOrder < 1:
        raise ValueError("LobattoIntegrate: integration order must be odd (1,3,5,...) and >= 1")
    
    points = lobattoIntegrationPoints[int(integrationOrder/2)]
    weights = lobattoIntegrationWeights[int(integrationOrder/2)]
    
    for p in points:
        x = 0.5*(b - a)*p + 0.5*(b + a)
        value += 0.5*(b - a)*weights[cnt]*functionOfX(x);
        cnt += 1

    return value


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#model helpers, sensor recorder, solution files and restart (moved from utilities.py)

def GetOtherMarker(mbs, bodyNumber, existingMarker, show=True):
    """creates a new marker for body with bodyNumber using another marker existingMarker, such that the new marker has the same reference position as the existing marker, working for MarkerBodyPosition (no rotations included); this alleviates creation of markers and calculation of localPosition

    Args:
        mbs: multibody system where new marker is added to
        bodyNumber: body where new marker shall be attached to
        existingMarker: marker number which serves as a reference
        show: if True, marker is shown

    Returns:
        returns marker number of new marker

    Example:
        #oBody0 = mbs.CreateRigidBody(...)
        #oBody1 = mbs.CreateRigidBody(...)
        marker0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oBody0,localPosition=[1,0,0]))
        #create joint from one marker (with rotation) and other body
        mbs.AddObject(SphericalJoint(markerNumbers=[marker0, GetOtherMarker(mbs, oBody1, marker0)]))
    """
    #reference position and rotation of body:
    pRefBody = mbs.GetObjectOutputBody(bodyNumber,exudyn.OutputVariableType.Position,
                                       localPosition=[0,0,0],
                                       configuration=exudyn.ConfigurationType.Reference)

    rotRefBody = mbs.GetObjectOutputBody(bodyNumber,exudyn.OutputVariableType.RotationMatrix,
                                         localPosition=[0,0,0],
                                         configuration=exudyn.ConfigurationType.Reference).reshape((3,3))

    pMarker = mbs.GetMarkerOutput(existingMarker, exudyn.OutputVariableType.Position, 
                                  configuration=exudyn.ConfigurationType.Reference)
    pLocal = rotRefBody.T @ (pMarker - pRefBody)
    marker = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bodyNumber, localPosition=pLocal, 
                                           visualization=VMarkerBodyRigid(show=show)))

    return marker


def GetJointArgs(mbs, markerNumber0=None, markerNumber1=None, 
                 rotationMarker0=None, rotationMarker1=None, 
                 bodyNumber0=None, bodyNumber1=None):
    """creates input args for joints, based on an exiting marker (markerNumber, may be rigid or flex body), with optional existing rotationMarker and uses another rigid body (given as bodyNumber) to create a new MarkerBodyRigid and rotationMarker; this alleviates creation of joint args, see the example; inputs are either markerNumber0 [, rotationMarker0], bodyNumber1 OR markerNumber1 [, rotationMarker1], bodyNumber0

    Args:
        mbs: multibody system where new marker is added to
        markerNumber0: markerNumber of existing rigid body marker
        markerNumber1: markerNumber of existing rigid body marker
        rotationMarker0: rotation of the joint frame relative to the frame of markerNumber0; the joint then takes a copy of the marker turned by it
        rotationMarker1: the same for markerNumber1
        bodyNumber0: existing body used to create new marker
        bodyNumber1: existing body used to create new marker

    Returns:
        returns dict with the 'markerNumbers' list, ready to be used as args; the new marker carries the joint's rotation as its localHT; for a rotationMarker0/1 given, the existing marker is replaced by a copy turned by it

    Example:
        #oBody0 = mbs.CreateRigidBody(...)
        #oBody1 = mbs.CreateRigidBody(...)
        marker0 = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oBody0,
                                                localHT=exu.HT(rotation=RotationMatrixX(0.5*pi), translation=[1,0,0])))
        #create joint from one marker (with rotation) and other body
        mbs.AddObject(RevoluteJointZ(**GetJointArgs(mbs, markerNumber0=marker0, bodyNumber1=oBody1)
    """
    rotationMarkerThis = np.eye(3)
    if markerNumber0 is not None:
        existingMarker = markerNumber0
        if rotationMarker0 is not None:
            rotationMarkerThis = rotationMarker0
        if markerNumber1 is not None:
            raise ValueError('GetJointArgs: if markerNumber0 is given, markerNumber1 must be None')
        if bodyNumber1 is None:
            raise ValueError('GetJointArgs: if markerNumber0 is given, bodyNumber1 must be given too')
        bodyNumber = bodyNumber1
    else:
        existingMarker = markerNumber1
        if markerNumber1 is None:
            raise ValueError('GetJointArgs: if markerNumber0 is None, markerNumber1 must be provided')
        if rotationMarker1 is not None:
            rotationMarkerThis = rotationMarker1
        if bodyNumber0 is None:
            raise ValueError('GetJointArgs: if markerNumber1 is given, bodyNumber0 must be given too')
        bodyNumber = bodyNumber0
        

    #reference position and rotation of body:
    pRefBody = mbs.GetObjectOutputBody(bodyNumber,exudyn.OutputVariableType.Position,
                                       localPosition=[0,0,0],
                                       configuration=exudyn.ConfigurationType.Reference)
    rotRefBody = mbs.GetObjectOutputBody(bodyNumber,exudyn.OutputVariableType.RotationMatrix,
                                         localPosition=[0,0,0],
                                         configuration=exudyn.ConfigurationType.Reference).reshape((3,3))

    pMarker = mbs.GetMarkerOutput(existingMarker, exudyn.OutputVariableType.Position, 
                                  configuration=exudyn.ConfigurationType.Reference)
    rotationMarkerNew = mbs.GetMarkerOutput(existingMarker, exudyn.OutputVariableType.RotationMatrix, 
                                  configuration=exudyn.ConfigurationType.Reference).reshape((3,3))
    
    pLocal = rotRefBody.T @ (pMarker - pRefBody)
    #the new marker carries the rotation of the joint as its localHT (#2745)
    localHT = exudyn.HT(rotation=rotRefBody.T @ rotationMarkerNew @ rotationMarkerThis, translation=pLocal)
    markerNumberNew = mbs.AddMarker(MarkerBodyRigid(bodyNumber=bodyNumber, localHT=localHT))

    #the existing marker: a copy turned by the rotation, so that the joint takes no deprecated rotationMarker0/1 (#2804)
    from exudyn.rigidBodyUtilities import _MarkerWithRotation
    (existingTurned, rotationMarkerRemaining) = _MarkerWithRotation(mbs, existingMarker, rotationMarkerThis)
    args = {'markerNumbers': [existingTurned, markerNumberNew] if markerNumber0 is not None else [markerNumberNew, existingTurned]}
    if np.linalg.norm(rotationMarkerRemaining - np.eye(3)) != 0: #a marker without localHT: only the deprecated way remains
        args['rotationMarker0' if markerNumber0 is not None else 'rotationMarker1'] = rotationMarkerRemaining
    return args


def ShowOnlyObjects(mbs, objectNumbers=[], showOthers=False):
    """function to hide all objects in mbs except for those listed in objectNumbers

    Args:
        mbs: mbs containing object
        objectNumbers: integer object number or list of object numbers to be shown; if empty list [], then all objects are shown
        showOthers: if True, then all other objects are shown again

    Returns:
        changes all colors in mbs, which is NOT reversible
    """
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
    SC=mbs.GetSystemContainer()
    SC.renderer.SendRedrawSignal()


def HighlightItem(SC, mbs, itemNumber, itemType=exudyn.ItemType.Object, showNumbers=True):
    """highlight a certain item with number itemNumber; set itemNumber to -1 to show again all objects

    Args:
        mbs: mbs containing object
        itemNumbers: integer object/node/etc number to be highlighted
        itemType: type of items to be highlighted
        showNumbers: if True, then the numbers of these items are shown
    """
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

    SC.renderer.SendRedrawSignal()


def UFsensorRecord(mbs, t, sensorNumbers, factors, configuration):
    """Internal SensorUserFunction of the deprecated function AddSensorRecorder

    Note:
        Warning: this method is DEPRECATED, use storeInternal in Sensors, which is much more performant; Note, that a sensor usually just passes through values of an existing sensor, while recording the values to a numpy array row-wise (time in first column, data in remaining columns)
    """
    iSensor = sensorNumbers[0]
    val = mbs.GetSensorValues(iSensor, configuration=configuration) #get all values
    if type(val) == float:# or type(x) == nd.float64:
        val = np.array([val]) #for scalar values
    ti = int((t+1e-9)/factors[0]) #add 1e-10 safety factor due to rounding errors when adding time steps (may lead to small errors after 1e7 steps)
    if ti >= 0 and ti < len(mbs.variables['sensorRecord'+str(iSensor)]):
        mbs.variables['sensorRecord'+str(iSensor)][ti,0] = t
        mbs.variables['sensorRecord'+str(iSensor)][ti,1:] = val
        
    return val #return value usually not used further


@Deprecated('1.11.0', 2029, use='a sensor with storeInternal=True')
def AddSensorRecorder(mbs, sensorNumber, endTime, sensorsWritePeriod, sensorOutputSize=3):
    """DEPRECATED: Add a SensorUserFunction object in order to record sensor output internally; this avoids creation of files for sensors, which can speedup and simplify evaluation in ParameterVariation and GeneticOptimization; values are stored internally in mbs.variables['sensorRecord'+str(sensorNumber)] where sensorNumber is the mbs sensor number

    Args:
        mbs: mbs containing object
        sensorNumber: integer sensor number to be recorded
        endTime: end time of simulation, as given in simulationSettings.timeIntegration.endTime
        sensorsWritePeriod: as given in simulationSettings.solution.sensors.writePeriod
        sensorOutputSize: size of sensor data: 3 for Displacement, Position, etc. sensors; may be larger for RotationMatrix or Coordinates sensors; check this size by calling mbs.GetSensorValues(sensorNumber)

    Returns:
        adds an according SensorUserFunction sensor to mbs; returns new sensor number; during initialization a new numpy array is allocated in  mbs.variables['sensorRecord'+str(sensorNumber)] and the information is written row-wise: [time, sensorValue1, sensorValue2, ...]

    Note:
        Warning: this method is DEPRECATED, use storeInternal in Sensors, which is much more performant; Note, that a sensor usually just passes through values of an existing sensor, while recording the values to a numpy array row-wise (time in first column, data in remaining columns)
    """
    nSteps = int(endTime/sensorsWritePeriod)
    mbs.variables['sensorRecord'+str(sensorNumber)] = np.zeros((nSteps+1,1+sensorOutputSize)) #time+3 sensor values

    sUserRecord = mbs.AddSensor(SensorUserFunction(sensorNumbers=[sensorNumber], 
                                                   factors=[sensorsWritePeriod],
                                                   writeToFile=False,
                                                   sensorUserFunction=UFsensorRecord))
    
    return sUserRecord


#remembers which kinds already said it, so that the notice is printed ONCE per kind (#2477)
_uiWindowNoticePrinted = set()

def UIWindowSuppressed(kind, callerInfo=''):
    """ask exudyn.special.userInterface whether this kind of window must not open (#2477)

    Note:
        The flags are set for automated runs - test runners, CI, AI-assisted development - where a
        window that waits for a human stops everything. A suppressed call is a silent no-op, except
        that the FIRST suppression of each kind prints one line, so that a window-less session is
        never a mystery.

    Args:
        kind: one of 'Renderer', 'SolutionViewer', 'Plots', 'Dialogs'; the name of the flag without
              the 'suppress' prefix
        callerInfo: name of the calling function, shown in the notice

    Returns:
        True if the window must not be opened

    Example:
        if UIWindowSuppressed('Plots', 'PlotSensor'): return plt
    """
    suppressed = bool(getattr(exudyn.special.userInterface, 'suppress' + kind))
    if suppressed and kind not in _uiWindowNoticePrinted:
        _uiWindowNoticePrinted.add(kind)
        print('NOTE: ' + (callerInfo + ' ' if callerInfo != '' else '')
              + 'opens no window, because exudyn.special.userInterface.suppress' + kind + '=True')
    return suppressed


def OutputFilePath(fileName, callerInfo=''):
    """merge a local file name with the global exudyn.config.outputDirectory, exactly as the solver
    does when it writes solution, sensor, image and print files (#2454)

    Note:
        The rule in Exudyn is: everything WRITTEN as output of a run follows
        exudyn.config.outputDirectory, and a file is READ from there only if its name comes from
        Exudyn itself - the simulation settings (SolutionViewer) or a sensor definition
        (PlotSensor). A file name that you pass to a function such as LoadSolutionFile is read
        exactly as given; wrap it in OutputFilePath(...) yourself if you want the output directory.
        Model data (mesh import, FEMinterface/ObjectFFRFreducedOrderInterface SaveToFile and
        LoadFromFile, SaveDictToHDF5/LoadDictFromHDF5) is never redirected.

    Args:
        fileName: file name as given by the user or stored in a sensor or in the simulation settings
        callerInfo: name of the calling function, used in the error message

    Returns:
        fileName unchanged if exudyn.config.outputDirectory is empty, otherwise the merged path

    Example:
        exudyn.config.outputDirectory = 'run17'
        OutputFilePath('solution/sensor.txt') #'run17/solution/sensor.txt'
    """
    outputDirectory = exudyn.config.outputDirectory
    if outputDirectory == '' or fileName == '':
        return fileName

    #an absolute file name and a set output directory contradict each other; raise the same way as
    #the C++ writers, but name the function the user called
    if (fileName[0] == '/' or fileName[0] == '\\'
        or (len(fileName) > 1 and fileName[1] == ':')):
        raise ValueError((callerInfo + ': ' if callerInfo != '' else '')
                         + 'the file name "' + fileName + '" is an absolute path, while '
                         'exudyn.config.outputDirectory is set to "' + outputDirectory + '"; '
                         'use a relative file name or reset exudyn.config.outputDirectory = ""')

    if outputDirectory[-1] in '/\\':
        return outputDirectory + fileName
    return outputDirectory + '/' + fileName


def CreateDirectoryForFile(fileName):
    """create the directory a file is going to be written into, if it does not exist yet

    Note:
        Every Exudyn function that writes a file calls this first, so that a path such as
        'solution/sensor.txt' - or anything under exudyn.config.outputDirectory - works without the
        caller having to create the directory. Failure is deliberately ignored: creating a
        directory can fail for reasons that do not stop the write (a read-only parent on a network
        share, a race with another process that just created it), and the write itself reports the
        real problem with a better message (#2493).

    Args:
        fileName: file name including its path; a name without any path is left alone

    Returns:
        fileName unchanged, so that the call can wrap the file name at the point of use
    """
    directoryName = os.path.dirname(fileName)
    if directoryName != '':
        try:
            os.makedirs(directoryName, exist_ok=True)
        except Exception:
            pass #see the note above: the write reports what actually went wrong

    return fileName


def LoadSolutionFile(fileName, safeMode=False, maxRows=-1, verbose=True, hasHeader=True):
    """read coordinates solution file (exported during static or dynamic simulation with option exu.SimulationSettings().solution.file.name='...') into dictionary:

    Args:
        fileName: string containing directory and filename of stored coordinatesSolutionFile
        saveMode: if True, it loads lines directly to load inconsistent lines as well; use this for huge files (>2GB); is slower but needs less memory!
        verbose: if True, some information is written when importing file (use for huge files to track progress)
        maxRows: maximum number of data rows loaded, if saveMode=True; use this for huge files to reduce loading time; set -1 to load all rows
        hasHeader: set to False, if file is expected to have no header; if False, then some error checks related to file header are not performed

    Returns:
        dictionary with 'data': the matrix of stored solution vectors, 'columnsExported': a list with integer values showing the exported sizes [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData], 'nColumns': the number of data columns and 'nRows': the number of data rows
    """

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
    if hasHeader:
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
        
        if verbose: exudyn.Print('text file loaded ... converting ...')
        
        cntDataRows = 0
        dataRowStart = -1
        dataRowLast = 0

        cnt = 0
        cntComments = 0
        for line in lines: 
            if line[0]!='#': 
                if dataRowStart == -1:
                    dataRowStart = cnt
                cntDataRows+=1
                dataRowLast = cnt
            else:
                cntComments += 1
            cnt+=1
        
        if verbose: exudyn.Print('found',cntComments,'lines with comments, which are ignored')
            
        if maxRows != -1 and cntDataRows > maxRows:
            cntDataRows = maxRows
        
        if cntDataRows == 0 or dataRowStart == -1:
            raise ValueError('LoadSolutionFile: no rows found')
        else:
            if verbose: exudyn.Print('data starts at ',dataRowStart, ', found ', cntDataRows, ' rows', sep='')

        if verbose: exudyn.Print('check columns ...')
        
        cols = len(lines[dataRowStart].split(','))
        if hasHeader:
            if cols != nColumns+1:
                raise ValueError('ERROR in LoadSolution: number of columns in first data row is inconsistent: got ',cols,' columns, but expected ', nColumns+1)
        else:
            nColumns=cols-1
            columnsExported=[] #unknown ...
            
        #check last line, which may be incomplete:
        #colsLastLine = len(lines[dataRowStart+cntDataRows-1].split(','))
        colsLastLine = len(lines[dataRowLast].split(','))
        skipLast = 0
        if colsLastLine != cols:
            if verbose: exudyn.Print('LoadSolution: WARNING number of columns in last data row is inconsistent; will be skipped')
            skipLast = 1
        
        if verbose: exudyn.Print('file contains ',cntDataRows, ' rows and ', cols, ' columns (incl. time)',sep='')
        
        data = np.zeros((cntDataRows, nColumns+1))
        
        progress = 0
        progressInfo = 5000000
        skipLine = 0 #counter for skipping additional lines with comments
        for i in range(cntDataRows-skipLast):
            if verbose and progress>=progressInfo: #update progress
                exudyn.Print('import data row', i, '/', cntDataRows)
                progress = 0
            progress+=nColumns
            
            while i+dataRowStart+skipLine<len(lines) and lines[i+dataRowStart+skipLine][0]=='#':
                skipLine += 1
            
            ylist = lines[i+dataRowStart+skipLine].split(',')
            if len(ylist) == nColumns+1:
                y=np.array(ylist, dtype=float)
                data[i,:] = y[:]
            elif verbose:
                exudyn.Print('  data row', i, 'is inconsistent:',len(ylist), nColumns+1,' ... skipped')

    if verbose: exudyn.Print('columns imported =', columnsExported)
    if verbose: exudyn.Print('total columns to be imported =', nColumns, ', array size of file =', np.size(data,1))

    if (nColumns + 1) != np.size(data,1): #one additional column for time!
        raise ValueError('ERROR in LoadSolution: number of columns is inconsistent')

    nRows = np.size(data,0)

    return dict({'data': data, 'columnsExported': columnsExported,'nColumns': nColumns,'nRows': nRows})


def NumpyInt8ArrayToString(npArray):
    """simple conversion of int8 arrays into strings (not highly efficient, so use only for short strings)
    """
    s=''
    for x in npArray:
        s+=chr(x)
    return s


def BinaryReadIndex(file, intType):
    """read single Index from current file position in binary solution file
    """
    data = np.fromfile(file, dtype=intType, count=1)
    if data.size != 1: return [0,True] #end of file
    return [data[0], False]


def BinaryReadReal(file, realType):
    """read single Real from current file position in binary solution file
    """
    data = np.fromfile(file, dtype=realType, count=1)
    if data.size != 1: return [0,True] #end of file
    return [data[0], False]


def BinaryReadString(file, intType):
    """read string from current file position in binary solution file
    """
    dataLength = np.fromfile(file, dtype=intType, count=1)[0]
    data = np.fromfile(file, dtype=np.byte, count=dataLength)
    return [NumpyInt8ArrayToString(data), False]


def BinaryReadArrayIndex(file, intType):
    """read Index array from current file position in binary solution file
    """
    dataLength = np.fromfile(file, dtype=intType, count=1)[0]
    data = np.fromfile(file, dtype=intType, count=dataLength)
    return [data, False]


def BinaryReadRealVector(file, intType, realType):
    """read Real vector from current file position in binary solution file

    Returns:
        return data as numpy array, or False if no data read
    """
    sizeData = np.fromfile(file, dtype=intType, count=1)
    if sizeData.size != 1: return [[],True] #end of file
    dataLength = sizeData[0]
    data = np.fromfile(file, dtype=realType, count=dataLength)
    if data.size != dataLength: return [[],True] #end of file
    return [data, False]


def LoadBinarySolutionFile(fileName, maxRows=-1, verbose=True):
    """read BINARY coordinates solution file (exported during static or dynamic simulation with option exu.SimulationSettings().solution.file.name='...') into dictionary

    Args:
        fileName: string containing directory and filename of stored coordinatesSolutionFile
        verbose: if True, some information is written when importing file (use for huge files to track progress)
        maxRows: maximum number of data rows loaded, if saveMode=True; use this for huge files to reduce loading time; set -1 to load all rows

    Returns:
        dictionary with 'data': the matrix of stored solution vectors, 'columnsExported': a list with integer values showing the exported sizes [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData], 'nColumns': the number of data columns and 'nRows': the number of data rows
    """
    with open(fileName, 'r') as file:
        data = np.fromfile(file, dtype=np.byte, count=6)
        s = NumpyInt8ArrayToString(data)
        if int(verbose)>1: exudyn.Print(s)
        if s!='EXUBIN':
            raise ValueError('LoadBinarySolutionFile: no binary header found!')

        if verbose: exudyn.Print('read binary file')
        fileEnd = False
#             ExuFile::BinaryWriteHeader(solFile, bfs);
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
            exudyn.Print('  indexSize=',indexSize)
            exudyn.Print('  realSize=',realSize)
            exudyn.Print('  pointerSize=',pointerSize)
            exudyn.Print('  bigEndian=',bigEndian)

#         ExuFile::BinaryWrite(EXUstd::exudynVersion, solFile, bfs);
        sVersion, fileEnd=BinaryReadString(file, intType)
        if verbose: exudyn.Print('  version=',sVersion)

#         ExuFile::BinaryWrite(STDstring("Mode0000"), solFile, bfs); //change this in future to add new features
        sMode, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: exudyn.Print('  mode=',sMode)

#             STDstring str = "Exudyn " + GetSolverName() + " ";
#             if (isStatic) { str+="static "; }
#             str+="solver solution file";
#             ExuFile::BinaryWrite(str, solFile, bfs);
        sSolver, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: exudyn.Print('  solver=',sSolver)

#             //solFile << "#simulation started=" << EXUstd::GetDateTimeString() << "\n";
#             ExuFile::BinaryWrite(EXUstd::GetDateTimeString(), solFile, bfs);
        sTime, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: exudyn.Print('  data/time=',sTime)

#             //not needed in binary format:
#             //solFile << "#columns contain: time, ODE2 displacements";
#             //if (solution.file.export.velocities) { solFile << ", ODE2 velocities"; }
#             //if (solution.file.export.accelerations) { solFile << ", ODE2 accelerations"; }
#             //if (nODE1) { solFile << ", ODE1 coordinates"; } //currently not available, but for future solFile structure necessary!
#             //if (nVel1) { solFile << ", ODE1 velocities"; }
#             //if (solution.file.export.algebraicCoordinates) { solFile << ", AE coordinates"; }
#             //if (solution.file.export.dataCoordinates) { solFile << ", ODE2 velocities"; }
#             //solFile << "\n";

#             //solFile << "#number of system coordinates [nODE2, nODE1, nAlgebraic, nData] = [" <<
#             //    nODE2 << "," << nODE1 << "," << nAE << "," << nData << "]\n"; //this will allow to know the system information, independently of coordinates written
#             ArrayIndex sysCoords({nODE2, nODE1, nAE, nData});
#             ExuFile::BinaryWrite(sysCoords, solFile, bfs);
        systemSizes, fileEnd=BinaryReadArrayIndex(file, intType)
        if verbose: exudyn.Print('  systemSizes=',systemSizes)


#             //solFile << "#number of written coordinates [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData] = [" << //these are the exported coordinates line-by-line
#             //    nODE2 << "," << nVel2 << "," << nAcc2 << "," << nODE1 << "," << nVel1 << "," << nAEexported << "," << nDataExported << "]\n"; //python convert line with v=eval(line.split('=')[1])
#             ArrayIndex writtenCoords({ nODE2, nVel2, nAcc2, nODE1, nVel1, nAEexported, nDataExported });
#             ExuFile::BinaryWrite(writtenCoords, solFile, bfs);
        columnsExported, fileEnd=BinaryReadArrayIndex(file, intType)
        if verbose: exudyn.Print('  columnsExported=',columnsExported)
        nColumns = sum(columnsExported) #total size of data per row
        
#             //solFile << "#total columns exported  (excl. time) = " << totalCoordinates << "\n";
#             ExuFile::BinaryWrite(totalCoordinates, solFile, bfs);
        totalCoordinates, fileEnd = BinaryReadIndex(file, intType)

#             Index numberOfSteps;
#             if (!isStatic) { numberOfSteps = timeint.numberOfSteps; }
#             else { numberOfSteps = staticSolver.numberOfLoadSteps; }
#             ExuFile::BinaryWrite(numberOfSteps, solFile, bfs);
        numberOfSteps, fileEnd = BinaryReadIndex(file, intType)
        
#             //solution information: always export string, even if has zero length:
#             ExuFile::BinaryWrite(solution.file.information, solFile, bfs);
        solutionInformation, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: exudyn.Print('  solutionInformation="'+solutionInformation+'"')

#             //add some checksum ...
#             ExuFile::BinaryWrite(STDstring("EndOfHeader"), solFile, bfs);
#             //next byte starts with solution
        EndOfHeader, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: exudyn.Print('  EndOfHeader found: ',EndOfHeader)
        if EndOfHeader!='EndOfHeader':
            raise ValueError('LoadBinarySolutionFile: EndOfHeader not found')


        #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        #read time steps
#         if (isBinary) //add size, only in binary mode
#         {
#             //including 1 real for time+1 Index for nVectors, but excluding bytes for this Index
#             Index lineSizeBytes = nVectors * bfs.indexSize + nValues * bfs.realSize + bfs.indexSize + bfs.realSize;
#             ExuFile::BinaryWrite(lineSizeBytes, solFile, bfs); //size of line, for fast skipping of solution line
#             ExuFile::BinaryWrite(nVectors, solFile, bfs); //number of vectors could vary if needed
#         }
        if fileEnd: exudyn.Print('  ==> end of file found during in header')
        fileEnd = False
        data = np.zeros((0,nColumns+1))
        nRows = 0
        line = np.zeros(nColumns+1)
        validEndFound = False
        dataList = [] #list is much faster than hstack !
        
        while not fileEnd:
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
                if int(verbose)>1: exudyn.Print('end of file reached')
                validEndFound = True
                break
            if int(verbose)>1: 
                exudyn.Print('  dataLength=',dataLength)
                
            line = np.fromfile(file, dtype=realType, count=dataLength)
            if line.size != dataLength: 
                fileEnd=True
                break
            #line, fileEnd=BinaryReadRealVector(file, intType, realType)
            if int(verbose)>1: 
                exudyn.Print('  read',line.size, 'columns (',nColumns+1,'expected)')
                exudyn.Print('  line=',line)
            if fileEnd: break
            if line.size != nColumns+1:
                raise ValueError('LoadBinarySolutionFile: rows are inconsistent')
            
            nRows += 1

            dataList+=[line]
            #data = np.vstack((data, line)) #slow!

        if verbose: exudyn.Print('  read '+str(nRows)+' rows from file')
        
        data = np.array(dataList)
        nRows = np.size(data,0)

        if not validEndFound:
            exudyn.Print('LoadBinarySolutionFile: WARNING: end of file inconsistent!')
        else:
            if int(verbose)>1: 
                exudyn.Print('  valid end of data found')
                exudyn.Print('LoadBinarySolutionFile finished')
    
        return dict({'data': data, 'columnsExported': columnsExported,'nColumns': nColumns,'nRows': nRows})


def RecoverSolutionFile(fileName, newFileName, verbose=0):
    """recover solution file with last row not completely written (e.g., if crashed, interrupted or no flush file option set)

    Args:
        fileName: string containing directory and filename of stored coordinatesSolutionFile
        newFileName: string containing directory and filename of new coordinatesSolutionFile
        verbose: 0=no information, 1=basic information, 2=information per row

    Returns:
        writes only consistent rows of file to file with name newFileName
    """
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
        exudyn.Print('columns imported =', columnsExported)
        exudyn.Print('total columns to be imported =', expectedColumns, '(incl. time)\n')


    with open(newFileName, 'w') as fileWrite:
        with open(fileName) as file:
            cnt = 0
            cntSolution = 0
            for line in file:
                if line[0] == '#':
                    if len(line) < 1000 and verbose >= 1:
                        exudyn.Print('HEADER:', line, end='')
                    fileWrite.write(line)
                else:
                    #cols = len(line.split(','))
                    cols = line.count(',')+1 #+1 needed, because two columns for one comma
                    if cols == expectedColumns:
                        if verbose >= 2:
                            exudyn.Print('data row ',cntSolution,', #cols=',cols, ', text=',line[0:12],'...', sep='')
                        fileWrite.write(line)
                    else:
                        if verbose >= 1:
                            exudyn.Print('\nWARNING: ignored solution data',cntSolution, '(file line',cnt,'), columns=', cols, '\n')
                    cntSolution += 1
                
                cnt += 1


#a prototype, not public: how a simulation continues from its restart file is still to be decided (#2850)
def _InitializeFromRestartFile(mbs, simulationSettings, restartFileName, verbose=True):
    """recover initial coordinates, time, etc. from given restart file; modifies simulationSettings and sets the according initial conditions in mbs

    Args:
        mbs: MainSystem to be operated with
        simulationSettings: simulationSettings which is updated and shall be used afterwards for SolveDynamic(...) or SolveStatic(...)
        restartFileName: string containing directory and filename of stored restart file, as given in solution.restart.name
        verbose: False=no information, True=basic information
    """
    raise ValueError('InitializeFromRestartFile: not fully implemented')

    fileRead=open(restartFileName,'r') 
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
        exudyn.Print('columns available in restart file =', columnsExported)
        exudyn.Print('total columns to be imported =', expectedColumns, '(incl. time)\n')

    if fileLines[-1][0:9]!='#FINISHED':
        raise ValueError('ERROR in InitializeFromRestartFile: last line does not contain "#FINISHED" and is thus expected to be corrupted!')

    #now everything should be ok and we can just read the line with numpy:
    data = np.loadtxt(restartFileName, comments='#', delimiter=',')
    nRows = np.size(data,0) #should be 1
    if nRows != 1:
        raise ValueError('ERROR in InitializeFromRestartFile: got more than one rows, but expected one')

    rowData = data[-1] #last row
    #cols = solution['columnsExported']
    [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData] = columnsExported

    #update several configurations:
    configurations = [exudyn.ConfigurationType.Current, 
                      exudyn.ConfigurationType.Initial, 
                      exudyn.ConfigurationType.Visualization]
    
    for configuration in configurations:
        mbs.systemData.SetODE2Coordinates(rowData[1:1+nODE2], configuration)
        if (nVel2): mbs.systemData.SetODE2Coordinates_t(rowData[1+nODE2:1+nODE2+nVel2], configuration)
        if (nAcc2): mbs.systemData.SetODE2Coordinates_tt(rowData[1+nODE2+nVel2:1+nODE2+nVel2+nAcc2], configuration)
        if (nODE1): mbs.systemData.SetODE1Coordinates(rowData[1+nODE2+nVel2+nAcc2:1+nODE2+nVel2+nAcc2+nODE1], configuration)
        if (nVel1): mbs.systemData.SetODE1Coordinates_t(rowData[1+nODE2+nVel2+nAcc2+nODE1:1+nODE2+nVel2+nAcc2+nODE1+nVel1], configuration)
        
        if (nAlgebraic): mbs.systemData.SetAECoordinates(rowData[1+nODE2+nVel2+nAcc2+nODE1+nVel1:1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic], configuration)
        if (nData): mbs.systemData.SetDataCoordinates(rowData[1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic:1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic+nData], configuration)
    
        if configuration == exudyn.ConfigurationType.Visualization:
            mbs.systemData.SetTime(rowData[0], exudyn.ConfigurationType.Visualization)
            SC=mbs.GetSystemContainer()
            SC.renderer.SendRedrawSignal()
    
    #add integration parameters to simulationSettings ...
    
    if verbose: exudyn.Print('\nInitializeFromRestartFile finished\n')


def SetSolutionState(mbs, solution, row, configuration=exudyn.ConfigurationType.Current, sendRedrawSignal=True):
    """load selected row of solution dictionary (previously loaded with LoadSolutionFile) into specific state; flag sendRedrawSignal is only used if configuration = exudyn.ConfigurationType.Visualization
    """
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
            SC=mbs.GetSystemContainer()
            SC.renderer.SendRedrawSignal()
    else:
        exudyn.Print("ERROR in SetVisualizationState: invalid row (out of range)")


def AnimateSolution(mbs, solution, rowIncrement = 1, timeout=0.04, createImages = False, runLoop = False):
    """This function is not further maintaned and should only be used if you do not have tkinter (like on some MacOS versions); use exudyn.interactive.SolutionViewer() instead! AnimateSolution consecutively load the rows of a solution file and visualize the result

    Args:
        mbs: the system used for animation
        solution: solution dictionary previously loaded with LoadSolutionFile; will be played from first to last row
        rowIncrement: can be set larger than 1 in order to skip solution frames: e.g. rowIncrement=10 visualizes every 10th row (frame)
        timeout: in seconds is used between frames in order to limit the speed of animation; e.g. use timeout=0.04 to achieve approximately 25 frames per second
        createImages: creates consecutively images from the animation, which can be converted into an animation
        runLoop: if True, the animation is played in a loop until 'q' is pressed in render window

    Returns:
        renders the scene in mbs and changes the visualization state in mbs continuously
    """
    SC = mbs.GetSystemContainer()
    nRows = solution['nRows']
    if nRows == 0:
        exudyn.Print('ERROR in AnimateSolution: solution file is empty')
        return
    if (rowIncrement < 1) or (rowIncrement > nRows):
        exudyn.Print('ERROR in AnimateSolution: rowIncrement must be at least 1 and must not be larger than the number of rows in the solution file')
    oldUpdateInterval = SC.visualizationSettings.general.graphicsUpdateInterval
    SC.visualizationSettings.general.graphicsUpdateInterval = 0.5*min(timeout, 2e-3) #avoid too small values to run multithreading properly
    mbs.SetRenderEngineStopFlag(False) #not to stop right at the beginning

    while runLoop and not mbs.GetRenderEngineStopFlag():
        for i in range(0,nRows,rowIncrement):
            if not(mbs.GetRenderEngineStopFlag()):
                #SetVisualizationState(exudyn, mbs, solution, i) #OLD
                SetSolutionState(mbs, solution, i, exudyn.ConfigurationType.Visualization)
                if createImages:
                    SC.renderer.RedrawAndSaveImage() #create images for animation
                #time.sleep(timeout)
                SC.renderer.DoIdleTasks(timeout)

    SC.visualizationSettings.general.graphicsUpdateInterval = oldUpdateInterval #set values back to original
