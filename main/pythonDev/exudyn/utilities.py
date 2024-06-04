#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Basic support functions for simpler creation of Exudyn models.
#           Advanced functions for loading and animating solutions and for drawing a graph of the mbs system.
#           This library requires numpy (as well as time and copy)
#
# Author:   Johannes Gerstmayr
# Date:     2019-07-26 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Utility functions and structures for Exudyn

import numpy as np
#import copy as copy #to be able to copy e.g. lists
from math import sqrt #, sin, cos, pi

import exudyn
from exudyn.basicUtilities import *
from exudyn.advancedUtilities import *
from exudyn.rigidBodyUtilities import *
from exudyn.graphicsDataUtilities import *
import exudyn.graphics #requires import for usage during __init__.py
from exudyn.itemInterface import *

#for compatibility with older models:
from exudyn.beams import GenerateStraightLineANCFCable2D, GenerateSlidingJoint, GenerateAleSlidingJoint,\
                         GenerateStraightBeam

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add deprecated GraphicsData functions such that "from exudyn.utilities import *" keeps working:

GraphicsDataOrthoCubePoint = exudyn.graphics.Brick
GraphicsDataCube = exudyn.graphics.Cuboid
GraphicsDataOrthoCube = exudyn.graphics.BrickXYZ
GraphicsDataSphere = exudyn.graphics.Sphere
GraphicsDataCylinder = exudyn.graphics.Cylinder

GraphicsDataLine = exudyn.graphics.Lines
GraphicsDataQuad = exudyn.graphics.Quad
GraphicsDataCircle = exudyn.graphics.Circle
GraphicsDataText = exudyn.graphics.Text

#advanced objects
GraphicsDataRigidLink = exudyn.graphics.RigidLink
GraphicsDataSolidOfRevolution = exudyn.graphics.SolidOfRevolution
GraphicsDataSolidExtrusion = exudyn.graphics.SolidExtrusion
GraphicsDataArrow = exudyn.graphics.Arrow
GraphicsDataBasis = exudyn.graphics.Basis
GraphicsDataFrame = exudyn.graphics.Frame
GraphicsDataCheckerBoard = exudyn.graphics.CheckerBoard

#import/export and transformations
GraphicsDataFromSTLfile = exudyn.graphics.FromSTLfile
GraphicsDataFromSTLfileTxt = exudyn.graphics.FromSTLfileASCII
GraphicsDataFromPointsAndTrigs = exudyn.graphics.FromPointsAndTrigs
GraphicsData2PointsAndTrigs = exudyn.graphics.ToPointsAndTrigs
ExportGraphicsData2STL = exudyn.graphics.ExportSTL

MoveGraphicsData = exudyn.graphics.Move
MergeGraphicsDataTriangleList = exudyn.graphics.MergeTriangleLists
AddEdgesAndSmoothenNormals = exudyn.graphics.AddEdgesAndSmoothenNormals

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#**function: internal function used for CreateDistanceSensor
def __UFsensorDistance(mbs, t, sensorNumbers, factors, configuration):

    generalContactIndex = int(factors[0])
    dirSensor = factors[5:8]
    markerNumber = int(factors[1])
    hasRotation = False
    if markerNumber != -1:
        p0 = mbs.GetMarkerOutput(markerNumber, variableType=exudyn.OutputVariableType.Position)
        hasRotation = ('Rigid' in mbs.GetMarker(markerNumber)['markerType'])
        if hasRotation:
            A0 = mbs.GetMarkerOutput(markerNumber, variableType=exudyn.OutputVariableType.RotationMatrix).reshape((3,3))
            dirSensor = A0 @ dirSensor
    else:
        p0 = np.array(factors[2:5])

    [minDistance, maxDistance, cylinderRadius, selectedTypeIndex, measureVelocity, graphicsObject, flags] = factors[8:15]
    selectedTypeIndex = exudyn.ContactTypeIndex(int(selectedTypeIndex)) #only converts from int
    measureVelocity = bool(measureVelocity)
    graphicsObject = int(graphicsObject)

    gContact = mbs.GetGeneralContact(generalContactIndex)
    data = gContact.ShortestDistanceAlongLine(pStart = p0, direction = dirSensor, 
                                           minDistance=minDistance, maxDistance=maxDistance,
                                           cylinderRadius=cylinderRadius, asDictionary=(measureVelocity==True),
                                           typeIndex=selectedTypeIndex,
                                           )
    if measureVelocity:
        d = data['distance']
        v = data['velocityAlongLine']
        rv = [d, v]
    else:
        d = data
        rv = [d]

    if graphicsObject != -1:
        factLen = 1
        if int(flags) == 0:
            factLen = 0

        mbs.SetObjectParameter(graphicsObject,'referencePosition',p0 + factLen*(d*np.array(Normalize(dirSensor))) )
        if hasRotation:
            mbs.SetObjectParameter(graphicsObject,'referenceRotation', A0)
            
    return rv



#**function: Add geometry for distance sensor given by points and triangles (point indices) to mbs; use a rigid body marker where the geometry is put on; 
#           Creates a GeneralContact for efficient search on background. If you have several sets of points and trigs, first merge them or add them manually to the contact
#**input: 
#  mbs: MainSystem where contact is created
#  meshPoints: list of points (3D), as returned by graphics.ToPointsAndTrigs()
#  meshTrigs: list of trigs (3 node indices each), as returned by graphics.ToPointsAndTrigs()
#  rigidBodyMarkerIndex: rigid body marker to which the triangles are fixed on (ground or moving object)
#  searchTreeCellSize: size of search tree (X,Y,Z); use larger values in directions where more triangles are located
#**output: int; returns ngc, which is the number of GeneralContact in mbs, to be used in CreateDistanceSensor(...); keep the gContact as deletion may corrupt data
#**notes: should be used by CreateDistanceSensor(...) and AddLidar(...) for simple initialization of GeneralContact; old name: DistanceSensorSetupGeometry(...)
#**belongsTo: MainSystem
def CreateDistanceSensorGeometry(mbs, meshPoints, meshTrigs, rigidBodyMarkerIndex, searchTreeCellSize=[8,8,8]):
    gContact = mbs.AddGeneralContact()
    gContact.SetFrictionPairings(0*np.eye(1)) #may not be empty
    gContact.SetSearchTreeCellSize(numberOfCells=searchTreeCellSize)
    # [meshPoints, meshTrigs] = RefineMesh(meshPoints, meshTrigs) #just to have more triangles on floor
    gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=rigidBodyMarkerIndex,
                                        contactStiffness=1, contactDamping=1, #dummy values
                                        frictionMaterialIndex=0, pointList=meshPoints, triangleList=meshTrigs)
    gContact.isActive=False #no contact computation; could also be done later on, if many moving objects are used ...
    ngc = mbs.NumberOfGeneralContacts()-1
    gContact = mbs.GetGeneralContact(ngc) #keeps reference to gContact, while other functions work with automatic ...

    return ngc

#**function: Function to create distance sensor based on GeneralContact in mbs; sensor can be either placed on absolute position or attached to rigid body marker; in case of marker, dirSensor is relative to the marker
#**input:
#  mbs: the MainSystem where distance sensor is created
#  generalContactIndex: the number of the GeneralContact object in mbs; the index of the GeneralContact object which has been added with last AddGeneralContact(...) command is generalContactIndex=mbs.NumberOfGeneralContacts()-1
#  positionOrMarker: either a 3D position as list or np.array, or a MarkerIndex with according rigid body marker
#  dirSensor: the direction (no need to normalize) along which the distance is measured (must not be normalized); in case of marker, the direction is relative to marker orientation if marker contains orientation (BodyRigid, NodeRigid)
#  minDistance: the minimum distance which is accepted; smaller distance will be ignored
#  maxDistance: the maximum distance which is accepted; items being at maxDistance or futher are ignored; if no items are found, the function returns maxDistance
#  cylinderRadius: in case of spheres (selectedTypeIndex=ContactTypeIndex.IndexSpheresMarkerBased), a cylinder can be used which measures the shortest distance at a certain radius (geometrically interpreted as cylinder)
#  selectedTypeIndex: either this type has default value, meaning that all items in GeneralContact are measured, or there is a specific type index, which is the only type that is considered during measurement
#  storeInternal: like with any SensorUserFunction, setting to True stores sensor data internally
#  fileName: if defined, recorded data of SensorUserFunction is written to specified file
#  measureVelocity: if True, the sensor measures additionally the velocity (component 0=distance, component 1=velocity); velocity is the velocity in direction 'dirSensor' and does not account for changes in geometry, thus it may be different from the time derivative of the distance!
#  addGraphicsObject: if True, the distance sensor is also visualized graphically in a simplified manner with a red line having the length of dirSensor; NOTE that updates are ONLY performed during computation, not in visualization; for this reason, solutionSettings.sensorsWritePeriod should be accordingly small
#  drawDisplaced: if True, the red line is drawn backwards such that it moves along the measured surface; if False, the beam is fixed to marker or position
#  color: optional color for 'laser beam' to be drawn
#**output: SensorIndex; creates sensor and returns according sensor number of SensorUserFunction
#**notes: use generalContactIndex = CreateDistanceSensorGeometry(...) before to create GeneralContact module containing geometry; old name: AddDistanceSensor(...)
#**belongsTo: MainSystem
def CreateDistanceSensor(mbs, generalContactIndex,
                      positionOrMarker, dirSensor, minDistance=-1e7, 
                      maxDistance=1e7, cylinderRadius=0, 
                      selectedTypeIndex=exudyn.ContactTypeIndex.IndexEndOfEnumList,
                      storeInternal = False, fileName = '', measureVelocity = False,
                      addGraphicsObject=False, drawDisplaced=True, color=exudyn.graphics.color.red):
    
    markerNumber = -1
    p0list = [0,0,0]
    if type(positionOrMarker) == list or type(positionOrMarker) == np.ndarray:
        p0list = list(positionOrMarker)
    elif type(positionOrMarker)==exudyn.MarkerIndex:
        markerNumber = float(int(positionOrMarker))
        try:
            p0list = mbs.GetMarkerOutput(markerNumber=positionOrMarker,variableType=exu.OutputVariableType.Position, configuration=exu.ConfigurationType.Reference)
            p0list = list(p0list)
        except:
            p0list = [0,0,0] #this was just a trial, otherwise initialize with zeros (e.g. for special objects where this does not work)
    else:
        raise ValueError('CreateDistanceSensor: positionOrMarker must be either MarkerIndex or 3D position as list or numpy.array')

    graphicsObject = -1 #signals that there is no graphics object
    sign = 1.
    if drawDisplaced:
        sign = -1.
    if addGraphicsObject:
        if cylinderRadius == 0:
            gData = exudyn.graphics.Lines([[0,0,0],list(sign*np.array(dirSensor))], color = color)
        else: 
            gData = exudyn.graphics.Cylinder([0,0,0],sign*np.array(dirSensor), radius=cylinderRadius, color = color)
            
        graphicsObject=mbs.AddObject(ObjectGround(referencePosition= p0list,
                                      visualization=VObjectGround(graphicsData=[gData])))

    flags = int(drawDisplaced)
    dataUF = [float(generalContactIndex)]
    dataUF += [markerNumber] + p0list + list(dirSensor)
    dataUF += [ minDistance, maxDistance, cylinderRadius, float(int(selectedTypeIndex)), float(measureVelocity), float(int(graphicsObject)), float(flags)] 


    sUF = mbs.AddSensor(SensorUserFunction(sensorNumbers=[], factors=dataUF,
                                              storeInternal=True,
                                              sensorUserFunction=__UFsensorDistance))

    return sUF


#**function: DEPRECATED: Internal SensorUserFunction, used in function AddSensorRecorder
#**notes: Warning: this method is DEPRECATED, use storeInternal in Sensors, which is much more performant; Note, that a sensor usually just passes through values of an existing sensor, while recording the values to a numpy array row-wise (time in first column, data in remaining columns)
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

#**function: DEPRECATED: Add a SensorUserFunction object in order to record sensor output internally; this avoids creation of files for sensors, which can speedup and simplify evaluation in ParameterVariation and GeneticOptimization; values are stored internally in mbs.variables['sensorRecord'+str(sensorNumber)] where sensorNumber is the mbs sensor number
#**input: 
#  mbs: mbs containing object
#  sensorNumber: integer sensor number to be recorded
#  endTime: end time of simulation, as given in simulationSettings.timeIntegration.endTime 
#  sensorsWritePeriod: as given in simulationSettings.solutionSettings.sensorsWritePeriod
#  sensorOutputSize: size of sensor data: 3 for Displacement, Position, etc. sensors; may be larger for RotationMatrix or Coordinates sensors; check this size by calling mbs.GetSensorValues(sensorNumber)
#**output: adds an according SensorUserFunction sensor to mbs; returns new sensor number; during initialization a new numpy array is allocated in  mbs.variables['sensorRecord'+str(sensorNumber)] and the information is written row-wise: [time, sensorValue1, sensorValue2, ...]
#**notes: Warning: this method is DEPRECATED, use storeInternal in Sensors, which is much more performant; Note, that a sensor usually just passes through values of an existing sensor, while recording the values to a numpy array row-wise (time in first column, data in remaining columns)
def AddSensorRecorder(mbs, sensorNumber, endTime, sensorsWritePeriod, sensorOutputSize=3):
    print('WARNING: AddSensorRecorder is DEPRECATED, use sensors and set storeInternal=True to achieve similar functionality')
    nSteps = int(endTime/sensorsWritePeriod)
    mbs.variables['sensorRecord'+str(sensorNumber)] = np.zeros((nSteps+1,1+sensorOutputSize)) #time+3 sensor values

    sUserRecord = mbs.AddSensor(SensorUserFunction(sensorNumbers=[sensorNumber], 
                                                   factors=[sensorsWritePeriod],
                                                   writeToFile=False,
                                                   sensorUserFunction=UFsensorRecord))
    
    return sUserRecord    
    
    
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
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
#  hasHeader: set to False, if file is expected to have no header; if False, then some error checks related to file header are not performed
#**output: dictionary with 'data': the matrix of stored solution vectors, 'columnsExported': a list with integer values showing the exported sizes [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData], 'nColumns': the number of data columns and 'nRows': the number of data rows
def LoadSolutionFile(fileName, safeMode=False, maxRows=-1, verbose=True, hasHeader=True):

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
        
        if verbose: print('text file loaded ... converting ...')
        
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
        
        if verbose: print('found',cntComments,'lines with comments, which are ignored')
            
        if maxRows != -1 and cntDataRows > maxRows:
            cntDataRows = maxRows
        
        if cntDataRows == 0 or dataRowStart == -1:
            raise ValueError('LoadSolutionFile: no rows found')
        else:
            if verbose: print('data starts at ',dataRowStart, ', found ', cntDataRows, ' rows', sep='')

        if verbose: print('check columns ...')
        
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
            if verbose: print('LoadSolution: WARNING number of columns in last data row is inconsistent; will be skipped')
            skipLast = 1
        
        if verbose: print('file contains ',cntDataRows, ' rows and ', cols, ' columns (incl. time)',sep='')
        
        data = np.zeros((cntDataRows, nColumns+1))
        
        progress = 0
        progressInfo = 5000000
        skipLine = 0 #counter for skipping additional lines with comments
        for i in range(cntDataRows-skipLast):
            if verbose and progress>=progressInfo: #update progress
                print('import data row', i, '/', cntDataRows)
                progress = 0
            progress+=nColumns
            
            while i+dataRowStart+skipLine<len(lines) and lines[i+dataRowStart+skipLine][0]=='#':
                skipLine += 1
            
            ylist = lines[i+dataRowStart+skipLine].split(',')
            if len(ylist) == nColumns+1:
                y=np.array(ylist, dtype=float)
                data[i,:] = y[:]
            elif verbose:
                print('  data row', i, 'is inconsistent:',len(ylist), nColumns+1,' ... skipped')

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
            print('  indexSize=',indexSize)
            print('  realSize=',realSize)
            print('  pointerSize=',pointerSize)
            print('  bigEndian=',bigEndian)

#         ExuFile::BinaryWrite(EXUstd::exudynVersion, solFile, bfs);
        sVersion, fileEnd=BinaryReadString(file, intType)
        if verbose: print('  version=',sVersion)

#         ExuFile::BinaryWrite(STDstring("Mode0000"), solFile, bfs); //change this in future to add new features
        sMode, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: print('  mode=',sMode)

#             STDstring str = "Exudyn " + GetSolverName() + " ";
#             if (isStatic) { str+="static "; }
#             str+="solver solution file";
#             ExuFile::BinaryWrite(str, solFile, bfs);
        sSolver, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: print('  solver=',sSolver)

#             //solFile << "#simulation started=" << EXUstd::GetDateTimeString() << "\n";
#             ExuFile::BinaryWrite(EXUstd::GetDateTimeString(), solFile, bfs);
        sTime, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: print('  data/time=',sTime)

#             //not needed in binary format:
#             //solFile << "#columns contain: time, ODE2 displacements";
#             //if (solutionSettings.exportVelocities) { solFile << ", ODE2 velocities"; }
#             //if (solutionSettings.exportAccelerations) { solFile << ", ODE2 accelerations"; }
#             //if (nODE1) { solFile << ", ODE1 coordinates"; } //currently not available, but for future solFile structure necessary!
#             //if (nVel1) { solFile << ", ODE1 velocities"; }
#             //if (solutionSettings.exportAlgebraicCoordinates) { solFile << ", AE coordinates"; }
#             //if (solutionSettings.exportDataCoordinates) { solFile << ", ODE2 velocities"; }
#             //solFile << "\n";

#             //solFile << "#number of system coordinates [nODE2, nODE1, nAlgebraic, nData] = [" <<
#             //    nODE2 << "," << nODE1 << "," << nAE << "," << nData << "]\n"; //this will allow to know the system information, independently of coordinates written
#             ArrayIndex sysCoords({nODE2, nODE1, nAE, nData});
#             ExuFile::BinaryWrite(sysCoords, solFile, bfs);
        systemSizes, fileEnd=BinaryReadArrayIndex(file, intType)
        if verbose: print('  systemSizes=',systemSizes)


#             //solFile << "#number of written coordinates [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData] = [" << //these are the exported coordinates line-by-line
#             //    nODE2 << "," << nVel2 << "," << nAcc2 << "," << nODE1 << "," << nVel1 << "," << nAEexported << "," << nDataExported << "]\n"; //python convert line with v=eval(line.split('=')[1])
#             ArrayIndex writtenCoords({ nODE2, nVel2, nAcc2, nODE1, nVel1, nAEexported, nDataExported });
#             ExuFile::BinaryWrite(writtenCoords, solFile, bfs);
        columnsExported, fileEnd=BinaryReadArrayIndex(file, intType)
        if verbose: print('  columnsExported=',columnsExported)
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
#             ExuFile::BinaryWrite(solutionSettings.solutionInformation, solFile, bfs);
        solutionInformation, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: print('  solutionInformation="'+solutionInformation+'"')

#             //add some checksum ...
#             ExuFile::BinaryWrite(STDstring("EndOfHeader"), solFile, bfs);
#             //next byte starts with solution
        EndOfHeader, fileEnd=BinaryReadString(file, intType)
        if int(verbose)>1: print('  EndOfHeader found: ',EndOfHeader)
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
#**function: This function is not further maintaned and should only be used if you do not have tkinter (like on some MacOS versions); use exudyn.interactive.SolutionViewer() instead! AnimateSolution consecutively load the rows of a solution file and visualize the result
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
#**function: helper function which draws system graph of a MainSystem (mbs); several options let adjust the appearance of the graph; the graph visualization uses randomizer, which results in different graphs after every run!
#**input:
#   mbs: MainSystem to be operated with
#   showLoads: toggle appearance of loads in mbs
#   showSensors: toggle appearance of sensors in mbs
#   useItemNames: if True, object names are shown instead of basic object types (Node, Load, ...)
#   useItemTypes: if True, object type names (MassPoint, JointRevolute, ...) are shown instead of basic object types (Node, Load, ...); Note that Node, Object, is omitted at the beginning of itemName (as compared to theDoc.pdf); item classes become clear from the legend
#   addItemTypeNames: if True, type nymes (Node, Load, etc.) are added
#   multiLine: if True, labels are multiline, improving readability
#   fontSizeFactor: use this factor to scale fonts, allowing to fit larger graphs on the screen with values < 1
#   showLegend: shows legend for different item types
#   layoutDistanceFactor: this factor influences the arrangement of labels; larger distance values lead to circle-like results
#   layoutIterations: more iterations lead to better arrangement of the layout, but need more time for larger systems (use 1000-10000 to get good results)
#   tightLayout: if True, uses matplotlib plt.tight\_layout() which may raise warning
#**output: [Any, Any, Any]; returns [networkx, G, items] with nx being networkx, G the graph and item what is returned by nx.draw\_networkx\_labels(...)
#**belongsTo: MainSystem
def DrawSystemGraph(mbs, showLoads=True, showSensors=True, useItemNames = False, 
                    useItemTypes = False, addItemTypeNames=True, multiLine=True, fontSizeFactor=1., 
                    layoutDistanceFactor=3., layoutIterations=100, showLegend = True, tightLayout = True):
    
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
    
    #showLegend = False
    #addItemTypeNames = False #Object, Node, ... not added but legend added
    # if useItemTypes or useItemNames:
    #     showLegend = True

    sLineBreak = ''
    if multiLine:
        sLineBreak = '-\n'

    #+++++++++++++++++++++++++++++++++++++++++++++++++++++
    itemType = 'Node'
    n = mbs.systemData.NumberOfNodes()
    for i in range(n):
        item = mbs.GetNode(i)
        itemName=itemType+str(i)
    
        nodeName = 'Node'
        if item['nodeType'].find('Ground') != -1:
            nodeName = nodeName + 'Ground'
    
        if useItemNames:
            itemName=item['name'] #+str(i)
        elif useItemTypes:
            itemName=item['nodeType']+str(i)
            if addItemTypeNames:
                itemName = nodeName + sLineBreak + itemName

            if sLineBreak != '':
                itemName=itemName.replace('Node'+sLineBreak+'Rigid','NodeRigid'+sLineBreak)
                itemName=itemName.replace('Node'+sLineBreak+'Generic','NodeGeneric'+sLineBreak)
    
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
            itemName=item['name']#+str(i)
        elif useItemTypes:
            itemName=item['markerType']+str(i)
            if addItemTypeNames:
                itemName = 'Marker' + sLineBreak + itemName

            if sLineBreak != '':
                itemName=itemName.replace('Marker'+sLineBreak+'Body','MarkerBody'+sLineBreak)
                itemName=itemName.replace('Marker'+sLineBreak+'Object','MarkerObject'+sLineBreak)
                itemName=itemName.replace('Marker'+sLineBreak+'Node','MarkerNode'+sLineBreak)
                itemName=itemName.replace('Marker'+sLineBreak+'SuperElement','MarkerSuperElement'+sLineBreak)
                itemName=itemName.replace('Marker'+sLineBreak+'Kinematic','MarkerKinematic'+sLineBreak)
    
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
            itemName=item['name']#+str(i)
        elif useItemTypes:
            itemName=item['objectType']+str(i)
            if addItemTypeNames:
                itemName = 'Object' + sLineBreak + itemName
            
            if sLineBreak != '':
                itemName=itemName.replace('Object'+sLineBreak+'Joint','ObjectJoint'+sLineBreak)
                itemName=itemName.replace('Object'+sLineBreak+'Mass','ObjectMass'+sLineBreak)
                itemName=itemName.replace('Object'+sLineBreak+'Beam','ObjectBeam'+sLineBreak)
                itemName=itemName.replace('Object'+sLineBreak+'ANCF','ObjectANCF'+sLineBreak)
                itemName=itemName.replace('Object'+sLineBreak+'Contact','ObjectContact'+sLineBreak)
                itemName=itemName.replace('Object'+sLineBreak+'Connector','ObjectConnector'+sLineBreak)
                itemName=itemName.replace('Object'+sLineBreak+'Rigid','ObjectRigid'+sLineBreak)
                itemName=itemName.replace('Object'+sLineBreak+'FFRFr','ObjectFFRF'+sLineBreak+'r')
            
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
                itemName=item['name']#+str(i)
            elif useItemTypes:
                itemName=item['loadType']+str(i)
                if addItemTypeNames:
                    itemName = 'Load' + sLineBreak + itemName

                if sLineBreak != '':
                    itemName=itemName.replace('Load'+sLineBreak+'Mass','LoadMass'+sLineBreak)

        
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
                itemName=item['name']#+str(i)
            elif useItemTypes:
                itemName=item['sensorType']+str(i)
                if addItemTypeNames:
                    itemName = 'Sensor' + sLineBreak + itemName
        
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
        
        fontSizeLegend = 10
        if fontSizeFactor > 1: #do not make font size smaller!
            fontSizeLegend *= fontSizeFactor
        plt.legend(fontsize=fontSizeLegend)

    
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
    
    pos = nx.drawing.spring_layout(G, scale=0.5, k=layoutDistanceFactor*1/sqrt(G.size()), 
                                   threshold = 1e-5, iterations = layoutIterations)
    nx.draw_networkx_nodes(G, pos, node_size=1)
    nx.draw_networkx_edges(G, pos, edge_color=edgeColorMap, width=edgeWidths)#width=2)
    
    #reproduce what draw_networkx_labels does, allowing different colors for nodes
    #check: https://networkx.github.io/documentation/stable/_modules/networkx/drawing/nx_pylab.html
    items = nx.draw_networkx_labels(G, pos, font_size=10*fontSizeFactor, clip_on=False, #clip at plot boundary
                                    bbox=dict(facecolor='skyblue', edgecolor='black', 
                                              boxstyle='round,pad=0.1', lw=10*fontSizeFactor)) #lw is border size (no effect?)

    
    #now assign correct colors:
    for i in range(len(itemNames)):
        currentColor = itemColorMap[i]
        itemType = itemTypes[i]
        boxStyle = 'round,pad=0.2'
        fontSize = 10*fontSizeFactor
        if itemType == 'Object':
            boxStyle = 'round,pad=0.2'
            fontSize = 12*fontSizeFactor
        if itemType == 'Node':  
            boxStyle = 'square,pad=0.1'
            fontSize = 10*fontSizeFactor
        if itemType == 'Marker':  
            boxStyle = 'square,pad=0.1'
            fontSize = 8*fontSizeFactor
    
        #print("color=",currentColor)
        items[itemNames[i]].set_bbox(dict(facecolor=currentColor,  
              edgecolor=currentColor, boxstyle=boxStyle))
        items[itemNames[i]].set_fontsize(fontSize)
    
    plt.axis('off') #do not show frame, because usually some nodes are very close to frame ...
    if tightLayout:
        plt.tight_layout()
    plt.margins(x=0.1*fontSizeFactor, y=0.1*fontSizeFactor) #larger margin, to avoid clipping of long texts
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
    


