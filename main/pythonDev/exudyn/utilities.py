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

#**function: get index from value in given data vector (numpy array); usually used to get specific index of time vector
#**input: 
#  data: containing (almost) equidistant values of time
#  value: e.g., time to be found in data
#  tolerance: tolerance, which is accepted (default: tolerance=1e-7)
#**output: index
def IndexFromValue(data, value, tolerance=1e-7):
    bestTol = 1e37
    index  = -1
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
def ShowOnlyObjects(mbs, objectNumbers=[]):
    if not isinstance(objectNumbers,list):
        listObjects = [objectNumbers]
    else:
        listObjects = objectNumbers
    isEmpty = len(listObjects) == 0
    
    for objectIndex in range(mbs.systemData.NumberOfObjects()):
        oDict = mbs.GetObject(objectIndex)
        flag = showOthers
        if objectIndex in listObjects or isEmpty: #if no objects to show,  
            flag = True
        if 'Vshow' in oDict:
            mbs.SetObjectParameter(objectIndex,'Vshow', flag)
    mbs.SendRedrawSignal()

#**function to highlight a certain item with number itemNumber; set itemNumber to -1 to show again all objects
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


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++   LOAD SOLUTION AND ANIMATION   ++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#++++++++++++++++++++++++++++++++++++++++++++
#**function: read coordinates solution file (exported during static or dynamic simulation with option exu.SimulationSettings().solutionSettings.coordinatesSolutionFileName='...') into dictionary:
#**input: fileName: string containing directory and filename of stored coordinatesSolutionFile
#**output: dictionary with 'data': the matrix of stored solution vectors, 'columnsExported': a list with binary values showing the exported columns [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData],'nColumns': the number of data columns and 'nRows': the number of data rows
def LoadSolutionFile(fileName):
    data = np.loadtxt(fileName, comments='#', delimiter=',')

    fileRead=open(fileName,'r') 
    fileLines = fileRead.readlines()
    fileRead.close()
    leftStr=fileLines[4].split('=')[0]
    if leftStr[0:30] != '#number of written coordinates': 
        print('ERROR in LoadSolution: file header corrupted')

    columnsExported = eval(fileLines[4].split('=')[1]) #load according column information into vector: [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData]
    print('columns imported =', columnsExported)
    nColumns = sum(columnsExported)
    print('total columns to be imported =', nColumns, ', array size of file =', np.size(data,1))

    if (nColumns + 1) != np.size(data,1): #one additional column for time!
        print('ERROR in LoadSolution: number of columns is inconsistent')

    nRows = np.size(data,0)

    return dict({'data': data, 'columnsExported': columnsExported,'nColumns': nColumns,'nRows': nRows})
    
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

    
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: generate cable elements along straight line with certain discretization
#**input:
#  mbs: the system where ANCF cables are added
#  positionOfNode0: 3D position (list or np.array) for starting point of line
#  positionOfNode1: 3D position (list or np.array) for end point of line
#  numberOfElements: for discretization of line
#  cableTemplate: a ObjectANCFCable2D object, containing the desired cable properties; cable length and node numbers are set automatically
#  massProportionalLoad: a 3D list or np.array, containing the gravity vector or zero
#  fixedConstraintsNode0: a list of 4 binary values, indicating the coordinate contraints on the first node (x,y-position and x,y-slope)
#  fixedConstraintsNode1: a list of 4 binary values, indicating the coordinate contraints on the last node (x,y-position and x,y-slope)
#  vALE: used for ObjectALEANCFCable2D objects
#**output: returns a list [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]
#**example: see Examples/ANCF_cantilever_test.py
def GenerateStraightLineANCFCable2D(mbs, positionOfNode0, positionOfNode1, numberOfElements, cableTemplate,
                                massProportionalLoad=[0,0,0], fixedConstraintsNode0=[0,0,0,0], fixedConstraintsNode1=[0,0,0,0],
                                vALE=0, ConstrainAleCoordinate=True):
    

    cableNodeList=[]
    cableNodePositionList=[positionOfNode0]
    cableObjectList=[]
    loadList=[]
    cableCoordinateConstraintList=[]

    # length of one element, calculated from first and last node position:
    cableLength = NormL2(VSub(positionOfNode1, positionOfNode0))/numberOfElements
    
    # slope of elements in reference position, calculated from first and last node position:
    cableSlopeVec = Normalize(VSub(positionOfNode1, positionOfNode0))
   
    # add first ANCF node (straight reference configuration):
    nCable0 = mbs.AddNode(Point2DS1(referenceCoordinates=[positionOfNode0[0],positionOfNode0[1],cableSlopeVec[0],cableSlopeVec[1]])) 
    cableNodeList+=[nCable0]
    
    
    cableTemplate.physicsLength = cableLength
    
#    #ALE node: 
#    if vALE !=0:
#        nALE = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=1, referenceCoordinates=[0], initialCoordinates=[0], initialCoordinates_t=[vALE]))
#        mALE = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nALE, coordinate=0)) #ALE velocity  marker

    # add all other ANCF nodes (straight reference configuration) and attach Gravity marker to them:
    for i in range(numberOfElements): 
        
        positionOfCurrentNode=[positionOfNode0[0]+cableLength*cableSlopeVec[0]*(i+1),positionOfNode0[1]+cableLength*cableSlopeVec[1]*(i+1)]
        cableNodePositionList+=[positionOfCurrentNode]
        
        nCableLast = mbs.AddNode(Point2DS1(referenceCoordinates=[positionOfCurrentNode[0],positionOfCurrentNode[1],cableSlopeVec[0],cableSlopeVec[1]]))
        cableNodeList+=[nCableLast]
        

#        if vALE ==0:
#            cableTemplate.nodeNumbers=[cableNodeList[i],cableNodeList[i+1]]
#        else:
        cableTemplate.nodeNumbers[0:2]=[cableNodeList[i],cableNodeList[i+1]]
            
        oCable=mbs.AddObject(cableTemplate)
        cableObjectList+=[oCable]

        if NormL2(massProportionalLoad) != 0:
            mBodyMassLast = mbs.AddMarker(MarkerBodyMass(bodyNumber=oCable))
            lLoadLast=mbs.AddLoad(Gravity(markerNumber=mBodyMassLast,loadVector=massProportionalLoad))
            loadList+=[lLoadLast]
        
    
    if (NormL2(fixedConstraintsNode0+fixedConstraintsNode1)) != 0:
        # ground "node" at 0,0,0:
        nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) 
        # add marker to ground "node": 
        mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0))
    

        for j in range(4):            
            if fixedConstraintsNode0[j] != 0:            
                #fix ANCF coordinates of first node
                mCableCoordinateConstraint0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nCable0, coordinate=j)) #add marker
                cBoundaryCondition=mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mCableCoordinateConstraint0])) #add constraint
                cableCoordinateConstraintList+=[cBoundaryCondition]
            
        for j in range(4):            
            if fixedConstraintsNode1[j] != 0:                 
                # fix right end position coordinates, i.e., add markers and constraints:
                mCableCoordinateConstraint1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nCableLast, coordinate=j))#add marker
                cBoundaryCondition=mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mCableCoordinateConstraint1])) #add constraint  
                cableCoordinateConstraintList+=[cBoundaryCondition]
        
#        if vALE !=0 and ConstrainAleCoordinate:
#            cConstrainAle=mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mALE]))
            
    
    return [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]


#**function: generate a sliding joint from a list of cables, marker to a sliding body, etc.
def GenerateSlidingJoint(mbs,cableObjectList,markerBodyPositionOfSlidingBody,localMarkerIndexOfStartCable=0,slidingCoordinateStartPosition=0):

    cableMarkerList = []#list of Cable2DCoordinates markers
    offsetList = []     #list of offsets counted from first cable element; needed in sliding joint
    offset = 0          #first cable element has offset 0
    
    for item in cableObjectList: #create markers for cable elements
        m = mbs.AddMarker(MarkerBodyCable2DCoordinates(bodyNumber = item))
        cableMarkerList += [m]
        offsetList += [offset]  
        offset += mbs.GetObjectParameter(item,'physicsLength')
    
    nodeDataSlidingJoint = mbs.AddNode(NodeGenericData(initialCoordinates=[localMarkerIndexOfStartCable,slidingCoordinateStartPosition],numberOfDataCoordinates=2)) #initial index in cable list
    
    oSlidingJoint = mbs.AddObject(ObjectJointSliding2D(markerNumbers=[markerBodyPositionOfSlidingBody,cableMarkerList[localMarkerIndexOfStartCable]], 
                                                      slidingMarkerNumbers=cableMarkerList, 
                                                      slidingMarkerOffsets=offsetList, 
                                                      nodeNumber=nodeDataSlidingJoint))

    return [oSlidingJoint]





#**function: generate an ALE sliding joint from a list of cables, marker to a sliding body, etc.
def GenerateAleSlidingJoint(mbs,cableObjectList,markerBodyPositionOfSlidingBody,AleNode,
                            localMarkerIndexOfStartCable=0,AleSlidingOffset=0,
                            activeConnector=True, penaltyStiffness=0):

    cableMarkerList = []#list of Cable2DCoordinates markers
    offsetList = []     #list of offsets counted from first cable element; needed in sliding joint
    offset = 0          #first cable element has offset 0
    usePenalty = (penaltyStiffness!=0) #penaltyStiffness=0 ==> no penalty formulation!
    
    for item in cableObjectList: #create markers for cable elements
        m = mbs.AddMarker(MarkerBodyCable2DCoordinates(bodyNumber = item))
        cableMarkerList += [m]
        offsetList += [offset]  
        offset += mbs.GetObjectParameter(item,'physicsLength')
    
    nodeDataAleSlidingJoint = mbs.AddNode(NodeGenericData(initialCoordinates=[localMarkerIndexOfStartCable],numberOfDataCoordinates=1)) #initial index in cable list   
    oAleSlidingJoint = mbs.AddObject(ObjectJointALEMoving2D(markerNumbers=[markerBodyPositionOfSlidingBody,cableMarkerList[localMarkerIndexOfStartCable]], 
                                                      slidingMarkerNumbers=cableMarkerList, slidingMarkerOffsets=offsetList,
                                                      nodeNumbers=[nodeDataAleSlidingJoint, AleNode], slidingOffset=AleSlidingOffset,activeConnector=activeConnector,
                                                      usePenaltyFormulation = usePenalty, penaltyStiffness=penaltyStiffness))


    return [oAleSlidingJoint]

