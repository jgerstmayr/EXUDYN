# Utility functions and structures for Exudyn
"""
Created on Fri Jul 26 10:53:30 2019

@author: Johannes Gerstmayr

goal: support functions, which simplify the generation of models
"""
#constants and fixed structures:
import numpy as np #LoadSolutionFile
import time        #AnimateSolution
import copy as copy #to be able to copy e.g. lists

from exudyn.basicUtilities import *
from exudyn.rigidBodyUtilities import *
from exudyn.graphicsDataUtilities import *


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
    return np.sin(2*np.pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!

#**function: compute cos sweep at given time t
#**input: 
#  t: evaluate of sweep at time t
#  t1: end time of sweep frequency range
#  f0: start of frequency interval [f0,f1] in Hz
#  f1: end of frequency interval [f0,f1] in Hz
#**output: evaluation of cos sweep (in range -1..+1)
def SweepCos(t, t1, f0, f1):
    k = (f1-f0)/t1
    return np.cos(2*np.pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!

#**function: frequency according to given sweep functions SweepSin, SweepCos
#**input: 
#  t: evaluate of frequency at time t
#  t1: end time of sweep frequency range
#  f0: start of frequency interval [f0,f1] in Hz
#  f1: end of frequency interval [f0,f1] in Hz
#**output: frequency in Hz
def FrequencySweep(t, t1, f0, f1):
    return t*(f1-f0)/t1 + f0

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
#**function: load selected row of solution dictionary (previously loaded with LoadSolutionFile) into specific state
def SetSolutionState(exu, mbs, solution, row, configuration):
    if row < solution['nRows']:
        rowData = solution['data'][row]
        #cols = solution['columnsExported']
        [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData] = solution['columnsExported']

        #note that these visualization updates are not threading safe!
        mbs.systemData.SetODE2Coordinates(rowData[1:1+nODE2], configuration)
        if (nVel2): mbs.systemData.SetODE2Coordinates_t(rowData[1+nODE2:1+nODE2+nVel2], configuration)
        if (nAlgebraic): mbs.systemData.SetAECoordinates(rowData[1+nODE2+nVel2+nAcc2+nODE1+nVel1:1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic], configuration)
        if (nData): mbs.systemData.SetDataCoordinates(rowData[1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic:1+nODE2+nVel2+nAcc2+nODE1+nVel1+nAlgebraic+nData], configuration)

        if configuration == exu.ConfigurationType.Visualization:
            mbs.systemData.SetTime(rowData[0], exu.ConfigurationType.Visualization)
            mbs.SendRedrawSignal()
    else:
        print("ERROR in SetVisualizationState: invalid row (out of range)")

#++++++++++++++++++++++++++++++++++++++++++++
#**function: load selected row of solution dictionary into visualization state and redraw
#**input: 
#  exu: the exudyn library
#  mbs: the system, where the state is applied to
#  solution: solution dictionary previously loaded with LoadSolutionFile
#  row: the according row of the solution file which is visualized
#**output: renders the scene in mbs and changes the visualization state in mbs
def SetVisualizationState(exu, mbs, solution, row):
    SetSolutionState(exu, mbs, solution, row, exu.ConfigurationType.Visualization)

#++++++++++++++++++++++++++++++++++++++++++++
#**function: consecutively load the rows of a solution file and visualize the result
#**input: 
#  exu: the exudyn library
#  SC: the system container, where the mbs lives in
#  mbs: the system used for animation
#  solution: solution dictionary previously loaded with LoadSolutionFile; will be played from first to last row
#  rowIncrement: can be set larger than 1 in order to skip solution frames: e.g. rowIncrement=10 visualizes every 10th row (frame)
#  timeout: in seconds is used between frames in order to limit the speed of animation; e.g. use timeout=0.04 to achieve approximately 25 frames per second
#  createImages: creates consecutively images from the animation, which can be converted into an animation
#  runLoop: if True, the animation is played in a loop until 'q' is pressed in render window
#**output: renders the scene in mbs and changes the visualization state in mbs continuously
def AnimateSolution(exu, SC, mbs, solution, rowIncrement = 1, timeout=0.04, createImages = False, runLoop = False):
    nRows = solution['nRows']
    if (rowIncrement < 1) | (rowIncrement > nRows):
        print('ERROR in AnimateSolution: rowIncrement must be at least 1 and must not be larger than the number of rows in the solution file')
    oldUpdateInterval = SC.visualizationSettings.general.graphicsUpdateInterval
    SC.visualizationSettings.general.graphicsUpdateInterval = 0.5*min(timeout, 2e-3) #avoid too small values to run multithreading properly

    while runLoop and not mbs.GetRenderEngineStopFlag():
        for i in range(0,nRows,rowIncrement):
            if not(mbs.GetRenderEngineStopFlag()):
                SetVisualizationState(exu, mbs, solution, i)
                if createImages:
                    SC.RedrawAndSaveImage() #create images for animation
                time.sleep(timeout)

    SC.visualizationSettings.general.graphicsUpdateInterval = oldUpdateInterval #set values back to original









#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#                              SPECIAL FUNCTIONS FOR EXUDYN
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

    
from exudyn.itemInterface import *    
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

