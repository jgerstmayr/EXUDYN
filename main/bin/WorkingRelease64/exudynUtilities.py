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

from exudynBasicUtilities import *
from exudynRigidBodyUtilities import *
from exudynGraphicsDataUtilities import *


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#helper functions for matplotlib
def PlotLineCode(index):
    CC = ['r-','g-','b-','k-','c-','m-','y-','r:','g:','b:','k:','c:','m:','y:','r--','g--','b--','k--','c--','m--','y--','r-.','g-.','b-.','k-.','c-.','m-.','y-.']
    if index < len(CC):
        return CC[index]
    else:
        return 'k:' #black line

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++   LOAD SOLUTION AND ANIMATION   ++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#fill submatrix into given destinationMatrix at destination row 'destRow' and destination column 'destColumn'; all matrices must be numpy arrays 
def FillInSubMatrix(subMatrix, destinationMatrix, destRow, destColumn):
    nRows = subMatrix.shape[0]
    nColumns = subMatrix.shape[1]

    destinationMatrix[destRow:destRow+nRows, destColumn:destColumn+nColumns] = subMatrix

    #for i in range(nRows):
    #    for j in range(nColumns):
    #        destinationMatrix[i+destRow, j+destColumn] = subMatrix[i,j]


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++
#sweep in time interval [0,t1], frequency interval [f0,f1] Hz; use sin function
def SweepSin(t, t1, f0, f1):
    k = (f1-f0)/t1
    return np.sin(2*np.pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!

#sweep in time interval [0,t1], frequency interval [f0,f1] Hz; use cos function
def SweepCos(t, t1, f0, f1):
    k = (f1-f0)/t1
    return np.cos(2*np.pi*(f0+k*0.5*t)*t) #take care of factor 0.5 in k*0.5*t, in order to obtain correct frequencies!!!

#frequency sweep in time interval [0,t1], frequency interval [f0,f1] Hz
def FrequencySweep(t, t1, f0, f1):
    return t*(f1-f0)/t1 + f0

#++++++++++++++++++++++++++++++++++++++++++++
#read Solution file into dictionary:
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
#load selected row of solution into specific state
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
#load selected row of solution into visualization state
def SetVisualizationState(exu, mbs, solution, row):
    SetSolutionState(exu, mbs, solution, row, exu.ConfigurationType.Visualization)

#++++++++++++++++++++++++++++++++++++++++++++
#consecutively load the rows of a solution file and visualize the result
#timeout (in seconds) is used between frames in order to limit the speed of animation; e.g. use timeout=0.04 to achieve approximately 25 frames per second
#rowIncrement can be set larger than one in order to skip solution frames: e.g. rowIncrement=10 visualizes every 10th row (frame)
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

    
from itemInterface import *    
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#inputs:
# mbs: exudyn.MainSystem()
# positionOfNode0: 
#
# cableTemplate: e.g. ObjectANCFCable2D object with according properties; physicsLength and nodeNumbers will be filled automatically
#output:
#[...]
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

