#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Beam utility functions, e.g. for creation of sequences of straight or curved beams.
#
# Author:   Johannes Gerstmayr
# Date:     2022-01-30 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# Notes:    For a list of plot colors useful for matplotlib, see also utilities.PlotLineCode(...)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from math import sin, cos, pi, asin, atan2 #, sqrt, acos
import copy 
import numpy as np #for loading
import exudyn #for sensor index
import exudyn.itemInterface as eii


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: generate 2D ANCF cable elements along straight line given by two points; applies discretization (numberOfElements) and may apply gravity as well as nodal constraints
#**input:
#  mbs: the system where ANCF cables are added
#  positionOfNode0: 3D position (list or np.array) for starting point of line
#  positionOfNode1: 3D position (list or np.array) for end point of line
#  numberOfElements: for discretization of line
#  cableTemplate: a ObjectANCFCable2D object, containing the desired cable properties; cable length and node numbers are set automatically
#  massProportionalLoad: a 3D list or np.array, containing the gravity vector or zero
#  fixedConstraintsNode0: a list of 4 binary values, indicating the coordinate contraints on the first node (x,y-position and x,y-slope); use None in order to apply no constraints
#  fixedConstraintsNode1: a list of 4 binary values, indicating the coordinate contraints on the last node (x,y-position and x,y-slope); use None in order to apply no constraints
#  nodeNumber0: if set other than -1, this node number defines the node that shall be used at positionOfNode0
#  nodeNumber1: if set other than -1, this node number defines the node that shall be used at positionOfNode1
#**output: returns a list containing created items [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]
#**notes: use GenerateStraightBeam instead
#**example: 
# see Examples/ANCF_cantilever_test.py
def GenerateStraightLineANCFCable2D(mbs, positionOfNode0, positionOfNode1, numberOfElements, cableTemplate,
                                massProportionalLoad=[0,0,0], 
                                fixedConstraintsNode0=[0,0,0,0], fixedConstraintsNode1=[0,0,0,0],
                                nodeNumber0=-1, nodeNumber1=-1):

    return GenerateStraightBeam(mbs=mbs, positionOfNode0=positionOfNode0, positionOfNode1=positionOfNode1, 
                                numberOfElements=numberOfElements, beamTemplate=cableTemplate,
                                gravity=massProportionalLoad, 
                                fixedConstraintsNode0=fixedConstraintsNode0, fixedConstraintsNode1=fixedConstraintsNode1,
                                nodeNumber0=nodeNumber0, nodeNumber1=nodeNumber1)
    

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: generate 3D ANCF cable elements along straight line given by two points; applies discretization (numberOfElements) and may apply gravity as well as nodal constraints
#**input:
#  mbs: the system where ANCF cables are added
#  positionOfNode0: 3D position (list or np.array) for starting point of line
#  positionOfNode1: 3D position (list or np.array) for end point of line
#  numberOfElements: for discretization of line
#  cableTemplate: a ObjectANCFCable object, containing the desired cable properties; cable length and node numbers are set automatically
#  massProportionalLoad: a 3D list or np.array, containing the gravity vector or zero
#  fixedConstraintsNode0: a list of binary values, indicating the coordinate contraints on the first node (position and slope); 4 coordinates for 2D and 6 coordinates for 3D node; use None in order to apply no constraints
#  fixedConstraintsNode1: a list of binary values, indicating the coordinate contraints on the last node (position and slope); 4 coordinates for 2D and 6 coordinates for 3D node; use None in order to apply no constraints
#  nodeNumber0: if set other than -1, this node number defines the node that shall be used at positionOfNode0
#  nodeNumber1: if set other than -1, this node number defines the node that shall be used at positionOfNode1
#**output: returns a list containing created items [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]
#**example: 
# see Examples/ANCF_cantilever_test.py
def GenerateStraightLineANCFCable(mbs, positionOfNode0, positionOfNode1, numberOfElements, cableTemplate,
                                massProportionalLoad=[0,0,0], fixedConstraintsNode0=[0,0,0, 0,0,0], fixedConstraintsNode1=[0,0,0, 0,0,0],
                                nodeNumber0=-1, nodeNumber1=-1):
    return GenerateStraightBeam(mbs=mbs, positionOfNode0=positionOfNode0, positionOfNode1=positionOfNode1, 
                                numberOfElements=numberOfElements, beamTemplate=cableTemplate,
                                gravity=massProportionalLoad, 
                                fixedConstraintsNode0=fixedConstraintsNode0, fixedConstraintsNode1=fixedConstraintsNode1,
                                nodeNumber0=nodeNumber0, nodeNumber1=nodeNumber1)
    
    
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: generic function to create beam elements along straight line given by two points; applies discretization (numberOfElements) and may apply gravity as well as nodal constraints
#**input:
#  mbs: the system where beam elements are added
#  positionOfNode0: 3D position (list or np.array) for starting point of line
#  positionOfNode1: 3D position (list or np.array) for end point of line
#  numberOfElements: for discretization of line
#  beamTemplate: a Beam object (ObjectANCFCable2D, ObjectBeamGeometricallyExact2D, ObjectALEANCFCable2D, etc.), containing the desired beam type and properties; finite (beam) element length and node numbers are set automatically; for ALE element, the beamTemplate.nodeNumbers[2] must be set in the template and will not be overwritten
#  gravity: a 3D list or np.array, containing the gravity vector or zero
#  fixedConstraintsNode0: a list of binary values, indicating the coordinate contraints on the first node (position and slope); must agree with the number of coordinates in the node; use None to add no constraints
#  fixedConstraintsNode1: a list of binary values, indicating the coordinate contraints on the last node (position and slope); must agree with the number of coordinates in the node; use None to add no constraints
#  nodeNumber0: if set other than -1, this node number defines the node that shall be used at positionOfNode0
#  nodeNumber1: if set other than -1, this node number defines the node that shall be used at positionOfNode1
#**output: returns a list containing created items [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]
#**example: 
# import exudyn as exu
# from exudyn.utilities import * #includes exudyn.beams
# SC = exu.SystemContainer()
# mbs = SC.AddSystem()
# #example of flexible pendulum
# beamTemplate = ObjectBeamGeometricallyExact2D(physicsMassPerLength=0.02,
#                     physicsCrossSectionInertia=8e-9,
#                     physicsBendingStiffness=8e-4,
#                     physicsAxialStiffness=2000,
#                     physicsShearStiffness=650,
#                     visualization=VObjectBeamGeometricallyExact2D(drawHeight = 0.002))
# 
# #create straight beam with 10 elements, apply gravity and fix (x,y) position of node 0 (rotation left free)
# beamInfo = GenerateStraightBeam(mbs, positionOfNode0=[0,0,0], positionOfNode1=[0.5,0,0], 
#                                 numberOfElements=10, beamTemplate=beamTemplate,
#                                 gravity=[0,-9.81,0], fixedConstraintsNode0=[1,1,0],)
# #beamInfo contains nodes, beamObjects, loads, etc.
# #Assemble and solve
def GenerateStraightBeam(mbs, positionOfNode0, positionOfNode1, numberOfElements, beamTemplate,
                         gravity=[0,0,0], fixedConstraintsNode0=None, fixedConstraintsNode1=None,
                         nodeNumber0=-1, nodeNumber1=-1):
    
    beamNodeList=[]
    beamNodePositionList=[positionOfNode0]
    beamObjectList=[]
    loadList=[]
    beamCoordinateConstraintList=[]
    
    if len(positionOfNode0) != 3:
        exudyn.Print('WARNING: GenerateStraightBeam: positionOfNode0 should be a 3D vector')
    if len(positionOfNode1) != 3:
        exudyn.Print('WARNING: GenerateStraightBeam: positionOfNode1 should be a 3D vector')

    
    if '__class__' in beamTemplate.__dir__():
        className = str(beamTemplate.__class__.__name__)
        is2D = False
        isGeomExact = False
        if className == 'ObjectANCFCable2D' or className == 'ObjectALEANCFCable2D':
            NodeTemplate = eii.NodePoint2DSlope1
            nDOFnode = 4
            is2D = True
        elif className == 'ObjectANCFCable':
            NodeTemplate = eii.NodePointSlope1
            nDOFnode = 6
        elif className == 'ObjectBeamGeometricallyExact2D':
            NodeTemplate = eii.NodeRigidBody2D
            nDOFnode = 3
            is2D = True
            isGeomExact = True
        else:
            raise ValueError('GenerateStraightBeam: invalid beamTemplate "'+className+'" (maybe not implemented)')
            
        if is2D and (positionOfNode0[2] != 0 or positionOfNode1[2] != 0):
            ValueError('GenerateStraightBeam: positionOfNode0 and positionOfNode1 must have zero z-components for 2D beam elements')
        
        if not(fixedConstraintsNode0 is None) and len(fixedConstraintsNode0) != nDOFnode:
            ValueError('GenerateStraightBeam: fixedConstraintsNode0 incompatible has incompatible size')
        if not(fixedConstraintsNode1 is None) and len(fixedConstraintsNode1) != nDOFnode:
            ValueError('GenerateStraightBeam: fixedConstraintsNode1 incompatible has incompatible size')
    else:
        raise ValueError('GenerateStraightBeam: beamTemplate may is invalid')

    #helper function to convert position into list for 2D or 3D
    def ConvertVector(pos, is2D):
        if is2D: return list(pos[0:2])
        else: return list(pos)

    # length of one element, calculated from first and last node position:
    vDiff = np.array(positionOfNode1)-np.array(positionOfNode0)
    lDiff = np.linalg.norm(vDiff)
    if (numberOfElements <= 0 or numberOfElements != int(numberOfElements)): 
        raise ValueError('GenerateStraightBeam: number of elements must be integer, non-zero and positive')

    beamLength = lDiff/numberOfElements
    
    # slope of elements in reference position, calculated from first and last node position:
    if (lDiff == 0.): 
        raise ValueError('GenerateStraightBeam: distance between positionOfNode0 and positionOfNode1 is zero; terminating')
    beamSlopeVec = (1./lDiff)*vDiff
    
    if isGeomExact:
        if not is2D: 
            raise ValueError('GenerateStraightBeam: only works for 2D geometrically exact beam for now!')
        beamRotations = [atan2(beamSlopeVec[1], beamSlopeVec[0])]
    else:
        beamRotations = ConvertVector(beamSlopeVec, is2D)
   
    # add first ANCF node (straight reference configuration):
    if (nodeNumber0!=-1):
        beamNodeList+=[nodeNumber0]
        nBeam0=nodeNumber0
    else:
        nBeam0 = mbs.AddNode(NodeTemplate(referenceCoordinates=ConvertVector(positionOfNode0, is2D) + beamRotations)) 
        beamNodeList+=[nBeam0]
    
    beamTemplate.physicsLength = beamLength
    
    # add all other ANCF nodes (straight reference configuration) and attach Gravity marker to them:
    for i in range(numberOfElements): 
        
        positionOfCurrentNode = (np.array(positionOfNode0)+beamLength*beamSlopeVec*(i+1)).tolist()
        # positionOfCurrentNode=[positionOfNode0[0]+beamLength*beamSlopeVec[0]*(i+1),
        #                        positionOfNode0[1]+beamLength*beamSlopeVec[1]*(i+1), 
        #                        positionOfNode0[2]+beamLength*beamSlopeVec[2]*(i+1), 
        #                        ]
        #if is2D: positionOfCurrentNode[2] = 0
        
        beamNodePositionList+=[positionOfCurrentNode]
        
        if (i==numberOfElements-1 and nodeNumber1!=-1):
            nBeamLast = nodeNumber1
        else:
            nBeamLast = mbs.AddNode(NodeTemplate(referenceCoordinates=ConvertVector(positionOfCurrentNode, is2D) + beamRotations)) 
            # if is2D:
            #     nBeamLast = mbs.AddNode(eii.NodePoint2DSlope1(referenceCoordinates=[positionOfCurrentNode[0],positionOfCurrentNode[1],beamSlopeVec[0],beamSlopeVec[1]]))
            # else:
            #     nBeamLast = mbs.AddNode(eii.NodePointSlope1(referenceCoordinates=list(positionOfCurrentNode) + list(beamSlopeVec) ))
        
        beamNodeList+=[nBeamLast]
        
        #do not override all node numbers, as nodeNumbers[2] may be ALE node
        beamTemplate.nodeNumbers[0:2]=[beamNodeList[i],beamNodeList[i+1]]
            
        oBeam=mbs.AddObject(beamTemplate)
        beamObjectList+=[oBeam]

        if np.linalg.norm(gravity) != 0:
            mBodyMassLast = mbs.AddMarker(eii.MarkerBodyMass(bodyNumber=oBeam))
            lLoadLast=mbs.AddLoad(eii.Gravity(markerNumber=mBodyMassLast,loadVector=gravity))
            loadList+=[lLoadLast]
        
    
    if not(fixedConstraintsNode0 is None):
        # ground "node" at first node:
        nGround = mbs.AddNode(eii.NodePointGround(referenceCoordinates=positionOfNode0)) 
        # add marker to ground "node": 
        mGround = mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0))

        for j in range(len(fixedConstraintsNode0)):            
            if fixedConstraintsNode0[j] != 0:
                #fix ANCF coordinates of first node
                mBeamCoordinateConstraint0 = mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber = nBeam0, coordinate=j)) #add marker
                cBoundaryCondition=mbs.AddObject(eii.CoordinateConstraint(markerNumbers=[mGround,mBeamCoordinateConstraint0])) #add constraint
                beamCoordinateConstraintList+=[cBoundaryCondition]
            
    if not(fixedConstraintsNode1 is None):
        # ground "node" at first node:
        nGround = mbs.AddNode(eii.NodePointGround(referenceCoordinates=positionOfNode1)) 
        # add marker to ground "node": 
        mGround = mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0))

        for j in range(len(fixedConstraintsNode1)):            
            if fixedConstraintsNode1[j] != 0:
                # fix right end position coordinates, i.e., add markers and constraints:
                mBeamCoordinateConstraint1 = mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber = nBeamLast, coordinate=j))#add marker
                cBoundaryCondition=mbs.AddObject(eii.CoordinateConstraint(markerNumbers=[mGround,mBeamCoordinateConstraint1])) #add constraint  
                beamCoordinateConstraintList+=[cBoundaryCondition]
            
    
    return [beamNodeList, beamObjectList, loadList, beamNodePositionList, beamCoordinateConstraintList]


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: generate cable elements along circular arc with given start point, radius, start angle (measured relative to $x$-axis, in positive rotation sense) and angle of arc
#**input:
#  mbs: the system where ANCF cables are added
#  positionOfNode0: 3D position (list or np.array) for starting point of line
#  radius: radius of arc
#  startAngle: start angle of arc in radians  ($0 \ldots 2 \pi$), defines the direction of the slope vector, measured relative to $x$-axis, in positive rotation sense
#  arcAngle: total angle of arc in radians ($0 \ldots 2 \pi$), measured in positive rotation sense (negative angle reverts curvature and center point of circle)
#  numberOfElements: for discretization of arc
#  cableTemplate: a ObjectANCFCable2D object, containing the desired cable properties; cable length and node numbers are set automatically
#  massProportionalLoad: a 3D list or np.array, containing the gravity vector or zero
#  fixedConstraintsNode0: a list of 4 binary values, indicating the coordinate contraints on the first node (x,y-position and x,y-slope)
#  fixedConstraintsNode1: a list of 4 binary values, indicating the coordinate contraints on the last node (x,y-position and x,y-slope)
#  nodeNumber0: if set other than -1, this node number defines the node that shall be used at positionOfNode0
#  nodeNumber1: if set other than -1, this node number defines the node that shall be used at positionOfNode1
#  setCurvedReferenceConfiguration: if True, the curvature +/-(1/radius) is set as a reference configuration (sign depends on arcAngle); if False, the reference configuration is straight
#  verboseMode: if True, prints out information on created nodes
#**output: returns a list [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]
def GenerateCircularArcANCFCable2D(mbs, positionOfNode0, radius, startAngle, arcAngle, 
                                   numberOfElements, cableTemplate,
                                   massProportionalLoad=[0,0,0], fixedConstraintsNode0=[0,0,0,0], fixedConstraintsNode1=[0,0,0,0],
                                   nodeNumber0=-1, nodeNumber1=-1, setCurvedReferenceConfiguration=True, verboseMode=False):
    

    cableNodeList=[]
    cableNodePositionList=[positionOfNode0]
    cableObjectList=[]
    loadList=[]
    cableCoordinateConstraintList=[]
    if len(positionOfNode0) != 3:
        exudyn.Print('WARNING: GenerateCircularArcANCFCable2D: positionOfNode0 should be a 3D vector')

    if (numberOfElements <= 0 or numberOfElements != int(numberOfElements)): 
        raise ValueError('GenerateCircularArcANCFCable2D: number of elements must be integer, non-zero and positive')

    if (radius <= 0.): 
        raise ValueError('GenerateCircularArcANCFCable2D: radius must be > 0')

    if (arcAngle == 0.): 
        raise ValueError('GenerateCircularArcANCFCable2D: arcAngle must be non-zero')

    #length of one element:
    cableLength = radius*abs(arcAngle)/numberOfElements
    # exudyn.Print('cableLength=',cableLength)
    # exudyn.Print('p0=', positionOfNode0)
    
    #reverse normal and slope, means changing the curvature and center point of arc
    arcSign = 1.
    if arcAngle < 0.:
        arcSign = -1.

    # slope vector of first node
    cableSlopeVec = np.array([ np.cos(startAngle),np.sin(startAngle),0.])
    cableNormal =   np.array([-np.sin(startAngle),np.cos(startAngle),0.])
    
    #center point of circle for creation of arc:
    pCenter = arcSign*radius*cableNormal + np.array(positionOfNode0)
    # exudyn.Print('pCenter=',pCenter)

    # add first ANCF node (straight reference configuration):
    if (nodeNumber0!=-1):
        cableNodeList+=[nodeNumber0]
    else:
        nCable0 = mbs.AddNode(eii.Point2DS1(referenceCoordinates=[positionOfNode0[0],positionOfNode0[1],cableSlopeVec[0],cableSlopeVec[1]])) 
        cableNodeList+=[nCable0]
        if verboseMode:
            exudyn.Print('  node 0: pos=', positionOfNode0[0],',',positionOfNode0[1],', slope=',cableSlopeVec[0],',',cableSlopeVec[1])
    
    cableTemplate.physicsLength = cableLength

    oldReferenceCurvature = cableTemplate.physicsReferenceCurvature 
    if setCurvedReferenceConfiguration:
        cableTemplate.physicsReferenceCurvature = arcSign/radius #set reference curvature, such that beam is pre-curved
    
    # add all other ANCF nodes (straight reference configuration) and attach Gravity marker to them:
    for i in range(numberOfElements): 
        phi = (i+1)*arcAngle/float(numberOfElements)
        #new node:
        pArc = pCenter + arcSign*radius*np.sin(phi)*cableSlopeVec - arcSign*radius*np.cos(phi)*cableNormal
        #new slope:
        vArc = np.cos(phi)*cableSlopeVec + np.sin(phi)*cableNormal
        # exudyn.Print('pArc'+str(i)+'=',pArc)
        # exudyn.Print('vArc'+str(i)+'=',vArc)
        
        positionOfCurrentNode=[pArc[0],pArc[1],0.]
        cableNodePositionList+=[positionOfCurrentNode]

        if verboseMode:
            exudyn.Print('  node '+str(i+1)+': pos=', positionOfCurrentNode[0],',',positionOfCurrentNode[1],', slope=',vArc[0],',',vArc[1])
        
        if (i==numberOfElements-1 and nodeNumber1!=-1):
            nCableLast = nodeNumber1
        else:
            nCableLast = mbs.AddNode(eii.Point2DS1(referenceCoordinates=[positionOfCurrentNode[0],positionOfCurrentNode[1],
                                                                     vArc[0],vArc[1]]))

        # exudyn.Print('cableNodeList=',cableNodeList)
        
        cableNodeList+=[nCableLast]
        
        cableTemplate.nodeNumbers[0:2]=[cableNodeList[i],cableNodeList[i+1]]
            
        oCable=mbs.AddObject(cableTemplate)
        cableObjectList+=[oCable]

        if np.linalg.norm(massProportionalLoad) != 0:
            mBodyMassLast = mbs.AddMarker(eii.MarkerBodyMass(bodyNumber=oCable))
            lLoadLast=mbs.AddLoad(eii.Gravity(markerNumber=mBodyMassLast,loadVector=massProportionalLoad))
            loadList+=[lLoadLast]
        
    
    if (np.linalg.norm(list(fixedConstraintsNode0)+list(fixedConstraintsNode1)) ) != 0:
        # ground "node" at 0,0,0:
        nGround = mbs.AddNode(eii.NodePointGround(referenceCoordinates=[0,0,0])) 
        # add marker to ground "node": 
        mGround = mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0))
    

        for j in range(4):            
            if fixedConstraintsNode0[j] != 0:            
                #fix ANCF coordinates of first node
                mCableCoordinateConstraint0 = mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber = nCable0, coordinate=j)) #add marker
                cBoundaryCondition=mbs.AddObject(eii.CoordinateConstraint(markerNumbers=[mGround,mCableCoordinateConstraint0])) #add constraint
                cableCoordinateConstraintList+=[cBoundaryCondition]
            
        for j in range(4):            
            if fixedConstraintsNode1[j] != 0:                 
                # fix right end position coordinates, i.e., add markers and constraints:
                mCableCoordinateConstraint1 = mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber = nCableLast, coordinate=j))#add marker
                cBoundaryCondition=mbs.AddObject(eii.CoordinateConstraint(markerNumbers=[mGround,mCableCoordinateConstraint1])) #add constraint  
                cableCoordinateConstraintList+=[cBoundaryCondition]

     
    if setCurvedReferenceConfiguration:
        cableTemplate.physicsReferenceCurvature = oldReferenceCurvature        
    
    return [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]








#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: CreateReevingCurve for creating the geometry of a reeving system based on circles with radius and left/right side of passing the circles; left/right is seen in the direction passing from one to the next circle
#**input:
#  circleList: list containing center position, radius and 'L' (left) or 'R' (right) passing of circle
#  radialOffset: additional offset added to circles to account for half height of rope or beam
#  closedCurve: if True, the system adds circleList[0] and  circleList[1] at end of list and sets removeLastLine=True and removeFirstLine=False, in order to generate a closed curve according to given circles; furthermore, the number of nodes becomes equal to the number of elements in this case
#  drawingLinesPerCircle: number of lines in lineData per one revolution
#  numberOfANCFnodes: if not -1, function also generates nodes with equidistant distribution along curve!
#  graphicsElementsPerCircle: number of drawing lines generated in graphicsDataLines per circle revolution (larger generates better approximation of circles)
#  graphicsNodeSize: if not 0, addes graphics representation of nodes generated; for check if mesh is correct
#  removeFirstLine: removes first line generated, which may be unwanted
#  removeLastLine: removes last line generated, which may be unwanted
#  colorCircles: RGBA color for circles
#  colorLines: RGBA color for lines
#**output: return a dictionary with {'ancfPointsSlopes':ancfPointsSlopes, 'elementLengths':elementLengths, 'elementCurvatures':elementCurvatures, 'totalLength':totalLength, 'circleData':circle2D, 'graphicsDataLines':graphicsDataLines, 'graphicsDataCircles':graphicsDataCircles }; 'ancfPointsSlopes' denotes 4-dimensional vector with (x/y) position and (x/y) slope coordinates in a row; 'elementLengths' is the list of curved lengths for elements between nodes (size is 1 smaller than number of nodes), 'elementCurvatures' is the list of scalar curvatures between nodes (according to list of elementLengths), 'totalLength' is the total length of the reeving line, 'circleData' represents the lines and arcs calculated for the reeving system, 'graphicsDataLines' is the graphicsData for the lines and 'graphicsDataCircles' represents the graphicsData for the circles
#**example:
# #list with circle center, radius and side at which rope runs
# circleList = [[[0,0],0.2,'L'],
#               [[0,1],0.2,'L'],
#               [[0.8,0.8],0.4,'L'],
#               [[1,0],0.2,'L'],
#               [[0,0],0.2,'L'],
#               [[0,1],0.2,'L'],
#               ]
# [] = CreateReevingCurve(circleList, 
#                         removeLastLine=True, #allows closed curve
#                         numberOfANCFnodes=50)
def CreateReevingCurve(circleList, drawingLinesPerCircle = 64, numberOfANCFnodes=-1, 
                       removeLastLine=False, removeFirstLine=False,
                       radialOffset = 0., closedCurve=False,
                       graphicsElementsPerCircle=64, graphicsNodeSize=0,
                       colorCircles=[0.,0.5,1.,1.], colorLines=[1.,0.5,0.,1.]):

    nodesANCFcreated = numberOfANCFnodes
    minP=np.array([ 1e30, 1e30, 1e30])
    maxP=np.array([-1e30,-1e30,-1e30])
    
    circleListExt = copy.deepcopy(circleList)
    
    if closedCurve:
        circleListExt += [circleList[0]]
        circleListExt += [circleList[1]]
        removeLastLine=True 
        removeFirstLine=False
        if nodesANCFcreated!=-1:
            nodesANCFcreated += 1
    
    graphicsDataCircles = [] #optional graphics for circles
    for circle in circleListExt:
        #exudyn.Print(circle)
        pos = [circle[0][0],circle[0][1],0]
        r = circle[1]+radialOffset
        if circle[1] <= 0.:
            raise ValueError('CreateReevingCurve: radius in circleList may not be zero or negative!')

        graphicsDataCircles += [{'type':'Circle', 'color':colorCircles,
                  'position':pos,'radius':r}]
        minP = np.minimum(minP, np.array(pos)-[r,r,0])
        maxP = np.maximum(maxP, np.array(pos)+[r,r,0])
    
    #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #create list of lines and arcs:
    
    #list of dictionaries for lines and arcs; everything in 2D
    #lines: {'type':'LINE', 'point0':np.array([0,0]), 'point1':np.array([0,0])}
    #circularArcs: {'type':'ARC', 'point0':np.array([0,0]), 'center':np.array([0,0]), 'radius':1, 'startAngle':angle0, 'arcAngle':angle1}
    curve2D = []

    #start points from circle 0:
    circle = circleListExt[0]
    pos0 = np.array([circle[0][0],circle[0][1]])
    r0 = circle[1]+radialOffset
    side0=circle[2]
    p1Last = [] #point from last arc; empty if no arc to continue
    # pos0 = np.array([-1,-0.8])
    # r0 = 0.2
    # side0='L'
    # p1Last = [] #point from last arc; empty if no arc to continue


    totalLength = 0.
    
    for i, circle in enumerate(circleListExt):
        #exudyn.Print(circle)
        if i==0: #first circle is used already
            continue 
    
        pos1 = np.array([circle[0][0],circle[0][1]])
        r1 = circle[1]+radialOffset
        side1 = circle[2]
        if r0 < 0. or r1 < 0.: continue
    
        vv = pos1-pos0
        L = np.linalg.norm(vv)
        if L==0: 
            raise ValueError('CreateReevingCurve: distance of circles is zero')
        vv0 = (1./L)*vv
        nv0 = np.array([-vv0[1],vv0[0]])
    
        sign0 = 1
        sign1 = 1
        if side0 == 'R':
            sign0 = -1
        if side1 == 'R':
            sign1 = -1
        alpha = asin((sign1*r1 - sign0*r0)/L)
    
        p0 = pos0+sign0*nv0*cos(alpha)*r0+vv0*sin(-sign0*alpha)*r0
        p1 = pos1+sign1*nv0*cos(alpha)*r1+vv0*sin(-sign1*alpha)*r1
        
        # exudyn.Print('circle',i,', alpha=',alpha,', p0=',p0,', p1=',p1)
        # exudyn.Print('  vv0=',vv0,', nv0=', nv0, ', sign0=',sign0, ', sign1=',sign1)
        # exudyn.Print('  r0=',r0,', r1=', r1,', pos0=', pos0,', pos1=', pos1)
    
    
        #add arc from end of last line
        if list(p1Last) != []:
            vr0 = p1Last-pos0
            angle0=atan2(vr0[1],vr0[0])-sign0*0.5*pi
            vr1 = p0-pos0
            angle1=atan2(vr1[1],vr1[0])-sign0*0.5*pi
            arcAngle = (angle1-angle0)
            if arcAngle < 0 and side0=='R':
                arcAngle+=2.*pi
            if arcAngle > 0 and side0=='L':
                arcAngle-=2.*pi
            # exudyn.Print('aa=',arcAngle)
            curve2D+=[{'type':'ARC', 'point0':p1Last, 
                       'radius':r0, 'startAngle':angle0, 'endAngle':angle1, 'arcAngle':arcAngle}]
            totalLength += abs(r0*arcAngle)
    
        #only add this line, if not excluded!
        if not ((removeFirstLine and i==1) or
                (removeLastLine and i==len(circleListExt)-1)):
            segmentLength = np.linalg.norm(p1-p0)
            if segmentLength == 0.:
                print('WARNING: CreateReevingCurve: curve seems to be degenerated, check location of circles')
            curve2D += [{'type':'LINE', 'point0':p0, 'point1':p1}]
            totalLength += segmentLength
    
        #for next circle:
        p1Last = copy.deepcopy(p1)
        pos0 = copy.deepcopy(pos1)
        r0 = copy.deepcopy(r1)
        side0 = copy.deepcopy(side1)
    
    #create single line:
    lineData=[]
    
    if nodesANCFcreated > 0 and nodesANCFcreated < 2:
        raise ValueError('CreateReevingCurve: numberOfANCFnodes must be either -1 [deactivated] or > 2 !')

    numberOfElements = nodesANCFcreated-1
    lElem = totalLength/numberOfElements
    ancfPointsSlopes = [] #list of 4D vectors with nodal points+slopes
    elementLengths = [] #curved length for elements stored
    elementCurvatures = [] #curvature of element i, between nodes i and i+1
    
    arcCoord = 0 #remember how far already gone!
    nextArcCoord = 0 #this is the acrCoord with the next node
    tol = 1e-10 #make sure to add last node, which can have some round-off errors
    
    #now create nodes for ANCF elements:
    
    for item in curve2D:
        if item['type']=='LINE':
            p0 = item['point0']
            p1 = item['point1']
            lineData+=list(p0)+[0]
            lineData+=list(p1)+[0]
            L = np.linalg.norm(p1-p0)
            startCoord = arcCoord  #this is the arcCoordinate of the start of this line/curve
            endCoord = arcCoord+L  #this is the arcCoordinate of the end of this line/curve
            while nextArcCoord <= endCoord*(1.+tol):
                slope = (1/L)*(p1-p0)
                p = p0 + (nextArcCoord-startCoord)*slope
                ancfPointsSlopes+=[list(p)+list(slope)]
                if nextArcCoord != 0:
                    elementLengths += [lElem]
                    elementCurvatures += [0.] #represents straight line!
                nextArcCoord += lElem
            arcCoord = endCoord
        elif item['type']=='ARC':
            startAngle = item['startAngle']
            arcAngle = item['arcAngle']
            p0 = item['point0']
            r = item['radius']
    
            arcSign = 1.
            if arcAngle < 0.:
                arcSign = -1.
    
            # slope vector of first node
            vv0 = np.array([ np.cos(startAngle),np.sin(startAngle)])
            nv0=   np.array([-np.sin(startAngle),np.cos(startAngle)])
    
            #center point of circle for creation of arc:
            pCenter = arcSign*r*nv0 + p0

            nARC = int(abs(arcAngle)*graphicsElementsPerCircle/(2*pi))+1
        
            for i in range(nARC):
                phi = (i+1)*arcAngle/float(nARC)
                #new node:
                pArc = pCenter + arcSign*r*np.sin(phi)*vv0 - arcSign*r*np.cos(phi)*nv0
                lineData+=list(pArc)+[0]
            
            L = abs(r*arcAngle)
            startCoord = arcCoord  #this is the arcCoordinate of the start of this line/curve
            endCoord = arcCoord+L  #this is the arcCoordinate of the end of this line/curve
            while nextArcCoord <= endCoord*(1.+tol):
                phi = (nextArcCoord-startCoord)/L*arcAngle
    
                pArc = pCenter + arcSign*r*np.sin(phi)*vv0 - arcSign*r*np.cos(phi)*nv0
                vArc = np.cos(phi)*vv0 + np.sin(phi)*nv0
    
                ancfPointsSlopes+=[list(pArc)+list(vArc)]
                if nextArcCoord != 0:
                    elementLengths += [lElem]
                    curvature = arcSign*1./r #correct sign, whether it is left or right
                    #print('curvature=',curvature, ', sign=',arcSign)
                    elementCurvatures += [curvature] #r may not be zero!
                nextArcCoord += lElem
            
            arcCoord = endCoord
    
    exudyn.Print('Reeving system total length=', totalLength)
    # exudyn.Print('ANCF nodes=', ancfPointsSlopes)
    
    graphicsDataLines = [{'type':'Line', 'color':colorLines, 
                  'data':lineData} ]

    if graphicsNodeSize!= 0:
        for ps in ancfPointsSlopes:
            graphicsDataLines += [{'type':'Circle', 'color':colorLines,
                  'position':[ps[0],ps[1],0],'radius':graphicsNodeSize}]

    return {'ancfPointsSlopes':ancfPointsSlopes, 'elementLengths':elementLengths, 
            'elementCurvatures':elementCurvatures, 'totalLength':totalLength, 'circleData':curve2D, 
            'lineData':lineData, 'graphicsDataLines':graphicsDataLines, 'graphicsDataCircles':graphicsDataCircles }


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: Create nodes and ANCFCable2D elements in MainSystem mbs from a given set of nodes, elements lengths and a template for the cable, based on output of function CreateReevingCurve(...); function works similar to GenerateStraightLineANCFCable2D, but for arbitrary geometry (curved elements); optionally add loads and constraints
#**input:
#  mbs: the system where ANCF elements and nodes are added
#  ancfPointsSlopes: list of position and slopes for nodes, provided as 4D numpy arrays, as returned by CreateReevingCurve(...)
#  elementLengths: list of element lengths per element, as returned by CreateReevingCurve(...)
#  cableTemplate: a ObjectANCFCable2D object, containing the desired cable properties; cable length and node numbers are set automatically
#  massProportionalLoad: a 3D list or np.array, containing the gravity vector to be applied to all elements or zero
#  fixedConstraintsNode0: a list of 4 binary values, indicating the coordinate contraints on the first node (x,y-position and x,y-slope)
#  fixedConstraintsNode1: a list of 4 binary values, indicating the coordinate contraints on the last node (x,y-position and x,y-slope)
#  firstNodeIsLastNode: if True, then the last node is using the node number of the first node and the curve is closed; otherwise, the first and last nodes are different, and the curve is open
#  elementCurvatures: optional list of pre-curvatures of elements, used to override the cableTemplate entry 'physicsReferenceCurvature'; use 0. for straight lines!
#  graphicsSizeConstraints: if set other than -1, it will be used as the size for drawing applied coordinate constraints
#**output: returns a list [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]
def PointsAndSlopes2ANCFCable2D(mbs, ancfPointsSlopes, elementLengths, cableTemplate, massProportionalLoad=[0,0,0], 
                                fixedConstraintsNode0=[0,0,0,0], fixedConstraintsNode1=[0,0,0,0], firstNodeIsLastNode=True,
                                elementCurvatures=[], graphicsSizeConstraints=-1):

    cableNodeList=[]
    cableNodePositionList=[]
    cableObjectList=[]
    loadList=[]
    cableCoordinateConstraintList=[]
    
    # add first ANCF node (straight reference configuration):
    nodeNumber0 = mbs.AddNode(eii.Point2DS1(referenceCoordinates=ancfPointsSlopes[0])) 
    cableNodeList+=[nodeNumber0]
    cableNodePositionList+=[ancfPointsSlopes[0][0],ancfPointsSlopes[0][1],0] #3D
    numberOfElements = len(ancfPointsSlopes)-1
    
    # add all other ANCF nodes (straight reference configuration) and attach Gravity marker to them:
    for i in range(numberOfElements): 
        # pointSlope0 = ancfPointsSlopes[i]
        # pointSlope1 = ancfPointsSlopes[i+1]

        cableTemplate.physicsLength = elementLengths[i]
        if elementCurvatures != []:
            cableTemplate.physicsReferenceCurvature = elementCurvatures[i]
        
        if (i==numberOfElements-1 and firstNodeIsLastNode):
            nodeNumberLast = nodeNumber0
        else:
            nodeNumberLast = mbs.AddNode(eii.Point2DS1(referenceCoordinates=ancfPointsSlopes[i+1]))
        cableNodeList+=[nodeNumberLast]

        cableNodePositionList+=[ancfPointsSlopes[i+1][0],ancfPointsSlopes[i+1][1],0] #3D
       
        
        cableTemplate.nodeNumbers[0:2]=[cableNodeList[i],cableNodeList[i+1]]
            
        oCable=mbs.AddObject(cableTemplate)
        cableObjectList+=[oCable]

        if np.linalg.norm(massProportionalLoad) != 0:
            mBodyMassLast = mbs.AddMarker(eii.MarkerBodyMass(bodyNumber=oCable))
            lLoadLast=mbs.AddLoad(eii.Gravity(markerNumber=mBodyMassLast,loadVector=massProportionalLoad))
            loadList+=[lLoadLast]
        
    
    if (np.linalg.norm(list(fixedConstraintsNode0)+list(fixedConstraintsNode1)) ) != 0:
        # ground "node" at 0,0,0:
        nGround = mbs.AddNode(eii.NodePointGround(referenceCoordinates=[0,0,0])) 
        # add marker to ground "node": 
        mGround = mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0))
    
        if (firstNodeIsLastNode and
            np.linalg.norm(fixedConstraintsNode1)  != 0):
            raise ValueError('PointsAndSlopes2ANCFCable2D: fixedConstraintsNode1 must be [0,0,0,0] in case t firstNodeIsLastNode==True')

        for j in range(4):            
            if fixedConstraintsNode0[j] != 0:            
                #fix ANCF coordinates of first node
                mCableCoordinateConstraint0 = mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber = nodeNumber0, coordinate=j)) #add marker
                cBoundaryCondition=mbs.AddObject(eii.CoordinateConstraint(markerNumbers=[mGround,mCableCoordinateConstraint0],
                                                                      visualization=eii.VCoordinateConstraint(drawSize=graphicsSizeConstraints))) #add constraint
                cableCoordinateConstraintList+=[cBoundaryCondition]
            
        for j in range(4):            
            if fixedConstraintsNode1[j] != 0:                 
                # fix right end position coordinates, i.e., add markers and constraints:
                mCableCoordinateConstraint1 = mbs.AddMarker(eii.MarkerNodeCoordinate(nodeNumber = nodeNumberLast, 
                                                                                 coordinate=j))#add marker
                cBoundaryCondition=mbs.AddObject(eii.CoordinateConstraint(markerNumbers=[mGround,mCableCoordinateConstraint1],
                                                                      visualization=eii.VCoordinateConstraint(drawSize=graphicsSizeConstraints))) #add constraint
                cableCoordinateConstraintList+=[cBoundaryCondition]
            
    
    return [cableNodeList, cableObjectList, loadList, cableNodePositionList, cableCoordinateConstraintList]





#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#sliding joint utilities

#**function: generate a sliding joint from a list of cables, marker to a sliding body, etc.
#**output: returns the sliding joint object
def GenerateSlidingJoint(mbs,cableObjectList,markerBodyPositionOfSlidingBody,localMarkerIndexOfStartCable=0,slidingCoordinateStartPosition=0):

    cableMarkerList = []#list of Cable2DCoordinates markers
    offsetList = []     #list of offsets counted from first cable element; needed in sliding joint
    offset = 0          #first cable element has offset 0
    
    for item in cableObjectList: #create markers for cable elements
        m = mbs.AddMarker(eii.MarkerBodyCable2DCoordinates(bodyNumber = item))
        cableMarkerList += [m]
        offsetList += [offset]  
        offset += mbs.GetObjectParameter(item,'physicsLength')
    
    nodeDataSlidingJoint = mbs.AddNode(eii.NodeGenericData(initialCoordinates=[localMarkerIndexOfStartCable,slidingCoordinateStartPosition],numberOfDataCoordinates=2)) #initial index in cable list
    
    oSlidingJoint = mbs.AddObject(eii.ObjectJointSliding2D(markerNumbers=[markerBodyPositionOfSlidingBody,cableMarkerList[localMarkerIndexOfStartCable]], 
                                                      slidingMarkerNumbers=cableMarkerList, 
                                                      slidingMarkerOffsets=offsetList, 
                                                      nodeNumber=nodeDataSlidingJoint))

    return [oSlidingJoint]





#**function: generate an ALE sliding joint from a list of cables, marker to a sliding body, etc.
#**output: returns the sliding joint object
def GenerateAleSlidingJoint(mbs,cableObjectList,markerBodyPositionOfSlidingBody,AleNode,
                            localMarkerIndexOfStartCable=0,AleSlidingOffset=0,
                            activeConnector=True, penaltyStiffness=0):

    cableMarkerList = []#list of Cable2DCoordinates markers
    offsetList = []     #list of offsets counted from first cable element; needed in sliding joint
    offset = 0          #first cable element has offset 0
    usePenalty = (penaltyStiffness!=0) #penaltyStiffness=0 ==> no penalty formulation!
    
    for item in cableObjectList: #create markers for cable elements
        m = mbs.AddMarker(eii.MarkerBodyCable2DCoordinates(bodyNumber = item))
        cableMarkerList += [m]
        offsetList += [offset]  
        offset += mbs.GetObjectParameter(item,'physicsLength')
    
    nodeDataAleSlidingJoint = mbs.AddNode(eii.NodeGenericData(initialCoordinates=[localMarkerIndexOfStartCable],numberOfDataCoordinates=1)) #initial index in cable list   
    oAleSlidingJoint = mbs.AddObject(eii.ObjectJointALEMoving2D(markerNumbers=[markerBodyPositionOfSlidingBody,cableMarkerList[localMarkerIndexOfStartCable]], 
                                                      slidingMarkerNumbers=cableMarkerList, slidingMarkerOffsets=offsetList,
                                                      nodeNumbers=[nodeDataAleSlidingJoint, AleNode], slidingOffset=AleSlidingOffset,activeConnector=activeConnector,
                                                      usePenaltyFormulation = usePenalty, penaltyStiffness=penaltyStiffness))


    return [oAleSlidingJoint]




