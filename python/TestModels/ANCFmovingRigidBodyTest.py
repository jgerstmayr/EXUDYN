#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test model for moving rigid body on two cables
#
# Author:   Andreas Zwölfer, Johannes Gerstmayr
# Date:     2019-12-16
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict

useGraphics = True #without test
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#you can erase the following lines and all exudynTestGlobals related operations if this is not intended to be used as TestModel:
try: #only if called from test suite
    from modelUnitTests import exudynTestGlobals #for globally storing test results
    useGraphics = exudynTestGlobals.useGraphics
except:
    class ExudynTestGlobals:
        pass
    exudynTestGlobals = ExudynTestGlobals()
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

SC = exu.SystemContainer()
mbs = SC.AddSystem()


#Import function  
#import GenerateStraightLineANCFCable2D as func

#background
rect = [-2,-2,4,2] #xmin,ymin,xmax,ymax
background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
background1 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[0,-1,0, 2,-1,0]} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))

L=10
gravityFieldConstant=9.81 #9000
complianceFactBend = 0.1
complianceFactAxial = 0.1
nEl=10 #must be even number
vALE=2*5
offset=-1
 

nGlobalGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) 
mGlobalGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGlobalGround, coordinate=0))

fixANCFRotation = 1

#######################SUSPENSION ROPE##################################################################################################################################################################
suspensionCableTemplate=Cable2D(physicsMassPerLength=20.87, 
                                physicsBendingStiffness=78878*complianceFactBend, 
                                physicsAxialStiffness=398240000*complianceFactAxial)

[suspensionCableNodeList, suspensionCableObjectList, suspensionLoadList, suspensionCableNodePositionList, dummy]=GenerateStraightLineANCFCable2D(mbs=mbs, positionOfNode0=[0,0,0], positionOfNode1=[L,0,0], numberOfElements=nEl, cableTemplate=suspensionCableTemplate,
                                                                  massProportionalLoad=[0,-gravityFieldConstant,0], fixedConstraintsNode0=[1,1,0,fixANCFRotation], fixedConstraintsNode1=[1,1,0,fixANCFRotation])
##################################################################################################################################################################

 

######################Haulage ROPE##################################################################################################################################################################
nALE = mbs.AddNode(NodeGenericODE2(numberOfODE2Coordinates=1, referenceCoordinates=[0], initialCoordinates=[0], initialCoordinates_t=[vALE]))
mALE = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nALE, coordinate=0)) #ALE velocity  marker

haulageCableTemplate=ALECable2D(physicsMassPerLength=6.96, 
                                physicsBendingStiffness=5956*complianceFactBend, 
                                physicsAxialStiffness=96725000*complianceFactAxial,
                                physicsAddALEvariation=False) #for compatibility with test suite results
haulageCableTemplate.nodeNumbers[2]=nALE #this will not be overwritten!

[haulageCableNodeList, haulageCableObjectList, haulageLoadList, haulageCableNodePositionList, dummy]=GenerateStraightLineANCFCable2D(mbs=mbs, 
                     positionOfNode0=[0,offset,0], positionOfNode1=[L,offset,0], numberOfElements=nEl, cableTemplate=haulageCableTemplate,
                     massProportionalLoad=[0,-gravityFieldConstant,0], fixedConstraintsNode0=[1,1,0,fixANCFRotation], fixedConstraintsNode1=[1,1,0,fixANCFRotation])

cAleConstraint=mbs.AddObject(CoordinateConstraint(markerNumbers=[mGlobalGround,mALE]))
##################################################################################################################################################################################################################################################


#slack carrier test
slackCarrierWheelRadius=0.75
mSuspensionRopeAttachmentNodeX=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = suspensionCableNodeList[int(nEl/2)], coordinate=0))
mSuspensionRopeAttachmentNodeY=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = suspensionCableNodeList[int(nEl/2)], coordinate=1))

graphicsSlackCarrier={'type':'Circle', 'color':[.1,0.1,0.8,1], 'position':[0,0,0], 'radius': slackCarrierWheelRadius}
nSlackCarrierRigidBody = mbs.AddNode(Rigid2D(referenceCoordinates=[5,offset-slackCarrierWheelRadius,0]))
oSlackCarrierRigidBody = mbs.AddObject(RigidBody2D(physicsMass=1, physicsInertia=1, nodeNumber=nSlackCarrierRigidBody,visualization=VObjectRigidBody2D(graphicsData= [graphicsSlackCarrier]))) #, visualization=VObjectRigidBody2D(graphicsData= [graphicsSupportWheels])

mSlackCarrierX = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nSlackCarrierRigidBody,coordinate=0))
mSlackCarrierY = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nSlackCarrierRigidBody,coordinate=1))
mSlackCarrierRot = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nSlackCarrierRigidBody,coordinate=2))

mbs.AddObject(CoordinateConstraint(markerNumbers=[mSuspensionRopeAttachmentNodeX,mSlackCarrierX])) 
mbs.AddObject(CoordinateConstraint(markerNumbers=[mSuspensionRopeAttachmentNodeY,mSlackCarrierY])) 
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGlobalGround,mSlackCarrierRot]))
               
nSegments = 4 #number of contact segments; must be consistent between nodedata and contact element
useFriction = False
nFactFriction = 1
if useFriction: nFactFriction = 3

initialGapList = [0.1]*(nSegments*nFactFriction) #initial gap of 0.1
cStiffness = 1e7
mContactSlackCarrier=mbs.AddMarker(MarkerBodyRigid(bodyNumber = oSlackCarrierRigidBody))  


for i in haulageCableObjectList:
    mContactCable = mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=i, numberOfSegments = nSegments))  
    nodeDataContactCable = mbs.AddNode(NodeGenericData(initialCoordinates=initialGapList,numberOfDataCoordinates=nSegments*nFactFriction)) 
    if useFriction: 
        mbs.AddObject(ObjectContactFrictionCircleCable2D(markerNumbers=[mContactSlackCarrier, mContactCable], nodeNumber = nodeDataContactCable, numberOfContactSegments=nSegments, contactStiffness = cStiffness, circleRadius = slackCarrierWheelRadius, offset = 0))  
    else:
        mbs.AddObject(ObjectContactCircleCable2D(markerNumbers=[mContactSlackCarrier, mContactCable], nodeNumber = nodeDataContactCable, numberOfContactSegments=nSegments, contactStiffness = cStiffness, circleRadius = slackCarrierWheelRadius, offset = 0))  
      




##################################################################################
    
a = 0.8     #y-dim/2 of gondula
b = 0.02   #x-dim/2 of gondula
yCOM = a    #COM distance to attachment point; in vertical direction
massRigid = 60 #12
#vInit=40
inertiaRigid = massRigid/12*(2*a)**2
g = 9.81    # gravity
    
refPos = [0,offset,0]
#    refPos = [fieldData['stationData'][0]['referencePointCoordinates'][1][0],fieldData['maxVerticalPositionSuspensionRopeShoes'][0],0]

#rigid body which slides:
graphicsRigid1 = GraphicsDataRectangle(-b,0,b,a) #drawing of rigid body
graphicsRigid2 = GraphicsDataRectangle(-a,-a,a,0) #drawing of rigid body

nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[refPos[0],refPos[1]-yCOM,0], initialVelocities=[vALE,0,0]));
oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,visualization=VObjectRigidBody2D(graphicsData= [graphicsRigid1,graphicsRigid2])))


markerRigidTopAle = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[0.,yCOM,0.])) #support point
mR2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0.,0.,0.])) #center of mass (for load)
mbs.AddLoad(Force(markerNumber=mR2, loadVector=[0,-massRigid*g,0]))

aleCableMarkerList = []#list of Cable2DCoordinates markers
aleOffsetList = []     #list of offsets counted from first cable element; needed in sliding joint
aleOffset = 0          #first cable element has offset 0
aleSlidingCoordinateInit = 0 
aleInitialLocalMarker = 0 


for i in range(len(haulageCableObjectList)): #create markers for cable elements
    m = mbs.AddMarker(MarkerBodyCable2DCoordinates(bodyNumber = haulageCableObjectList[i]))
    aleCableMarkerList += [m] #list containing 'MarkerBodyCable2DCoordinates' marker for sliding joint
    aleOffsetList += [aleOffset] #list of relative (arclength) coordinates of the starting point of a cable
    aleOffset += L/nEl
 
nodeDataAleSlidingJoint = mbs.AddNode(NodeGenericData(initialCoordinates=[aleInitialLocalMarker],numberOfDataCoordinates=1)) #initial index in cable list   
aleSlidingJoint = mbs.AddObject(ObjectJointALEMoving2D(name='aleSlider', markerNumbers=[markerRigidTopAle,aleCableMarkerList[aleInitialLocalMarker]], 
                                                  slidingMarkerNumbers=aleCableMarkerList, slidingMarkerOffsets=aleOffsetList,nodeNumbers=[nodeDataAleSlidingJoint, nALE], activeConnector = False))
 
mnRigid0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRigid, coordinate=0)) #add rigid body marker
mnRigid1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRigid, coordinate=1)) #add rigid body marker
mnRigid2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRigid, coordinate=2)) #add rigid body marker
nCCRigid0 = mbs.AddObject(CoordinateConstraint(markerNumbers=[mGlobalGround,mnRigid0])) 
nCCRigid1 = mbs.AddObject(CoordinateConstraint(markerNumbers=[mGlobalGround,mnRigid1])) 
nCCRigid2 = mbs.AddObject(CoordinateConstraint(markerNumbers=[mGlobalGround,mnRigid2])) 

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# Assemble multibody system
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
mbs.Assemble()
#exu.Print(mbs)

#SC.renderer.DoIdleTasks()

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# Simualtion settings:
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++#

simulationSettings = exu.SimulationSettings()
simulationSettings.staticSolver.numberOfLoadSteps  = 2
 
SC.visualizationSettings.general.circleTiling = 64
SC.visualizationSettings.nodes.defaultSize=0.125

SC.visualizationSettings.contour.outputVariable = exu.OutputVariableType.Displacement
SC.visualizationSettings.contour.outputVariableComponent = 1 # plot y-component


SC.visualizationSettings.contact.contactPointsDefaultSize = .005
SC.visualizationSettings.connectors.showContact = True
 


if useGraphics: 
    SC.renderer.Start()

#get initial velocities
vInit = mbs.systemData.GetODE2Coordinates_t(configuration = exu.ConfigurationType.Initial)

#SC.renderer.DoIdleTasks()

mbs.SolveStatic(simulationSettings) 

#prolong solution for next computation
u = mbs.systemData.GetODE2Coordinates()
#v = mbs.systemData.GetODE2Coordinates_t()
data = mbs.systemData.GetDataCoordinates()
mbs.systemData.SetODE2Coordinates(u,configuration = exu.ConfigurationType.Initial)
mbs.systemData.SetODE2Coordinates_t(vInit,configuration = exu.ConfigurationType.Initial)
mbs.systemData.SetDataCoordinates(data,configuration = exu.ConfigurationType.Initial)

#store some reference value:
ncables = len(suspensionCableNodeList)
sol = mbs.systemData.GetODE2Coordinates(); 
u = sol[int(ncables/4)*4+1]; #y-displacement of node at midpoint of rope
     

#SC.renderer.DoIdleTasks()

mbs.SetObjectParameter(aleSlidingJoint, 'activeConnector', True)
mbs.SetObjectParameter(nCCRigid0, 'activeConnector', False)
mbs.SetObjectParameter(nCCRigid1, 'activeConnector', False)
mbs.SetObjectParameter(nCCRigid2, 'activeConnector', False)
mbs.SetObjectParameter(cAleConstraint, 'activeConnector', False)


solveDynamic = True
if solveDynamic:
    # time related settings:
    steps=200
    tend=0.1
    h=tend/steps
    
    #fact = 15000
    simulationSettings.timeIntegration.numberOfSteps = steps #1*fact
    simulationSettings.timeIntegration.endTime = tend #0.002*fact
    # Integrator related settings:
    # simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
    # simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.3
    simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False

    simulationSettings.timeIntegration.verboseMode = 1
    simulationSettings.timeIntegration.verboseModeFile = 0

    
    mbs.SolveDynamic(simulationSettings)
    

if useGraphics: 
    #SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

#compute error for test suite:
ncables = len(suspensionCableNodeList)
sol2 = mbs.systemData.GetODE2Coordinates(); 
u2 = sol2[int(ncables/4)*4+1]; #y-displacement of node in first quater of rope
exu.Print('static deflection  =',u)      #2020-03-05(corrected Cable2DshapeMarker): -0.06446474690480661    2019-12-17(new static solver): -0.06446474690512931;  2019-12-16: -0.06446474679809994
exu.Print('dynamic deflection =',u2)       #2020-03-05(corrected Cable2DshapeMarker):0.06446627698400298; 2020-01-09: -0.06446627698121662(computeInitialAccelerations = False) 2020-01-09: -0.06446627843202835; 2019-12-26: -0.06446627698104967; 2019-12-17(update residual): -0.06446627698121662;  2019-12-16 (late): -0.06446627699890756; 2019-12-16: -0.06446610364603222
#exudynTestGlobals.testError = u + u2 - (-0.06446474690480661-0.06446627698400298)
exu.Print('ANCFmovingRigidBodyTest=',u+u2)       
exudynTestGlobals.testError = u + u2 - (-0.06446474690612931 - 0.06446622244370685) #updated 2022-12-25
exudynTestGlobals.testResult = u + u2

