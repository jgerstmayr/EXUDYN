#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Model for belt drive; special case according to: A. Pechstein, J. Gerstmayr. A Lagrange-Eulerian formulation of an axially moving beam based on the absolute nodal coordinate formulation, Multibody System Dynamics, Vol. 30 (3), 343 – 358, 2013.
#           Note that the model is set up slightly different as in the paper, and that some parameters have not been described in the 2013 paper; therfore, only the overall behaviour is the same
#
# Author:   Johannes Gerstmayr
# Date:     2022-02-27
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.beams import *

import numpy as np
from math import sin, cos, pi, sqrt , asin, acos, atan2
import copy 


SC = exu.SystemContainer()
mbs = SC.AddSystem()


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Parameters for the belt
gVec = [0,-9.81,0]      # gravity
Emodulus=1e7            # Young's modulus of ANCF element in N/m^2
b=0.08 #0.002            # width of rectangular ANCF element in m
hc=0.01#0.002           # height of rectangular ANCF element in m
rhoBeam=1036.       # density of ANCF element in kg/m^3
A=b*hc                  # cross sectional area of ANCF element in m^2
I=(b*hc**3)/12            # second moment of area of ANCF element in m^4
EI = Emodulus*I
EA = Emodulus*A
rhoA = rhoBeam*A
dEI = 0*Emodulus*I         #REMARK: bending proportional damping, not used in the 2013 paper
dEA = 1             #dEA=1 in paper PechsteinGerstmayr 2013, according to HOTINT C++ files ...


#%%
#settings:
useGraphics= True
useContact = True
doDynamic = True
velocityControl = True
staticEqulibrium = False #False in 2013 paper

#in 2013 paper, reference curvature is set according to initial geometry and released until tAccStart
preCurved = False #uses preCurvature according to straight and curved initial segments; here, alternative way is used with reference configuration, leading to less initial vibrations
strainIsRelativeToReference = 1. #0: straight reference, 1.: curved reference
useContactCircleFriction = True

movePulley = True #as in 2013 paper, move within first 0.05 seconds

tEnd = 1#*0.1 #*5 #end time of dynamic simulation
#h = 1e-3 #step size
tAccStart = 0.05
tAccEnd = 0.6
omegaFinal = 12

useFriction = True
dryFriction = 0.5#1.2
contactStiffness = 1e8#2e5
contactDamping = 0#1e-3*contactStiffness

nSegments = 8 #8
nANCFnodes = 2*30#120 works well

wheelMass = 50#1 the wheel mass is not given in the paper, only the inertia 
# for the second pulley
wheelInertia = 0.25#0.01
rotationDampingWheels = 0 #torque proportional to rotation speed

#torque = 1

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create circles
#complicated shape:
initialDisplacement = -0.0025
initialDisplacement0 = initialDisplacement*int(movePulley-1) #this is set at t=0

#h = 0.25e-3
radiusPulley = 0.09995
positionPulley2x = 0.1*pi
#preStretch = -1*(pi*0.4099+0.005)/ pi*0.4099
initialDistance = positionPulley2x - 0
initialLength = 2*initialDistance +2* pi*(radiusPulley + hc/2)
finalLength = initialLength - 2* initialDisplacement0
preStretch = -(finalLength - initialLength)/ initialLength 

factorStriplen = (2*initialDistance+2*pi*radiusPulley)/(2*initialDistance+2*pi*(radiusPulley + hc/2));
print('factorStriplen =', factorStriplen )

preStretch += (1-1./factorStriplen) #this is due to an error in the original paper 2013

print('preStretch=', preStretch)
circleList = [[[initialDisplacement0,0], radiusPulley,'L'],
              [[positionPulley2x,0], radiusPulley,'L'],
              # [[initialDisplacement0,0], radiusPulley,'L'],              
              # [[positionPulley2x,0], radiusPulley,'L'],
              ]
#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create geometry:
reevingDict = CreateReevingCurve(circleList, drawingLinesPerCircle = 64, 
                                radialOffset=0.5*hc, closedCurve=True, #allows closed curve
                                numberOfANCFnodes=nANCFnodes, graphicsNodeSize= 0.01)

# #symmetric mesh (starts at left-most point on left circle):
# circleList = [[[initialDisplacement0,-positionPulley2x], radiusPulley,'L'],
#               [[initialDisplacement0,0], radiusPulley,'L'],
#               [[positionPulley2x,0], radiusPulley,'L'],
#               [[initialDisplacement0,0], radiusPulley,'L'],
#               [[initialDisplacement0,+positionPulley2x], radiusPulley,'L'],
#               # [[initialDisplacement0,0], radiusPulley,'L'],              
#               # [[positionPulley2x,0], radiusPulley,'L'],
#               ]

# reevingDict = CreateReevingCurve(circleList, drawingLinesPerCircle = 64, 
#                                  removeLastLine=True, removeFirstLine=True,
#                                  radialOffset=0.5*hc, #closedCurve=True, #allows closed curve
#                                  numberOfANCFnodes=1+nANCFnodes, graphicsNodeSize= 0.01) #+1 because of duplicated first/last node

# del circleList[-1]
# del circleList[-1]
# del circleList[0]

# NOT generally working: 
# elementLength = reevingDict['elementLengths'][0] #if all are the same!


# set precurvature at location of pulleys:
elementCurvatures = [] #no pre-curvatures
if preCurved:
    elementCurvatures = reevingDict['elementCurvatures']

gList=[]
if False: #visualize reeving curve, without simulation
    gList = reevingDict['graphicsDataLines'] + reevingDict['graphicsDataCircles']

oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(show=False)))#, visualization = {'show : False'}
nGround = mbs.AddNode(NodePointGround())
mCoordinateGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))

    
#mbs.SetObjectParameter(objectNumber = oGround, parameterName = 'Vshow', value=False)
#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create ANCF elements:
dimZ = b #z.dimension

cableTemplate = Cable2D(#physicsLength = L / nElements, #set in GenerateStraightLineANCFCable2D(...)
                        physicsMassPerLength = rhoA,
                        physicsBendingStiffness = EI,
                        physicsAxialStiffness = EA,
                        physicsBendingDamping = dEI,
                        physicsAxialDamping = dEA,
                        physicsReferenceAxialStrain = preStretch*0., #prestretch
                        physicsReferenceCurvature = 0.,#-1/(radiusPulley + hc/2),
                        useReducedOrderIntegration = True,
                        strainIsRelativeToReference = strainIsRelativeToReference,
                        visualization=VCable2D(drawHeight=hc),
                        )

ancf = PointsAndSlopes2ANCFCable2D(mbs, reevingDict['ancfPointsSlopes'], 
                                   reevingDict['elementLengths'], 
                                   cableTemplate, massProportionalLoad=gVec, 
                                   #fixedConstraintsNode0=[1,0,0,1], #fixedConstraintsNode1=[1,1,1,1],
                                   elementCurvatures  = elementCurvatures,
                                   firstNodeIsLastNode=True, graphicsSizeConstraints=0.01)

if useContactCircleFriction:
    lElem = reevingDict['totalLength'] / nANCFnodes
    cFact=b*lElem/nSegments #stiffness shall be per area, but is applied at every segment
    # preCurved = True
    contactStiffness*=cFact
    contactDamping = 2000*cFact #according to Dufva 2008 paper ... seems also to be used in 2013 PEchstein Gerstmayr
    frictionVelocityPenalty = 1e7*cFact
    print('cFact=',cFact, ', lElem=', lElem)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create sensors for all nodes
sNodeVel = []
sObjectForce = []

ancfNodes = ancf[0]
ancfObjects = ancf[1]
if True:
    for i,obj in enumerate(ancfObjects):
        lElem = reevingDict['elementLengths'][i]
        sObjectForce += [mbs.AddSensor(SensorBody(bodyNumber = obj, 
                                                  storeInternal=True,
                                                  localPosition=[0.*lElem,0,0], #0=at left node!
                                                  #fileName = 'solution/obj'+str(i)+'.txt',
                                                  outputVariableType=exu.OutputVariableType.ForceLocal))]
        sNodeVel += [mbs.AddSensor(SensorBody(bodyNumber = obj, 
                                              storeInternal=True,
                                              localPosition=[0.*lElem,0,0], #0=at left node
                                              #fileName = 'solution/obj'+str(i)+'.txt',
                                              outputVariableType=exu.OutputVariableType.Velocity))]

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#add contact:
if useContact:

    dimZ= 0.01 #for drawing
    sWheelRot = [] #sensors for angular velocity

    nMassList = []
    wheelSprings = [] #for static computation
    for i, wheel in enumerate(circleList):
        p = [wheel[0][0], wheel[0][1], 0] #position of wheel center
        r = wheel[1]
    
        rot0 = 0 #initial rotation
        pRef = [p[0], p[1], rot0]
        gList = [GraphicsDataCylinder(pAxis=[0,0,-dimZ],vAxis=[0,0,-dimZ], radius=r,
                                      color= color4dodgerblue, nTiles=64),
                 GraphicsDataArrow(pAxis=[0,0,0], vAxis=[-0.9*r,0,0], radius=0.01*r, color=color4orange),
                 GraphicsDataArrow(pAxis=[0,0,0], vAxis=[0.9*r,0,0], radius=0.01*r, color=color4orange)]

        omega0 = 0 #initial angular velocity
        v0 = np.array([0,0,omega0]) 

        nMass = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=pRef, initialVelocities=v0,
                                            visualization=VNodeRigidBody2D(drawSize=dimZ*2)))
        nMassList += [nMass]
        oMass = mbs.AddObject(ObjectRigidBody2D(physicsMass=wheelMass, physicsInertia=wheelInertia,
                                                nodeNumber=nMass, visualization=
                                                VObjectRigidBody2D(graphicsData=gList)))
        mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
        mGroundWheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=p, visualization = VMarkerBodyRigid(show = False)))
    
        #mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGroundWheel, mNode], visualization=VRevoluteJoint2D(show=False)))

        mCoordinateWheelX = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=0))
        mCoordinateWheelY = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=1))
        constraintX = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheelX],
                                                 visualization=VCoordinateConstraint(show = False)))
        constraintY = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheelY],
                                                 visualization=VCoordinateConstraint(show = False)))
        if i==0:
            constraintPulleyLeftX = constraintX

        if True:
        
            sWheelRot += [mbs.AddSensor(SensorNode(nodeNumber=nMass, 
                                                   storeInternal=True,
                                                   fileName='solution/wheel'+str(i)+'angVel.txt',
                                                   outputVariableType=exu.OutputVariableType.AngularVelocity))]
        tdisplacement = 0.05
  
                         
        def UFvelocityDrive(mbs, t, itemNumber, lOffset): #time derivative of UFoffset
            if t < tAccStart:
                v = 0
            if t >= tAccStart and t < tAccEnd:
                v = omegaFinal/(tAccEnd-tAccStart)*(t-tAccStart)
            elif t >= tAccEnd:
                v = omegaFinal
            return v    
        
        if doDynamic:    
            if i == 0:
                if velocityControl:
                    mCoordinateWheel = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=2))
                    velControl = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheel],
                                                        velocityLevel=True, offsetUserFunction_t= UFvelocityDrive,
                                                        visualization=VCoordinateConstraint(show = False)))#UFvelocityDrive

        if staticEqulibrium:
            mCoordinateWheel = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=2))
            csd = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheel],
                                                     visualization=VCoordinateConstraint(show = False)))
            wheelSprings += [csd]
        

        else:
            cableList = ancf[1]
            mCircleBody = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oMass))
            for i in range(len(cableList)):
                initialGapList = [0.1]*nSegments + [0]*(2*nSegments) #initial gap of 0., corresponds to contact ...

                mCable = mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=cableList[i], numberOfSegments = nSegments))
                nodeDataContactCable = mbs.AddNode(NodeGenericData(initialCoordinates=initialGapList,
                                                                   numberOfDataCoordinates=nSegments*(1+2) ))
                mbs.AddObject(ObjectContactFrictionCircleCable2D(markerNumbers=[mCircleBody, mCable], nodeNumber = nodeDataContactCable, 
                                                         numberOfContactSegments=nSegments, 
                                                         contactStiffness = contactStiffness, 
                                                         contactDamping=contactDamping, 
                                                         frictionVelocityPenalty = frictionVelocityPenalty, 
                                                         frictionCoefficient=int(useFriction)*dryFriction,
                                                         circleRadius = r+hc/2, offset = 0*hc/2))
    


#user function to smoothly transform from curved to straight reference configuration as
#in paper 2013, Pechstein, Gerstmayr
def PreStepUserFunction(mbs, t):

    if True and t <= tAccStart+1e-10:
        cableList = ancf[1]
        fact = (tAccStart-t)/tAccStart #from 0 to 1
        if fact < 1e-12: fact = 0. #for very small values ...
        #curvatures = reevingDict['elementCurvatures']
        #print('fact=', fact)
        for i in range(len(cableList)):
            oANCF = cableList[i]
            mbs.SetObjectParameter(oANCF, 'strainIsRelativeToReference', 
                                   fact)
            mbs.SetObjectParameter(oANCF, 'physicsReferenceAxialStrain', 
                                   preStretch*(1.-fact))

        if movePulley:
            mbs.SetObjectParameter(constraintPulleyLeftX, 'offset', initialDisplacement*(1.-fact))
    
    return True


mbs.Assemble()


h = 0.00005*4 #0.1
simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/coordinatesSolution.txt'
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.005
simulationSettings.solutionSettings.sensorsWritePeriod = 0.001
#simulationSettings.displayComputationTime = True
simulationSettings.parallel.numberOfThreads = 1 #use 4 to speed up for > 100 ANCF elements
simulationSettings.displayStatistics = True

simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.stepInformation= 255

simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.useModifiedNewton = True
#simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
#simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
simulationSettings.displayStatistics = True
# if useContactCircleFriction:
#     simulationSettings.timeIntegration.discontinuous.iterationTolerance = 1e-7
#     #simulationSettings.timeIntegration.stepInformation = 511

SC.visualizationSettings.general.circleTiling = 24
SC.visualizationSettings.loads.show=False
SC.visualizationSettings.nodes.defaultSize = 0.01
SC.visualizationSettings.openGL.multiSampling = 4

if True:
    SC.visualizationSettings.contour.outputVariableComponent=0
    SC.visualizationSettings.contour.outputVariable=exu.OutputVariableType.ForceLocal

#visualize contact:
if False:
    SC.visualizationSettings.contact.showSearchTree =True
    SC.visualizationSettings.contact.showSearchTreeCells =True
    SC.visualizationSettings.contact.showBoundingBoxes = True

if useGraphics: 
    exu.StartRenderer()
    mbs.WaitForUserToContinue()

#simulationSettings.staticSolver.newton.absoluteTolerance = 1e-10
simulationSettings.staticSolver.loadStepGeometric = True;
simulationSettings.staticSolver.loadStepGeometricRange=1e4
simulationSettings.staticSolver.numberOfLoadSteps = 10
#simulationSettings.staticSolver.useLoadFactor = False
simulationSettings.staticSolver.stabilizerODE2term = 1
simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
simulationSettings.staticSolver.newton.absoluteTolerance = 1e-6

if staticEqulibrium: #precompute static equilibrium
    mbs.SetObjectParameter(velControl, 'activeConnector', False)
    exu.SolveStatic(mbs, simulationSettings, updateInitialValues=True)
    
    mbs.SetObjectParameter(velControl, 'activeConnector', True)
    for csd in wheelSprings:
        mbs.SetObjectParameter(csd, 'activeConnector', False)
    
mbs.SetPreStepUserFunction(PreStepUserFunction)

exu.SolveDynamic(mbs, simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2) #183 Newton iterations, 0.114 seconds

if useGraphics and True:
    SC.visualizationSettings.general.autoFitScene = False
    SC.visualizationSettings.general.graphicsUpdateInterval=0.02
    from exudyn.interactive import SolutionViewer
    sol = LoadSolutionFile('solution/coordinatesSolution.txt', safeMode=True)#, maxRows=100)
    SolutionViewer(mbs, sol)


if useGraphics: 
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!

#%%++++++++++++++++++++++++++++++++++++++++
if True:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
    from exudyn.plot import PlotSensor

    PlotSensor(mbs, closeAll=True)
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #velocity and axial forces along belt:
    nodeVec = np.zeros(len(sNodeVel))
    nodePos = np.zeros(len(sNodeVel))
    objectForce = np.zeros(len(sObjectForce))

    elementLengths = reevingDict['elementLengths']
    posX = 0.
    for i, sensor in enumerate(sNodeVel):
        vel = mbs.GetSensorValues(sensor) #at end of simulation
        #note that in 2013 paper, the velocity was measured at the "contact surface" of the belt, at y=-h/2
        nodeVec[i] = np.linalg.norm(vel)  #in 2023 paper, this was the absolute value of velocity!!!
        nodePos[i] = posX
        if i < len(elementLengths): #does not work for last node ...
            posX += elementLengths[i]
    for i, sensor in enumerate(sObjectForce):
        objectForce[i] = mbs.GetSensorValues(sensor)
        # nodeForce[i] = i*elementLength
    
    
    plt.figure('velocity versus axial position')
    ax=plt.gca() # get current axes
    plt.plot(nodePos[:], nodeVec[:], 'r-')  # , label='deformation for ALE beam' 
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
    plt.legend() #show labels as legend
    plt.tight_layout() 


    plt.figure('axial force versus axial position')
    ax=plt.gca() # get current axes
    # plt.plot(nodePos[:-1], objectForce[:], 'b-')
    plt.plot(nodePos[:], objectForce[:], 'b-')
    ax.grid(True, 'major', 'both')
    ax.xaxis.set_major_locator(ticker.MaxNLocator(10)) 
    ax.yaxis.set_major_locator(ticker.MaxNLocator(10)) 
    plt.legend() #show labels as legend
    plt.tight_layout() 

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    PlotSensor(mbs, sensorNumbers=[sWheelRot[0], sWheelRot[1]], components=[2,2])#,sWheelRot[1]

    # PlotSensor(mbs, sensorNumbers=['solution/wheel1angVelRef60Trap5e-5.txt'], components=[2], colorCodeOffset=2, newFigure=False)#,sWheelRot[1]
    # PlotSensor(mbs, sensorNumbers=['solution/wheel1angVelRef60Trap2.5e-5.txt'], components=[2], colorCodeOffset=3, newFigure=False)#,sWheelRot[1]
    # PlotSensor(mbs, sensorNumbers='solution/Belt_nel60_vel0_f0.5_md0_sd1X.txt', components=[3], colorCodeOffset=4, 
    #            newFigure=False, fileCommentChar = '%', fileDelimiterChar = ' ')#,sWheelRot[1]
    # PlotSensor(mbs, sensorNumbers='solution/Belt_nel120_vel0_f0.5_md0_sd1_dt1e-4X.txt', components=[0], colorCodeOffset=5,
    #            newFigure=False, fileCommentChar = '%', fileDelimiterChar = ' ')#,sWheelRot[1]
    # PlotSensor(mbs, sensorNumbers='solution/Belt_nel120_vel0_f0.5_md0_sd1_dt2.5e-5X.txt', components=[0], colorCodeOffset=6,
    #            newFigure=False, fileCommentChar = '%', fileDelimiterChar = ' ')#,sWheelRot[1]
            
