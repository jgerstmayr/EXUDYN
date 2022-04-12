#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ANCF cable elements in contact with circles defined by GeneralContact
#
# Author:   Johannes Gerstmayr
# Date:     2022-01-31
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import exudyn as exu
from exudyn.itemInterface import *
from exudyn.utilities import *
from exudyn.beams import *
from math import sin, cos, sqrt, pi

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

#background
rect = [-1,-1.5,3,1.5] #xmin,ymin,xmax,ymax
background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], 
                                   visualization=VObjectGround(graphicsData= [background0])))
nGround = mbs.AddNode(NodePointGround())
mCoordinateGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#contact
doImplicit = True
useContact = True
useFriction = True
dryFriction = 0.5
contactStiffness = 1e5
contactDamping = 1e-3*contactStiffness

if useContact:
    gContact = mbs.AddGeneralContact()
    gContact.verboseMode = 1*0
    gContact.frictionProportionalZone = 1
    gContact.ancfCableUseExactMethod = False
    gContact.ancfCableNumberOfContactSegments = 8
    ssx = 16#32 #search tree size
    ssy = 8#32 #search tree size
    ssz = 1 #search tree size
    gContact.SetSearchTreeCellSize(numberOfCells=[ssx,ssy,ssz])
    #gContact.SetSearchTreeBox(pMin=np.array([-1,-1,-1]), pMax=np.array([4,1,1]))
torque=10*2
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#wheel:
dampingWheel1 = 1  #5 #add breaking torque, to limit velocity
#cable:

numberOfElements = 32    # per section
curvedRefConf=False     # this flag could initialize the elements to be produced curved -> not suitable for belt drive!
L=2                     # length of ANCF element in m
E=1e10                  # Young's modulus of ANCF element in N/m^2
rhoBeam=1000            # density of ANCF element in kg/m^3
b=0.002                 # width of rectangular ANCF element in m
h=0.002                 # height of rectangular ANCF element in m
A=b*h                   # cross sectional area of ANCF element in m^2
I=b*h**3/12             # second moment of area of ANCF element in m^4
dEI = 0*1e-3*E*I
dEA = 1e-2*E*A
# f=3*E*I/L**2            # tip load applied to ANCF element in N
g=-9.81
dimZ = b #z.dimension
preStretch=-0.002
# exu.Print("load f="+str(f))
# exu.Print("EI="+str(E*I))

# nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
# mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

cableTemplate = Cable2D(#physicsLength = L / nElements, #set in GenerateStraightLineANCFCable2D(...)
                        physicsMassPerLength = rhoBeam*A,
                        physicsBendingStiffness = E*I,
                        physicsAxialStiffness = E*A,
                        physicsBendingDamping = dEI,
                        physicsAxialDamping = dEA,
                        physicsReferenceAxialStrain = preStretch, #prestretch
                        #nodeNumbers = [0, 0], #will be filled in GenerateStraightLineANCFCable2D(...)
                        visualization=VCable2D(drawHeight=2*h),
                        )
print("pre-stretch force=", preStretch*E*A)
print("beam mass per length=", rhoBeam*A)
#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#create belt drive:
distanceWheels = 2 #distance of wheel centers
wheelCenter0 = np.array([0,0,0])
wheelCenter1 = np.array([distanceWheels,0,0])

rWheel0 = 0.5
rWheel1 = 0.5
rRoll = 0.1
rollCenter1 = np.array([0.5*distanceWheels,-rWheel0+rRoll,0])

mWheel = 4
mRoll = 1e-6 #small as compared to beam with rhoA=4e-3
kRoll = 1e-2
dRelRoll = 0.005
fRoll = 0.01

velocityControl = True

yAxis = np.array([0,1.,0])
ancfList=[]

if True:
    startAngle = -pi
    arcAngle = -pi
    positionOfNode0 = wheelCenter0-rWheel0*yAxis # starting point of line
    ancf=GenerateCircularArcANCFCable2D(mbs, positionOfNode0, 
                                        rWheel0, startAngle, arcAngle, numberOfElements, 
                                        cableTemplate,
                                        massProportionalLoad = [0,g,0], #optionally add gravity
                                        #fixedConstraintsNode0 = [1,1,1,1], #add constraints for pos and rot (r'_y)
                                        #fixedConstraintsNode1 = [1,1,1,1],
                                        setCurvedReferenceConfiguration=curvedRefConf, 
                                        #verboseMode=True
                                        )
    ancfList+=[ancf]
    ancf=GenerateStraightLineANCFCable2D(mbs,
                                         ancf[3][-1], wheelCenter1+rWheel1*yAxis,
                                         numberOfElements,
                                         cableTemplate, #this defines the beam element properties
                                         massProportionalLoad = [0,g,0], #optionally add gravity
                                          # fixedConstraintsNode0 = [1,1,1,1], #add constraints for pos and rot (r'_y)
                                          # fixedConstraintsNode1 = [1,1,1,1],
                                         nodeNumber0=ancf[0][-1]
                                         )
    ancfList+=[ancf]

    if True:
        startAngle = 0
        arcAngle = -pi
        ancf=GenerateCircularArcANCFCable2D(mbs, ancf[3][-1], 
                                            rWheel1, startAngle, arcAngle, numberOfElements, 
                                            cableTemplate,
                                            massProportionalLoad = [0,g,0], #optionally add gravity
                                            #fixedConstraintsNode0 = [1,1,1,1], #add constraints for pos and rot (r'_y)
                                            #fixedConstraintsNode1 = [1,1,1,1],
                                            setCurvedReferenceConfiguration=curvedRefConf, 
                                            nodeNumber0=ancf[0][-1]
                                            )
        ancfList+=[ancf]
        ancf=GenerateStraightLineANCFCable2D(mbs,
                                             ancf[3][-1], ancfList[0][3][0],
                                             numberOfElements,
                                             cableTemplate, #this defines the beam element properties
                                             massProportionalLoad = [0,g,0], #optionally add gravity
                                             nodeNumber0=ancf[0][-1],
                                             nodeNumber1=ancfList[0][0][0]
                                             )
        ancfList+=[ancf]



def UFvelocityDrive(mbs, t, itemNumber, lOffset): #time derivative of UFoffset
    vMax = 10 #5m/s
    if 0.5*t < vMax:
        return 0.5*t
    return vMax

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
sAngVel=[]
#add contact:
if useContact:


    halfHeight = 0.5*h*0
    wheels = [{'center':wheelCenter0, 'radius':rWheel0-halfHeight, 'mass':mWheel}, 
              {'center':wheelCenter1, 'radius':rWheel1-halfHeight, 'mass':mWheel}, 
              {'center':rollCenter1, 'radius':rRoll-halfHeight, 'mass':mRoll}, #small mass for roll, not to influence beam
              ]

    for i, wheel in enumerate(wheels):
        r = wheel['radius']
        p = wheel['center']
        mass = wheel['mass']
        rot0 = 0 #initial rotation
        pRef = [p[0], p[1], rot0]
        gList = [GraphicsDataCylinder(vAxis=[0,0,dimZ], radius=r*0.99, #draw smaller to see cable element 
                                      color= color4dodgerblue, nTiles=32*2),
                 GraphicsDataArrow(pAxis=[0,0,0.02*r], vAxis=[r,0,0], radius=0.02*r, color=color4orange)]

        omega0 = 0 #initial angular velocity
        v0 = np.array([0,0,omega0]) 

        RBinertia = InertiaCylinder(mass/(r**2*np.pi*b), b, r, axis=2)

        nMass = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=pRef, initialVelocities=v0,
                                            visualization=VNodeRigidBody2D(drawSize=dimZ*2)))
        oMass = mbs.AddObject(ObjectRigidBody2D(physicsMass=RBinertia.mass, physicsInertia=RBinertia.GetInertia6D()[2],
                                                nodeNumber=nMass, visualization=
                                                VObjectRigidBody2D(graphicsData=gList)))
        mNode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
        mGroundWheel = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=p))
        frictionMaterialIndex=0
        if i < 2:
            mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGroundWheel, mNode]))
        else:
            #add measurement roll:
            dRoll = dRelRoll * (2*sqrt(kRoll*mRoll))
            print('measurement roll resonance=',sqrt(kRoll/mRoll)/(2*pi))
            frictionMaterialIndex=1 #no friction
            mCWheel0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=0))
            mCWheel1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=1))
            mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCWheel0]))
            mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mCoordinateGround, mCWheel1],
                                                 stiffness = 1e-2, damping=dRoll,
                                                 offset=-fRoll/kRoll,
                                                 visualization=VCoordinateSpringDamper(show=False)))
            sMeasureRoll = mbs.AddSensor(SensorNode(nodeNumber=nMass, 
                                                    fileName='solution/wheel'+str(i)+'pos.txt',
                                                    outputVariableType=exu.OutputVariableType.Displacement))
        
        if i == 0:
            if velocityControl:
                mCoordinateWheel = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=2))
                mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordinateGround, mCoordinateWheel],
                                                   velocityLevel=True, offsetUserFunction_t=UFvelocityDrive))
                
            else:
                mbs.AddLoad(LoadTorqueVector(markerNumber=mNode, loadVector=[0,0,torque]))
        if i == 1:
            mCoordinateWheel = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=2))
            mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mCoordinateGround, mCoordinateWheel], 
                                                 damping=dampingWheel1, 
                                                 visualization=VCoordinateSpringDamper(show=False)))

        gContact.AddSphereWithMarker(mNode, radius=r, contactStiffness=contactStiffness, 
                                     contactDamping=contactDamping, frictionMaterialIndex=frictionMaterialIndex)
        
        sAngVel += [mbs.AddSensor(SensorNode(nodeNumber=nMass, fileName='solution/wheel'+str(i)+'angVel.txt',
                                  outputVariableType=exu.OutputVariableType.AngularVelocity))]

    allCables = []
    for ancf in ancfList:
        allCables += ancf[1]
        
    for oIndex in allCables:
        gContact.AddANCFCable(objectIndex=oIndex, halfHeight=halfHeight, #halfHeight should be h/2, but then cylinders should be smaller
                              contactStiffness=contactStiffness, contactDamping=contactDamping, 
                              frictionMaterialIndex=0)

    frictionMatrix = np.zeros((2,2))
    frictionMatrix[0,0]=int(useFriction)*dryFriction
    frictionMatrix[0,1]=0 #no friction between some rolls and cable
    frictionMatrix[1,0]=0 #no friction between some rolls and cable
    gContact.SetFrictionPairings(frictionMatrix)




mbs.Assemble()
#exu.Print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/coordinatesSolution.txt'
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = 0.005
simulationSettings.solutionSettings.sensorsWritePeriod = 0.001
#simulationSettings.displayComputationTime = True
simulationSettings.parallel.numberOfThreads = 1 #use 4 to speed up for > 100 ANCF elements
simulationSettings.displayStatistics = True

doDynamic = True
tEnd = 10#0.25
h = 0.5e-3
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.stepInformation= 3+128+256

simulationSettings.timeIntegration.verboseMode = 1 #otherwise, load steps are shown ...

simulationSettings.timeIntegration.newton.useModifiedNewton = True

#+++++++++++++++++++++++++++++++++++++++++++++
#static:
simulationSettings.staticSolver.verboseMode = 1 #otherwise, load steps are shown ...
simulationSettings.staticSolver.numberOfLoadSteps  = 40
simulationSettings.staticSolver.discontinuous.iterationTolerance = 1
simulationSettings.staticSolver.stabilizerODE2term = 2 #may only act on position degrees of freedom
#+++++++++++++++++++++++++++++++++++++++++++++

SC.visualizationSettings.general.drawWorldBasis=True
SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.nodes.defaultSize = h*20
SC.visualizationSettings.loads.show = False

SC.visualizationSettings.contour.outputVariableComponent=0
SC.visualizationSettings.contour.outputVariable=exu.OutputVariableType.ForceLocal

#visualize contact:
if False:
    SC.visualizationSettings.contact.showSearchTree =True
    SC.visualizationSettings.contact.showSearchTreeCells =True
    SC.visualizationSettings.contact.showBoundingBoxes = True

if exudynTestGlobals.useGraphics: 
    exu.StartRenderer()
    mbs.WaitForUserToContinue()

if doDynamic :
    exu.SolveDynamic(mbs, simulationSettings) #183 Newton iterations, 0.114 seconds
else:
    exu.SolveStatic(mbs, simulationSettings) #183 Newton iterations, 0.114 seconds


if exudynTestGlobals.useGraphics and True:
    SC.visualizationSettings.general.autoFitScene = False
    SC.visualizationSettings.general.graphicsUpdateInterval=0.02
    from exudyn.interactive import SolutionViewer
    sol = LoadSolutionFile('solution/coordinatesSolution.txt', safeMode=True)#, maxRows=100)
    print('start SolutionViewer')
    SolutionViewer(mbs, sol)


if exudynTestGlobals.useGraphics: 
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!
    
    if True:
        from exudyn.plot import PlotSensor
        PlotSensor(mbs, sensorNumbers=[sAngVel[0],sAngVel[1]], components=2, closeAll=True)
        PlotSensor(mbs, sensorNumbers=sMeasureRoll, components=1)
        

#print representative result:
sol = mbs.systemData.GetODE2Coordinates()
n = len(sol)
exu.Print('tip displacement: x='+str(sol[n-4])+', y='+str(sol[n-3])) 
exudynTestGlobals.testError = sol[n-3] - (-0.4842656133238705) #2021-05-07 (deactivated StaticSolveOldSolver):-0.4842656133238705  #2019-12-17(relTol=1e-7 / up to 7 digits accurate): -0.4842656547442095;  2019-11-22: (-0.4844812763485709) (with relTol=1e-5);  y-displacement
exudynTestGlobals.testResult = sol[n-3]


