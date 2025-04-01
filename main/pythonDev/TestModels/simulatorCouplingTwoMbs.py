#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Coupling of two MainSystem (mbs0, mbs1) as simple simulator coupling;
#           mbs0 contains joint constraints and is solved implicit; it receives forces from mbs1;
#           mbs1 is containing particles and shall be solved explicit; it receives displaced and rotated bodies from mbs0
# 
# Author:   Johannes Gerstmayr
# Date:     2024-10-20
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict
import numpy as np
import sys
import copy

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
mbs0 = SC.AddSystem()

g = [0,0,-9.81]     #gravity in m/s^2

doSimulatorCoupling = True #if False, mbs1 is not added

#++++++++++++++++++++++++++++++
#wheel parameters:
rhoWheel = 500      #density kg/m^3
rWheel = 0.4            #radius of disc in m
wWheel = 0.2             #width of disc in m, just for drawing
p0Wheel = [0,0,rWheel]        #origin of disc center point at reference, such that initial contact point is at [0,0,0]
initialRotationCar = RotationMatrixZ(0)

v0 = 10 #initial car velocity in y-direction
rSteering = -200*0

#v0 = [0,0,0]                                   #initial translational velocity
#print("v0Car=",v0)

#%%++++++++++++++++++++++++++++++
#ground parameters
lGround = 50
wGround = 10
hGround = 0.5

#%%++++++++++++++++++++++++++++++
#car parameters and inertia:
p0Car = [0,0,rWheel]        #origin of disc center point at reference, such that initial contact point is at [0,0,0]
lCar = 6
wCar = 2.5
hCar = rWheel
mCar = 5000
omega0Car = [0,0,0]                   #initial angular velocity around z-axis
omega0Wheel = [-1*v0/rWheel,0,0]                   #initial angular velocity around z-axis
v0Car = [0,v0*1,0]                  #initial velocity of car center point
pWheel = [0.5*wCar, 0.35*lCar] #local position of wheels

kDrive = 5000 #velocity P-controller
omegaDriveSet = -v0/rWheel

#inertia for infinitely small ring:
inertiaWheel = InertiaCylinder(density=rhoWheel, length=wWheel, outerRadius=rWheel, axis=0)
#exu.Print(inertiaWheel)

inertiaCar = InertiaCuboid(density=mCar/(lCar*wCar*hCar),sideLengths=[wCar, lCar, hCar])
#exu.Print(inertiaCar)

#%%++++++++++++++++++++++++++++++
#create car node and body:
graphicsCar = []
graphicsCar += [graphics.Brick(centerPoint=[0,0,0],size=[wCar-4*wWheel, lCar, hCar], color=graphics.color.lightred)]
graphicsCar += [graphics.Brick(centerPoint=[0,pWheel[1],0],size=[wCar-1*wWheel, 0.5*rWheel, 0.5*rWheel], color=graphics.color.grey)]
graphicsCar += [graphics.Brick(centerPoint=[0,-pWheel[1],0],size=[wCar-1*wWheel, 0.5*rWheel, 0.5*rWheel], color=graphics.color.grey)]
graphicsCar += [graphics.Brick(centerPoint=[0,0.4*lCar,3*hCar],size=[wCar-4*wWheel, 0.2*lCar, 6*hCar], color=graphics.color.lightred)]

dShield = 0.05
hShield = 2
offyShield = 0.5
phiShield = 16/180*pi #angle of shield
nShield = 15 #number of segments
curvedShield = 0.5 #curved parameter
graphicsShield = []

zShield = np.linspace(0.5*hShield,-0.5*hShield,nShield)
yShield = np.append(zShield**2*curvedShield,zShield**2*curvedShield+dShield)
zShield = np.append(zShield, np.linspace(-0.5*hShield,0.5*hShield,nShield))
vertShield = np.vstack((yShield,zShield)).T.tolist()
segShield = np.column_stack((np.arange(2*nShield), np.arange(1, 2*nShield + 1)%(2*nShield)))

#the shield is then only used for contact with particles!
graphicsShield += [graphics.SolidExtrusion(vertices=vertShield, segments=segShield, height=wCar*1.2, 
                        rot = RotationMatrixZ(0.5*pi-phiShield)@RotationMatrixX(0.5*pi), 
                        pOff=[-0.6*wCar,0.5*lCar+0*dShield+offyShield + wCar*0.6*sin(phiShield),0.5*hShield-rWheel],
                        color=graphics.color.orange, smoothNormals=True)]

graphicsCar += [graphics.Brick(centerPoint=[0,0.5*lCar+0.5*0.75*offyShield,0.5*hShield-rWheel],
                                  size=[wCar*0.2, 0.75*offyShield, 0.5*offyShield], color=graphics.color.grey)]
graphicsCar += [graphics.Cylinder(pAxis=[0,0.5*lCar+0.8*offyShield+dShield*0.25,0.5*hShield-rWheel-0.2*offyShield],
                                     vAxis=[0,0, 0.4*offyShield], 
                                     radius=0.2*offyShield, 
                                     color=graphics.color.grey, nTiles=32)]

bCar=mbs0.CreateRigidBody(inertia = inertiaCar, 
                          nodeType = exu.NodeType.RotationRotationVector, #use same node type as for explicit integrator
                         referencePosition = p0Car, 
                         referenceRotationMatrix = initialRotationCar,
                         initialAngularVelocity = omega0Car,
                         initialVelocity = v0Car,
                         gravity = g, 
                         #graphicsDataList = graphicsCar+graphicsShield
                         )

nCarMbs0 = mbs0.GetObject(bCar)['nodeNumber']

#add zero force to car, used the to interact with particles
carParticlesForce = mbs0.CreateForce(bodyNumber=bCar)
#the zero torque; it is bodyFixed, which agrees with the rotation vector formulation
carParticlesTorque = mbs0.CreateTorque(bodyNumber=bCar, bodyFixed=True)


nWheels = 4
markerWheels=[]
markerCarAxles=[]
oRollingDiscs=[]

# car setup:
# ^Y, lCar
# | W2 +---+ W3
# |    |   |
# |    | + | car center point
# |    |   |
# | W0 +---+ W1
# +---->X, wCar

#ground body and marker
gGround = []
if not doSimulatorCoupling:
    gGround += [graphics.CheckerBoard(point=[0.,0.5*lGround-lCar,-0.01],size=wGround, size2=lGround, nTiles=8,nTiles2=80)]
    
oGround = mbs0.AddObject(ObjectGround(visualization=VObjectGround(graphicsData=gGround)))
markerGround = mbs0.AddMarker(MarkerBodyRigid(bodyNumber=oGround, localPosition=[0,0,0]))

sCarVel = mbs0.AddSensor(SensorBody(bodyNumber=bCar, #fileName='solution/rollingDiscCarVel.txt', 
                                   storeInternal=True,
                                   outputVariableType = exu.OutputVariableType.Velocity))

sAngVels=[]
sWheelPos=[]
sRollPos=[]
sRollForce=[]

#%%++++++++++++++++++++++++++++++
#create wheels bodies and nodes:
for iWheel in range(nWheels):
    #additional graphics for visualization of rotation:
    graphicsWheel = []
    graphicsWheel += [graphics.Brick(centerPoint=[0,0,0],size=[wWheel*1.1,0.7*rWheel,0.7*rWheel], color=graphics.color.lightred)]
    graphicsWheel += [graphics.Cylinder(pAxis=[-0.5*wWheel,0,0],vAxis=[wWheel,0,0],radius=rWheel, color=graphics.color.darkgrey,nTiles=64)]

    dx = -pWheel[0]
    dy = -pWheel[1]
    if iWheel > 1: dy *= -1
    if iWheel % 2 == 1: dx *= -1

    kRolling = 1e6
    dRolling = kRolling*0.01

    phiZwheelLeft = 0
    phiZwheelRight = 0
    if rSteering != 0:
        phiZwheelLeft = np.arctan(lCar/rSteering) #5/180*np.pi   #steering angle
        phiZwheelRight = np.arctan(lCar/(wCar+rSteering)) #5/180*np.pi   #steering angle

    initialRotationWheelLeft = RotationMatrixZ(phiZwheelLeft)
    initialRotationWheelRight = RotationMatrixZ(phiZwheelRight)

    initialRotation = RotationMatrixZ(0)
    if iWheel == 2:
        initialRotation = initialRotationWheelLeft
    if iWheel == 3:
        initialRotation = initialRotationWheelRight

    #v0Wheel = Skew(omega0Wheel) @ initialRotationWheel @ [0,0,rWheel]   #initial angular velocity of center point
    v0Wheel = v0Car #approx.

    pOff = [dx,dy,0]


    #add wheel body
    b0 = mbs0.CreateRigidBody(inertia = inertiaWheel, 
                             referencePosition = VAdd(p0Wheel,pOff), 
                             referenceRotationMatrix = initialRotation, #np.diag([1,1,1]),
                             initialAngularVelocity = omega0Wheel,
                             initialVelocity = v0Wheel,
                             gravity = g, 
                             #graphicsDataList = graphicsWheel
                             )

    n0 = mbs0.GetObject(b0)['nodeNumber']

    #markers for wheel body:
    mWheel = mbs0.AddMarker(MarkerBodyRigid(bodyNumber=b0, localPosition=[0,0,0]))
    markerWheels += [mWheel]

    mCarAxle = mbs0.AddMarker(MarkerBodyRigid(bodyNumber=bCar, localPosition=pOff))
    markerCarAxles += [mCarAxle]

    mbs0.CreateRevoluteJoint(bodyNumbers=[bCar, b0], 
                             position = pOff,
                             useGlobalFrame=False,
                             axis=initialRotation @ [1,0,0], 
                             show=False,
                             axisLength=wWheel*1.05)
    
    nGeneric = mbs0.AddNode(NodeGenericData(initialCoordinates=[0,0,0], numberOfDataCoordinates=3))
    oRolling = mbs0.AddObject(ObjectConnectorRollingDiscPenalty(markerNumbers=[markerGround, mWheel], 
                                                                nodeNumber = nGeneric,
                                                                discRadius=rWheel, 
                                                                dryFriction=[0.8,0.8], 
                                                                dryFrictionProportionalZone=1e-1, 
                                                                rollingFrictionViscous=0.2*0,
                                                                contactStiffness=kRolling, contactDamping=dRolling,
                                                                visualization=VObjectConnectorRollingDiscPenalty(discWidth=wWheel, color=graphics.color.blue, show=False)))
    oRollingDiscs += [oRolling]


    #drive:
    if iWheel < 2: #only on back wheels
        #print('wheel ', iWheel, ' has TSD')
        #torsional spring damper acts around Z-axis -> requires rotation marker to rotate from x to z axis
        #damping here represents a P-control on andular velocity using the offset
        oTSD = mbs0.AddObject(TorsionalSpringDamper(markerNumbers=[mCarAxle, mWheel],
                                                    rotationMarker0 = RotationMatrixY(0.5*pi)@initialRotation,
                                                    rotationMarker1 = RotationMatrixY(0.5*pi),
                                                    damping=kDrive,
                                                    velocityOffset=omegaDriveSet
                                                    ))

    strNum = str(iWheel)
    if useGraphics:
        sAngVels+=[mbs0.AddSensor(SensorBody(bodyNumber=b0,
                                 storeInternal=True,
                                 outputVariableType = exu.OutputVariableType.AngularVelocityLocal))]
    
        sWheelPos+=[mbs0.AddSensor(SensorBody(bodyNumber=b0, 
                                     storeInternal=True,
                                     outputVariableType = exu.OutputVariableType.Position))]
    
        sRollPos+=[mbs0.AddSensor(SensorObject(objectNumber=oRolling, 
                                            storeInternal=True,
                                            outputVariableType = exu.OutputVariableType.Position))]
    
        sRollForce+=[mbs0.AddSensor(SensorObject(name='wheelForce'+strNum,objectNumber=oRolling, 
                                               storeInternal=True,
                                               outputVariableType = exu.OutputVariableType.ForceLocal))]


# #user function for time-dependent torque on two wheels 0,1
# def UFtorque(mbs0, t, torque):
#     if t < 2:
#         return torque
#     else:
#         return [0,0,0]

# mbs0.AddLoad(Torque(markerNumber=markerWheels[0],loadVector=[-2000,0,0], bodyFixed = True, loadVectorUserFunction=UFtorque))
# mbs0.AddLoad(Torque(markerNumber=markerWheels[1],loadVector=[-2000,0,0], bodyFixed = True, loadVectorUserFunction=UFtorque))


mbs0.Assemble()

tEnd = 0.5 #40#1.2
stepSize = 0.001 #for car only
if useGraphics:
    tEnd = 4

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
simulationSettings.solutionSettings.outputPrecision = 5 #make files smaller
simulationSettings.solutionSettings.exportAccelerations = False
simulationSettings.solutionSettings.exportVelocities = True
simulationSettings.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False #speeds up

simulationSettings.solutionSettings.writeSolutionToFile = False #don't write dynamic solver's solution
simulationSettings.solutionSettings.solutionWritePeriod = 0.01
simulationSettings.solutionSettings.coordinatesSolutionFileName = 'solution/test.txt'
#simulationSettings.displayComputationTime = True
#simulationSettings.displayStatistics = True
simulationSettings.timeIntegration.verboseMode = 1
simulationSettings.parallel.numberOfThreads = 1
simulationSettings.timeIntegration.generalizedAlpha.lieGroupAddTangentOperator = False #for lie group nodes
simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = False

simulationSettings.timeIntegration.newton.useModifiedNewton = True

SC.visualizationSettings.nodes.show = True
SC.visualizationSettings.nodes.drawNodesAsPoint  = False
SC.visualizationSettings.nodes.showBasis = False
SC.visualizationSettings.nodes.tiling = 4
SC.visualizationSettings.nodes.drawNodesAsPoint = False
SC.visualizationSettings.nodes.basisSize = 0.015

SC.visualizationSettings.loads.show = False

SC.visualizationSettings.window.renderWindowSize = [2000,1600]
SC.visualizationSettings.openGL.shadow = 0.25
SC.visualizationSettings.openGL.perspective = 1
SC.visualizationSettings.openGL.light0position = [20,-20,40,0]

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#%%explicit mbs1 for particles:
if doSimulatorCoupling:
    mbs1 = SC.AddSystem()


    #+++++++++++++++++++++++++++++++++++
    #contact parameters:
    fact = 2
    if fact <= 2:
        hGround*=2
        
    frictionCoeff = 0.05*2
    contactStiffness = 1e4/fact*400
    contactDamping = 0.001*contactStiffness
    stepSize = 0.5e-3/fact

    #+++++++++++++++++++++++++++++++++++
    #initialize GeneralContact
    distParticles = -0.25*lCar #distance to car
    rParticles = 0.25/fact

    particlesArea = np.array([wGround*0.2, lGround-2*lCar-distParticles, hGround])
    particlesOffset = np.array([0,
                                0.5*lGround+distParticles*0.5+0*lCar,
                                0.5*hGround-0*rWheel])
        
    stCells = (particlesArea * (0.3/rParticles)).astype(int)
    stCells = np.maximum(stCells, 1)
    
    gContact = mbs1.AddGeneralContact()
    gContact.verboseMode = 1
    gContact.SetFrictionPairings(frictionCoeff*np.eye(1))
    gContact.SetSearchTreeCellSize(numberOfCells=stCells)
    
    graphicsWheels = []
    graphicsWheels += [graphics.Cylinder(pAxis=[-0.5*wWheel+pWheel[0], pWheel[1],0],vAxis=[wWheel,0,0],radius=rWheel, color=graphics.color.darkgrey,nTiles=64)]
    graphicsWheels += [graphics.Cylinder(pAxis=[-0.5*wWheel+pWheel[0],-pWheel[1],0],vAxis=[wWheel,0,0],radius=rWheel, color=graphics.color.darkgrey,nTiles=64)]
    graphicsWheels += [graphics.Cylinder(pAxis=[-0.5*wWheel-pWheel[0], pWheel[1],0],vAxis=[wWheel,0,0],radius=rWheel, color=graphics.color.darkgrey,nTiles=64)]
    graphicsWheels += [graphics.Cylinder(pAxis=[-0.5*wWheel-pWheel[0],-pWheel[1],0],vAxis=[wWheel,0,0],radius=rWheel, color=graphics.color.darkgrey,nTiles=64)]
    
    #+++++++++++++++++++++++++++++++++++
    #copy of car, which is then mapped:
    bCarMbs1=mbs1.CreateRigidBody(inertia = inertiaCar, 
                             referencePosition = p0Car,
                             nodeType = exu.NodeType.RotationRotationVector,
                             referenceRotationMatrix = initialRotationCar,
                             initialAngularVelocity = omega0Car,
                             initialVelocity = v0Car,
                             #gravity = g, 
                             graphicsDataList = graphicsCar+graphicsShield+graphicsWheels)
    nCarMbs1 = mbs1.GetObject(bCarMbs1)['nodeNumber']
    
    mCarMbs1 = mbs1.AddMarker(MarkerBodyRigid(bodyNumber=bCarMbs1))
    [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(graphicsShield[0])
    gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mCarMbs1, 
                                        contactStiffness=contactStiffness, 
                                        contactDamping=contactDamping, 
                                        frictionMaterialIndex=0,
                                        pointList=meshPoints,  triangleList=meshTrigs)
    
    #+++++++++++++++++++++++++++++++++++
    #set up container for particles
    
    tWall = 0.1
    p0 = np.array([0,0.5*lGround-lCar,-0.5*tWall])
    hWall = 4*hGround

    color4wall = [0.6,0.6,0.6,0.5]
    addNormals = False
    gFloor =    graphics.Brick(p0,[wGround,lGround,tWall],graphics.color.steelblue,addNormals)
    gFloorAdd = graphics.Brick(p0+[-0.5*wGround,0,0.5*hWall],[tWall,lGround,hWall],color4wall,addNormals)
    gFloor =    graphics.MergeTriangleLists(gFloor, gFloorAdd)
    gFloorAdd = graphics.Brick(p0+[ 0.5*wGround,0,0.5*hWall],[tWall,lGround,hWall],color4wall,addNormals)
    gFloor =    graphics.MergeTriangleLists(gFloor, gFloorAdd)
    gFloorAdd = graphics.Brick(p0+[0,-0.5*lGround,0.5*hWall],[wGround,tWall,hWall],color4wall,addNormals)
    gFloor =    graphics.MergeTriangleLists(gFloor, gFloorAdd)
    gFloorAdd = graphics.Brick(p0+[0, 0.5*lGround,0.5*hWall],[wGround,tWall,hWall],color4wall,addNormals)
    gFloor =    graphics.MergeTriangleLists(gFloor, gFloorAdd)
    
    gDataList = [gFloor]
    
    
    nGround = mbs1.AddNode(NodePointGround(referenceCoordinates=[0,0,0] ))
    mGround = mbs1.AddMarker(MarkerNodeRigid(nodeNumber=nGround))
    
    [meshPoints, meshTrigs] = graphics.ToPointsAndTrigs(gFloor)
    gContact.AddTrianglesRigidBodyBased(rigidBodyMarkerIndex=mGround, 
                                        contactStiffness=contactStiffness, 
                                        contactDamping=contactDamping, 
                                        frictionMaterialIndex=0,
        pointList=meshPoints,  triangleList=meshTrigs)

    #+++++++++++++++++++++++++++++++++++
    #set up particles
    from exudyn.particles import CreateParticlesInBox
    
    particles = CreateParticlesInBox(minPointBox = particlesOffset - 0.5*particlesArea, 
                                     maxPointBox = particlesOffset + 0.5*particlesArea, 
                                     minRadius = 0.7*rParticles,
                                     maxRadius = rParticles,
                                     maxNumberOfParticles = 60000,#0000,
                                     verbose=1)

    for (p, r) in particles:
        colorInd = int(p[2]*10)%16

        density = 200 #kg/m^3
        mass = 4/3*pi*rParticles**3*density
        # mass = 100
        RBinertia = InertiaSphere(mass, r)
        massDict = mbs1.CreateRigidBody(inertia=RBinertia,
                                nodeType=exu.NodeType.RotationRotationVector,
                                #initialVelocity=[1,-1,0],
                                referencePosition=p,
                                gravity=[0,0,-9.81],
                                #graphicsDataList=[graphics.Sphere(radius=rParticles, 
                                #color=graphics.color.brown),
                                returnDict = True,
                                )
        nMass = massDict['nodeNumber']
        oMass = massDict['bodyNumber']
        mbs1.SetNodeParameter(nMass, 'VdrawSize', 2*rParticles)
        mbs1.SetNodeParameter(nMass, 'Vcolor', graphics.colorList[colorInd])
        mThis = mbs1.AddMarker(MarkerNodeRigid(nodeNumber=nMass))
    
        gContact.AddSphereWithMarker(mThis, radius=r, 
                                     contactStiffness=contactStiffness, 
                                     contactDamping=contactDamping,
                                     frictionMaterialIndex=0)
    
    oGround2 = mbs1.CreateGround(graphicsDataList=gDataList)

    #+++++++++++++++++++++++++++++++++++
    mbs1.Assemble()
    carMbs1ODE2index = mbs1.GetNodeODE2Index(nCarMbs1) #this is where we like to write coordinates

    solExplicit = 'solution/testExplicit.txt'
    
    #+++++++++++++++++++++++++++++++++++++++++
    #initialize implicit solver for vehicle
    simulationSettings1 = copy.copy(simulationSettings)
    simulationSettings1.timeIntegration.numberOfSteps = 1
    simulationSettings1.timeIntegration.endTime = stepSize
    simulationSettings1.solutionSettings.writeSolutionToFile = True
    simulationSettings1.solutionSettings.solutionWritePeriod = 1
    simulationSettings1.linearSolverType = exu.LinearSolverType.EigenSparse
    simulationSettings1.solutionSettings.outputPrecision = 5 #make files smaller
    simulationSettings1.solutionSettings.exportAccelerations = False
    simulationSettings1.solutionSettings.exportVelocities = True
    simulationSettings1.timeIntegration.explicitIntegration.computeEndOfStepAccelerations = False #speeds up
    
    simulationSettings1.solutionSettings.writeFileFooter = False
    simulationSettings1.solutionSettings.coordinatesSolutionFileName = solExplicit
    simulationSettings1.parallel.numberOfThreads = 1

    simulationSettings1.timeIntegration.explicitIntegration.dynamicSolverType = exu.DynamicSolverType.VelocityVerlet
    explicitSolver = exudyn.MainSolverExplicit()

    #+++++++++++++++++++++++++++++++++++++++++
    #initialize implicit solver for vehicle
    # tEnd = 1
    # simulationSettings.timeIntegration.numberOfSteps = int(1/stepSize)
    # simulationSettings.timeIntegration.endTime = 1
    simulationSettings.timeIntegration.numberOfSteps = 1
    simulationSettings.timeIntegration.endTime = stepSize #solve one step to initialize solver
    mbs0.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)
    implicitSolver = mbs0.sys['dynamicSolver']
    #+++++++++++++++++++++++++++++++++++++++++


    mbs0.SetRenderEngineStopFlag(False)
    mbs1.SetRenderEngineStopFlag(False)

    simulationSettings.timeIntegration.numberOfSteps = 1
    simulationSettings.timeIntegration.endTime = stepSize #solve one step to initialize solver

    #explicitSolver.InitializeSolverInitialConditions(mbs1, simulationSettings)

    if useGraphics:
        if False: #for testing only
            exu.StartRenderer()
            if 'renderState' in exu.sys:
                SC.SetRenderState(exu.sys['renderState'])



            # compare performance without simulator coupling:
            if True:
                mbs1.ActivateRendering()
                simulationSettings1.timeIntegration.numberOfSteps = int(1/stepSize)
                simulationSettings1.timeIntegration.endTime = 1
                explicitSolver.InitializeSolver(mbs1, simulationSettings1)
                explicitSolver.SolveSystem(mbs1, simulationSettings1)
                
            if False:
                import time
                start = time.time()
                explicitSolver.InitializeSolver(mbs1, simulationSettings1)
    
                for i in range(int(tEnd/stepSize)+1):
                    t = i * stepSize #end time
                    explicitSolver.output.verboseMode = i%100==0
        
                    explicitSolver.it.startTime = t
                    explicitSolver.it.endTime = t+stepSize
                    explicitSolver.SolveSteps(mbs1, simulationSettings)
        
                    #no effect on render window:
                    mbs0.systemData.SetTime(t, exu.ConfigurationType.Visualization) #for visualization
                    mbs0.systemData.SetTime(t, exu.ConfigurationType.Current) #for visualization
                    if i%200==0 or t==tEnd: 
                        mbs0.SendRedrawSignal()
                    stop = mbs1.GetRenderEngineStopFlag()
                    if stop: break
        
                    #print('t=',t)
                explicitSolver.FinalizeSolver(mbs1, simulationSettings)
                print('time spent=',time.time()-start)
                
                if useGraphics:
                    SC.WaitForRenderEngineStopFlag()
                    exu.StopRenderer() #safely close rendering window!
                
            sys.exit()
    
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#%%
explicitSolver.it.numberOfSteps = 1

explicitSolver.output.verboseMode = 0
simulationSettings1.solutionSettings.solutionWritePeriod = 0.02
simulationSettings1.solutionSettings.flushFilesImmediately = True
simulationSettings1.solutionSettings.writeInitialValues = False
simulationSettings1.solutionSettings.writeSolutionToFile = True
simulationSettings1.timeIntegration.verboseMode = 0
simulationSettings1.timeIntegration.automaticStepSize = False

#initialize solver and file writing:
mbs1.sys['simulationSettings'] = simulationSettings1
mbs1.sys['dynamicSolver'] = explicitSolver
explicitSolver.InitializeSolver(mbs1, simulationSettings1)
simulationSettings1.solutionSettings.writeFileHeader = False
simulationSettings1.solutionSettings.appendToFile = True

lastWritten = 0
stepCnt = 0
gContact.computeContactForces = True
contactForces = np.zeros(0) #initialize array
carContact = np.zeros(6)
#++++++++++++++++++++++++++++++++++++++
#couple to explicit solver:
def PreStepUserFunction(mbs, t):
    global mbs0, mbs1, explicitSolver, stepSize, simulationSettings1,\
        nCarMbs0, carMbs1ODE2index, lastWritten, contactForces, carContact, stepCnt
    #print('*t=',t)    
    # if t < stepSize*2:
    #     print('mbs0:\n',mbs0)
    #     print('mbs1:\n',mbs1)

    #+++++++++++++++++++++++++++++++++
    #retrieve state of car:
    u = mbs0.GetNodeOutput(nCarMbs0, exu.OutputVariableType.Coordinates)
    v = mbs0.GetNodeOutput(nCarMbs0, exu.OutputVariableType.Coordinates_t)
    sysDict = mbs1.systemData.GetSystemStateDict(reference=True)
    ode2=sysDict['ODE2Coords']
    ode2_t=sysDict['ODE2Coords_t']

    #+++++++++++++++++++++++++++++++++
    #apply state of car in mbs1 (using references ode2 and ode2_t)
    ode2[carMbs1ODE2index:carMbs1ODE2index+6] = u
    ode2_t[carMbs1ODE2index:carMbs1ODE2index+6] = v

    #only write every 0.01 seconds:
    explicitSolver.output.writeToSolutionFile = False
    if abs(t-lastWritten) >= 0.01-1e-7:
        explicitSolver.output.writeToSolutionFile = True
        lastWritten = t
        # print('car vel=',np.round(v,2))
        # print('carContact:',np.round(carContact,2))
        stepCnt+=1
        if stepCnt == 10:
            stepCnt = 0
            print('car vel=',np.round(v[:3],3))
            
    
    # # #solve one explicit step:
    explicitSolver.it.startTime = t-stepSize
    explicitSolver.it.endTime = t
    explicitSolver.SolveSteps(mbs1, simulationSettings1)

    #+++++++++++++++++++++++++++++++++
    if abs(t-stepSize) < 1e-8: #contact forces only exist after first step ...
        contactForces = gContact.GetSystemODE2RhsContactForces(copy=False) #contains forces at end of step

    carContact = contactForces[0:6]
    
    #note that carContact contains coordinate loads; 
    #but in this case, they agree with global force and local torque vector
    mbs0.SetLoadParameter(carParticlesForce, 'loadVector', carContact[0:3])
    mbs0.SetLoadParameter(carParticlesTorque, 'loadVector', carContact[3:6])
    
    return True

mbs0.SetPreStepUserFunction(PreStepUserFunction)


#++++++++++++++++++++++++++++++++++++++
#start implicit solver which calls explicit solver in every preStepUserFunction
if useGraphics:
    exu.StartRenderer()
    if 'renderState' in exu.sys:
        SC.SetRenderState(exu.sys['renderState'])
    #mbs0.WaitForUserToContinue()

simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize)
simulationSettings.timeIntegration.endTime = tEnd
#simulationSettings.parallel.numberOfThreads = 1

if True:
    mbs0.SolveDynamic(simulationSettings, solverType=exu.DynamicSolverType.TrapezoidalIndex2)
else:
    import time
    start = time.time()
    simulationSettings.timeIntegration.numberOfSteps = 1
    simulationSettings.timeIntegration.endTime = stepSize #solve one step to initialize solver

    for i in range(int(tEnd/stepSize)+1):
        t = i * stepSize #end time
        #print(t)
        explicitSolver.output.verboseMode = i%100==0
        # implicitSolver.output.verboseMode = i%100==0

        
        simulationSettings.timeIntegration.startTime = t
        simulationSettings.timeIntegration.endTime = t+stepSize
        simulationSettings.timeIntegration.numberOfSteps = 1
        mbs0.SolveDynamic(simulationSettings, 
                          solverType=exu.DynamicSolverType.TrapezoidalIndex2, 
                          updateInitialValues=True)

        #if i%200==0 or t==tEnd: mbs0.SendRedrawSignal()
        stop = mbs1.GetRenderEngineStopFlag()
        if stop: break

        #print('t=',t)

    #finalize solver and adjust solver parameters for solution viewer
    simulationSettings1.solutionSettings.writeSolutionToFile = True
    simulationSettings1.solutionSettings.writeFileFooter = False
    explicitSolver.FinalizeSolver(mbs1, simulationSettings)
    
    print('time spent=',time.time()-start)



if useGraphics:
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!

if True:
    #in order to see updates of displayed time, we need to make mbs1 to the master
    SC.DetachFromRenderEngine() #detach from current renderer
    SC2 = exu.SystemContainer()
    SC2.visualizationSettings = SC.visualizationSettings
    SC2.AppendSystem(mbs1)
    mbs1.SolutionViewer()

c=mbs0.GetNodeOutput(n0, variableType=exu.OutputVariableType.Coordinates)
u=sum(c)
exu.Print("simulatorCouplingTwoMbs: u=",u)

exudynTestGlobals.testError = u - 0.
exudynTestGlobals.testResult = u

##++++++++++++++++++++++++++++++++++++++++++++++q+++++++
#plot results
if useGraphics and False:
    
    
    mbs0.PlotSensor(sensorNumbers=sCarVel, components=[0,1,2], title='car velocitiy', closeAll=True)
    for i in range(4):
        mbs0.PlotSensor(sensorNumbers=sRollPos[i], componentsX=0, components=1, 
                   labels='wheel trail '+str(i), newFigure=(i==0), colorCodeOffset=i)
        #trail and wheel pos are almost same, just if car slightly tilts, there is a deviation
        mbs0.PlotSensor(sensorNumbers=sWheelPos[i], componentsX=0, components=1, 
                   labels='wheel pos '+str(i), newFigure=False, colorCodeOffset=i+7,
                   lineStyles='', markerStyles='x')
    
    mbs0.PlotSensor(sensorNumbers=sRollForce, components=[2]*4, title='wheel contact forces')

    mbs0.PlotSensor(sensorNumbers=sRollForce*2, components=[0]*4+[1]*4, title='wheel lateral (X) and drive/acceleration (Y) forces')

    mbs0.PlotSensor(sensorNumbers=sAngVels, components=[0]*4, title='wheel local angular velocity')

