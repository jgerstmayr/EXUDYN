#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test model for cable with contact and friction; test model for ObjectContactFrictionCircleCable2D,
#           which models frictional contact between 2D ANCF element and circular object, using contact and stick-slip friction;
#
# Author:   Johannes Gerstmayr
# Date:     2019-08-15 (created)
# Date:     2022-07-20 (last modified)
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


#background
rect = [-0.5,-1,2.5,1] #xmin,ymin,xmax,ymax
background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
background1 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[0,-1,0, 2,-1,0]} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0, background1])))

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#cable:
mypi = 3.141592653589793

L=2                     # length of ANCF element in m
#L=mypi                 # length of ANCF element in m
E=2.07e11               # Young's modulus of ANCF element in N/m^2
rho=7800                # density of ANCF element in kg/m^3
b=0.001*10                 # width of rectangular ANCF element in m
h=0.001*10                 # height of rectangular ANCF element in m
A=b*h                   # cross sectional area of ANCF element in m^2
I=b*h**3/12             # second moment of area of ANCF element in m^4
f=3*E*I/L**2            # tip load applied to ANCF element in N

exu.Print("load f="+str(f))
exu.Print("EI="+str(E*I))

nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

cableList=[]        #for cable elements
nodeList=[]  #for nodes of cable
markerList=[]       #for nodes

#%%+++++++++++++++++++++
#create nodes and cable elements; 
#alternatively, GenerateStraightLineANCFCable2D from exudyn.beams could be used
nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
nodeList+=[nc0]
nElements = 8 #32*4
lElem = L / nElements
for i in range(nElements):
    nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
    nodeList+=[nLast]
    elem=mbs.AddObject(Cable2D(physicsLength=lElem, physicsMassPerLength=rho*A, 
                               physicsBendingStiffness=E*I, physicsAxialStiffness=E*A*0.1, 
                               physicsBendingDamping=E*I*0.025*0, physicsAxialDamping=E*A*0.1, 
                               nodeNumbers=[int(nc0)+i,int(nc0)+i+1]))
    cableList+=[elem]

#%%+++++++++++++++++++++
mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3))
    
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))

#add gravity:
markerList=[]
for i in range(len(nodeList)):
    m = mbs.AddMarker(MarkerNodePosition(nodeNumber=nodeList[i])) 
    markerList+=[m]
    fact = 1 #add (half) weight of two elements to node
    if (i==0) | (i==len(nodeList)-1): fact = 0.5 # first and last node only weighted half
    mbs.AddLoad(Force(markerNumber = m, loadVector = [0, -0.1*400*rho*A*fact*lElem, 0])) #will be changed in load steps


cStiffness = 1e3
cDamping = 0.02*cStiffness*2
r1 = 0.1
r2 = 0.3

posRoll1 = [0.25*L,-0.15,0]
posRoll2 = [0.75*L,-0.5,0]

useFriction = 1
useCircleContact = True
if useCircleContact:
    nSegments = 2*4 #4; number of contact segments; must be consistent between nodedata and contact element
    initialGapList = [0.1]*nSegments #initial gap of 0.1
    if (useFriction):
        initialGapList += [0]*(2*nSegments)

    mGroundCircle = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oGround, localPosition=posRoll1)) 
    mGroundCircle2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oGround, localPosition=posRoll2)) 


    rGraphics = GraphicsDataRectangle(0.,0.,0.1*r2,r2)
    vRigidBody = VObjectRigidBody2D(graphicsData = [rGraphics])
    nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=posRoll2))
    oRigid = mbs.AddObject(RigidBody2D(nodeNumber = nRigid, physicsMass = 1, physicsInertia=0.001, visualization=vRigidBody))
    mRigid = mbs.AddMarker(MarkerBodyRigid(bodyNumber = oRigid, localPosition=[0,0,0]))
    # mRigid = mbs.AddMarker(MarkerNodeRigid(nodeNumber = nRigid)) #gives identical result

    mbs.AddLoad(Torque(markerNumber=mRigid, loadVector=[0,0,-0.1]))

    #fix position of roll:
    for i in range(0,2):
        mRigidC = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nRigid, coordinate=i))
        #mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mRigidC]))
        mbs.AddObject(CoordinateSpringDamper(markerNumbers=[mGround,mRigidC], stiffness = 100, damping=5, visualization= VObjectConnectorCoordinateSpringDamper(show = False)))


    for i in range(len(cableList)):
        #exu.Print("cable="+str(cableList[i]))
        mCable = mbs.AddMarker(MarkerBodyCable2DShape(bodyNumber=cableList[i], numberOfSegments = nSegments))

        nodeDataContactCable = mbs.AddNode(NodeGenericData(initialCoordinates=initialGapList,numberOfDataCoordinates=nSegments*(1+2*useFriction)))
        mbs.AddObject(ObjectContactFrictionCircleCable2D(markerNumbers=[mRigid, mCable], nodeNumber = nodeDataContactCable, 
                                                 numberOfContactSegments=nSegments, contactStiffness = cStiffness, contactDamping=cDamping, 
                                                 frictionVelocityPenalty = 10, frictionCoefficient=2,
                                                 useSegmentNormals=False, #for this test
                                                 circleRadius = r2))

#%%++++++++++++++++++++++++++++++++++
#finally assemble and start computation
mbs.Assemble()
#exu.Print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values

fact = 300
simulationSettings.timeIntegration.numberOfSteps = fact
simulationSettings.timeIntegration.endTime = 0.0005*fact
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/fact
#simulationSettings.solutionSettings.outputPrecision = 4
#simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8 #10000
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10*100

#simulationSettings.timeIntegration.discontinuous.maxIterations = 5
# simulationSettings.timeIntegration.discontinuous.iterationTolerance = 0.001 #with 

simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.timeIntegration.newton.maxModifiedNewtonIterations = 5
#simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
#simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 6.055454452393343e-06*0.1 #eps^(1/3)
simulationSettings.timeIntegration.newton.modifiedNewtonContractivity = 1e8
simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 
simulationSettings.displayStatistics = True #just in this example ...

SC.visualizationSettings.bodies.showNumbers = False
SC.visualizationSettings.nodes.defaultSize = 0.01
SC.visualizationSettings.markers.defaultSize = 0.01
SC.visualizationSettings.connectors.defaultSize = 0.01
SC.visualizationSettings.contact.contactPointsDefaultSize = 0.005
SC.visualizationSettings.connectors.showContact = 1

simulationSettings.solutionSettings.solutionInformation = "ANCF cable with rigid contact"

# useGraphics=False
if useGraphics: 
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

solveDynamic = True
if solveDynamic: 

    mbs.SolveDynamic(simulationSettings)

    sol = mbs.systemData.GetODE2Coordinates()
    u = sol[len(sol)-3]
    exu.Print('tip displacement: y='+str(u))
else:
    simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-10*100 #can be quite small; WHY?
    simulationSettings.staticSolver.verboseMode = 1
    simulationSettings.staticSolver.numberOfLoadSteps  = 20*2
    simulationSettings.staticSolver.loadStepGeometric = False;
    simulationSettings.staticSolver.loadStepGeometricRange = 5e3;

    simulationSettings.staticSolver.newton.relativeTolerance = 1e-5*100 #10000
    simulationSettings.staticSolver.newton.absoluteTolerance = 1e-10
    simulationSettings.staticSolver.newton.maxIterations = 30 #50 for bending into circle

    simulationSettings.staticSolver.discontinuous.iterationTolerance = 0.1
    #simulationSettings.staticSolver.discontinuous.maxIterations = 5
    simulationSettings.staticSolver.pauseAfterEachStep = False
    simulationSettings.staticSolver.stabilizerODE2term = 50 

    mbs.SolveStatic(simulationSettings)

    sol = mbs.systemData.GetODE2Coordinates()
    n = len(sol)
    u=sol[n-4]
    exu.Print('static tip displacement: x='+str(sol[n-4])+', y='+str(sol[n-3])) 

#put outside if
exudynTestGlobals.testError = u - (-0.014187561328096003) #until 2022-03-09 (old ObjectContactFrictionCircleCable2D): -0.014188649931870346   #2019-12-26: -0.014188649931870346; 2019-12-16: (-0.01418281035370442);
exudynTestGlobals.testResult = u
exu.Print("test result=",exudynTestGlobals.testResult)

if useGraphics: 
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

