#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  A rigid body sliding along a chain of `ANCFCable2D` elements with an
#           `ObjectJointSliding2D`, which moves the contact from one element to the next as it goes:
#           the vertical position of its centre of mass after a short dynamic step.
#           The model compares against a reference value written into it, so its
#           result is that difference and its reference solution is 0 (#2632).
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-01, reworked 2026-09-24
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics

testIsActive = exu.sys.get('testIsActive', False)
exu.sys['testTolerance'] = 4e-13 #the tolerance RunAllModelUnitTests used for these ten

SC = exu.SystemContainer()
mbs = SC.AddSystem()


#background
rect = [-2.5,-2,2.5,1] #xmin,ymin,xmax,ymax
background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
background1 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[0,-1,0, 2,-1,0]} #background
oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0])))


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#cable:
mypi = 3.141592653589793

L=2                     # length of ANCF element in m
#L=mypi                 # length of ANCF element in m
E=2.07e11               # Young's modulus of ANCF element in N/m^2
rho=7800                # density of ANCF element in kg/m^3
b=0.001                 # width of rectangular ANCF element in m
h=0.001                 # height of rectangular ANCF element in m
A=b*h                   # cross sectional area of ANCF element in m^2
I=b*h**3/12             # second moment of area of ANCF element in m^4
f=3*E*I/L**2            # tip load applied to ANCF element in N
g=9.81

exu.Print("load f="+str(f))
exu.Print("EI="+str(E*I))

nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

cableList=[]        #for cable elements
nodeList=[]  #for nodes of cable
markerList=[]       #for nodes
nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
nodeList+=[nc0]
nElements = 3
lElem = L / nElements
for i in range(nElements):
    nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
    nodeList+=[nLast]
    elem=mbs.AddObject(Cable2D(length=lElem, massPerLength=rho*A, 
                               bendingStiffness=E*I, axialStiffness=E*A, 
                               nodeNumbers=[int(nc0)+i,int(nc0)+i+1]))
    cableList+=[elem]
    mBody = mbs.AddMarker(MarkerBodyMass(bodyNumber = elem))
    mbs.AddLoad(Gravity(markerNumber=mBody, loadVector=[0,-g,0]))

mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3))

mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))


a = 0.1     #y-dim/2 of gondula
b = 0.001    #x-dim/2 of gondula
massRigid = 12*0.01
inertiaRigid = massRigid/12*(2*a)**2

slidingCoordinateInit = lElem*1.5 #0.75*L
initialLocalMarker = 1 #second element
if nElements<2:
    slidingCoordinateInit /= 3.
    initialLocalMarker = 0

addRigidBody = True
nRigid = -1
if addRigidBody:
    #rigid body which slides:
    graphicsRigid = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-b,-a,0, b,-a,0, b,a,0, -b,a,0, -b,-a,0]} #drawing of rigid body
    nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[slidingCoordinateInit,-a,0], initialVelocities=[0,0,0]));
    oRigid = mbs.AddObject(RigidBody2D(mass=massRigid, inertia=inertiaRigid,nodeNumber=nRigid,visualization=VObjectRigidBody2D(graphicsData= [graphicsRigid])))

    markerRigidTop = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[0.,a,0.])) #support point
    mR2 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0.,0.,0.])) #center of mass (for load)

    mbs.AddLoad(Force(markerNumber = mR2, loadVector = [massRigid*g*0.1, -massRigid*g, 0]))


#slidingJoint:
addSlidingJoint = True
if addSlidingJoint:
    cableMarkerList = []#list of Cable2DCoordinates markers
    offsetList = []     #list of offsets counted from first cable element; needed in sliding joint
    offset = 0          #first cable element has offset 0
    for item in cableList: #create markers for cable elements
        m = mbs.AddMarker(MarkerBodyCable2DCoordinates(bodyNumber = item))
        cableMarkerList += [m]
        offsetList += [offset]
        offset += lElem

    #mGroundSJ = mbs.AddMarker(MarkerBodyPosition(bodyNumber = oGround, localPosition=[0.*lElem+0.75*L,0.,0.])) 
    nodeDataSJ = mbs.AddNode(NodeGenericData(initialCoordinates=[initialLocalMarker,slidingCoordinateInit],numberOfDataCoordinates=2)) #initial index in cable list
    slidingJoint = mbs.AddObject(ObjectJointSliding2D(name='slider', markerNumbers=[markerRigidTop,cableMarkerList[initialLocalMarker]], 
                                                      slidingMarkerNumbers=cableMarkerList, slidingMarkerOffsets=offsetList, 
                                                      nodeNumber=nodeDataSJ, useClassicalFormulation = False))


mbs.Assemble()

simulationSettings = exu.SimulationSettings() #takes currently set values or default values
simulationSettings.solution.file.write=False

fact = 200
simulationSettings.timeIntegration.numberOfSteps = 1*fact
simulationSettings.timeIntegration.endTime = 0.001*fact*0.5
simulationSettings.solution.file.write = True
simulationSettings.solution.file.writePeriod = simulationSettings.timeIntegration.endTime/fact
simulationSettings.timeIntegration.verboseMode = 1

simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8*100 #10000
simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10*100

simulationSettings.timeIntegration.newton.useModifiedNewton = False
simulationSettings.timeIntegration.newton.maxModifiedNewtonIterations = 5
simulationSettings.timeIntegration.newton.numericalDifferentiation.addReferenceCoordinatesToEpsilon = False
simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1.e-3
simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-8 #6.055454452393343e-06*0.0001 #eps^(1/3)
simulationSettings.timeIntegration.newton.modifiedNewtonContractivity = 1e8
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 
simulationSettings.show.statistics = False

if not testIsActive: 
    SC.renderer.Start()

mbs.SolveDynamic(simulationSettings)


error = 0
if nRigid != -1:
    u = mbs.GetNodeOutput(nRigid, exu.OutputVariableType.Position) #tip node
    if True: 
        error = u[1] - (-0.14920183348514676 ) #2021-09-27: new JacobianODE2RHS
        #error = u[1] -(-0.14920182499994944 ) #2021-02-06: -0.14920182499994944 (1e-9 different from old solver) 
    elif True:
        error = u[1] - (-0.14920182666080586) #2021-02-04: -0.14920182666080586
    else:
        error = u[1] - (-0.14920151345936586) #2019-12-26: -0.14920151345936586; 15.12.2019: (-0.1489879442762764); before 15.12.2019: (-0.14898795617249422) #2019-11-22; #20.10.2019: (-0.1489879501348149); 17.10.2019:-0.14898795468724652; old? :(-0.14898795002032308) #old error before projected sliding joint: (-0.14898792622401774) #y-position of COM of sliding body
                    
    exu.Print('value SlidingJoint2DTest=',u[1])
    #exu.Print('error SlidingJoint2DTest=',error)

if not testIsActive: 
    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

testResult = abs(error)
exu.Print('solution of SlidingJoint2DTest=', testResult)
exu.sys['testResult'] = testResult
