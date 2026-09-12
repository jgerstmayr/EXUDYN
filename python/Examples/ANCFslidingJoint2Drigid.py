#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  ANCF Cable2D element with sliding joint test
#
# Author:   Johannes Gerstmayr
# Date:     2019-09-15
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.itemInterface import *

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

print("load f="+str(f))
print("EI="+str(E*I))

nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

cableList=[]        #for cable elements
nodeList=[]  #for nodes of cable
markerList=[]       #for nodes
nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
nodeList+=[nc0]
nElements = 32
lElem = L / nElements
for i in range(nElements):
    nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
    nodeList+=[nLast]
    elem=mbs.AddObject(Cable2D(physicsLength=lElem, physicsMassPerLength=rho*A, 
                               physicsBendingStiffness=E*I, physicsAxialStiffness=E*A, nodeNumbers=[int(nc0)+i,int(nc0)+i+1]))
    cableList+=[elem]
    mBody = mbs.AddMarker(MarkerBodyMass(bodyNumber = elem))
    mbs.AddLoad(Gravity(markerNumber=mBody, loadVector=[0,-g,0]))

addPointMass = False
if addPointMass:
    massTip = 0.01 #tip mass
    nMass = mbs.AddNode(Point2D(referenceCoordinates=[L,0],visualization=VNodePoint2D(drawSize=0.3)))
    mTip0 = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
    mTip1 = mbs.AddMarker(MarkerNodePosition(nodeNumber=nLast))
    mbs.AddObject(MassPoint2D(physicsMass = massTip, nodeNumber=nMass))
    mbs.AddLoad(Force(markerNumber=mTip0, loadVector=[0,-massTip*g,0]))
    mbs.AddObject(RevoluteJoint2D(markerNumbers=[mTip0,mTip1]))


mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = int(nc0)+1*0, coordinate=0))
mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = int(nc0)+1*0, coordinate=1))
mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = int(nc0)+1*0, coordinate=3))
    
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))

#mANCF3 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nLast, coordinate=1))
#mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF3]))
#mANCF4 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nLast, coordinate=0))
#mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF4]))

#add gravity:
markerList=[]
for i in range(len(nodeList)):
    m = mbs.AddMarker(MarkerNodePosition(nodeNumber=nodeList[i])) 
    markerList+=[m]
    #fact = 1 #add (half) weight of two elements to node
    #if (i==0) | (i==len(nodeList)-1): 
    #    fact = 0.5 # first and last node only weighted half
    #mbs.AddLoad(Force(markerNumber = m, loadVector = [0., -rho*A*fact*lElem*g, 0])) #will be changed in load steps

a = 0.1     #y-dim/2 of gondula
b = 0.001    #x-dim/2 of gondula
massRigid = 12*0.01
inertiaRigid = massRigid/12*(2*a)**2
g = 9.81    # gravity

slidingCoordinateInit = lElem*1.5 #0.75*L
initialLocalMarker = 1 #second element
if nElements<2:
    slidingCoordinateInit /= 3.
    initialLocalMarker = 0

addRigidBody = True
if addRigidBody:
    #rigid body which slides:
    graphicsRigid = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-b,-a,0, b,-a,0, b,a,0, -b,a,0, -b,-a,0]} #drawing of rigid body
    nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[slidingCoordinateInit,-a,0], initialVelocities=[0,0,0]));
    oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,visualization=VObjectRigidBody2D(graphicsData= [graphicsRigid])))

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
                                                      nodeNumber=nodeDataSJ))


mbs.Assemble()
print(mbs)

simulationSettings = exu.SimulationSettings() #takes currently set values or default values
#simulationSettings.solutionSettings.coordinatesSolutionFileName = 'ANCFCable2Dbending' + str(nElements) + '.txt'

h=5e-4
tEnd = 0.6
simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)
simulationSettings.timeIntegration.endTime = tEnd
simulationSettings.solutionSettings.writeSolutionToFile = True
simulationSettings.solutionSettings.solutionWritePeriod = h
#simulationSettings.solutionSettings.outputPrecision = 4
simulationSettings.displayComputationTime = True
simulationSettings.timeIntegration.verboseMode = 1

# simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8*100 #10000
# simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-10*100

simulationSettings.timeIntegration.newton.useModifiedNewton = True
simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 #0.6 works well 
simulationSettings.pauseAfterEachStep = False
simulationSettings.displayStatistics = True

#SC.visualizationSettings.nodes.showNumbers = True
SC.visualizationSettings.bodies.showNumbers = False
SC.visualizationSettings.loads.show = False
#SC.visualizationSettings.connectors.showNumbers = True
SC.visualizationSettings.nodes.defaultSize = 0.01
SC.visualizationSettings.markers.defaultSize = 0.01
SC.visualizationSettings.connectors.defaultSize = 0.01
SC.visualizationSettings.contact.contactPointsDefaultSize = 0.005
SC.visualizationSettings.connectors.showContact = 1

simulationSettings.solutionSettings.solutionInformation = "ANCF cable with imposed curvature or applied tip force/torque"

solveDynamic = True
if solveDynamic: 
    SC.renderer.Start()
    SC.renderer.DoIdleTasks()

    mbs.SolveDynamic(simulationSettings)

    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!

else:
    simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-10*100 #can be quite small; WHY?
    simulationSettings.staticSolver.newton.numericalDifferentiation.doSystemWideDifferentiation = False
    simulationSettings.staticSolver.newton.useNumericalDifferentiation = False
    simulationSettings.staticSolver.verboseMode = 3
    simulationSettings.staticSolver.numberOfLoadSteps  = 20*2
    simulationSettings.staticSolver.loadStepGeometric = False;
    simulationSettings.staticSolver.loadStepGeometricRange = 5e3;

    simulationSettings.staticSolver.newton.relativeTolerance = 1e-5*100 #10000
    simulationSettings.staticSolver.newton.absoluteTolerance = 1e-10
    simulationSettings.staticSolver.newton.maxIterations = 30 #50 for bending into circle

    simulationSettings.staticSolver.discontinuous.iterationTolerance = 0.1
    #simulationSettings.staticSolver.discontinuous.maxIterations = 5
    simulationSettings.staticSolver.pauseAfterEachStep = False
    simulationSettings.staticSolver.stabilizerODE2term = 100

    SC.renderer.Start()

    mbs.SolveStatic(simulationSettings)

    #sol = mbs.systemData.GetODE2Coordinates()
    #n = len(sol)
    #print('tip displacement: x='+str(sol[n-4])+', y='+str(sol[n-3])) 

    SC.renderer.DoIdleTasks()
    SC.renderer.Stop() #safely close rendering window!





#class MyDialog:
#    def __init__(self, parent):
#        top = self.top = Toplevel(parent)
#        Label(top, text="Value").pack()
#        self.e = Entry(top)
#        self.e.pack(padx=5)
#        b = Button(top, text="OK", command=self.ok)
#        b.pack(pady=5)
#    def ok(self):
#        #print("value is " + self.e.get())
#        exec(self.e.get())
#        self.top.destroy()

#root = Tk()
#Button(root, text="Exudyn").pack()
#root.update()
#d = MyDialog(root)
#root.wait_window(d.top)

