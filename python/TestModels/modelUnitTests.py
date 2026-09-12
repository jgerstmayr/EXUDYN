#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This file contains a collection of smaller, old examples for the test suite
# it contains the older interface for test suite functions, now replaced by standalone .py tests in TestModels
#
# Author:   Johannes Gerstmayr
# Date:     2019-11-01
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
import exudyn.graphics as graphics #only import if it does not conflict 

#general test class for test of functions; 
#exudyn ... must contain the exudyn module (exu)
#systemContainer ... must contain a valid system container (SC) within exu
#useGraphics ... if true, the computations are visualized and a user interaction is needed after every test
class TestInterface:
    def __init__(self, exudyn, systemContainer, useGraphics=False):
        self.exu = exudyn
        self.SC = systemContainer
        self.useGraphics = useGraphics
        self.useCorrectedAccGenAlpha = True #always corrected
        self.useNewGenAlphaSolver = True    #active by default

#this class is for interaction of test suite with examples given as (autonomous) .py file
class ExudynTestStructure:
    def __init__(self, useGraphics = True, performTests = False, testError = 0, 
                 testResult = 0, testTolFact = 1, isPerformanceTest = False):
        self.useGraphics = useGraphics
        self.testError = testError      #for regular test models (store reference solution inside)
        self.testError = testError      #for regular test models (store reference solution inside)
        self.testTolFact = testTolFact  #additional factor to raise tolerance
        self.performTests = performTests #this variable is only used for testing if example is calculated outside test mode
        self.isPerformanceTest = isPerformanceTest #only true for performance tests; allows to use test both for error and performance test

        self.useCorrectedAccGenAlpha = True  #always corrected
        self.useNewGenAlphaSolver = True    #active by default
        
exudynTestGlobals = ExudynTestStructure() #variable used as global variable during testing


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#test 1 graphicsData and Assemble without unkowns; test background graphics on ground object
def GraphicsDataTest(mbs, testInterface):
    rect = [-2.5,-2,2.5,1] #xmin,ymin,xmax,ymax
    background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
    graphicsRigid1 ={'type':'Circle', 'color':[1,0.1,0.8,1], 'position':[1,2,0], 'radius': 1}  

    oGround = mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0,graphicsRigid1])))

    mbs.Assemble()

    if testInterface.useGraphics: 
        testInterface.testinterface.SC.renderer.Start()
        testInterface.SC.renderer.DoIdleTasks()
        testInterface.SC.renderer.Stop() #safely close rendering window!

    return 0 #this test only fails (crashes) but does not give an error value



#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Test 2 ANCF cable
def ANCFCable2DBendingTest(mbs, testInterface): 

    totalError = 0 #total error computed in test

    L=2                    # length of ANCF element in m
    Em=2.07e11             # Young's modulus of ANCF element in N/m^2
    rho=7800               # density of ANCF element in kg/m^3
    b=0.1                  # width of rectangular ANCF element in m
    h=0.1                  # height of rectangular ANCF element in m
    A=b*h                  # cross sectional area of ANCF element in m^2
    I=b*h**3/12            # second moment of area of ANCF element in m^4
    f=3*Em*I/L**2           # tip load applied to ANCF element in N

    EI = Em*I
    rhoA = rho*A
    EA = Em*A


    nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))

    nElements = 1
    lElem = L / nElements

    for i in range(nElements):
        nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
        elem = mbs.AddObject(Cable2D(physicsLength=lElem, physicsMassPerLength=rhoA, 
                                     physicsBendingStiffness=EI, physicsAxialStiffness=EA, 
                                     nodeNumbers=[int(nc0)+i,int(nc0)+i+1]))

    #tip node / force
    mANCFnode = mbs.AddMarker(MarkerNodePosition(nodeNumber=nLast))
    mbs.AddLoad(Force(markerNumber = mANCFnode, loadVector = [0, -f, 0]))


    #ground node / coordinate
    nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action; coordinate number does not matter
    
    #constraints
    mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
    mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
    mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3)) #3
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))
    #mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0],velocityLevel=True))
    #mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1],velocityLevel=True))
    #mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2],velocityLevel=True))

    #Assemble and Solve:
    mbs.Assemble()

    simulationSettings = testInterface.exu.SimulationSettings() #takes currently set values or default values
    simulationSettings.solutionSettings.coordinatesSolutionFileName = "solution/ANCFCable2D_bending_test.txt"
    simulationSettings.solutionSettings.writeSolutionToFile=False
    simulationSettings.timeIntegration.numberOfSteps = 1000
    #simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/1000
    simulationSettings.timeIntegration.endTime = 0.1
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.timeIntegration.newton.useModifiedNewton = False
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6
    simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
    simulationSettings.displayStatistics = True

    if testInterface.useGraphics: 
        testInterface.SC.renderer.Start()

    mbs.SolveDynamic(simulationSettings)

    sol = mbs.systemData.GetODE2Coordinates(); n = len(sol)
    #tip displacements:
    u = sol[n-4];v = sol[n-3] #15.12.2019:(-1.040678127615946 -1.444419986874761); 20.10.2019: (-1.0406781266430292 -1.4444199866881322); 17.10.2019: sol= -1.040678126647053 -1.4444199866858678; #28.7.2009: -1.040678126643273, -1.444419986688082
    if testInterface.useCorrectedAccGenAlpha:
        totalError += u+v - (-1.0611305415798655 -1.4396907464589017)  #2021-09-27: new JacobianODE2RHS
        #totalError += u+v -(-1.0611305415779122 -1.4396907464583975)  #2021-02-04: (-1.0611305415779122 -1.4396907464583975)
    else:
        totalError += u+v - (-1.0391620828192676 -1.4443521331339881)  #2019-12-26: (-1.0391620828192676 -1.4443521331339881) old (before correct initial accelerations): (-1.040678127615946 -1.444419986874761)
    testInterface.exu.Print('sol dynamic=',u,v)
    #testInterface.exu.Print('time integration error =',totalError)

    mbs.SolveStatic(simulationSettings)

    sol = mbs.systemData.GetODE2Coordinates(); n = len(sol)
    u = sol[n-4]; v = sol[n-3]; #20.10.2019: -0.3622447299987188 -0.9941447593196007; 17.10.2019: sol= -0.3622447299990847 -0.9941447593206921; #28.7.2019: -0.3622447300008477, -0.994144759326213
    testInterface.exu.Print('sol static (standardTol)=',u,v)
    totalError += u+v - (-0.36224473018839665 -0.9941447595447153) #2021-09-27: new JacobianODE2RHS
    # totalError += u+v-(-0.3622447299987188 -0.9941447593196007)

    simulationSettings.staticSolver.newton.relativeTolerance = 1e-14 #in order to converge to MATLAB results
    simulationSettings.staticSolver.newton.absoluteTolerance = 1e-14

    mbs.SolveStatic(simulationSettings)

    sol = mbs.systemData.GetODE2Coordinates(); n = len(sol)
    #tip displacements: paper GerstmIschrik2008: 1Element: u=-0.362244729891,  v=-0.994144758725; 4 Elements: 0.507428715119 1.205533702233
    u = sol[n-4]; v = sol[n-3];                 #2019-12-17: -0.3622447298904951 -0.9941447587249616
    testInterface.exu.Print('sol static (tol=1e-14)=',u,v)
    totalError += u+v - (-0.36224472989050654 -0.9941447587249747) #2021-09-27: new JacobianODE2RHS
    # totalError += u+v-(-0.3622447298904951 -0.9941447587249616)

    
    #totalError -= -1.3563894893270607-2.4850981133313548 #reference solution with one element and standard settings except: gen-alpha=0.6, useModifiedNewton = False

    if testInterface.useGraphics: 
        testInterface.SC.renderer.DoIdleTasks()
        testInterface.SC.renderer.Stop() #safely close rendering window!

    return totalError


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#spring-damper test3
def SpringDamperMesh(mbs, testInterface):
    nBodies = 6
    nBodies2 = 3

    for j in range(nBodies2): 
        body = mbs.AddObject({'objectType': 'Ground', 'referencePosition': [0,j,0]})
        mbs.AddMarker({'markerType': 'BodyPosition',  'bodyNumber': body,  'localPosition': [0.0, 0.0, 0.0], 'bodyFixed': False})
        for i in range(nBodies-1): 
            #does not work for analytical jacobian in static case: z-coordinate unconstrained
            # node = mbs.AddNode({'nodeType': 'Point','referenceCoordinates': [i+1, j, 0.0],
            #                     'initialCoordinates': [(i+1)*0.05*0, 0.0, 0.0], 
            #                     'initialVelocities': [0., 0., 0.],})
            # body = mbs.AddObject({'objectType': 'MassPoint', 'physicsMass': 10, 'nodeNumber': node})
            node = mbs.AddNode(NodePoint2D(referenceCoordinates=[i+1, j],
                                           initialCoordinates=[(i+1)*0.05*0, 0]))
            body = mbs.AddObject(ObjectMassPoint2D(physicsMass= 10, nodeNumber= node))
            mbs.AddMarker({'markerType': 'BodyPosition',  'bodyNumber': body,  'localPosition': [0.0, 0.0, 0.0], 'bodyFixed': False})

    #add spring-dampers:
    for j in range(nBodies2-1): 
        for i in range(nBodies-1): 
            mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                            'referenceLength':1, 'markerNumbers': [j*nBodies + i,j*nBodies + i+1]})
            mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                            'referenceLength':1, 'markerNumbers': [j*nBodies + i,(j+1)*nBodies + i]})
            #diagonal spring: l*sqrt(2)
            mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                            'referenceLength':sqrt2, 'markerNumbers': [j*nBodies + i,(j+1)*nBodies + i+1]})

    for i in range(nBodies-1): 
        j = nBodies2-1
        mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                        'referenceLength':1, 'markerNumbers': [j*nBodies + i,j*nBodies + i+1]})
    for j in range(nBodies2-1): 
        i = nBodies-1
        mbs.AddObject({'objectType': 'ConnectorSpringDamper', 'stiffness': 4000, 'damping': 10, 'force': 0,
                        'referenceLength':1, 'markerNumbers': [j*nBodies + i,(j+1)*nBodies + i]})

    #add loads:
    mbs.AddLoad({'loadType': 'ForceVector',  'markerNumber': nBodies*nBodies2-1,  'loadVector': [0, -50*2, 0]})

    #add constraints for testing:
    nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[-0.5,0,0])) #ground node for coordinate constraint
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action

    mNC1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = 1, coordinate=1))
    mNC2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = 2, coordinate=1))

    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mNC1]))

    mbs.Assemble()

    if testInterface.useGraphics: 
        testInterface.SC.renderer.Start()

    simulationSettings = testInterface.exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = 100
    simulationSettings.timeIntegration.endTime = 1
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6
    simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False
    simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False
    simulationSettings.timeIntegration.verboseMode = 0
    simulationSettings.timeIntegration.newton.useModifiedNewton = True
    simulationSettings.displayStatistics = True
    simulationSettings.solutionSettings.writeSolutionToFile=False

    testInterface.SC.visualizationSettings.nodes.defaultSize = 0.05

#    if testInterface.useGraphics: 
#        testInterface.SC.renderer.DoIdleTasks()

    mbs.SolveDynamic(simulationSettings)

    if testInterface.useGraphics: 
        testInterface.SC.renderer.DoIdleTasks()

    u = mbs.GetNodeOutput(nBodies-2, testInterface.exu.OutputVariableType.Position) #tip node
    testInterface.exu.Print('dynamic tip displacement (y)=', u[1])
    if testInterface.useCorrectedAccGenAlpha:
        dynamicError = u[1]-(-0.6385807469187298 )  #2021-02-04: -0.6385807469187298 
    else:
        dynamicError = u[1]-(-0.6383785907891227)  #2019-12-26: -0.6383785907891227; 2019-12-15: (-0.6349442849103891); before 15.12.2019: (-0.6349442850473246)
       
    #dynamic tip displacement for                                             s=1000: -0.6386766492418571,s=10000: -0.6386985511667669, s=100000: -0.6387006546281098
    #dynamic tip displacement for index2 Newmark: s=100: -0.6386060431598312, s=1000: -0.638699952155624, s=10000: -0.6387008780175608, s=100000: -0.6387008872717617


    #simulationSettings.solutionSettings.coordinatesSolutionFileName = "staticSolution.txt"
    #simulationSettings.solutionSettings.appendToFile = False
    simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-5
    simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
    simulationSettings.staticSolver.newton.absoluteTolerance = 1e-1
    simulationSettings.staticSolver.newton.numericalDifferentiation.forODE2connectors = True #be compatible with old solution
    #simulationSettings.staticSolver.verboseMode = 1

    mbs.SolveStatic(simulationSettings)

    u = mbs.GetNodeOutput(nBodies-2, testInterface.exu.OutputVariableType.Position) #tip node
    testInterface.exu.Print('static tip displacement (y)=', u[1])
    staticError = u[1]-(-0.44056224799446486)

    totalError = abs(staticError) + abs(dynamicError)

    if testInterface.useGraphics: 
        testInterface.SC.renderer.DoIdleTasks()
        testInterface.SC.renderer.Stop() 

    return totalError


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#test4 PointMass, SpringDamper, DistanceConstraint
def MathematicalPendulumTest(mbs, testInterface):
    L = 0.8 #distance
    nodeList = [0,0]

    for k in range(2):
        n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0], initialCoordinates = [0,0,0]))
        nodeList[k] = n1

        mass = 2.5
        g = 9.81

        #add mass points and ground object:
        objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0]))
        massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1))

        #marker for constraint / springDamper
        groundMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber = objectGround, localPosition= [0, 0, 0]))
        bodyMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber = massPoint, localPosition= [0, 0, 0]))

        if k==0:
            mbs.AddObject(DistanceConstraint(distance = L, markerNumbers = [groundMarker,bodyMarker]))
        else:
            k = 40000 #spring stiffness
            d = 200  #damping coefficient
            mbs.AddObject(SpringDamper(stiffness = k, damping = d, force = 0, referenceLength = L, 
                                       markerNumbers = [groundMarker,bodyMarker]))

        #add loads:
        mbs.AddLoad({'loadType': 'ForceVector',  'markerNumber': bodyMarker,  'loadVector': [0, -mass*g, 0]}) 

    #testInterface.exu.Print(mbs)
    mbs.Assemble()
    if testInterface.useGraphics: 
        testInterface.SC.renderer.Start()

    simulationSettings = testInterface.exu.SimulationSettings()

    simulationSettings.solutionSettings.writeSolutionToFile=False
    simulationSettings.timeIntegration.numberOfSteps = 1000
    simulationSettings.timeIntegration.endTime = 2

    simulationSettings.timeIntegration.generalizedAlpha.useNewmark = False #better convergence with index2 and correct initial accelerations
    simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = False

    #simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6 #SHOULD work with standard values ...
    #simulationSettings.timeIntegration.newton.useModifiedNewton = False #CHECK if works with modified Newton ...
    simulationSettings.timeIntegration.generalizedAlpha.computeInitialAccelerations = True
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.6 
    #simulationSettings.displayStatistics = False

    mbs.SolveDynamic(simulationSettings)
    if testInterface.useGraphics: 
        testInterface.SC.renderer.DoIdleTasks()
        testInterface.SC.renderer.Stop() #safely close rendering window!

    u1 = mbs.GetNodeOutput(nodeList[0], testInterface.exu.OutputVariableType.Position) #tip node
    testInterface.exu.Print('solution mathematicalPendulum Constraint=',u1[1])

    u2 = mbs.GetNodeOutput(nodeList[1], testInterface.exu.OutputVariableType.Position) #tip node
    testInterface.exu.Print('solution mathematicalPendulum SpringDamper=',u2[1])
    if testInterface.useCorrectedAccGenAlpha:
        errorConstraint= u1[1] -   (-0.06808284314701757)   #2021-02-04: -0.06808284314701757
        errorSpringDamper= u2[1] - (-0.07148800507819238)   #2021-02-04: -0.07148800507819238
    else:
        errorConstraint= u1[1] - (-0.0714264053422459)  #2019-12-26: -0.0714264053422459; 15.12.2019: -0.07242503089584812; before 15.12.2019: (-0.0724256565815142) #-(-0.7823882479152345) with endtime=10 and 10000 steps
        errorSpringDamper= u2[1] - (-0.07477852383438113) #2019-12-26: -0.07477852383438113; 15.12.2019: (-0.07579968609949864); before 15.12.2019: (-0.07579967194309412)  #-(-0.7738842923525007)

    return abs(errorConstraint)+abs(errorSpringDamper)

    #solution converged for 6 digits with index2 + accelerations initial conditions:
    #100000
    #solution mathematicalPendulum Constraint= -0.068039063184649
    #solution mathematicalPendulum SpringDamper= -0.071442303558492


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#rigid pendulum: test5 RigidBody2D, MarkerBodyPosition, RevoluteJoint2D, MarkerBodyMass, Force, Gravity
def RigidPendulumTest(mbs, testInterface):
    rect = [-2,-2,2,2] #xmin,ymin,xmax,ymax
    background = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
    oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
    a = 0.5     #half x-dim of pendulum
    b = 0.05    #half y-dim of pendulum
    massRigid = 12
    inertiaRigid = massRigid/12*(2*a)**2
    g = 9.81    # gravity

    graphics2 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-a,-b,0, a,-b,0, a,b,0, -a,b,0, -a,-b,0]} #background
    nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[a,0,0], initialVelocities=[0,0,0*2]));
    oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,visualization=VObjectRigidBody2D(graphicsData= [graphics2])))

    mRigidSupport = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[-a,0.,0.])) #support point
    mRigidMidPoint = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[ 0., 0.,0.])) #mid point
    mRigidMass = mbs.AddMarker(MarkerBodyMass(bodyNumber=oRigid)) 

    mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0.]))
    mbs.AddObject(RevoluteJoint2D(markerNumbers=[mGround,mRigidSupport]))

    mbs.AddLoad(Force(markerNumber = mRigidMidPoint, loadVector = [0, -0.5*massRigid*g, 0])) #split force into two parts: gravity and force ...
    mbs.AddLoad(Gravity(markerNumber = mRigidMass, loadVector = [0, -0.5*g, 0])) 

    mbs.Assemble()
    #mbs.systemData.Info()

    simulationSettings = testInterface.exu.SimulationSettings() #takes currently set values or default values
    simulationSettings.solutionSettings.writeSolutionToFile=False

    simulationSettings.timeIntegration.numberOfSteps = 1000
    simulationSettings.timeIntegration.endTime = 0.5
    simulationSettings.timeIntegration.newton.relativeTolerance = 1e-10 
    simulationSettings.timeIntegration.verboseMode = 1 
    simulationSettings.displayStatistics = False

    simulationSettings.solutionSettings.solutionWritePeriod = 1e-4

    simulationSettings.timeIntegration.newton.useModifiedNewton = True

    #simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
    #simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5

    if testInterface.useGraphics: 
        testInterface.SC.renderer.Start()

    mbs.SolveDynamic(simulationSettings)

    if testInterface.useGraphics: 
        testInterface.SC.renderer.DoIdleTasks()
        testInterface.SC.renderer.Stop() #safely close rendering window!


    u = mbs.GetNodeOutput(nRigid, testInterface.exu.OutputVariableType.Position) #tip node
    # if testInterface.useNewGenAlphaSolver:
    #     errorRigidPendulum = u[1] - (-0.49796067298096375 ) #2021-02-06: --0.49796067298096375 
    if testInterface.useCorrectedAccGenAlpha:
        errorRigidPendulum = u[1] - (-0.4979606729809297) #2021-02-04: -0.4979606729809297
    else:
        errorRigidPendulum = u[1] - (-0.4979662392961769) #2019-12-26(new initial acc): -0.4979662392961769; 15.12.2019: (-0.4980200584148534); before 15.12.2019: (-0.4980200584133354) # old test (load at tip)   0*(- 0.4905431986572512) #0*(-0.037344780490849015) #yield-displacement
    testInterface.exu.Print('solution rigid pendulum=',u[1])

    return abs(errorRigidPendulum)


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#slider-crank test6; test nonlinear model; index2 and index3-formulation for ConnectorCoordinate and RevoluteJoint2D
def SliderCrank2DTest(mbs, testInterface):

    #++++++++++++++++++++++++++++++++
    #ground object/node:

    rect = [-1,-2,3,2] #xmin,ymin,xmax,ymax
    background = GraphicsDataRectangle(-1, -2, 3, 2, color=[0.9,0.9,0.9,1.])
    #{'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
    oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
    nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint

    #++++++++++++++++++++++++++++++++
    #nodes and bodies
    #crank is mounted at (0,0,0); crank length = 2*a0, connecting rod length = 2*a1
    a0 = 0.25     #half x-dim of body
    b0 = 0.05    #half y-dim of body
    massRigid0 = 2
    inertiaRigid0 = massRigid0/12*(2*a0)**2
    graphics0 = GraphicsDataRectangle(-a0,-b0,a0,b0)
    #{'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-a0,-b0,0, a0,-b0,0, a0,b0,0, -a0,b0,0, -a0,-b0,0]} #background

    a1 = 0.5     #half x-dim of body
    b1 = 0.05    #half y-dim of body
    massRigid1 = 4
    inertiaRigid1 = massRigid1/12*(2*a1)**2
    graphics1 = GraphicsDataRectangle(-a1,-b1,a1,b1)

    nRigid0 = mbs.AddNode(Rigid2D(referenceCoordinates=[a0,0,0], initialVelocities=[0,0,0]));
    oRigid0 = mbs.AddObject(RigidBody2D(physicsMass=massRigid0, physicsInertia=inertiaRigid0,nodeNumber=nRigid0,visualization=VObjectRigidBody2D(graphicsData= [graphics0])))

    nRigid1 = mbs.AddNode(Rigid2D(referenceCoordinates=[2*a0+a1,0,0], initialVelocities=[0,0,0]));
    oRigid1 = mbs.AddObject(RigidBody2D(physicsMass=massRigid1, physicsInertia=inertiaRigid1,nodeNumber=nRigid1,visualization=VObjectRigidBody2D(graphicsData= [graphics1])))

    c=0.05 #dimension of mass
    sliderMass = 1
    graphics2 = GraphicsDataRectangle(-c,-c,c,c)

    nMass = mbs.AddNode(Point2D(referenceCoordinates=[2*a0+2*a1,0]))
    oMass = mbs.AddObject(MassPoint2D(physicsMass=sliderMass, nodeNumber=nMass,visualization=VObjectRigidBody2D(graphicsData= [graphics2])))

    #++++++++++++++++++++++++++++++++
    #markers for joints:
    mR0Left = mbs.AddMarker(MarkerBodyRigid(bodyNumber=oRigid0, localPosition=[-a0,0.,0.])) #support point # MUST be a rigidBodyMarker, because a torque is applied
    mR0Right = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid0, localPosition=[ a0,0.,0.])) #end point; connection to connecting rod

    mR1Left = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid1, localPosition=[-a1,0.,0.])) #connection to crank
    mR1Right = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid1, localPosition=[ a1,0.,0.])) #end point; connection to slider

    mMass = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oMass, localPosition=[ 0.,0.,0.]))
    mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0.]))

    #++++++++++++++++++++++++++++++++
    #joints:
    mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR0Left]))
    mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR0Right,mR1Left]))
    mbs.AddObject(RevoluteJoint2D(markerNumbers=[mR1Right,mMass]))

    #++++++++++++++++++++++++++++++++
    #markers for node constraints:
    mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
    mNodeSlider = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nMass, coordinate=1)) #y-coordinate is constrained

    #++++++++++++++++++++++++++++++++
    #coordinate constraints
    mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mNodeSlider]))

    #loads and driving forces:
    mbs.AddLoad(Torque(markerNumber = mR0Left, loadVector = [0, 0, 10])) #apply torque at crank

    #++++++++++++++++++++++++++++++++
    #assemble, adjust settings and start time integration
    mbs.Assemble()

    simulationSettings = testInterface.exu.SimulationSettings() #takes currently set values or default values
    simulationSettings.solutionSettings.writeSolutionToFile=False

    simulationSettings.timeIntegration.numberOfSteps = 1000
    simulationSettings.timeIntegration.endTime = 1
    simulationSettings.timeIntegration.newton.useModifiedNewton = True

    simulationSettings.timeIntegration.newton.relativeTolerance = 1e-10 #10000
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
    simulationSettings.displayStatistics = False

    if testInterface.useGraphics: 
        testInterface.SC.renderer.Start()

    #solve generalized alpha / index3:
    mbs.SolveDynamic(simulationSettings)

    u = mbs.GetNodeOutput(nMass, testInterface.exu.OutputVariableType.Position) #tip node
    if testInterface.useCorrectedAccGenAlpha:
        errorSliderCrankIndex3 = u[0] - 1.3550008762955048 #2021-02-04: 1.3550008762955048
    else:
        errorSliderCrankIndex3 = u[0] - 1.353298442702153  #2019-12-26: 1.353298442702153; 15.12.2019: 1.3513750614337234; before 15.12.2019: 1.3513750614326427 #2019-11-22; previous: 1.3513750614331235 #x-position of slider
    testInterface.exu.Print('solution SliderCrankIndex3  =',u[0])
    testInterface.exu.Print('error errorSliderCrankIndex3=',errorSliderCrankIndex3)

    simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
    simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True

    #solve index 2 / trapezoidal rule:
    mbs.SolveDynamic(simulationSettings)

    u = mbs.GetNodeOutput(nMass, testInterface.exu.OutputVariableType.Position) #tip node
    errorSliderCrankIndex2 = u[0] - 1.3550413308333111 #2019-12-26: 1.3550413308333111; 15.12.2019: 1.352878631961969; before 15.12.2019: 1.3528786319585846 #2019-11-22; previous: 1.3528786319585837 #x-position of slider
    testInterface.exu.Print('solution SliderCrankIndex2  =',u[0])
    testInterface.exu.Print('error errorSliderCrankIndex2=',errorSliderCrankIndex2)

    if testInterface.useGraphics: 
        testInterface.SC.renderer.DoIdleTasks()
        testInterface.SC.renderer.Stop() #safely close rendering window!


    return abs(errorSliderCrankIndex3)+abs(errorSliderCrankIndex2)



#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#test7 ANCF Cable2D, SlidingJoint2D
def SlidingJoint2DTest(mbs, testInterface):

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

    testInterface.exu.Print("load f="+str(f))
    testInterface.exu.Print("EI="+str(E*I))

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
        elem=mbs.AddObject(Cable2D(physicsLength=lElem, physicsMassPerLength=rho*A, 
                                   physicsBendingStiffness=E*I, physicsAxialStiffness=E*A, 
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
                                                          nodeNumber=nodeDataSJ, classicalFormulation = False))


    mbs.Assemble()

    simulationSettings = testInterface.exu.SimulationSettings() #takes currently set values or default values
    simulationSettings.solutionSettings.writeSolutionToFile=False

    fact = 200
    simulationSettings.timeIntegration.numberOfSteps = 1*fact
    simulationSettings.timeIntegration.endTime = 0.001*fact*0.5
    simulationSettings.solutionSettings.writeSolutionToFile = True
    simulationSettings.solutionSettings.solutionWritePeriod = simulationSettings.timeIntegration.endTime/fact
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
    simulationSettings.displayStatistics = False

    if testInterface.useGraphics: 
        testInterface.SC.renderer.Start()

    mbs.SolveDynamic(simulationSettings)


    error = 0
    if nRigid != -1:
        u = mbs.GetNodeOutput(nRigid, testInterface.exu.OutputVariableType.Position) #tip node
        if testInterface.useNewGenAlphaSolver: 
            error = u[1] - (-0.14920183348514676 ) #2021-09-27: new JacobianODE2RHS
            #error = u[1] -(-0.14920182499994944 ) #2021-02-06: -0.14920182499994944 (1e-9 different from old solver) 
        elif testInterface.useCorrectedAccGenAlpha:
            error = u[1] - (-0.14920182666080586) #2021-02-04: -0.14920182666080586
        else:
            error = u[1] - (-0.14920151345936586) #2019-12-26: -0.14920151345936586; 15.12.2019: (-0.1489879442762764); before 15.12.2019: (-0.14898795617249422) #2019-11-22; #20.10.2019: (-0.1489879501348149); 17.10.2019:-0.14898795468724652; old? :(-0.14898795002032308) #old error before projected sliding joint: (-0.14898792622401774) #y-position of COM of sliding body
                        
        testInterface.exu.Print('value SlidingJoint2DTest=',u[1])
        #testInterface.exu.Print('error SlidingJoint2DTest=',error)

    if testInterface.useGraphics: 
        testInterface.SC.renderer.DoIdleTasks()
        testInterface.SC.renderer.Stop() #safely close rendering window!

    return abs(error)


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#test8 linear spring-damper: CartesianSpringDamper, initialCoordinates, initialVelocities
def CartesianSpringDamperTest(mbs, testInterface):
    L=0.5
    mass = 1.6
    k = 4000
    omega0 = 50 # sqrt(4000/1.6)
    dRel = 0.05
    d = dRel * 2 * 80 #80=sqrt(1.6*4000)
    u0=-0.08
    v0=1
    f = 80
    x0 = f/k

    #node for mass point:
    n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0], initialCoordinates = [u0,0,0], initialVelocities= [v0,0,0]))

    #add mass points and ground object:
    objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0]))
    massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1))

    #marker for constraint / springDamper
    groundMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber = objectGround, localPosition= [0, 0, 0]))
    bodyMarker = mbs.AddMarker(MarkerBodyPosition(bodyNumber = massPoint, localPosition= [0, 0, 0]))

    mbs.AddObject(CartesianSpringDamper(markerNumbers = [groundMarker, bodyMarker], stiffness = [k,k,k], damping = [d,0,0], offset = [L,0,0]))

    #add loads:
    mbs.AddLoad({'loadType': 'ForceVector',  'markerNumber': bodyMarker,  'loadVector': [f, 0, 0]}) 

    #testInterface.exu.Print(mbs)
    mbs.Assemble()

    simulationSettings = testInterface.exu.SimulationSettings()
    simulationSettings.solutionSettings.writeSolutionToFile=False
    tEnd = 1
    steps = 1000
    simulationSettings.timeIntegration.numberOfSteps = steps
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #SHOULD work with 0.9 as well
    simulationSettings.displayStatistics = False

    mbs.SolveDynamic(simulationSettings)

    u = mbs.GetNodeOutput(n1, testInterface.exu.OutputVariableType.Position)
    uCartesianSpringDamper= u[0] - L
    if testInterface.useCorrectedAccGenAlpha:
        errorCartesianSpringDamper = uCartesianSpringDamper - 0.011834933407061654 #until 2022-01-25 (changed jacobians): 0.011834933406052683 #2021-09-27: new JacobianODE2RHS
        #errorCartesianSpringDamper = uCartesianSpringDamper -0.011834933407364412 #2021-02-04: 0.011834933407364412 
    else:
        errorCartesianSpringDamper = uCartesianSpringDamper - 0.011834933407594783 #15.12.2019: 0.011834933407594783; beofre 15.12.2019: 0.011834933407038783 #for 1000 steps, endtime=1; accurate up to 3e-6 to exact solution
    testInterface.exu.Print('solution cartesianSpringDamper=',uCartesianSpringDamper)

    return abs(errorCartesianSpringDamper)

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#Test9 ANCF cable
def CoordinateSpringDamperTest(mbs, testInterface): 
    L=0.5
    mass = 1.6
    k = 4000
    omega0 = 50 # sqrt(4000/1.6)
    dRel = 0.05
    d = dRel * 2 * 80 #80=sqrt(1.6*4000)
    u0=-0.08
    v0=1
    f = 80
    x0 = f/k
    fFriction = 20 #force in Newton, only depends on direction of velocity
    
    #node for mass point:
    n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0], initialCoordinates = [u0,0,0], initialVelocities= [v0,0,0]))
    nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [L,0,0]))
    
    #add mass points and ground object:
    objectGround = mbs.AddObject(ObjectGround(referencePosition = [0,0,0]))
    massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1))
    
    #marker for constraint / springDamper
    groundCoordinateMarker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
    nodeCoordinateMarker0  = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 0))
    nodeCoordinateMarker1  = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 1))
    nodeCoordinateMarker2  = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, coordinate = 2))
    
    #Spring-Dampers
    mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundCoordinateMarker, nodeCoordinateMarker0], 
                                         stiffness = k, damping = d) ) #changed 2023-01-21: dryFriction=0*fFriction, dryFrictionProportionalZone=0.01)) #offset must be zero, because coordinates just represent the displacements
    mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundCoordinateMarker, nodeCoordinateMarker1], stiffness = k)) 
    mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundCoordinateMarker, nodeCoordinateMarker2], stiffness = k)) 
    
    #add loads:
    mbs.AddLoad(LoadCoordinate(markerNumber = nodeCoordinateMarker0, load = f))
    
    mbs.Assemble()
    
    simulationSettings = testInterface.exu.SimulationSettings()
    tEnd = 1 #1
    steps = 1000    #1000
    simulationSettings.solutionSettings.solutionWritePeriod = 1e-3
    simulationSettings.timeIntegration.numberOfSteps = steps
    simulationSettings.timeIntegration.endTime = tEnd
    simulationSettings.displayStatistics = False
    
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1 #SHOULD work with 0.9 as well
    simulationSettings.solutionSettings.writeSolutionToFile=False

    if testInterface.useGraphics: 
        testInterface.SC.renderer.Start()
    
    mbs.SolveDynamic(simulationSettings)
    
    if testInterface.useGraphics: 
        testInterface.SC.renderer.DoIdleTasks()
        testInterface.SC.renderer.Stop() #safely close rendering window!
    
    u = mbs.GetNodeOutput(n1, testInterface.exu.OutputVariableType.Position)
    uCoordinateSpringDamper= u[0] - L
    if testInterface.useCorrectedAccGenAlpha:
        errorCoordinateSpringDamper = uCoordinateSpringDamper - 0.011834933407061654 #until 2022-01-25 (changed jacobians): 0.01183493340619235 #2021-09-27: new JacobianODE2RHS
        #errorCoordinateSpringDamper = uCoordinateSpringDamper -0.011834933407368853 #2021-02-04: 0.011834933407368853
    else:
        errorCoordinateSpringDamper = uCoordinateSpringDamper - 0.011834933406690284 #15.12.2019: 0.011834933406690284; beofre 15.12.2019: 0.011834933407047 #for 1000 steps, endtime=1; this is different from CartesianSpringDamper because of offset L (rounding errors around 1e-14)
    

    testInterface.exu.Print('solution CoordinateSpringDamper=',uCoordinateSpringDamper)
    return abs(errorCoordinateSpringDamper)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#switching constraints test with: mbs.SetPreStepUserFunction, Get/SetParameters, activeConnector, 
def SwitchingConstraintsTest(mbs, testInterface):

    rect = [-2.5,-1.5,0.5,1.5] #xmin,ymin,xmax,ymax
    background = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
    oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background])))
    #nGround=mbs.AddNode(NodePointGround(referencePosition= [0,0,0]))
    a = 0.5     #x-dim of pendulum
    b = 0.05    #y-dim of pendulum
    massRigid = 12
    mass = 2 #of additional mass
    inertiaRigid = massRigid/12*(2*a)**2
    omega0 = 4

    graphics2 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[-a,-b,0, a,-b,0, a,b,0, -a,b,0, -a,-b,0]} #background
    nRigid = mbs.AddNode(Rigid2D(referenceCoordinates=[-0.5,0,0], initialVelocities=[0,omega0*a,omega0]));
    oRigid = mbs.AddObject(RigidBody2D(physicsMass=massRigid, physicsInertia=inertiaRigid,nodeNumber=nRigid,visualization=VObjectRigidBody2D(graphicsData= [graphics2])))

    mR1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oRigid, localPosition=[-0.5,0.,0.])) #support point
    mG0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[-0.5-a,0.,0.]))
    oRJoint = mbs.AddObject(RevoluteJoint2D(markerNumbers=[mG0,mR1],activeConnector=True))

    #mass point is attached with coordinate constraints:
    mCoordR0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRigid, coordinate=0)) 
    mCoordR1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nRigid, coordinate=1)) 

    #additional mass point attached to COM of rigid body:
    nMass = mbs.AddNode(Point2D(referenceCoordinates=[-0.5,0], initialVelocities=[0,omega0*a]));
    oMass = mbs.AddObject(MassPoint2D(physicsMass=mass, nodeNumber=nMass) )

    mCoordM0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=0)) 
    mCoordM1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nMass, coordinate=1)) 

    oConstraint0 = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordM0, mCoordR0],activeConnector=True))
    oConstraint1 = mbs.AddObject(CoordinateConstraint(markerNumbers=[mCoordM1, mCoordR1],activeConnector=True))

    mbs.Assemble()

    simulationSettings = testInterface.exu.SimulationSettings() #takes currently set values or default values
    simulationSettings.timeIntegration.numberOfSteps = 4000
    simulationSettings.timeIntegration.endTime = 2
    simulationSettings.timeIntegration.newton.relativeTolerance = 1e-8
    simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-6
    #simulationSettings.timeIntegration.verboseMode = 1

    #execute this python code in the same scope as this file at beginning of every time step:
    #2019-12-18: subtract deltaT=1e-8 in order to avoid round off effects in 1e-16 regime
    def UFswitchConnector(mbs, t):
        if t > (0.3 + 1e-8): 
            mbs.SetObjectParameter(oRJoint, 'activeConnector', False)
        if t > (0.1 + 1e-8): 
            mbs.SetObjectParameter(oConstraint0, 'activeConnector', False)
            mbs.SetObjectParameter(oConstraint1, 'activeConnector', False)
        return True #True, means that everything is alright, False=stop simulation
    
    mbs.SetPreStepUserFunction(UFswitchConnector)

    #simulationSettings.timeIntegration.verboseMode = 1
    simulationSettings.timeIntegration.newton.useModifiedNewton = False
    simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1
    #simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1
    simulationSettings.timeIntegration.generalizedAlpha.useNewmark = True
    simulationSettings.timeIntegration.generalizedAlpha.useIndex2Constraints = True
    simulationSettings.solutionSettings.solutionInformation = "Rigid pendulum with switching constraints"
    simulationSettings.displayStatistics = False
    simulationSettings.solutionSettings.writeSolutionToFile=False

    #testInterface.useGraphics = True
    if testInterface.useGraphics: 
        testInterface.SC.renderer.Start()

    mbs.SolveDynamic(simulationSettings)

    if testInterface.useGraphics: 
        testInterface.SC.renderer.DoIdleTasks()
        testInterface.SC.renderer.Stop() #safely close rendering window!

    u = mbs.systemData.GetODE2Coordinates()
    testInterface.exu.Print('u =',u)
    testInterface.exu.Print('sum(u) =',sum(u))

    return abs(sum(u)-8.376384072333597) #2020-01-09: 376384072333597; before 13.12.2019: 8.342236349593959


#run all unit tests; return True if success, False if unit tests failed
def RunAllModelUnitTests(mbs, testInterface):
    #testInterface.useGraphics = False

    totalError = 0 #accumulated error
    totalTests = 0 #number of tests accomplished
    testsFailed = [] #number of tests failed
    
    #errTol = 1e-13
    errTol = 4e-13 #changed this since the new solver
    
    listTests=[GraphicsDataTest,ANCFCable2DBendingTest,SpringDamperMesh,MathematicalPendulumTest,
               RigidPendulumTest,SliderCrank2DTest,SlidingJoint2DTest,CartesianSpringDamperTest,
               CoordinateSpringDamperTest, SwitchingConstraintsTest]

    for TEST in listTests:    
        testInterface.exu.Print("\n\n****************\n "+TEST.__name__+":[TEST " + str(totalTests+1) + "]\n****************\n")
        err = 1e38 #some large value
        try:
            err = TEST(mbs, testInterface); 
            if (abs(err) > errTol): 
                testsFailed+=[totalTests]
                testInterface.exu.Print('UNITTEST *FAILED*: error in '+TEST.__name__+' = ',err)
            else:
                testInterface.exu.Print('UNITTEST OK: error in '+TEST.__name__+' = ',err)
        except Exception as e:
            testsFailed+=[totalTests+1]
            #testInterface.exu.Print('TEST '+TEST.__name__+' *FAILED* and raised exception:\n'+str(e))
            #print('TEST '+TEST.__name__+' *FAILED* and raised exception:\n'+str(e), flush=True)
        finally:
            totalError += err; totalTests += 1; 
            mbs.Reset()
    
    # testInterface.exu.Print("\n\n****************\n GraphicsDataTest:[TEST " + str(totalTests+1) + "]\n****************\n")
    # err = GraphicsDataTest(mbs, testInterface); totalError += err; totalTests += 1; 
    # if (abs(err) > errTol): testsFailed+=[totalTests]
    # testInterface.exu.Print('error in GraphicsDataTest = ',err)
    # mbs.Reset()

    # testInterface.exu.Print("\n\n****************\n ANCFCable2DBendingTest:[TEST " + str(totalTests+1) + "]\n****************\n")
    # err = ANCFCable2DBendingTest(mbs, testInterface); totalError += err; totalTests += 1
    # if (abs(err) > errTol): testsFailed+=[totalTests]
    # testInterface.exu.Print('error in ANCFCable2DBendingTest = ',err)
    # mbs.Reset()

    # testInterface.exu.Print("\n\n****************\n SpringDamperMesh:[TEST " + str(totalTests+1) + "]\n****************\n")
    # err = SpringDamperMesh(mbs, testInterface); totalError += err; totalTests += 1
    # if (abs(err) > errTol): testsFailed+=[totalTests]
    # testInterface.exu.Print('error in SpringDamperMesh = ',err)
    # mbs.Reset()

    # testInterface.exu.Print("\n\n****************\n MathematicalPendulumTest:[TEST " + str(totalTests+1) + "]\n****************\n")
    # err = MathematicalPendulumTest(mbs, testInterface); totalError += err; totalTests += 1
    # if (abs(err) > errTol): testsFailed+=[totalTests]
    # testInterface.exu.Print('error in MathematicalPendulumTest = ',err)
    # mbs.Reset()

    # testInterface.exu.Print("\n\n****************\n RigidPendulumTest:[TEST " + str(totalTests+1) + "]\n****************\n")
    # err = RigidPendulumTest(mbs, testInterface); totalError += err; totalTests += 1
    # if (abs(err) > errTol): testsFailed+=[totalTests]
    # testInterface.exu.Print('error in RigidPendulumTest = ',err)
    # mbs.Reset()

    # testInterface.exu.Print("\n\n****************\n SliderCrank2DTest:[TEST " + str(totalTests+1) + "]\n****************\n")
    # err = SliderCrank2DTest(mbs, testInterface); totalError += err; totalTests += 1
    # if (abs(err) > errTol): testsFailed+=[totalTests]
    # testInterface.exu.Print('error in SliderCrank2DTest = ',err)
    # mbs.Reset()

    # testInterface.exu.Print("\n\n****************\n SlidingJoint2DTest:[TEST " + str(totalTests+1) + "]\n****************\n")
    # err = SlidingJoint2DTest(mbs, testInterface); totalError += err; totalTests += 1
    # if (abs(err) > errTol): testsFailed+=[totalTests]
    # testInterface.exu.Print('error in SlidingJoint2DTest = ',err)
    # mbs.Reset()

    # testInterface.exu.Print("\n\n****************\n CartesianSpringDamperTest:[TEST " + str(totalTests+1) + "]\n****************\n")
    # err = CartesianSpringDamperTest(mbs, testInterface); totalError += err; totalTests += 1
    # if (abs(err) > errTol): testsFailed+=[totalTests]
    # testInterface.exu.Print('error in CartesianSpringDamperTest = ',err)
    # mbs.Reset()

    # testInterface.exu.Print("\n\n****************\n CoordinateSpringDamperTest:[TEST " + str(totalTests+1) + "]\n****************\n")
    # err = CoordinateSpringDamperTest(mbs, testInterface); totalError += err; totalTests += 1
    # if (abs(err) > errTol): testsFailed+=[totalTests]
    # testInterface.exu.Print('error in CoordinateSpringDamperTest = ',err)
    # mbs.Reset()

    # testInterface.exu.Print("\n\n****************\n SwitchingConstraintsTest: [TEST " + str(totalTests+1) + "]\n****************\n")
    # err = SwitchingConstraintsTest(mbs, testInterface); totalError += err; totalTests += 1
    # if (abs(err) > errTol): testsFailed+=[totalTests]
    # testInterface.exu.Print('error in SwitchingConstraintsTest = ',err)
    # mbs.Reset()




    testInterface.exu.Print('\n\n********************************')
    testInterface.exu.Print('********************************')
    rv = True
    if len(testsFailed) == 0:
        testInterface.exu.Print('  ALL ' + str(totalTests) + ' UNIT TESTS SUCCESSFUL')
    else:
        testInterface.exu.Print('  ' + str(len(testsFailed)) + ' UNIT TESTS FAILED: ' + str(testsFailed))
        rv = False

    testInterface.exu.Print('********************************')
    testInterface.exu.Print('********************************\n')

    return [rv, testsFailed]




