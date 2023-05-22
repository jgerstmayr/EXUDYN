#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test models for ANCFBeam and GeometricallyExactBeam (2-node shear deformable beam, 
#           Lie group formulation for work of elastic forces); 
#           test model: right angle frame under end load;
#           K. Nachbagauer, J. Gerstmayr. Structural and Continuum Mechanics Approaches for a 3D Shear Deformable ANCF Beam Finite Element: Application to Buckling and Nonlinear Dynamic Examples, Journal of Computational and Nonlinear Dynamics, Vol. 9(1), pp. 011013-1 – 011013-8, 2013.
#
# Author:   Johannes Gerstmayr
# Date:     2023-05-02
#
# Notes:    currently, this model shows very bad convergence for ANCFBeam and no convergence for GeometricallyExactBeam
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import exudyn as exu
from exudyn.utilities import *

import numpy as np

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

useGeometricallyExact = True

nElements = 4*2*2


useGraphics = True
verbose = 1

useEP = True #for geometrically exact beam node
angleZ = -pi/2
rotZ = RotationMatrixZ(-pi/2)
if useEP:
    NodeClass = NodeRigidBodyEP
    initialRotationsGE = eulerParameters0
    initialRotationsGE2 = list(RotationMatrix2EulerParameters(rotZ))
else: #does not work for static case, as static solver currently (2023-04) cannot solve for Lie group nodes 
    NodeClass = NodeRigidBodyRotVecLG
    initialRotationsGE = [0,0,0]
    initialRotationsGE2 = [0,0,angleZ]


testErrorSum = 0

printCase = True

if True:
    mbs.Reset()
    
    computeEigenmodes = False
    csFact = 1
    sectionData = exu.BeamSection()
    ks1=1 #shear correction, torsion
    ks2=1 #shear correction, bending
    ks3=1 #shear correction, bending
    ff=1 #drawing factor

    L = 0.24 #length of beam
    w = 0.0006 #width of beam
    h = 0.03 #height Y

    Em = 7.124e10
    rho = 1e2 #unused in static computation

    A=h*w
    nu = 0.31              # Poisson ratio

    #not mentioned in Simo's paper
    # ks1= 0.5768 #torsion correction factor if J=Jyy+Jzz
    # ks2= 10*(1+nu)/(12+11*nu)
    # ks3=ks2

    
    Gm = Em/(2*(1+nu))      # Shear modulus

    # Cross-section properties
    Iyy = h*w**3/12 # Second moment of area of the beam cross-section
    Izz = w*h**3/12 # Second moment of area of the beam cross-section
    #J = (Iyy+Izz)   # approximation; Polar moment of area of the beam cross-section
    #thin rectangle:
    beta = 1/3 #for infinitely thin beam
    J = beta*h*w**3

    sectionData.stiffnessMatrix = np.diag([Em*A, Gm*A*ks2, Gm*A*ks3, Gm*J*ks1, Em*Iyy, Em*Izz])


    rhoA = rho*A

    sectionData.inertia= rho*J*np.eye(3)
    sectionData.massPerLength = rhoA

    sectionGeometry = exu.BeamSectionGeometry()

    #points, in positive rotation sense viewing in x-direction, points in [Y,Z]-plane
    #points do not need to be closed!
    lp = exu.Vector2DList()
    if True:
        lp.Append([h*ff,-w*ff])
        lp.Append([h*ff,w*ff])
        lp.Append([-h*ff,w*ff])
        lp.Append([-h*ff,-w*ff])

    sectionGeometry.polygonalPoints = lp
    #exu.Print('HERE\n',sectionGeometry.polygonalPoints)
    nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
    mnGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
 

    lElem = L/nElements

    #first arm:
    eY=[0,1,0]
    eZ=[0,0,1]
    if useGeometricallyExact:
        n0 = mbs.AddNode(NodeClass(referenceCoordinates=[0,0,0]+initialRotationsGE))
    else:
        initialRotations = eY+eZ
        n0 = mbs.AddNode(NodePoint3DSlope23(referenceCoordinates=[0,0,0]+initialRotations))
    nInit = n0
    for k in range(nElements):
        if useGeometricallyExact:
            n1 = mbs.AddNode(NodeClass(referenceCoordinates=[lElem*(k+1),0,0]+initialRotationsGE))
        
            oBeam = mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=[n0,n1], physicsLength = lElem, 
                                                                 sectionData = sectionData,
                                                                 visualization=VBeam3D(sectionGeometry=sectionGeometry)))
        else:
            n1 = mbs.AddNode(NodePoint3DSlope23(referenceCoordinates=[lElem*(k+1),0,0]+initialRotations))
            oBeam = mbs.AddObject(ObjectANCFBeam(nodeNumbers=[n0,n1], physicsLength = lElem, 
                                                   #testBeamRectangularSize = [h,w],
                                                   sectionData = sectionData,
                                                   crossSectionPenaltyFactor = [csFact,csFact,csFact],
                                                   visualization=VANCFBeam(sectionGeometry=sectionGeometry)))
        n0 = n1

    #second arm:
    eY=[1,0,0]
    eZ=[0,0,1]
    if useGeometricallyExact:
        n0B = mbs.AddNode(NodeClass(referenceCoordinates=[L,0,0]+initialRotationsGE2))
    else:
        initialRotations = eY+eZ
        n0B = mbs.AddNode(NodePoint3DSlope23(referenceCoordinates=[L,0,0]+initialRotations))

    nInitB = n0B
    for k in range(nElements):
        if useGeometricallyExact:
            n1B = mbs.AddNode(NodeClass(referenceCoordinates=[L,-lElem*(k+1),0]+initialRotationsGE2))
        
            oBeam = mbs.AddObject(ObjectBeamGeometricallyExact(nodeNumbers=[n0B,n1B], physicsLength = lElem, 
                                                                  sectionData = sectionData,
                                                                  visualization=VBeam3D(sectionGeometry=sectionGeometry)))
        else:
            n1B = mbs.AddNode(NodePoint3DSlope23(referenceCoordinates=[L,-lElem*(k+1),0]+initialRotations))
            oBeam = mbs.AddObject(ObjectANCFBeam(nodeNumbers=[n0B,n1B], physicsLength = lElem, 
                                                    #testBeamRectangularSize = [h,w],
                                                    sectionData = sectionData,
                                                    crossSectionPenaltyFactor = [csFact,csFact,csFact],
                                                    visualization=VANCFBeam(sectionGeometry=sectionGeometry)))
        n0B = n1B


    
    mTip = mbs.AddMarker(MarkerNodeRigid(nodeNumber = n1))
    mTipB = mbs.AddMarker(MarkerNodeRigid(nodeNumber = n1B))
    #disturbance force:
    def UFloadTip(mbs, t, loadVector):
        tEnd = 1
        factDist = 0
        factLoad = 1
        if t < tEnd:
            factDist = t*(tEnd-t)/tEnd**2
            #factDist = (tEnd-t)/tEnd
            
        # if t < 1:
        factLoad = t
        # if factLoad > 0.6: #check if b
        #     factLoad = 0.6
        # else:
        #     factLoad = 0.715
        return [1.2*factLoad,0,0.0012*t]
    
    # mbs.AddLoad(Force(markerNumber=mTip, loadVector = [0,0,-0.2], 
    #                   #loadVectorUserFunction=UFloadTip
    #                   ))
    lTip = mbs.AddLoad(Force(markerNumber=mTipB, loadVector = [1,0,-0.1*2*0],
                             loadVectorUserFunction=UFloadTip
                             ))

    nGround = mbs.AddNode(NodePointGround(visualization=VNodePointGround(show=False)) )
    mCGround= mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
    # mNode = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n1, coordinate=2))
    # oCC = mbs.AddObject(CoordinateConstraint(markerNumbers = [mCGround, mNode], offset = 0.02,))

    mNodeB = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=n1B, coordinate=0))
    #oCC = mbs.AddObject(CoordinateConstraint(markerNumbers = [mCGround, mNodeB], offset = 0.,))

    # def UFoffset(mbs, t, itemNumber, lOffset):
    #     fact = 0
    #     if t>0.5:
    #         fact = 2*(t-0.5)
    #     return 0.02*fact

    # oCC = mbs.AddObject(CoordinateConstraint(markerNumbers = [mCGround, mNodeB],
    #                                 offsetUserFunction = UFoffset,
    #                                 ))

    if useGeometricallyExact:
        nm0 = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nInit))
        nmGround = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nGround))
        mbs.AddObject(GenericJoint(markerNumbers=[nmGround, nm0],
                                   visualization=VGenericJoint(axesRadius=2*w, axesLength=2*w)))
    else:
        for i in range(9):
            #if i != 4 and i != 8: #exclude constraining the slope lengths
            if True:
                nm0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nInit, coordinate=i))
                mbs.AddObject(CoordinateConstraint(markerNumbers=[mnGround, nm0]))

    nm0Last = mbs.AddMarker(MarkerNodeRigid(nodeNumber=n1))
    nm0B = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nInitB))
    mbs.AddObject(GenericJoint(markerNumbers=[nm0Last, nm0B],
                               rotationMarker1=rotZ,
                               constrainedAxes=[1,1,1,1,1,1],
                               visualization=VGenericJoint(axesRadius=2*w, axesLength=2*w)))


    sLoad = mbs.AddSensor(SensorLoad(loadNumber = lTip, storeInternal=True))
    # sLoad = mbs.AddSensor(SensorObject(objectNumber = oCC, storeInternal=True,outputVariableType=exu.OutputVariableType.Force))
    sDisp = mbs.AddSensor(SensorNode(nodeNumber=n1B, outputVariableType=exu.OutputVariableType.Displacement, storeInternal=True))

    # exu.Print(mbs)
    mbs.Assemble()

    tEnd = 1     #end time of simulation
    stepSize = 1e-4   #step size; leads to 1000 steps

    simulationSettings = exu.SimulationSettings()
    simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize) #must be integer
    simulationSettings.timeIntegration.endTime = tEnd

    simulationSettings.solutionSettings.solutionWritePeriod = 2e-2  #output interval general
    simulationSettings.solutionSettings.sensorsWritePeriod = 1e-5  #output interval of sensors

    simulationSettings.staticSolver.verboseMode = verbose

    # simulationSettings.displayComputationTime = True
    #simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
    # simulationSettings.parallel.numberOfThreads = 4

    #simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 5e-5
    #simulationSettings.staticSolver.newton.numericalDifferentiation.forODE2 = True
    #simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
    # simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-4

    simulationSettings.staticSolver.useLoadFactor = False #load is applied via user function
    simulationSettings.staticSolver.numberOfLoadSteps = 100
    # simulationSettings.linearSolverSettings.ignoreSingularJacobian
    
    # simulationSettings.staticSolver.loadStepGeometric=True
    # simulationSettings.staticSolver.loadStepGeometricRange = 1e4
    # simulationSettings.staticSolver.adaptiveStep = False
    # simulationSettings.staticSolver.stabilizerODE2term = 1
    # simulationSettings.staticSolver.computeLoadsJacobian = False

    # if useGeometricallyExact:
    #     simulationSettings.staticSolver.newton.relativeTolerance = 1e-4
    #     simulationSettings.staticSolver.newton.absoluteTolerance = 1e-5
    #     simulationSettings.staticSolver.numberOfLoadSteps = 1 #otherwise makes problems

    simulationSettings.staticSolver.newton.numericalDifferentiation.doSystemWideDifferentiation = True
    # simulationSettings.staticSolver.newton.numericalDifferentiation.forODE2=True

    simulationSettings.staticSolver.newton.relativeTolerance = 1e-5*2
    simulationSettings.staticSolver.newton.absoluteTolerance = 1e-5
    # simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-4
    # simulationSettings.staticSolver.stabilizerODE2term = 1e4

    simulationSettings.timeIntegration.newton.relativeTolerance = 1e-7
    simulationSettings.timeIntegration.newton.absoluteTolerance = 1e-7
    simulationSettings.timeIntegration.newton.useModifiedNewton = True

    #add some drawing parameters for this example
    SC.visualizationSettings.nodes.drawNodesAsPoint=False
    SC.visualizationSettings.nodes.defaultSize=0.01
    SC.visualizationSettings.nodes.basisSize=0.2
    SC.visualizationSettings.nodes.showNodalSlopes=True

    SC.visualizationSettings.bodies.beams.axialTiling = 50
    SC.visualizationSettings.general.drawWorldBasis = True
    SC.visualizationSettings.general.worldBasisSize = 0.1
    SC.visualizationSettings.openGL.multiSampling = 4
    SC.visualizationSettings.openGL.lineWidth=2

    if useGraphics:
        exu.StartRenderer()
        mbs.WaitForUserToContinue()


    # else:
    mbs.SolveStatic(simulationSettings)
    #mbs.SolveDynamic(simulationSettings)
    #mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.RK44)

    #%%+++++++++++++++++++++++++++++++++++    
    if useGraphics:
        SC.WaitForRenderEngineStopFlag()
        exu.StopRenderer() #safely close rendering window!
    
    ##evaluate final (=current) output values
    uTip = mbs.GetNodeOutput(n1, exu.OutputVariableType.Displacement)
    
    testErrorSum += np.linalg.norm(uTip)

    
    
    pTip = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
    exu.Print('ne=',nElements, ', ux=',L-pTip[0], ', uy=',pTip[1])


exu.Print('Solution of rightAngleFrameTip=', testErrorSum)
exudynTestGlobals.testError = testErrorSum - (0) 
exudynTestGlobals.testResult = testErrorSum

#%%+++++++++++++++++
if True:
    
    PlotSensor(None, closeAll=True)

    dataDisp = mbs.GetSensorStoredData(sDisp)
    dataLoad = mbs.GetSensorStoredData(sLoad)
    
    data = np.zeros((len(dataDisp),2))
    data[:,0] = dataDisp[:,3] #displacement in Z-direction
    data[:,1] = dataLoad[:,1] #force in X-direction
    data2 = np.zeros((len(dataDisp),2))
    data2[:,0] = dataDisp[:,1] #displacement in X-direction
    data2[:,1] = dataLoad[:,1] #force in X-direction
    
    mbs.PlotSensor(sensorNumbers=[data],components=[0], componentsX=[-1], 
                xLabel='displacement in Z direction',yLabel='Load (in X direction)')
    refSol = np.array([[0,1.088],[0.035,1.088]])
    mbs.PlotSensor(sensorNumbers=[refSol],components=[0],  
                xLabel='displacement in Z direction',yLabel='Load (in X direction)', newFigure=False,
                colorCodeOffset=1)
    # mbs.PlotSensor(sensorNumbers=[data2],components=[0], componentsX=[-1], 
    #             xLabel='displacement in X direction',yLabel='Load (in X direction)')


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#all results are taken from ANCFBeam (shear deformable 2-node 3D beam):
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#reference:
#critical load: 1.088


