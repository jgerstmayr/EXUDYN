#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test model for GeometricallyExactBeam2D, cantilever beam with tip force and torque
#
# Author:   Johannes Gerstmayr
# Date:     2021-03-25
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

compute2D = False
compute3D = True

#test examples
#2011 MUBO, Nachbagauer Pechstein Irschik Gerstmayr (2D)
#2013 CND, Nachbagauer Gruber Gerstmayr (static, 3D)
#2013 CND, Nachbagauer Gerstmayr (dynamic, 3D)
cases = ['CantileverLinear2011', 'Cantilever2011', 'GeneralBending2013', 'Eigenmodes2013']
nElementsList = [1,2,4,8,16,32,64,128,256,512,1024]
nElementsList = [8]
case = 2
useGraphics = False
verbose = False

bodyFixedLoad = False

nElements = 1

print('case=', cases[case])

for nElements in nElementsList:

    computeEigenmodes = False
    csFact = 1
    sectionData = exu.BeamSection()
    fTip = 0
    MxTip = 0
    MyTip = 0
    
    ks1=1 #shear correction, torsion
    ks2=1 #shear correction, bending
    ks3=1 #shear correction, bending

    if case == 0 or case == 1:
        caseName = cases[case]

        L = 2 #length of beam
        w = 0.1 #half width of beam
        h = 0.5 #height Y

        fTip = 5e5*h**3
        if case == 1:
            fTip *= 1000

        Em = 2.07e11
        rho = 1e2
        
        A=h*w
        nu = 0.3              # Poisson ratio
        ks2= 10*(1+nu)/(12+11*nu)
        ks3=ks2
    
    elif case == 2:
        L = 2 #length of beam
        h = 0.2 #height Y
        w = 0.4 #half width of beam
        Em = 2.07e11
        rho = 1e2

        A=h*w
        
        nu = 0.3              # Poisson ratio
        ks1= 0.5768 #torsion correction factor if J=Jyy+Jzz
        ks2= 0.8331
        ks3= 0.7961
        
        MxTip = 0.5e6
        MyTip = 2e6

        csFact = 100
        

    Gm = Em/(2*(1+nu))      # Shear modulus

    # Cross-section properties
    Iyy = h*w**3/12 # Second moment of area of the beam cross-section
    Izz = w*h**3/12 # Second moment of area of the beam cross-section
    J = (Iyy+Izz)   # approximation; Polar moment of area of the beam cross-section
    
    if case == 2:
        J = w*h**3/3 #approximation for thin rectangular cross sections
        ks1 = 0.229 #w/h=2
        
        print('J=', J, ', Iyy=', Iyy, ', Izz=', Izz)
        ==> shear correction factor questionable; results do not agree (implementation bug?)
    
    sectionData.stiffnessMatrix = np.diag([Em*A, Gm*A*ks2, Gm*A*ks3, Gm*J*ks1, Em*Iyy, Em*Izz])

    #print('stiffness=', np.diag(sectionData.stiffnessMatrix))
    # elif True:
    #     nElements = 16 #4 works
    #     L = 1
    #     fTip = -4.05e5*0.5*4 #*0.25
    #     bodyFixedLoad = True
    #     # Material properties
    #     Em = 2.1e11            # Young modulus
    #     nu = 0.3              # Poisson ratio
    #     Gm = Em/(2*(1+nu))      # Shear modulus
    #     k = 6*(1+nu)/(7+6*nu) # Shear correction factor
    
    #     # Cross-section properties
    #     r = 0.025              # Radius of the beam cross-section
    #     A = np.pi*r**2         # Area of the beam cross-section
    #     I = (1/4)*np.pi*(r**4) # Second moment of area of the beam cross-section
    #     J = 2*I                # Polar moment of area of the beam cross-section
    #     sectionData.stiffnessMatrix = np.diag([Em*A, k*Gm*A, k*Gm*A, Gm*J, Em*I, Em*I])
    #     print('stiffness=', np.diag(sectionData.stiffnessMatrix))
    
    rhoA = rho*A
    
    if False:
        EI = sectionData.stiffnessMatrix[5,5]
        #linear solution:
        uTip = fTip*L**3/(3*EI)
        print('u linear=',uTip)
    
    sectionData.inertia= rho*J*np.eye(3)
    sectionData.massPerLength = rhoA
    
    sectionGeometry = exu.BeamSectionGeometry()
    
    #points, in positive rotation sense viewing in x-direction, points in [Y,Z]-plane
    #points do not need to be closed!
    lp = exu.Vector2DList()
    if True:
        ff=0.2 #drawing
        lp.Append([h*ff,-w*ff])
        lp.Append([h*ff,w*ff])
        lp.Append([-h*ff,w*ff])
        lp.Append([-h*ff,-w*ff])
    
    sectionGeometry.polygonalPoints = lp
    #print('HERE\n',sectionGeometry.polygonalPoints)
    nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
    mnGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nGround, coordinate=0))
     
    
    eY=[0,1,0]
    eZ=[0,0,1]
    lElem = L/nElements
    if compute3D:
        initialRotations = eY+eZ
        n0 = mbs.AddNode(NodePoint3DSlope23(referenceCoordinates=[0,0,0]+initialRotations))
        nInit = n0
        for k in range(nElements):
            n1 = mbs.AddNode(NodePoint3DSlope23(referenceCoordinates=[lElem*(k+1),0,0]+initialRotations))
    
            oBeam = mbs.AddObject(ObjectANCFBeam(nodeNumbers=[n0,n1], physicsLength = lElem, 
                                                   testBeamRectangularSize = [h,w],
                                                   sectionData = sectionData,
                                                   crossSectionPenaltyFactor = [csFact,csFact,csFact],
                                                   visualization=VANCFBeam(sectionGeometry=sectionGeometry)))
            n0 = n1
    
    
        mTip = mbs.AddMarker(MarkerNodeRigid(nodeNumber = n1))
        if fTip != 0:
            mbs.AddLoad(Force(markerNumber=mTip, loadVector = [0,fTip,0], bodyFixed = bodyFixedLoad))

        if MxTip != 0 or MyTip != 0:
            mbs.AddLoad(Torque(markerNumber=mTip, loadVector = [MxTip, MyTip,0] ))

        for i in range(9):
            nm0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nInit, coordinate=i))
            mbs.AddObject(CoordinateConstraint(markerNumbers=[mnGround, nm0]))
    
    
    # if compute2D:
    #     lElem = L/nElements
    #     n2d0 = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0,0,0]))
    #     nInit = n2d0
    #     for k in range(nElements):
    #         n2d1 = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[lElem*(k+1),0,0]))
    
    #         oBeam = mbs.AddObject(ObjectBeamGeometricallyExact2D(nodeNumbers=[n2d0,n2d1], physicsLength = lElem,
    #                                                              physicsMassPerLength=sectionData.massPerLength,
    #                                                              physicsAxialStiffness=sectionData.stiffnessMatrix[0,0],
    #                                                              physicsBendingStiffness=EI,
    #                                                              physicsShearStiffness=sectionData.stiffnessMatrix[1,1],
    #                                                              physicsCrossSectionInertia=sectionData.inertia[2,2],
    #                                                              ))
    #         n2d0 = n2d1
    
    #     if not computeEigenmodes:
    #         mTip = mbs.AddMarker(MarkerNodeRigid(nodeNumber = n2d1))
    #         mbs.AddLoad(Force(markerNumber=mTip, loadVector = [0,fTip,0], bodyFixed = bodyFixedLoad ))
        
    #         for i in range(3):
    #             nm0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nInit, coordinate=i))
    #             mbs.AddObject(CoordinateConstraint(markerNumbers=[mnGround, nm0]))
    
    
    # print('EA=', sectionData.stiffnessMatrix[0,0])
    # print('EI=', EI)
    
    #print(mbs.GetObject(oBeam))
    
    # print(mbs)
    mbs.Assemble()
    
    tEnd = 100     #end time of simulation
    stepSize = 0.5*0.01*0.1    #step size; leads to 1000 steps
    
    simulationSettings = exu.SimulationSettings()
    simulationSettings.solutionSettings.solutionWritePeriod = 2e-2  #output interval general
    simulationSettings.solutionSettings.sensorsWritePeriod = 1e-1  #output interval of sensors
    simulationSettings.timeIntegration.numberOfSteps = int(tEnd/stepSize) #must be integer
    simulationSettings.timeIntegration.endTime = tEnd
    #simulationSettings.solutionSettings.solutionInformation = "This is the info\nNew line\n and another new line \n"
    simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 0.5
    #simulationSettings.timeIntegration.simulateInRealtime=True
    #simulationSettings.timeIntegration.realtimeFactor=0.1
    
    simulationSettings.timeIntegration.verboseMode = verbose
    simulationSettings.staticSolver.verboseMode = verbose
    
    #simulationSettings.parallel.numberOfThreads = 4
    simulationSettings.timeIntegration.newton.useModifiedNewton = True
    #simulationSettings.timeIntegration.newton.numericalDifferentiation.minimumCoordinateSize = 1e0
    
    simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-4
    simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
    
    # simulationSettings.displayComputationTime = True
    simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
    # simulationSettings.parallel.numberOfThreads = 4
    
    #simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 5e-5
    #simulationSettings.staticSolver.newton.numericalDifferentiation.forODE2 = True
    #simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
    # simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-4
    
    if nElements > 32 and case==0: #change tolerance, because otherwise no convergence
        simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
    if case == 1: #tolerance changed from 1e-8 to 5e-10 to achieve values of paper (1024 has difference at last digit in paper)
        simulationSettings.staticSolver.newton.relativeTolerance = 0.5e-9


    simulationSettings.staticSolver.numberOfLoadSteps = 10
    
    
    #add some drawing parameters for this example
    SC.visualizationSettings.nodes.drawNodesAsPoint=False
    SC.visualizationSettings.nodes.defaultSize=0.01
    
    SC.visualizationSettings.bodies.beams.axialTiling = 50
    SC.visualizationSettings.general.drawWorldBasis = True
    SC.visualizationSettings.general.worldBasisSize = 0.1
    SC.visualizationSettings.openGL.multiSampling = 4
    
    
    # [M, K, D] = exu.solver.ComputeLinearizedSystem(mbs, simulationSettings, useSparseSolver=True)
    # print('M=',M.round(1))
    
    if useGraphics:
        exu.StartRenderer()
        mbs.WaitForUserToContinue()
    
    # if computeEigenmodes:
    #     nModes = 3*(1+int(compute3D))
    #     nRigidModes = 3*(1+int(compute3D))
    #     if compute2D:
    #         constrainedCoordinates=[0,1,mbs.systemData.ODE2Size()-2]
    #     else:
    #         constrainedCoordinates=[0,1,2,5,mbs.systemData.ODE2Size()-8,mbs.systemData.ODE2Size()-7]
        
    #     # constrainedCoordinates=[]
            
    #     compeig=exu.ComputeODE2Eigenvalues(mbs, simulationSettings, useSparseSolver=False, 
    #                                 numberOfEigenvalues= nRigidModes+nModes, 
    #                                 constrainedCoordinates=constrainedCoordinates,
    #                                 convert2Frequencies= False)
        
    #     print('eigvalues=',np.sqrt(compeig[0][nRigidModes:]))
            
    #     if False: #show modes:
    #         for i in range(nModes):
    #             iMode = nRigidModes+i
    #             mbs.systemData.SetODE2Coordinates(5*compeig[1][:,iMode], exudyn.ConfigurationType.Visualization)
    #             mbs.systemData.SetTime(np.sqrt(compeig[0][iMode]), exudyn.ConfigurationType.Visualization)
    #             mbs.SendRedrawSignal()
            
    #             mbs.WaitForUserToContinue()
    
    # else:
    exu.SolveStatic(mbs, simulationSettings)
    # exu.SolveDynamic(mbs, simulationSettings)
    #exu.SolveDynamic(mbs, simulationSettings, solverType = exu.DynamicSolverType.RK44)
        
    
    if useGraphics:
        SC.WaitForRenderEngineStopFlag()
        exu.StopRenderer() #safely close rendering window!
        
    ##evaluate final (=current) output values
    if case < 2:
        pTip = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
        print('ne=',nElements, ', ux=',L-pTip[0], ', uy=',pTip[1])
    elif case == 2:
        pTip = mbs.GetNodeOutput(n1, exu.OutputVariableType.Displacement)
        print('ne=',nElements, ', u=',list(pTip))


# exudynTestGlobals.testError = uLast[1] - (-2.2115028353806547) 
# exudynTestGlobals.testResult = uLast[1]


# case= CantileverLinear2011
# ne= 1 , ux= 9.122730371124987e-08 , uy= 0.0006166665660910742
# ne= 2 , ux= 1.612930911054633e-07 , uy= 0.000761594059956051
# ne= 4 , ux= 1.817632329093044e-07 , uy= 0.0007978259537503514
# ne= 8 , ux= 1.8706537385781985e-07 , uy= 0.0008068839288072357
# ne= 16 , ux= 1.8840244786488825e-07 , uy= 0.0008091484226773552
# ne= 32 , ux= 1.8873743745650984e-07 , uy= 0.0008097145461515806
# ne= 64 , ux= 1.8882122909680277e-07 , uy= 0.0008098560770205014
# ne= 128 , ux= 1.8884218011550047e-07 , uy= 0.0008098914597375441
# ne= 256 , ux= 1.8884741792568605e-07 , uy= 0.0008099003054159232

# case= Cantilever2011
# ne= 1 , ux= 0.07140273972210331 , uy= 0.5422582284347087
# ne= 2 , ux= 0.12379212049042398 , uy= 0.6568711098369214
# ne= 4 , ux= 0.14346766608034445 , uy= 0.6959356132426464
# ne= 8 , ux= 0.14904162136084342 , uy= 0.7068152601333508
# ne= 16 , ux= 0.15048521504207302 , uy= 0.7096238913684579
# ne= 32 , ux= 0.15084943688898345 , uy= 0.7103320154842049
# ne= 64 , ux= 0.15094070339339627 , uy= 0.7105094270056275
# ne= 128 , ux= 0.15096353448141975 , uy= 0.7105538063569327
# ne= 256 , ux= 0.15096924251664645 , uy= 0.7105649015025949
# ne= 512 , ux= 0.15097066767540968 , uy= 0.7105676713847702
# ne= 1024 , ux= 0.1509710143377565 , uy= 0.7105683436170535

