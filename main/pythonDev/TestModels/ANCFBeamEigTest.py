#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Test model for ANCFBeam3D computing eigenvalues
#           Test as in ASME JCND paper: Nachbagauer, Gruber, Gerstmayr, 2013, Structural and Continuum Mechanics Approaches for a 3D Shear Deformable ANCF Beam Finite Element: Application to Static and Linearized Dynamic Examples
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

ie = 4
if True:
#for ie in range(7):
    nElements = 2**ie
    
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    
    compute2D = False #compare to 2D geometrically exact
    compute3D = True
    
    
    L = 2 #length of beam
    w = 0.05 #half width of beam
    u = 0.1*w
    v = w-u #height of I inner
    h = w #height Y
    
    sectionData = exu.BeamSection()
    
    L = 2
    # Material properties
    h = 0.4
    w = 0.4
    E = 1e9            # Young modulus
    rho = 7850

    nu = 0.3              # Poisson ratio
    ks= 10*(1+nu)/(12+11*nu)
    G = E/(2*(1+nu))      # Shear modulus
    # Cross-section properties
    A=h*w
    I = w*h**3/12 # Second moment of area of the beam cross-section

    rhoA = rho*A
    rhoI = rho*I
    rhoJ = rho*(2*I)
    EA = E*A
    EI = E*I
    kt = 0.8436 #used for eigenvalue test in paper (not mentioned, but comes out from Eq. 31)
    GJ = G*(2*I)*kt
    csPenaltyFactor = 1000 #for accurate torsion, this needs to be larger than 1 (1000)

    sectionData.stiffnessMatrix = np.diag([EA, G*A*ks, G*A*ks, GJ, EI, EI])
    #print('stiffness=', np.diag(sectionData.stiffnessMatrix))
    
    
    sectionData.inertia= np.diag([rhoJ, rhoI, rhoI])
    sectionData.massPerLength = rhoA
    
    sectionGeometry = exu.BeamSectionGeometry()
    
    #print('omegaTorsion=',np.pi*np.sqrt(G*kt/(rho*L**2)))
    
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
                                                   #testBeamRectangularSize = [h,w],
                                                   sectionData = sectionData,
                                                   crossSectionPenaltyFactor = [csPenaltyFactor]*3,
                                                   visualization=VANCFBeam(sectionGeometry=sectionGeometry)))
            n0 = n1
    
    
    if compute2D:
        lElem = L/nElements
        n2d0 = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[0,0,0]))
        nInit = n2d0
        for k in range(nElements):
            n2d1 = mbs.AddNode(NodeRigidBody2D(referenceCoordinates=[lElem*(k+1),0,0]))
    
            oBeam = mbs.AddObject(ObjectBeamGeometricallyExact2D(nodeNumbers=[n2d0,n2d1], physicsLength = lElem,
                                                                 physicsMassPerLength=sectionData.massPerLength,
                                                                 physicsAxialStiffness=sectionData.stiffnessMatrix[0,0],
                                                                 physicsBendingStiffness=EI,
                                                                 physicsShearStiffness=sectionData.stiffnessMatrix[1,1],
                                                                 physicsCrossSectionInertia=sectionData.inertia[2,2],
                                                                 ))
            n2d0 = n2d1
    
    
    #print(mbs.GetObject(oBeam))
    
    # print(mbs)
    mbs.Assemble()
    
    
    simulationSettings = exu.SimulationSettings()
    simulationSettings.solutionSettings.solutionWritePeriod = 5e-3  #output interval general
    simulationSettings.solutionSettings.sensorsWritePeriod = 5e-3  #output interval of sensors
    #simulationSettings.timeIntegration.realtimeFactor=0.1
    
    simulationSettings.timeIntegration.verboseMode = 0
    
    simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
    
    SC.visualizationSettings.bodies.beams.axialTiling = 20 #for drawing of small number of beams
    SC.visualizationSettings.general.drawWorldBasis = True
    SC.visualizationSettings.general.worldBasisSize = 0.1
    SC.visualizationSettings.openGL.multiSampling = 4
        
    # [M, K, D] = exu.solver.ComputeLinearizedSystem(mbs, simulationSettings, useSparseSolver=True)
    # print('M=',M.round(1))
    
    
    nModes = 3*(1+int(compute3D))+2
    nRigidModes = 0*3*(1+int(compute3D))
    if compute2D:
        constrainedCoordinates=[0,1,mbs.systemData.ODE2Size()-2]
    else:
        constrainedCoordinates=[0,1,2,mbs.systemData.ODE2Size()-8,mbs.systemData.ODE2Size()-7]
        #constrainedCoordinates=[0,1,2,5,7,mbs.systemData.ODE2Size()-8,mbs.systemData.ODE2Size()-7] #fix rotation at left
    
    # constrainedCoordinates=[]
    
    exu.SetWriteToConsole(False)
    compeig=exu.ComputeODE2Eigenvalues(mbs, simulationSettings, useSparseSolver=True, 
                                numberOfEigenvalues= nRigidModes+nModes, 
                                constrainedCoordinates=constrainedCoordinates,
                                convert2Frequencies= False)

    exu.SetWriteToConsole(True)
    #print('eigvalues=',np.sqrt(compeig[0][nRigidModes:]))
    print('# '+str(nElements)+' element(s):',list(np.sqrt(compeig[0][nRigidModes:]).round(6)))

#simply supported, free rotation at both sides; csPenaltyFactor=1000:
#  1 element(s): [0.001899, 309.097734, 352.131224, 618.195442, 618.195442, 1766.994062, 1766.994062, 55877.258482]
#  2 element(s): [0.000931, 129.82301, 129.82301, 287.569472, 352.128893, 704.248462, 1004.592265, 1236.390885]
#  4 element(s): [0.000691, 102.829804, 102.829804, 282.125158, 327.602821, 427.041681, 427.041681, 704.229815]
#  8 element(s): [0.000674, 97.361214, 97.361214, 280.771187, 321.400378, 353.279185, 353.279185, 655.175156]
# 16 element(s): [0.000448, 96.061481, 96.061481, 280.433304, 319.857843, 337.338054, 337.338054, 642.769663]
# 32 element(s): [0.000331, 95.740589, 95.740589, 280.348948, 319.47286, 333.500724, 333.500724, 639.684442]
# 64 element(s): [0.000795, 95.660616, 95.660616, 280.328015, 319.376657, 332.550464, 332.550464, 636.863144]
#128 element(s): [0.000905, 95.640639, 95.640639, 280.32309, 319.352609, 332.313464, 332.313464, 635.987765]
#256 element(s): [0.001128, 95.635645, 95.635645, 280.322475, 319.346597, 332.254249, 332.254249, 635.769143]

    
#when only fixing ry_z (this is not correct):
#  1 element(s): [ 186.41511732  309.09773402  618.19544247  618.19544247 1548.68904697 1766.9940621 ]
#  8 element(s): [ 97.36121376  97.36121376 140.09329934 280.77118704 353.2791845 353.2791845 ]
# 32 element(s): [ 95.20586618  95.74058919  95.74058919 280.34894758 333.50072364 333.50072364]
# 64 element(s): [ 72.24604603  95.66061633  95.66061634 280.32801497 332.55046432 332.55046432]
#128 element(s): [ 53.06420013  95.64063871  95.64063871 280.32309013 332.31346374 332.31346374]
#256 element(s): [ 38.26768775  95.63564527  95.63564528 280.32247526 332.25424881 332.25424882]
#512 element(s): [ 27.3314354   95.63439697  95.63439698 280.3235542  332.23944728 332.23944728]

    
    
if True: #show modes:
    if useGraphics:
        exu.StartRenderer()
    for i in range(nModes):
        iMode = nRigidModes+i
        mbs.systemData.SetODE2Coordinates(5*compeig[1][:,iMode], exudyn.ConfigurationType.Visualization)
        mbs.systemData.SetTime(np.sqrt(compeig[0][iMode]), exudyn.ConfigurationType.Visualization)
        mbs.SendRedrawSignal()
    
        mbs.WaitForUserToContinue()    

    if useGraphics:
        SC.WaitForRenderEngineStopFlag()
        exu.StopRenderer() #safely close rendering window!

if False: #solve dynamic (but without forces, nothing happens ...)
    exu.StartRenderer()
    
    exu.SolveDynamic(mbs, simulationSettings)
    
    SC.WaitForRenderEngineStopFlag()
    exu.StopRenderer() #safely close rendering window!


# exudynTestGlobals.testError = uLast[1] - (-2.2115028353806547) 
# exudynTestGlobals.testResult = uLast[1]



