
.. _testmodels-ancfbeamtest:

***************
ANCFBeamTest.py
***************

You can view and download this file on Github: `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test models for ANCFBeam (2-node 3D shear deformable ANCF beam element 
   #           with 2 cross section slope vectors); 
   #           test models: cantilever beam with tip force and torque
   #
   # Author:   Johannes Gerstmayr
   # Date:     2023-04-05
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import * #includes itemInterface and rigidBodyUtilities
   import exudyn.graphics as graphics #only import if it does not conflict
   
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
   #2013 CND, Nachbagauer Gruber Gerstmayr (static, 3D); "Structural and Continuum Mechanics Approaches for a 3D Shear Deformable ANCF Beam Finite Element: Application to Static and Linearized Dynamic Examples"
   ### not yet: 2013 CND, Nachbagauer Gerstmayr (dynamic, 3D)
   cases = ['CantileverLinear2011', 'Cantilever2011', 'GeneralBending2013', 'PrincetonBeamF2', 'PrincetonBeamF3', 'Eigenmodes2013']
   nElementsList = [1,2,4,8,16,32,64,128,256,512,1024]
   # nElementsList = [8,32, 128]
   nElements = 8
   
   betaList = [0,15,30,45,60,75,90]
   betaDegree = 45
   
   caseList = [0,1,2,3,4]
   case=4
   
   useGraphics = False
   verbose = False
   
   bodyFixedLoad = False
   testErrorSum = 0
   printCase = True
   
   #for nElements in nElementsList:
   #for betaDegree in betaList:
   for case in caseList:
       if printCase:
           printCase=False
           exu.Print('case=', case, cases[case])
       mbs.Reset()
       
       computeEigenmodes = False
       csFact = 1
       sectionData = exu.BeamSection()
       fTip = 0
       MxTip = 0
       MyTip = 0
   
       ks1=1 #shear correction, torsion
       ks2=1 #shear correction, bending
       ks3=1 #shear correction, bending
       ff=1 #drawing factor
   
       #define beam parameters and loads for different cases
       if case == 0 or case == 1:
           caseName = cases[case]
   
           L = 2 #length of beam
           w = 0.1 #width of beam
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
           w = 0.4 #width Z of beam
           Em = 2.07e11
           rho = 1e2
   
           A=h*w
       
           nu = 0.3              # Poisson ratio
           ks1= 0.5768 #torsion correction factor if J=Jyy+Jzz
           ks2= 0.8331
           ks3= 0.7961
       
           MxTip = 0.5e6
           MyTip = 2e6
   
           csFact = 10
       elif case == 3 or case == 4: #Princeton beam example
           L = 0.508       #length of beam
           h = 12.3777e-3  #height Y; 12.3777e-3 with Obrezkov's paper
           w = 3.2024e-3   #width Z of beam
           Em = 71.7e9
           ks1=0.198
           nu = 0.31
   
           ks2=1
           ks3=1
           # ks2=0.9
           # ks3=0.9
   
   
           rho = 1e2       #unused
           A=h*w
       
           MxTip = 0
           MyTip = 0
           if case == 3:
               fTip = 8.896    #F2
           elif case == 4:
               fTip = 13.345 #F3
           #if kk==0: exu.Print('load=', fTip)
           
           beta = betaDegree/180*pi #beta=0 => negative y-axis
           bodyFixedLoad = False
   
           csFact = 10
       
       Gm = Em/(2*(1+nu))      # Shear modulus
   
       #compute sectionData
       
       # Cross-section properties
       Iyy = h*w**3/12 # Second moment of area of the beam cross-section
       Izz = w*h**3/12 # Second moment of area of the beam cross-section
       J = (Iyy+Izz)   # approximation; Polar moment of area of the beam cross-section
   
       sectionData.stiffnessMatrix = np.diag([Em*A, Gm*A*ks2, Gm*A*ks3, Gm*J*ks1, Em*Iyy, Em*Izz])
   
   
       rhoA = rho*A
   
       if False:
           #linear solution:
           uzTip = fTip*L**3/(3*Em*Iyy)
           exu.Print('uz linear=',uzTip)
           uyTip = fTip*L**3/(3*Em*Izz)
           exu.Print('uy linear=',uyTip)
   
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
    
   
       eY=[0,1,0]
       eZ=[0,0,1]
       lElem = L/nElements
       if compute3D:
           initialRotations = eY+eZ
           #create beam nodes and elements
           n0 = mbs.AddNode(NodePointSlope23(referenceCoordinates=[0,0,0]+initialRotations))
           nInit = n0
           for k in range(nElements):
               n1 = mbs.AddNode(NodePointSlope23(referenceCoordinates=[lElem*(k+1),0,0]+initialRotations))
   
               oBeam = mbs.AddObject(ObjectANCFBeam(nodeNumbers=[n0,n1], physicsLength = lElem, 
                                                      sectionData = sectionData,
                                                      crossSectionPenaltyFactor = [csFact,csFact,csFact],
                                                      visualization=VANCFBeam(sectionGeometry=sectionGeometry)))
               n0 = n1
   
   
           mTip = mbs.AddMarker(MarkerNodeRigid(nodeNumber = n1))
           if fTip != 0:
               if case < 3:
                   mbs.AddLoad(Force(markerNumber=mTip, loadVector = [0,fTip,0], bodyFixed = bodyFixedLoad))
               elif case >= 3:
                   mbs.AddLoad(Force(markerNumber=mTip, loadVector = [0,-fTip*cos(beta),fTip*sin(beta)], bodyFixed = bodyFixedLoad))
   
           if MxTip != 0 or MyTip != 0:
               mbs.AddLoad(Torque(markerNumber=mTip, loadVector = [MxTip, MyTip,0]))#, bodyFixed = True ))
   
           for i in range(9):
               #if i != 4 and i != 8: #exclude constraining the slope lengths
               if True:
                   nm0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=nInit, coordinate=i))
                   mbs.AddObject(CoordinateConstraint(markerNumbers=[mnGround, nm0]))
   
   
       # exu.Print(mbs)
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
   
       simulationSettings.timeIntegration.newton.useModifiedNewton = True
   
       simulationSettings.timeIntegration.newton.numericalDifferentiation.relativeEpsilon = 1e-4
       simulationSettings.timeIntegration.newton.relativeTolerance = 1e-6
   
       simulationSettings.linearSolverType = exu.LinearSolverType.EigenSparse
   
       if nElements > 32 and case==0: #change tolerance, because otherwise no convergence
           simulationSettings.staticSolver.newton.relativeTolerance = 1e-6
       if case == 1: #tolerance changed from 1e-8 to 5e-10 to achieve values of paper (1024 has difference at last digit in paper)
           simulationSettings.staticSolver.newton.relativeTolerance = 0.5e-9
   
   
       simulationSettings.staticSolver.numberOfLoadSteps = 5
   
   
       #add some drawing parameters for this example
       SC.visualizationSettings.nodes.drawNodesAsPoint=False
       SC.visualizationSettings.nodes.defaultSize=0.01
   
       SC.visualizationSettings.bodies.beams.axialTiling = 50
       SC.visualizationSettings.general.drawWorldBasis = True
       SC.visualizationSettings.general.worldBasisSize = 0.1
       SC.visualizationSettings.openGL.multiSampling = 4
   
   
       # [M, K, D] = exu.solver.ComputeLinearizedSystem(mbs, simulationSettings, useSparseSolver=True)
       # exu.Print('M=',M.round(1))
   
       if useGraphics:
           SC.renderer.Start()
           SC.renderer.DoIdleTasks()
   
       # else:
       mbs.SolveStatic(simulationSettings)
       # mbs.SolveDynamic(simulationSettings)
       #mbs.SolveDynamic(simulationSettings, solverType = exu.DynamicSolverType.RK44)
       
   
       if useGraphics:
           SC.renderer.DoIdleTasks()
           SC.renderer.Stop() #safely close rendering window!
       
       ##evaluate final (=current) output values
       uTip = mbs.GetNodeOutput(n1, exu.OutputVariableType.Displacement)
       
       errorFact = 1
       if case != 1:
           errorFact *= 100
   
       testErrorSum += np.linalg.norm(uTip)
   
       
       
       if case < 2:
           pTip = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
           exu.Print('ne=',nElements, ', ux=',L-pTip[0], ', uy=',pTip[1])
       elif case == 2:
           rotTip = mbs.GetNodeOutput(n1, exu.OutputVariableType.Rotation)
           exu.Print('ne=',nElements, ', u=',list(uTip))
           # exu.Print('ne=',nElements, ', rot=',rotTip)
       elif case == 3 or case == 4:
           exu.Print('ne=', nElements, ', beta=', round(beta*180/pi,1), ', u=',uTip.round(7))
   
   
   exu.Print('Solution of ANCFBeam3Dtest=', testErrorSum)
   exudynTestGlobals.testError = testErrorSum - (1.010486312300459 ) 
   exudynTestGlobals.testResult = testErrorSum
   
   
   
   # case= 0/CantileverLinear2011 
   #NachbagauerPechsteinIrschikGerstmayrMUBO2011 (2D):
   # ne=1,   9.12273046eâ€“8, 6.16666566eâ€“4, 0.000193
   # ne=2,   1.61293091eâ€“7, 7.61594059eâ€“4, 4.831eâ€“5
   # ne=4,   1.81763233eâ€“7, 7.97825954eâ€“4, 1.208eâ€“5
   # ne=256, 1.88847418eâ€“7, 8.09900305eâ€“4, 2.945eâ€“9
   #Exudyn: ksFact=1
   # ne= 1 , ux= 9.122730637578513e-08 , uy= 0.0006166665660910789 
   # ne= 2 , ux= 1.612930911054633e-07 , uy= 0.0007615940599560586 
   # ne= 4 , ux= 1.8176323512975046e-07 , uy= 0.0007978259537503566 
   # ne= 8 , ux= 1.8706537496804287e-07 , uy= 0.0008068839288072378 
   # ne= 16 , ux= 1.8840244964124508e-07 , uy= 0.0008091484226773518 
   # ne= 32 , ux= 1.887374359021976e-07 , uy= 0.0008097145461515286 
   # ne= 64 , ux= 1.888212299849812e-07 , uy= 0.0008098560770202866 
   # ne= 128 , ux= 1.8884218011550047e-07 , uy= 0.000809891459736643 
   # ne= 256 , ux= 1.8884741770364144e-07 , uy= 0.0008099003054122335
   
   
   # case= 1/Cantilever2011 
   #NachbagauerPechsteinIrschikGerstmayrMUBO2011 (2D):
   # ne=1,    0.07140274, 0.54225823, 0.168310
   # ne=2,    0.12379212, 0.65687111, 0.053697
   # ne=4,    0.14346767, 0.69593561, 0.014633
   # ne=1024, 0.15097103, 0.71056837, 2.280eâ€“7
   
   #Exudyn: ksFact=1
   # ne= 1 , ux= 0.07140273975041422 , uy= 0.5422582285449739 
   # ne= 2 , ux= 0.12379212054619537 , uy= 0.6568711099777776 
   # ne= 4 , ux= 0.14346766617229956 , uy= 0.695935613449867 
   # ne= 8 , ux= 0.14904162148449163 , uy= 0.7068152604035266 
   # ne= 16 , ux= 0.15048521526298897 , uy= 0.709623891842095 
   # ne= 32 , ux= 0.15084943688011565 , uy= 0.7103320154655514 
   # ne= 64 , ux= 0.15094070328691145 , uy= 0.7105094267817303 
   # ne= 128 , ux= 0.15096353326024237 , uy= 0.7105538037895819 
   # ne= 256 , ux= 0.15096924149743085 , uy= 0.7105648993600513 
   # ne= 512 , ux= 0.15097066651939461 , uy= 0.7105676689547459 
   # ne= 1024 , ux= 0.15097102364723924 , uy= 0.7105683631862169 
   
   # case = 2:
   #2013 CND, Nachbagauer Gruber Gerstmayr (static, 3D); "Structural and Continuum Mechanics Approaches for a 3D Shear Deformable ANCF Beam Finite Element: Application to Static and Linearized Dynamic Examples"
   #Table 4:
   # SMF
   # 8,  1.0943e-4, 1.8638e-4, 1.8117e-2
   # 32, 1.0943e-4, 1.8625e-4, 1.8117e-2
   # ANSYS
   # 40, 1.0939e-4, 1.8646e-4, 1.8117e-2    
   #Exudyn, ksFact=10:
   # ne= 8 , u= [-0.00010900977088157404, -0.0001902100873246334, -0.01811732779800177] 
   # ne= 32 , u= [-0.00010941122286522997, -0.00018667435478355072, -0.01811739809277171] 
   # ne= 128 , u= [-0.00010943631319815239, -0.000186451835025629, -0.018117402461210096] 
   #==> in 2013 paper, element performed slightly better, especially in ux and uy terms
   
   # case = 3:
   #Princeton beam with ANSYS (Leonid Obrezkov / Aki Mikkola / Marko Matikainen et al.,
   #       Performance review of locking alleviation methods for continuum ANCF beam elements,
   #       Nonlinear Dynamics, Vol. 109, pp. 31â€“546, May 2022
   # beta=[0 15 30 45 60 75 90];
   if (case==3 or case == 4) and False:
       # F2=8.896
       # % ANSYS beam (10-199 el)
       ANSYSF2y=np.array([1.071417630E-002,  1.061328706E-002, 1.011169630E-002,  8.837226265E-003, 6.604665004E-003, 3.538889001E-003, 0])
       ANSYSF2z=np.array([0, 4.208232124E-002, 7.939482948E-002, 0.108987937,  0.129887616, 0.142194370, 0.146245978])
       exu.Print('refsol ANSYS F2=8.896:\n',ANSYSF2y.round(6), '\n', ANSYSF2z.round(6),sep='')
       # % ANSYS solid (el) (4x12x500) - finer mesh doesn't have much influence see in Size effect file
       # ANSYS_solid_y=[1.069752828E-002 1.057180106E-002 9.938278402E-003 8.686786771E-003 6.500006282E-003 3.481999513E-003 0];
       # ANSYS_solid_z=[0 4.101165651E-002 7.696749069E-002 0.105976311 0.127251299 0.139594740 0.143848652];
       
       # F3=13.345 
       # % ANSYS beam (10-199 el)
       ANSYSF3y=np.array([1.606423724E-002, 1.645825752E-002, 1.665873206E-002, 1.518618440E-002, 1.157837500E-002, 6.248967384E-003, 0])
       ANSYSF3z=np.array([0,                6.435812858E-002, 0.117735994,      0.156467239,      0.181861627,      0.196097131,      0.200677707])
       # % ANSYS solid (el) (4x12x500) - finer mesh see in Size effect file
       #ANSYS_solid_y=[1.603700622E-002 1.637026068E-002 1.640440775E-002 1.485055210E-002 1.127173264E-002 6.062461977E-003  0])
       #ANSYS_solid_z=[0 6.270699533E-002 0.113752002 0.153554457 0.179978534  0.192972233 0.197669499])
       exu.Print('refsol ANSYS F3=13.345:\n',ANSYSF3y.round(6), '\n', ANSYSF3z.round(6),sep='')
   #Exudyn results for Princeton beam:
   #not exactly the same, but around the previous values with HOTINT
   #using 16 elements, csFact=10 (no influence)
   # F2=8.896
   # case= 3, PrincetonBeam
   # ne= 16 , beta= 0.0 , u= [-0.0001352 -0.0107023  0.       ]
   # ne= 16 , beta= 15.0 , u= [-0.0022414 -0.0106295  0.0421374]
   # ne= 16 , beta= 30.0 , u= [-0.0076567 -0.0101861  0.0794434]
   # ne= 16 , beta= 45.0 , u= [-0.0143664 -0.0089529  0.1089703]
   # ne= 16 , beta= 60.0 , u= [-0.0204225 -0.0067182  0.1297877]
   # ne= 16 , beta= 75.0 , u= [-0.0245093 -0.0036079  0.1420319]
   # ne= 16 , beta= 90.0 , u= [-0.0259403 -0.         0.1460608]
   
   # F3=13.345
   # case= 4, PrincetonBeam
   # ne= 16 , beta= 0.0 , u= [-0.0003039 -0.0160454  0.       ]
   # ne= 16 , beta= 15.0 , u= [-0.005319  -0.0165469  0.064622 ]
   # ne= 16 , beta= 30.0 , u= [-0.0171901 -0.0169316  0.1179818]
   # ne= 16 , beta= 45.0 , u= [-0.0303357 -0.0155488  0.1565214]
   # ne= 16 , beta= 60.0 , u= [-0.0411035 -0.0118996  0.1817173]
   # ne= 16 , beta= 75.0 , u= [-0.0479101 -0.0064334  0.1958343]
   # ne= 16 , beta= 90.0 , u= [-0.0502184 -0.         0.2003738]
   
   
   
   


