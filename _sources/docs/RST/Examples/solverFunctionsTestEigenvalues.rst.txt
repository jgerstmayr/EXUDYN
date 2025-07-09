
.. _examples-solverfunctionstesteigenvalues:

*********************************
solverFunctionsTestEigenvalues.py
*********************************

You can view and download this file on Github: `solverFunctionsTestEigenvalues.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/solverFunctionsTestEigenvalues.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Test of static solver, computing eigenvalues and showing eigenmodes; uses scipy.linalg
   #
   # Author:   Johannes Gerstmayr
   # Date:     2020-01-06
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.itemInterface import *
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   
   exu.Print("\n\n++++++++++++++++++++++++++\nStart EXUDYN version "+exu.config.Version()+"\n")
   
   #background
   rect = [-2,-2,2,2] #xmin,ymin,xmax,ymax
   background0 = {'type':'Line', 'color':[0.1,0.1,0.8,1], 'data':[rect[0],rect[1],0, rect[2],rect[1],0, rect[2],rect[3],0, rect[0],rect[3],0, rect[0],rect[1],0]} #background
   background1 = {'type':'Circle', 'radius': 0.1, 'position': [-1.5,0,0]} 
   background2 = {'type':'Text', 'position': [-1,-1,0], 'text':'Example with text\nin two lines:.=!'} #background
   oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0], visualization=VObjectGround(graphicsData= [background0, background1, background2])))
   
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   #cable:
   mypi = 3.141592653589793
   
   L=2.                   # length of ANCF element in m
   #L=mypi                 # length of ANCF element in m
   E=2.07e11              # Young's modulus of ANCF element in N/m^2
   rho=7800               # density of ANCF element in kg/m^3
   b=0.01                  # width of rectangular ANCF element in m
   h=0.01                  # height of rectangular ANCF element in m
   A=b*h                  # cross sectional area of ANCF element in m^2
   I=b*h**3/12            # second moment of area of ANCF element in m^4
   EI = E*I
   rhoA = rho*A
   f=3*EI/L**2           # tip load applied to ANCF element in N
   
   exu.Print("load f="+str(f))
   exu.Print("EI="+str(EI))
   exu.Print("rhoA="+str(rhoA))
   
   nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0])) #ground node for coordinate constraint
   mGround = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nGround, coordinate=0)) #Ground node ==> no action
   
   cableList=[]
   
   
   
   nc0 = mbs.AddNode(Point2DS1(referenceCoordinates=[0,0,1,0]))
   nElements = 32 #2020-01-03: now works even with 64 elements if relTol=1e-5; did not work with 16 elements (2019-12-07)
   lElem = L / nElements
   for i in range(nElements):
       nLast = mbs.AddNode(Point2DS1(referenceCoordinates=[lElem*(i+1),0,1,0]))
       elem=mbs.AddObject(Cable2D(physicsLength=lElem, physicsMassPerLength=rho*A, 
                                  physicsBendingStiffness=E*I, physicsAxialStiffness=E*A*0.1, 
                                  nodeNumbers=[int(nc0)+i,int(nc0)+i+1], useReducedOrderIntegration=True))
       cableList+=[elem]
   
   mANCF0 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=0))
   mANCF1 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=1))
   mANCF2 = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber = nc0, coordinate=3))
       
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF0]))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF1]))
   mbs.AddObject(CoordinateConstraint(markerNumbers=[mGround,mANCF2]))
   
   #mANCFLast = mbs.AddMarker(MarkerNodePosition(nodeNumber=nLast)) #force
   #nLoad = mbs.AddLoad(Force(markerNumber = mANCFLast, loadVector = [-450, 0, 0])) #will be changed in load steps
   
   mANCFrigid = mbs.AddMarker(MarkerBodyRigid(bodyNumber=elem, localPosition=[lElem,0,0])) #local position L = beam tip
   mbs.AddLoad(Torque(markerNumber = mANCFrigid, loadVector = [0, 0, E*I*0.25*mypi]))
   
   #mANCFnode = mbs.AddMarker(MarkerNodeRigid(nodeNumber=nLast)) #local position L = beam tip
   #mbs.AddLoad(Torque(markerNumber = mANCFnode, loadVector = [0, 0, 4*E*I*0.25*mypi]))
   #mbs.AddLoad(Force(markerNumber = mANCFnode, loadVector = [0, 0.4*E*I*0.25*mypi,0]))
   
   
   
   mbs.Assemble()
   #exu.Print(mbs)
   
   simulationSettings = exu.SimulationSettings() #takes currently set values or default values
   
   
   #SC.visualizationSettings.bodies.showNumbers = False
   SC.visualizationSettings.nodes.defaultSize = 0.025
   
   #simulationSettings.staticSolver.newton.numericalDifferentiation.relativeEpsilon = 1e-9
   simulationSettings.staticSolver.verboseMode = 1
   #simulationSettings.staticSolver.verboseModeFile = 0
   
   #simulationSettings.staticSolver.newton.absoluteTolerance = 1e-8
   simulationSettings.staticSolver.newton.relativeTolerance = 1e-6 #1e-5 works for 64 elements
   simulationSettings.staticSolver.newton.maxIterations = 20 #50 for bending into circle
   #simulationSettings.displayComputationTime = True
   
       
   SC.renderer.Start()
   
   simulationSettings.staticSolver.numberOfLoadSteps = 100
   simulationSettings.staticSolver.adaptiveStep = True
   
   staticSolver = exu.MainSolverStatic()
   #staticSolver.SolveSystem(mbs, simulationSettings)
   #print(staticSolver.timer)
   
   import numpy as np
   from scipy.linalg import eigh, eig #eigh for symmetric matrices, positive definite
   
   #+++++++++++++++++++++++++++++++++++++
   #compute eigenvalue problem:        
   
   staticSolver.InitializeSolver(mbs, simulationSettings)
   #staticSolver.SolveSteps(mbs, simulationSettings) #if preloaded
   #staticSolver.FinalizeSolver(mbs, simulationSettings)    
   
   
   nODE2 = staticSolver.GetODE2size()
   
   #raise ValueError("")
   #compute mass matrix:
   staticSolver.ComputeMassMatrix(mbs, 1)#simulationSettings)
   m = staticSolver.GetSystemMassMatrix()
   #print("m =",m)
   
   #compute stiffness matrix (systemJacobian is larger!)
   staticSolver.ComputeJacobianODE2RHS(mbs, scalarFactor_ODE2=-1,scalarFactor_ODE2_t=0,scalarFactor_ODE1=0)
   staticSolver.ComputeJacobianAE(mbs, 1)
   K = staticSolver.GetSystemJacobian()
   #print("K =",K)
   
   K2 = K[0:nODE2,0:nODE2]
   
   [eigvals, eigvecs] = eigh(K2, m) #this gives omega^2 ... squared eigen frequencies (rad/s)
   ev = np.sort(a=abs(eigvals)) #there may be very small eigenvalues
   print('eigvals=',eigvals)
   
   nEig = 4
   for i in range(len(ev)):
       ev[i] = ev[i]**0.5
   
   print("omega numerical =",ev[3:3+nEig])
   
   
   #analytical: bending eigenfrequency of free-free beam:
   #4.7300, 7.8532, 10.9956, 14.1371, 17.2787 (cosh(beta) * cos(beta) = 1)
   #find roots beta:
   #from mpmath import *
   #mp.dps = 16 #digits
   #for i in range(10): print(findroot(lambda x: cosh(x) * cos(x) - 1, 3*i+4.7))
   beta = [4.730040744862704, 7.853204624095838, 10.99560783800167, 14.13716549125746, 17.27875965739948, 20.42035224562606, 23.56194490204046, 26.70353755550819, 29.84513020910325]
   omega = np.zeros(nEig)
   for i in range(nEig):
       omega[i] = ((beta[i]/L)**4 * (EI/rhoA))**0.5
   
   print('omega analytical =',omega)
   
   
   #mbs.SolveStatic(simulationSettings)
   
   
   SC.renderer.DoIdleTasks()
   SC.renderer.Stop() #safely close rendering window!
   
   
   


