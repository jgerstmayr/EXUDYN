
.. _examples-shapeoptimization:

********************
shapeOptimization.py
********************

You can view and download this file on Github: `shapeOptimization.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/Examples/shapeOptimization.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example for simple shape optimization with NGsolve/Netgen
   #
   # Author:   Johannes Gerstmayr 
   # Date:     2023-05-23
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import exudyn as exu
   from exudyn.utilities import *
   from exudyn.FEM import *
   from exudyn.graphicsDataUtilities import *
   from exudyn.processing import GeneticOptimization, ParameterVariation, PlotOptimizationResults2D, Minimize
   
   import numpy as np
   import time
   #import timeit
   
   import exudyn.basicUtilities as eb
   import exudyn.rigidBodyUtilities as rb
   import exudyn.utilities as eu
   
   import numpy as np
   
   useGraphics = True
   
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #netgen/meshing part:
   fem = FEMinterface()
   #standard:
   L = 1     #Length of beam
   wy = 0.25*L
   #wzNominal = 0.25*L
   wzNominal = 0.25*L
   
   meshSize = wzNominal*0.25 #*0.25
   nModes = 8
   
   rho = 1000 #polymer or similar
   Emodulus=5e9
   nu=0.3
   meshCreated = False
   verbose = True
   
   import ngsolve as ngs
   import netgen
   from netgen.meshing import *
   
   from netgen.geom2d import unit_square
   #import netgen.libngpy as libng
   from netgen.csg import *
   
   
   #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++
   #if True: #needs netgen/ngsolve to be installed with pip install; to compute mesh, see e.g.: https://github.com/NGSolve/ngsolve/releases
   # rangeMax = 4
   # for param in range(rangeMax):
   def ParameterFunction(parameterSet):
       SC = exu.SystemContainer()
       mbs = SC.AddSystem()
       
       try:
           #++++++++++++++++++++++++++++++++++++++++++++++
           #++++++++++++++++++++++++++++++++++++++++++++++
           #store default parameters in structure (all these parameters can be varied!)
           class P: pass #create emtpy structure for parameters; simplifies way to update parameters
           P.x0 = 0.4*L
           P.x1 = 0.6*L
           P.x2 = 0.85*L
           P.r0 = wy*0.2
           P.r1 = wy*0.2
           P.r2 = wy*0.2
       
           P.computationIndex = 'Ref'
           
           # #now update parameters with parameterSet (will work with any parameters in structure P)
           for key,value in parameterSet.items():
               setattr(P,key,value)
       
           verbose = P.computationIndex == 'Ref'
           
           geo = CSGeometry()
           
           Vblock = L*wy*wzNominal
           Vcyls = P.r0**2*pi*wzNominal + P.r1**2*pi*wzNominal + P.r2**2*pi*wzNominal
           wz = wzNominal*Vblock/(Vblock-Vcyls) #increase thickness
           
           block = OrthoBrick(Pnt(0,-0.5*wy,-0.5*wz),Pnt(L,0.5*wy,0.5*wz))
           cyl0 = Cylinder(Pnt(P.x0,0,-0.5*wz),Pnt(P.x0,0,0.5*wz),P.r0)
           cyl1 = Cylinder(Pnt(P.x1,0,-0.5*wz),Pnt(P.x1,0,0.5*wz),P.r1)
           cyl2 = Cylinder(Pnt(P.x2,0,-0.5*wz),Pnt(P.x2,0,0.5*wz),P.r2)
           
           part = block - (cyl0+cyl1+cyl2)
           geo.Add(part)
           
           #Draw (geo)
           
           mesh = ngs.Mesh( geo.GenerateMesh(maxh=meshSize, curvaturesafety=1.25)) #smaller values give coarser mesh
           mesh.Curve(2)
       
           if False: #set this to true, if you want to visualize the mesh inside netgen/ngsolve
               # import netgen
               #+++++++++++++++++++
               import netgen.gui
               ngs.Draw (mesh)
               for i in range(40):
                   netgen.Redraw() #this makes the window interactive
                   time.sleep(0.05)
       
           meshCreated = True
           #+++++++++++++++++++++++++++++++++++++++++++++++++++++
           #Use fem to import FEM model and create FFRFreducedOrder object
           fem.ImportMeshFromNGsolve(mesh, density=rho, youngsModulus=Emodulus, poissonsRatio=nu, meshOrder=2)
       
           # print("nNodes=",fem.NumberOfNodes())
           #check total mass:
           if verbose:
               print("nNodes=",fem.NumberOfNodes())
               nNodes = fem.NumberOfNodes()
               M = CSRtoScipySparseCSR(fem.GetMassMatrix(sparse=True))
               Phit = np.kron(np.ones(nNodes),np.eye(3)).T
               totalMass = (Phit.T @ M @ Phit)[0,0]
       
               print("total mass=",totalMass)
           
           #+++++++++++++++++++++++++++++++++++++++++++++++++++++
           #compute Hurty-Craig-Bampton modes
       
           if True:
               pLeft = [0,-0.5*wy,-0.5*wz]
               pRight = [L,-0.5*wy,-0.5*wz]
               nTip = fem.GetNodeAtPoint(pRight) #tip node (do not use midpoint, as this may not be a mesh node ...)
               #print("nMid=",nMid)
               nodesLeftPlane = fem.GetNodesInPlane(pLeft, [-1,0,0])
               lenNodesLeftPlane = len(nodesLeftPlane)
               weightsLeftPlane = np.array((1./lenNodesLeftPlane)*np.ones(lenNodesLeftPlane))
           
               nodesRightPlane = fem.GetNodesInPlane(pRight, [-1,0,0])
               lenNodesRightPlane = len(nodesRightPlane)
               weightsRightPlane = np.array((1./lenNodesRightPlane)*np.ones(lenNodesRightPlane))
           
               #boundaryList = [nodesLeftPlane, nodesRightPlane] #second boudary (right plane) not needed ...
               boundaryList = [nodesLeftPlane] 
           
               # if verbose:        
               #     print("compute HCB modes... ")
           
               start_time = time.time()
               fem.ComputeHurtyCraigBamptonModes(boundaryNodesList=boundaryList, 
                                                 nEigenModes=nModes, 
                                                 useSparseSolver=True,
                                                 computationMode = HCBstaticModeSelection.RBE2,
                                                 verboseMode=False, 
                                                 timerTreshold=100000)
               
               fMin = min(fem.GetEigenFrequenciesHz())
               if verbose:        
                   # print("HCB modes needed %.3f seconds" % (time.time() - start_time))
                   print('smallest Eigenfrequency=',fMin)
               
           
               #alternatives:
               #fem.ComputeEigenModesWithBoundaryNodes(boundaryNodes=nodesLeftPlane, nEigenModes=nModes, useSparseSolver=False)
               #fem.ComputeEigenmodes(nModes, excludeRigidBodyModes = 6, useSparseSolver = True)
               #print("eigen freq.=", fem.GetEigenFrequenciesHz())
               
               
               #+++++++++++++++++++++++++++++++++++++++++++++++++++++
               #animate modes
               if verbose:
                   print("create CMS element ...")
                   cms = ObjectFFRFreducedOrderInterface(fem)
                   
                   objFFRF = cms.AddObjectFFRFreducedOrder(mbs, positionRef=[0,0,0], 
                                                                 initialVelocity=[0,0,0], 
                                                                 initialAngularVelocity=[0,0,0],
                                                                 gravity=[0,-9.81,0],
                                                                 color=[0.1,0.9,0.1,1.],
                                                                 )
                   
                   from exudyn.interactive import AnimateModes
                   mbs.Assemble()
                   SC.visualizationSettings.nodes.show = False
                   SC.visualizationSettings.openGL.showFaceEdges = True
                   SC.visualizationSettings.openGL.multiSampling=4
                   
                   #+++++++++++++++++++++++++++++++++++++++
                   #animate modes of ObjectFFRFreducedOrder (only needs generic node containing modal coordinates)
                   SC.visualizationSettings.general.autoFitScene = False #otherwise, model may be difficult to be moved
                   
                   nodeNumber = objFFRF['nGenericODE2'] #this is the node with the generalized coordinates
                   AnimateModes(SC, mbs, nodeNumber,  runOnStart= True)
                   # import sys
                   # sys.exit()
           
               #do not show values larger than -120 to have nicer plots ...
               return 140 + min(-100,-fMin) #maximize eigenfrequency => minimize ...
       except:
           return 140-99
   
   
   
   #%%
   #now perform parameter variation
   if __name__ == '__main__': #include this to enable parallel processing
       import time
   
       # refval = ParameterFunction({'computationIndex':'Ref'}) # compute reference solution
       # import sys
       # sys.exit()
       #%%++++++++++++++++++++++++++++++++++++++++++++++++++++
       #GeneticOptimization    
       start_time = time.time()
       rMax = 0.5*wy
       rRange = (0.3*rMax, 0.95*rMax)
   
       x0Range = (1*rMax, 3*rMax)
       x1Range = (4.5*rMax, 5*rMax)
       x2Range = (6*rMax, 7*rMax)
       
       
       if False:
           #this option does not work here, as it would require the ParameterFunction to restrict the ranges to feasible values
           [pOpt, vOpt, pList, values] = Minimize(
                                                 objectiveFunction = ParameterFunction, 
                                                 parameters = {'r0':rRange, 'r1':rRange, 'r2':rRange, 
                                                               'x0':x0Range, 'x1':x1Range, 'x2':x2Range, }, #parameters provide search range
                                                showProgress=True,
                                                enforceBounds=True,
                                                addComputationIndex = True,
                                                options={'maxiter':100},
                                                resultsFile='solution/shapeOptimization.txt'
                                                )
       else:
           [pOpt, vOpt, pList, values] = GeneticOptimization(
                                                 objectiveFunction = ParameterFunction, 
                                                 parameters = {'r0':rRange, 'r1':rRange, 'r2':rRange, 
                                                               'x0':x0Range, 'x1':x1Range, 'x2':x2Range, }, #parameters provide search range
                                                 numberOfGenerations = 10,
                                                 populationSize = 100,
                                                 elitistRatio = 0.1,
                                                 # crossoverProbability = 0, #makes no sense!
                                                 rangeReductionFactor = 0.7,
                                                 randomizerInitialization=0, #for reproducible results
                                                 #distanceFactor = 0.1, 
                                                 distanceFactor = 0., 
                                                 debugMode=False,
                                                 useMultiProcessing=True, #may be problematic for test
                                                 numberOfThreads = 20,
                                                 addComputationIndex=True,
                                                 showProgress=True, 
                                                 resultsFile = 'solution/shapeOptimization.txt',
                                                 )
   
       exu.Print("--- %s seconds ---" % (time.time() - start_time))
   
       exu.Print("[pOpt, vOpt]=", [pOpt, vOpt])
       u = vOpt
       exu.Print("optimized eigenfrequency=",-u)
   
       if True:
           # from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
           import matplotlib.pyplot as plt
   
           plt.close('all')
           [figList, axList] = PlotOptimizationResults2D(pList, values, yLogScale=False)
           
       refval = ParameterFunction({**pOpt,'computationIndex':'Ref'} ) # compute reference solution
   
   #i9, 14 cores, numberOfThreads = 20: 575.85 seconds (ngen=10, pposize=100)
   #with distanceFactor: 117.16 (ngen=12, popsize=200)
   #without distanceFactor: 117.5287   (ngen=10, popsize=100)
   
   #pOpt = {'r0': 0.04012197560525166, 'r1': 0.08694009640170029, 'r2': 0.11845643292562649, 'x0': 0.2828892936420842, 'x1': 0.6170637302367472, 'x2': 0.8749125935436654}
   #optimized eigenfrequency= 117.52874179749946 
   #p0 = {'r0': 0.0625, 'r1': 0.0625, 'r2': 0.0625, 'x0': 0.25, 'x1': 0.59375, 'x2': 0.8125}


