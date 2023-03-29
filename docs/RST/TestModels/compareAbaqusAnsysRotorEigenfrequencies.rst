
.. _testmodels-compareabaqusansysrotoreigenfrequencies:

******************************************
compareAbaqusAnsysRotorEigenfrequencies.py
******************************************

You can view and download this file on Github: `compareAbaqusAnsysRotorEigenfrequencies.py <https://github.com/jgerstmayr/EXUDYN/tree/master/main/pythonDev/TestModels/compareAbaqusAnsysRotorEigenfrequencies.py>`_

.. code-block:: python
   :linenos:

   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   # This is an EXUDYN example
   #
   # Details:  Example to compute eigenfrequencies of a rotor
   #           NOTE that this example requires files from a subdirectory testData as provided on github
   #
   # Author:   Stefan Holzinver, Johannes Gerstmayr
   # Date:     2020-05-18
   #
   # Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
   #
   #+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   
   import numpy as np
   from scipy.sparse import linalg
   import scipy as sp
   
   import exudyn as exu
   from exudyn.utilities import *
   from exudyn.FEM import *
   
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
   
   numberOfModes = 18
   useSparseSolverRoutine = False
   
   errorResult = 0
   
   testDataDir = "testData/"
   #if useGraphics:
   #    testDataDir = "testData/"
   
   ###############################################################################
   # Ansys - lumped mass matrix formulation - Sparse Matrix - MMF format
   ###############################################################################
   
   #read finite element model
   exudynTestGlobals.testResult = 0
   for testNumber in range(2):
       if testNumber == 1:
           useSparseSolverRoutine = True
       
       fem = FEMinterface()
       #exu.Print("read matrices ...")
       fem.ReadMassMatrixFromAnsys(fileName=testDataDir + 'rotorAnsysMassMatrixSparse.txt', 
                                   dofMappingVectorFile=testDataDir + 'rotorAnsysMatrixDofMappingVector.txt')
       fem.ReadStiffnessMatrixFromAnsys(fileName=testDataDir + 'rotorAnsysStiffnessMatrixSparse.txt',
                                   dofMappingVectorFile=testDataDir + 'rotorAnsysMatrixDofMappingVector.txt')
       
       #exu.Print("compute eigenmodes ...")
       fem.ComputeEigenmodes(numberOfModes, useSparseSolver = useSparseSolverRoutine)
       
       if useGraphics:
           exu.Print('natural frequencies from Ansys model (Lumped Mass Matrix, MMF-Format)', fem.GetEigenFrequenciesHz()[0:numberOfModes])
       
       if not useSparseSolverRoutine:
           f6 = fem.GetEigenFrequenciesHz()[6] - 104.63701055079783 #reference solution
       else:
           #compare with dense matrix solution; 
           #due to random initialization, the results change with every computation ==> approx. 1e-11 repeatability
           f6 = fem.GetEigenFrequenciesHz()[6] - 104.6370105507865 
       f6*=1e-6 #use offset also for abaqus, as it gives non-reproducible results in dense case (32/64bit?)
       exu.Print('natural frequencies from Ansys model, sparse=',str(useSparseSolverRoutine),":", fem.GetEigenFrequenciesHz()[6] )
       errorResult += f6
   
       exudynTestGlobals.testResult += 1e-6*fem.GetEigenFrequenciesHz()[6]
   
       ###############################################################################
       # Abaqus
       ###############################################################################
       
       #read finite element model
       fem = FEMinterface()
       #exu.Print("read matrices ...")
       fem.ReadMassMatrixFromAbaqus(fileName=testDataDir + 'rotorDiscTestMASS1.mtx')
       fem.ReadStiffnessMatrixFromAbaqus(fileName=testDataDir + 'rotorDiscTestSTIF1.mtx')
       fem.ScaleStiffnessMatrix(1e-2) #in Ansys, the stiffness matrix is already scaled!
           
       #exu.Print("compute eigenmodes ...")
       fem.ComputeEigenmodes(numberOfModes, useSparseSolver = useSparseSolverRoutine)
   
       if useGraphics:
           exu.Print('natural frequencies from Abaqus model (Lumped Mass Matrix)',fem.GetEigenFrequenciesHz()[0:numberOfModes])
       
       if not useSparseSolverRoutine:
           f6 = fem.GetEigenFrequenciesHz()[6] - 104.6370132606291 #reference solution
       else:
           #compare with dense matrix solution; 
           #due to random initialization, the results change with every computation ==> approx. 1e-11 repeatability
           f6 = fem.GetEigenFrequenciesHz()[6] - 104.6370132606291 
       f6*=1e-6 #use offset also for abaqus, as it gives non-reproducible results in dense case (32/64bit?)
       exu.Print('natural frequencies from Abaqus model, sparse=',str(useSparseSolverRoutine),":", fem.GetEigenFrequenciesHz()[6] )
       errorResult += f6
       exudynTestGlobals.testResult += 1e-6*fem.GetEigenFrequenciesHz()[6]
   
   exu.Print('error of compareAbaqusAnsysRotorEigenfrequencies (due to sparse solver)=',errorResult)
   if abs(errorResult) < 1e-15: #usually of size 1e-17
       errorResult = 0 #due to randomized sparse solver results, take this treshold!
       
   exu.Print('solution of compareAbaqusAnsysRotorEigenfrequencies (with treshold)=',errorResult)
   exudynTestGlobals.testError = errorResult #2020-05-22: 0
   #exudynTestGlobals.testResult computed above
   
   
   


