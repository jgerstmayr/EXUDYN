
.. _sec-module-solver:

Module: solver
==============

The solver module provides interfaces to static, dynamic and eigenvalue solvers.
Most of the solvers are implemented inside the C++ core.

- Author:    Johannes Gerstmayr  
- Date:      2020-12-02 
- Notes:     Solver functions are included directly in exudyn and can be used with exu.SolveStatic(...) 


.. _sec-solver-solvererrormessage:

Function: SolverErrorMessage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SolverErrorMessage <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L22>`__\ (\ ``solver``\ , \ ``mbs``\ , \ ``isStatic = False``\ , \ ``showCausingObjects = True``\ , \ ``showCausingNodes = True``\ , \ ``showHints = True``\ )

- | \ *function description*\ :
  | helper function for unique error and helper messages


----

.. _sec-solver-solvestatic:

Function: SolveStatic
^^^^^^^^^^^^^^^^^^^^^
`SolveStatic <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L151>`__\ (\ ``mbs``\ , \ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``updateInitialValues = False``\ , \ ``storeSolver = True``\ , \ ``showHints = False``\ , \ ``showCausingItems = True``\ )

- | \ *function description*\ :
  | solves the static mbs problem using simulationSettings; check theDoc.pdf for MainSolverStatic for further details of the static solver; NOTE that this function is directly available from exudyn (using exudyn.SolveStatic(...))
- | \ *input*\ :
  | \ ``mbs``\ : the MainSystem containing the assembled system; note that mbs may be changed upon several runs of this function
  | \ ``simulationSettings``\ : specific simulation settings out of exu.SimulationSettings(), as described in Section :ref:`sec-solutionsettings`\ ; use options for newton, discontinuous settings, etc., from staticSolver sub-items
  | \ ``updateInitialValues``\ : if True, the results are written to initial values, such at a consecutive simulation uses the results of this simulation as the initial values of the next simulation
  | \ ``storeSolver``\ : if True, the staticSolver object is stored in the mbs.sys dictionary as mbs.sys['staticSolver'], and simulationSettings are stored as mbs.sys['simulationSettings']
- | \ *output*\ :
  | returns True, if successful, False if fails; if storeSolver = True, mbs.sys contains staticSolver, which allows to investigate solver problems (check theDoc.pdf Section :ref:`sec-solversubstructures`\  and the items described in Section :ref:`sec-mainsolverstatic`\ )
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.itemInterface import *
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  #create simple system:
  ground = mbs.AddObject(ObjectGround())
  mbs.AddNode(NodePoint())
  body = mbs.AddObject(MassPoint(physicsMass=1, nodeNumber=0))
  m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=ground))
  m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body))
  mbs.AddObject(CartesianSpringDamper(markerNumbers=[m0,m1], stiffness=[100,100,100]))
  mbs.AddLoad(LoadForceVector(markerNumber=m1, loadVector=[10,10,10]))
  mbs.Assemble()
  sims = exu.SimulationSettings()
  sims.timeIntegration.endTime = 10
  success = exu.SolveStatic(mbs, sims, storeSolver = True)
  print("success =", success)
  print("iterations = ", mbs.sys['staticSolver'].it)
  print("pos=", mbs.GetObjectOutputBody(body,localPosition=[0,0,0],
        variableType=exu.OutputVariableType.Position))


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `3SpringsDistance.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/3SpringsDistance.py>`_\  (Ex), \ `ALEANCFpipe.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ALEANCFpipe.py>`_\  (Ex), \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Ex), \ `ANCFcantileverTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcantileverTest.py>`_\  (Ex), \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Ex), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TM), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFcontactCircleTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactCircleTest.py>`_\  (TM)


----

.. _sec-solver-solvedynamic:

Function: SolveDynamic
^^^^^^^^^^^^^^^^^^^^^^
`SolveDynamic <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L214>`__\ (\ ``mbs``\ , \ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``solverType = exudyn.DynamicSolverType.GeneralizedAlpha``\ , \ ``updateInitialValues = False``\ , \ ``storeSolver = True``\ , \ ``showHints = False``\ , \ ``showCausingItems = True``\ )

- | \ *function description*\ :
  | solves the dynamic mbs problem using simulationSettings and solver type; check theDoc.pdf for MainSolverImplicitSecondOrder for further details of the dynamic solver; NOTE that this function is directly available from exudyn (using exudyn.SolveDynamic(...))
- | \ *input*\ :
  | \ ``mbs``\ : the MainSystem containing the assembled system; note that mbs may be changed upon several runs of this function
  | \ ``simulationSettings``\ : specific simulation settings out of exu.SimulationSettings(), as described in Section :ref:`sec-solutionsettings`\ ; use options for newton, discontinuous settings, etc., from timeIntegration; therein, implicit second order solvers use settings from generalizedAlpha and explict solvers from explicitIntegration; be careful with settings, as the influence accuracy (step size!), convergence and performance (see special Section :ref:`sec-overview-basics-speedup`\ )
  | \ ``solverType``\ : use exudyn.DynamicSolverType to set specific solver (default=generalized alpha)
  | \ ``updateInitialValues``\ : if True, the results are written to initial values, such at a consecutive simulation uses the results of this simulation as the initial values of the next simulation
  | \ ``storeSolver``\ : if True, the staticSolver object is stored in the mbs.sys dictionary as mbs.sys['staticSolver'], and simulationSettings are stored as mbs.sys['simulationSettings']
  | \ ``showHints``\ : show additional hints, if solver fails
  | \ ``showCausingItems``\ : if linear solver fails, this option helps to identify objects, etc. which are related to a singularity in the linearized system matrix
- | \ *output*\ :
  | returns True, if successful, False if fails; if storeSolver = True, mbs.sys contains staticSolver, which allows to investigate solver problems (check theDoc.pdf Section :ref:`sec-solversubstructures`\  and the items described in Section :ref:`sec-mainsolverstatic`\ )
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.itemInterface import *
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  #create simple system:
  ground = mbs.AddObject(ObjectGround())
  mbs.AddNode(NodePoint())
  body = mbs.AddObject(MassPoint(physicsMass=1, nodeNumber=0))
  m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=ground))
  m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=body))
  mbs.AddObject(CartesianSpringDamper(markerNumbers=[m0,m1], stiffness=[100,100,100]))
  mbs.AddLoad(LoadForceVector(markerNumber=m1, loadVector=[10,10,10]))
  mbs.Assemble()
  sims = exu.SimulationSettings()
  sims.timeIntegration.endTime = 10
  success = exu.SolveDynamic(mbs, sims, storeSolver = True)
  print("success =", success)
  print("iterations = ", mbs.sys['dynamicSolver'].it)
  print("pos=", mbs.GetObjectOutputBody(body,localPosition=[0,0,0],
        variableType=exu.OutputVariableType.Position))


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `3SpringsDistance.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/3SpringsDistance.py>`_\  (Ex), \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `ALEANCFpipe.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ALEANCFpipe.py>`_\  (Ex), \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Ex), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TM), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TM)


----

.. _sec-solver-computelinearizedsystem:

Function: ComputeLinearizedSystem
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeLinearizedSystem <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L304>`__\ (\ ``mbs``\ , \ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``useSparseSolver = False``\ )

- | \ *function description*\ :
  | compute linearized system of equations for ODE2 part of mbs, not considering the effects of algebraic constraints
- | \ *input*\ :
  | \ ``mbs``\ : the MainSystem containing the assembled system
  | \ ``simulationSettings``\ : specific simulation settings used for computation of jacobian (e.g., sparse mode in static solver enables sparse computation)
  | \ ``useSparseSolver``\ : if False (only for small systems), all eigenvalues are computed in dense mode (slow for large systems!); if True, only the numberOfEigenvalues are computed (numberOfEigenvalues must be set!); Currently, the matrices are exported only in DENSE MODE from mbs! NOTE that the sparsesolver accuracy is much less than the dense solver
- | \ *output*\ :
  | [M, K, D]; list containing numpy mass matrix M, stiffness matrix K and damping matrix D
- | \ *example*\ :

.. code-block:: python

  #take any example from the Examples or TestModels folder, e.g., 'cartesianSpringDamper.py' and run it
  #then execute the following commands in the console (or add it to the file):
  [M, K, D] = exu.ComputeLinearizedSystem(mbs)
  print("M=", M)
  print("K=", K)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TM), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TM), \ `geometricallyExactBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/geometricallyExactBeamTest.py>`_\  (TM)


----

.. _sec-solver-computeode2eigenvalues:

Function: ComputeODE2Eigenvalues
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeODE2Eigenvalues <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L352>`__\ (\ ``mbs``\ , \ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``useSparseSolver = False``\ , \ ``numberOfEigenvalues = 0``\ , \ ``constrainedCoordinates = []``\ , \ ``convert2Frequencies = False``\ , \ ``useAbsoluteValues = True``\ )

- | \ *function description*\ :
  | compute eigenvalues for unconstrained ODE2 part of mbs, not considering the effects of algebraic constraints; the computation is done for the initial values of the mbs, independently of previous computations. If you would like to use the current state for the eigenvalue computation, you need to copy the current state to the initial state (using GetSystemState,SetSystemState, see Section :ref:`sec-mbs-systemdata`\ ); note that mass and stiffness matrix are computed in dense mode so far, while eigenvalues are computed according to useSparseSolver.
- | \ *input*\ :
  | \ ``mbs``\ : the MainSystem containing the assembled system
  | \ ``simulationSettings``\ : specific simulation settings used for computation of jacobian (e.g., sparse mode in static solver enables sparse computation)
  | \ ``useSparseSolver``\ : if False (only for small systems), all eigenvalues are computed in dense mode (slow for large systems!); if True, only the numberOfEigenvalues are computed (numberOfEigenvalues must be set!); Currently, the matrices are exported only in DENSE MODE from mbs! NOTE that the sparsesolver accuracy is much less than the dense solver
  | \ ``numberOfEigenvalues``\ : number of eigenvalues and eivenvectors to be computed; if numberOfEigenvalues==0, all eigenvalues will be computed (may be impossible for larger problems!)
  | \ ``constrainedCoordinates``\ : if this list is non-empty, the integer indices represent constrained coordinates of the system, which are fixed during eigenvalue/vector computation; according rows/columns of mass and stiffness matrices are erased
  | \ ``convert2Frequencies``\ : if True, the eigen values are converted into frequencies (Hz) and the output is [eigenFrequencies, eigenVectors]
  | \ ``useAbsoluteValues``\ : if True, abs(eigenvalues) is used, which avoids problems for small (close to zero) eigen values; needed, when converting to frequencies
- | \ *output*\ :
  | [eigenValues, eigenVectors]; eigenValues being a numpy array of eigen values (\ :math:`\omega_i^2`\ , being the squared eigen frequencies in (\ :math:`\omega_i`\  in rad/s)!), eigenVectors a numpy array containing the eigenvectors in every column
- | \ *example*\ :

.. code-block:: python

  #take any example from the Examples or TestModels folder, e.g., 'cartesianSpringDamper.py' and run it
  #then execute the following commands in the console (or add it to the file):
  [values, vectors] = exu.ComputeODE2Eigenvalues(mbs)
  print("eigenvalues=", values)
  #==>values contains the eigenvalues of the ODE2 part of the system in the current configuration


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `nMassOscillatorInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorInteractive.py>`_\  (Ex), \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TM), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TM), \ `computeODE2EigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2EigenvaluesTest.py>`_\  (TM), \ `geometricallyExactBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/geometricallyExactBeamTest.py>`_\  (TM)


----

.. _sec-solver-checksolverinfostatistics:

Function: CheckSolverInfoStatistics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CheckSolverInfoStatistics <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L435>`__\ (\ ``solverName``\ , \ ``infoStat``\ , \ ``numberOfEvaluations``\ )

- | \ *function description*\ :
  | helper function for solvers to check e.g. if high number of memory allocations happened during simulation
  | This can happen, if large amount of sensors are attached and output is written in every time step
- | \ *input*\ :
  | stat=exudyn.InfoStat() from previous step, numberOfEvaluations is a counter which is proportional to number of RHS evaluations in method

