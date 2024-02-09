
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
`SolverErrorMessage <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L23>`__\ (\ ``solver``\ , \ ``mbs``\ , \ ``isStatic = False``\ , \ ``showCausingObjects = True``\ , \ ``showCausingNodes = True``\ , \ ``showHints = True``\ )

- | \ *function description*\ :
  | (internal) helper function for unique error and helper messages



----

Function: SolveStatic
^^^^^^^^^^^^^^^^^^^^^
`SolveStatic <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L154>`__\ (\ ``mbs``\ , \ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``updateInitialValues = False``\ , \ ``storeSolver = True``\ , \ ``showHints = False``\ , \ ``showCausingItems = True``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.SolveStatic(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-solvestatic`\ 



----

Function: SolveDynamic
^^^^^^^^^^^^^^^^^^^^^^
`SolveDynamic <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L219>`__\ (\ ``mbs``\ , \ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``solverType = exudyn.DynamicSolverType.GeneralizedAlpha``\ , \ ``updateInitialValues = False``\ , \ ``storeSolver = True``\ , \ ``showHints = False``\ , \ ``showCausingItems = True``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.SolveDynamic(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-solvedynamic`\ 



----


.. _sec-solver-solversuccess:

Function: SolverSuccess
^^^^^^^^^^^^^^^^^^^^^^^
`SolverSuccess <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L314>`__\ (\ ``solverStructure``\ )

- | \ *function description*\ :
  | return success (True/False) and error message of solver after SolveSteps(...), SolveSystem(...), SolveDynamic(...) or SolveStatic(...) have been called. May also be set if other higher level functions called e.g. SolveSystem(...)
- | \ *input*\ :
  | solverStructure: solver structure, as stored in mbs.sys or as created e.g. by exudyn.MainSolverExplicit()
- | \ *output*\ :
  | [success, errorString], returns success=True or False and in case of no success, information is provided in errorString
- | \ *example*\ :

.. code-block:: python

  #assume MainSystem mbs, exu library and simulationSettings:
  try:
      mbs.SolveDynamic(simulationSettings)
  except:
      [success, msg] = exu.SolverSuccess(mbs.sys['dynamicSolver'])
      print('success=',success)
      print('error message=',msg)
  #alternative:
  solver=exu.MainSolverImplicitSecondOrder()
  ...
  [success, msg] = exu.SolverSuccess(solver)




----

Function: ComputeLinearizedSystem
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeLinearizedSystem <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L366>`__\ (\ ``mbs``\ , \ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``useSparseSolver = False``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.ComputeLinearizedSystem(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-computelinearizedsystem`\ 



----

Function: ComputeODE2Eigenvalues
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeODE2Eigenvalues <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L442>`__\ (\ ``mbs``\ , \ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``useSparseSolver = False``\ , \ ``numberOfEigenvalues = 0``\ , \ ``constrainedCoordinates = []``\ , \ ``convert2Frequencies = False``\ , \ ``useAbsoluteValues = True``\ , \ ``ignoreAlgebraicEquations = False``\ , \ ``singularValuesTolerance = 1e-12``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.ComputeODE2Eigenvalues(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-computeode2eigenvalues`\ 



----

Function: ComputeSystemDegreeOfFreedom
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeSystemDegreeOfFreedom <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L621>`__\ (\ ``mbs``\ , \ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``threshold = 1e-12``\ , \ ``verbose = False``\ , \ ``useSVD = False``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.ComputeSystemDegreeOfFreedom(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-computesystemdegreeoffreedom`\ 



----


.. _sec-solver-checksolverinfostatistics:

Function: CheckSolverInfoStatistics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CheckSolverInfoStatistics <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L708>`__\ (\ ``solverName``\ , \ ``infoStat``\ , \ ``numberOfEvaluations``\ )

- | \ *function description*\ :
  | helper function for solvers to check e.g. if high number of memory allocations happened during simulation
  | This can happen, if large amount of sensors are attached and output is written in every time step
- | \ *input*\ :
  | stat=exudyn.InfoStat() from previous step, numberOfEvaluations is a counter which is proportional to number of RHS evaluations in method

