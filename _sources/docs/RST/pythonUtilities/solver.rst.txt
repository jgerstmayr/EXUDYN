
.. _sec-module-solver:

Module: solver
==============

The solver module provides interfaces to static, dynamic and eigenvalue solvers.
Most of the solvers are implemented inside the C++ core.

- Author:    Johannes Gerstmayr  
- Date:      2020-12-02 
- Notes:     Solver functions are included directly in exudyn and can be used with mbs.SolveStatic(...) 


.. _sec-solver-solvererrormessage:

Function: SolverErrorMessage
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SolverErrorMessage <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L23>`__\ (\ ``solver``\ , \ ``mbs``\ , \ ``isStatic = False``\ , \ ``showCausingObjects = True``\ , \ ``showCausingNodes = True``\ , \ ``showHints = True``\ )

- | \ *function description*\ :
  | (internal) helper function for unique error and helper messages



----

Function: SolveStatic
^^^^^^^^^^^^^^^^^^^^^
`SolveStatic <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L157>`__\ (\ ``mbs``\ , \ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``updateInitialValues = False``\ , \ ``storeSolver = True``\ , \ ``showHints = False``\ , \ ``showCausingItems = True``\ , \ ``autoAssemble = True``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.SolveStatic(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-solvestatic`\ 



----

Function: SolveDynamic
^^^^^^^^^^^^^^^^^^^^^^
`SolveDynamic <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L224>`__\ (\ ``mbs``\ , \ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``solverType = exudyn.DynamicSolverType.GeneralizedAlpha``\ , \ ``updateInitialValues = False``\ , \ ``storeSolver = True``\ , \ ``showHints = False``\ , \ ``showCausingItems = True``\ , \ ``autoAssemble = True``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.SolveDynamic(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-solvedynamic`\ 



----


.. _sec-solver-solversuccess:

Function: SolverSuccess
^^^^^^^^^^^^^^^^^^^^^^^
`SolverSuccess <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L321>`__\ (\ ``solverStructure``\ )

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
      exu.Print('success=',success)
      exu.Print('error message=',msg)
  #alternative:
  solver=exu.MainSolverImplicitSecondOrder()
  ...
  [success, msg] = exu.SolverSuccess(solver)




----

Function: ComputeLinearizedSystem
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeLinearizedSystem <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L377>`__\ (\ ``mbs``\ , \ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``projectIntoConstraintNullspace = False``\ , \ ``singularValuesTolerance = 1e-12``\ , \ ``returnConstraintJacobian = False``\ , \ ``returnConstraintNullspace = False``\ , \ ``autoAssemble = True``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.ComputeLinearizedSystem(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-computelinearizedsystem`\ 



----

Function: ComputeODE2Eigenvalues
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeODE2Eigenvalues <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L515>`__\ (\ ``mbs``\ , \ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``useSparseSolver = False``\ , \ ``numberOfEigenvalues = 0``\ , \ ``constrainedCoordinates = []``\ , \ ``convert2Frequencies = False``\ , \ ``useAbsoluteValues = True``\ , \ ``computeComplexEigenvalues = False``\ , \ ``ignoreAlgebraicEquations = False``\ , \ ``singularValuesTolerance = 1e-12``\ , \ ``autoAssemble = True``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.ComputeODE2Eigenvalues(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-computeode2eigenvalues`\ 



----

Function: ComputeSystemDegreeOfFreedom
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeSystemDegreeOfFreedom <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L741>`__\ (\ ``mbs``\ , \ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``threshold = 1e-12``\ , \ ``verbose = False``\ , \ ``useSVD = False``\ , \ ``autoAssemble = True``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.ComputeSystemDegreeOfFreedom(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-computesystemdegreeoffreedom`\ 



----


.. _sec-solver-checksolverinfostatistics:

Function: CheckSolverInfoStatistics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CheckSolverInfoStatistics <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L833>`__\ (\ ``solverName``\ , \ ``infoStat``\ , \ ``numberOfEvaluations``\ )

- | \ *function description*\ :
  | helper function for solvers to check e.g. if high number of memory allocations happened during simulation
  | This can happen, if large amount of sensors are attached and output is written in every time step
- | \ *input*\ :
  | stat=exudyn.special.InfoStat() from previous step, numberOfEvaluations is a counter which is proportional to number of RHS evaluations in method

