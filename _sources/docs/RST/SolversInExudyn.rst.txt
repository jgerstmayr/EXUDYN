Solvers in Exudyn 
==================

The user has a couple of basic solvers available in Exudyn , see \ :numref:`fig-available-solvers`\ :

+  \ ``exudyn.SolveStatic(...)``\ : compute static solution for given problem (may also be used to compute kinematic behavior by prescribing joint motion)
+  \ ``exudyn.SolveDynamic(...)``\ : time integration of equations of motion
+  \ ``exudyn.ComputeLinearizedSystem(...)``\ : computes the linearized system of equations and returns mass, stiffness, damping matrices
+  \ ``exudyn.ComputeODE2Eigenvalues(...)``\ : computes the eigenvalues of the linearized system of equations; only possible if no algebraic constraints in system; uses scipy to compute eigenvalues



.. _fig-available-solvers:
.. figure:: ../theDoc/figures/solversAvailableSolvers.png
   :width: 500

   Basic and advanced solvers in Exudyn ; advanced solvers build upon any basic solver to perform more sophisticated operations



There are advanced solvers, like in \ ``exudyn.processing``\ :

+  \ **Optimization**\ :
  
+  \ ``GeneticOptimization(...)``\ : find optimum for given set of parameter ranges using genetic optimization; works in parallel
+  \ ``Minimize(...)``\ : find optimum with \ ``scipy.optimize.minimize(...)``\ 
  


+  \ ``ParameterVariation(...)``\ : compute a series of simulations for given set(s) of parameters; works in parallel
+  \ ``ComputeSensitivities(...)``\ : compute sensitivities for certain parameters; works in parallel

The advanced methods are build upon the basic solvers and essentially run single simulations in the background, see the according examples.

The basic solvers need a \ ``MainSystem``\ , usually denoted as \ ``mbs``\ , to be solved. Furthermore, a couple of options are usually to be given, which are explained shortly:

+  \ ``simulationSettings``\ : This is a big structure, containing all solver options; note that only the according options for \ ``staticSolver``\  or \ ``timeIntegration``\  are used. Look at the detailed description of these options in Section :ref:`sec-simulationsettingsmain`\ . These settings influence the output rate and output quantity of the solution, solver reporting, accuracy, solver type, etc. Specifically, the \ ``verboseMode``\  may be increased (2-4) to see the behavior of the solver and intermediate quantities.
+  \ ``solverType``\ : Only for \ ``exudyn.SolveDynamic(...)``\ : This is a simpler access to the solverType given in the internal structure of 
  
   \ ``timeIntegration.generalizedAlpha``\  and 
   \ ``simulationSettings.timeIntegration.explicitIntegration.dynamicSolverType``\ .
  

The function \ ``exudyn.SolveDynamic(...)``\  sets the according variables internally. For available solver types, see the description of \ ``exudyn.DynamicSolverType``\  in Section :ref:`sec-dynamicsolvertype`\ .

+  \ ``storeSolver``\ : if \ ``True``\ , the solver is stored in \ ``mbs.sys['staticSolver']``\  or \ ``mbs.sys['dynamicSolver']``\  and also solver settings are stored in \ ``mbs.sys['simulationSettings']``\ . After the solver has finished, \ ``mbs.sys['staticSolver']``\  can be used to retrieve additional information on convergence, system matrices, etc. (see the solver structure).
+  \ ``showHints``\ : This shows a lot of possible solutions in case of no convergence
+  \ ``showCausingItems``\ : This shows a potential causing item if the linear solver failed; the item number is computed from the coordinate number that caused problems (e.g., a row that became zero during factorization); note that this item may not be the real cause in your problem



System equations of motion
--------------------------

The system equations of motion in Exudyn follow the notations of Section :ref:`sec-nomenclatureeom`\  and are represented as 

.. math::
   :label: eq-system-eom

   {\mathbf{M}} \ddot {\mathbf{q}} + \frac{\partial {\mathbf{g}}}{\partial {\mathbf{q}}^\mathrm{T}} \tlambda_q + \frac{\partial {\mathbf{g}}}{\partial \dot {\mathbf{q}}^\mathrm{T}} \tlambda_{\dot q} & = &{\mathbf{f}}_\SO({\mathbf{q}}, \dot {\mathbf{q}}, t) \\
   \dot {\mathbf{y}} + \frac{\partial {\mathbf{g}}}{\partial {\mathbf{y}}^\mathrm{T}} \tlambda & = &{\mathbf{f}}_\FO({\mathbf{y}}, t) \\
   {\mathbf{g}}({\mathbf{q}}, \dot {\mathbf{q}}, {\mathbf{y}}, \tlambda, t) &=& 0 .


Here, we introduce different Lagrange multipliers \ :math:`\tlambda_q`\  and \ :math:`\tlambda_{\dot q}`\  which have equal sizes as \ :math:`\tlambda`\ , while those \ :math:`\lambda_i`\  which belong to holonomic constraints, are included in \ :math:`\tlambda_q`\  and \ :math:`\tlambda_i`\  belonging to non-holonomic constraints, are included in \ :math:`\tlambda_{\dot q}`\ , whereas other components in \ :math:`\tlambda_q`\  or \ :math:`\tlambda_{\dot q}`\  are zero.

It may help to know that for linear mechanical the term \ :math:`{\mathbf{f}}_\SO`\  becomes

.. math::

   {\mathbf{f}}^{lin}_\SO = {\mathbf{f}}_a - {\mathbf{K}} {\mathbf{q}} - {\mathbf{D}} \dot {\mathbf{q}}


in which \ :math:`{\mathbf{f}}^a`\  represents applied forces and stiffness matrix \ :math:`{\mathbf{K}}`\  and damping matrix \ :math:`{\mathbf{D}}`\  become part of the system Jacobian for time integration.

