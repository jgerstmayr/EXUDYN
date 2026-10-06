(sec-performance-errors)=
# Performance, errors and solver failures

What Exudyn says when something goes wrong, what to do about it, and how to make a
model run faster. None of this is needed to write a first model, and all of it is
needed sooner or later.

(sec-overview-basics-errors)=
## Errors: what Exudyn raises, and what to do about it

Every error that Exudyn reports from its C++ core arrives in Python as an exception with a
**type**. The type answers the first question a user has -- **whose mistake was it** --
before the message is even read.

All of them derive from `exudyn.ExudynError`, and each of them **also** derives from
the built-in exception that fits, so an `except ValueError` written before Exudyn 1.12
keeps working:

- `exudyn.ExudynTypeError` (also a `TypeError`): the object cannot be that parameter at all -- a list where a number belongs, a string where a function belongs.
- `exudyn.ExudynValueError` (also a `ValueError`): the kind of value is right and the value is not -- a vector of the wrong length, an unknown parameter name, `numberOfSteps=-1`, an output variable this item does not have.
- `exudyn.ExudynIndexError` (also an `IndexError`): an index outside its range -- `mbs.GetObject(99)` in a system with three objects.
- `exudyn.ModelError` (also a `ValueError`): the model **as built** does not hold together -- a load on a marker that does not exist, a node used by an object but never added, a function called before `mbs.Assemble()`. Most of these are raised by `Assemble()`, which checks the whole system.
- `exudyn.SolverError` (also a `RuntimeError`): the solver cannot continue -- a singular system matrix, no convergence, divergence. See {ref}`sec-overview-basics-convergenceproblems` for what to do.
- `exudyn.NotImplementedFeatureError` (also a `NotImplementedError`): the feature or the combination does not exist -- a jacobian that is not implemented for this element, a solver that cannot handle this constraint. **Neither your mistake nor a bug**; the message usually names a way around it.
- `exudyn.ExudynArithmeticError` (also an `ArithmeticError`): division by zero, root of a negative number.
- `exudyn.InternalError` (also a `RuntimeError`): an Exudyn invariant broke. **This one is a bug in Exudyn, not in your model** -- please report it with the message and, if possible, a model that shows it.

 So `except exudyn.ExudynError` catches everything Exudyn raises, while
`except IndexError` or `except ValueError` still do what they always did.

(sec-overview-basics-errors-solver)=
### Catching a solver failure

The case with a concrete action behind it: a solve that fails is a normal event in a parameter
study, and it should not end the study.

```{include} /docs/generated/notebooks/snippets/solving-solverError.md
```

 In a parameter variation, score the failed run instead of letting it stop the sweep:

```{include} /docs/generated/notebooks/snippets/solving-parameterFunction.md
```

 Note that `except exudyn.ExudynError` does **not** catch
`KeyboardInterrupt`: a long sweep can still be stopped with Ctrl-C.

### An error inside your own user function

If a Python user function -- `springForceUserFunction` and its kin -- raises, Exudyn
reports it as a `ModelError`, because a user function is part of the model. The original
exception is **not** lost: it is attached as `__cause__`, with its own traceback,
so Spyder and VS Code show the chain and jump to the line inside your function.

```{include} /docs/generated/notebooks/snippets/solving-userFunctionError.md
```

(sec-overview-basics-errors-where)=
### Where the message is written

The exception carries the message and the location, so the **console shows it once** -- as
the Python traceback, and not a second time as a printed block. A caught exception therefore
prints nothing at all, which is what makes a parameter variation readable.

 The log files are the other way round: an error is written to **every open log
file**, whatever raised it.

- the `exudyn` output file, if one was opened with `exu.SetWriteToFile(...)`;
- the solver information file, if `solution.solverInformationFileName` is set.

 On a long unattended run those files are the only record that anything happened.

(sec-overview-basics-errors-deprecation)=
### Deprecation warnings

A name that is on its way out raises a real Python `DeprecationWarning` -- once per place
in your code, not once per call. To find every use before a release removes the old name, run
your script with

```
  python -W error::DeprecationWarning yourModel.py
```

 which turns each of them into an error at the line that caused it.

### Behaviour that is not in your script

A script that behaves differently than it reads — a different output directory, a renderer that
looks unfamiliar, a dialog in a place you did not put it — may be reading something that was stored
on this machine. Exudyn keeps such settings in one folder, `~/.exudyn`, and
**deleting that folder returns everything to the defaults**; nothing in it is needed to run a model.
To find out whether it is the cause without deleting anything, run the script with
`EXUDYN_NO_USER_SETTINGS=1`: the difference is either gone (a stored setting caused it) or still
there (it did not). What was read is listed by `exudyn.misc.overrideSettings.Print()`, and the whole
mechanism is [](#sec-usersettings).

(sec-overview-basics-errors-switches)=
### Switches

- `exudyn.config.suppressWarnings = True`: no warnings, on the console or in a file.
- `exudyn.special.exceptions.parameterRangeChecks = False`: do not check parameter ranges; faster, and a wrong value then reaches the computation instead of being reported.
- `exudyn.special.exceptions.dictionaryVersionMismatch`, `.dictionaryNonCopyable`: turn those two specific errors off.

(sec-overview-basics-convergenceproblems)=
## Removing convergence problems and solver failures

Nonlinear formulations (such as most multibody systems, especially nonlinear finite elements) cause problems and there is no general nonlinear solver which may reliably and accurately solve such problems.
Tuning solver parameters is at hand of the user.
In general, the Newton solver tries to reduce the error by the factor given in

- `simulationSettings.staticSolver.newton.relativeTolerance` (for static solver),

which is not possible for very small (or zero) initial residuals. The absolute tolerance is helping out as a lower bound for the error, given in

- `simulationSettings.staticSolver.newton.absoluteTolerance` (for static solver),

which is by default rather low (1e-10) -- in order to achieve accurate results for small systems or small motion (in mm or $\mu$m regime). Increasing this value helps to solve such problems. Nevertheless, you should usually set tolerances as low as possible because otherwise, your solution may become inaccurate.

 The following hints / rules for described problems shall be followed.

- **static solver**: **load steps get very small** even if the solution seems to be smooth (or linear) and less steps are expected:
  - this may happen for **system without loads**; larger number of steps may happen for finer discretization;
  - you may adjust (increase) `.newton.relativeTolerance` / `.newton.absoluteTolerance` in static solver or in time integration to resolve such problems, but check if solution achieves according accuracy

- **static solver**:  load steps are reduced significantly for **highly nonlinear problems**:
  - solver repeatedly writes that steps are reduced $\ra$ try to use `loadStepGeometric` and use a large `loadStepGeometricRange`: this allows to start with very small loads in which the system is nearly linear (e.g. for thin strings or belts under gravity).

- **static solver**: system is (nearly) **kinematic**:
  - a static solution can be achieved using `stabilizerODE2term`, which adds mass-proportional stiffness terms during load steps $< 1$; see also hints for singular Jacobians below

- very small loads or even **zero loads** do not converge: `SolveDynamic` or `SolveStatic` **terminated due to errors**
  - the reason is the nonlinearity of formulations (nonlinear kinematics, nonlinear beam, etc.) and round off errors, which restrict Newton to achieve desired tolerances
  - adjust (increase) `.newton.relativeTolerance` / `.newton.absoluteTolerance` in static solver or in time integration
  - in many cases, especially for static problems, the `.newton.residualMode = 1` evaluates the increments; the nonlinear problems is assumed to be converged, if increments are within given absolute/relative tolerances; this also works usually better for kinematic solutions

- for **discontinuous problems**:
  - try to adjust solver parameters; especially the `discontinuous.iterationTolerance` and `discontinuous.maxIterations`; try to make smaller load or time steps in order to resolve switching points of contact or friction; generalized alpha solvers may cause troubles when reducing step sizes $\ra$ use TrapezoidalIndex2 solver
  - in case of **user functions**, make sure that there is no switching inside the user function (if or `sign` function); switching must be done in the PostNewtonStep, otherwise convergence severely suffers

- **singular Jacobians** or **redundant constraints**:
  - in case of systems that lead to a singular Jacobian due to redundant constraints or kinematic DOF in static solutions, you may switch to Eigen's FullPivotLU solver using:
  - `simulationSettings.linearSolver.solverType = exu.LinearSolverType.EigenDense` and
  - `simulationSettings.linearSolver.ignoreSingularJacobian=True` ;
  - however, check your results as they may be erroneous, because the solver tries to find an optimal solution / compromise which may not be what you intend to get!

- if you see further problems, please post them (including relevant example) at the Exudyn github page!

(sec-overview-basics-speedup)=
## Performance and ways to speed up computations

Multibody dynamics simulation should be accurate and reliable on the one hand side. Most solver settings are such that they lead to comparatively reliable results.
However, in some cases there is a significant possibility for speeding up computations, which are described in the following list. Not all recommendations may apply to your models.

The following examples refer to `simulationSettings = exu.SimulationSettings()`.
In general, to see where CPU time is lost, use the option turn on `simulationSettings.show.computationTime = True` to see which parts of the solver need most of the time (deactivated in exudynFast versions!).
In addition to Exudyn's internal time measurements, in Spyder (or IPython) you can use magic commands such as `%timeit -n10 mbs.SolveDynamic()` to evaluate the time spent for a specific command with number of repetitions given after `-n`. This may be particularly interesting in Python user functions to see where time is lost.

To activate the Exudyn C++ versions without range checks, which may be approx. 30 percent faster in some situations, use the following code snippet before first import of `exudyn`:

```python
  import sys
  sys.exudynFast = True #this variable is used to signal to load the fast exudyn module
  import exudyn as exu
```

The faster versions are available for all release versions, but only for some `.dev1` development versions (Python 3.10), which can be determined by trying `import exudyn.exudynCPPfast`.

 However, there are many **ways to speed up Exudyn in general**:

- for models with more than 50 coordinates, switching to sparse solvers might greatly improve speed: `simulationSettings.linearSolver.solverType = exu.LinearSolverType.EigenSparse`
- when preferring dense direct solvers, switching to Eigen's PartialPivLU solver might greatly improve speed: `simulationSettings.linearSolver.solverType = exu.LinearSolverType.EigenDense`; however, the flag `simulationSettings.linearSolver.ignoreSingularJacobian=True` will switch to the much slower (but more robust) Eigen's FullPivLU
- try to avoid Python functions or try to speed up Python functions; if this is not possible, see solutions below
- instead of user functions in objects or loads (computed in every iteration), some problems would also work if these parameters are only updated in `mbs.SetPreStepUserFunction(...)`
- Python user functions can be speed up (since Exudyn V1.7.40) by converting conventional Python functions into Exudyn (internal) symbolic user functions, which have similar performance as C++ functions with the ability to parallelize; see {ref}`sec-cinterface-symbolic`
- Alternatively, Python user functions can be speed up using the Python numba package, using `@jit` in front of functions (for more options, see [https://numba.pydata.org/numba-doc/dev/user/index.html](https://numba.pydata.org/numba-doc/dev/user/index.html)); Example given in `Examples/springDamperUserFunctionNumbaJIT.py` showing speedups of factor 4; more complicated Python functions may see speedups of 10 - 50
- for **discontinuous problems**, try to adjust solver parameters; especially the discontinuous.iterationTolerance which may be too tight and cause many iterations; iterations may be limited by discontinuous.maxIterations, which at larger values solely multiplies the computation time with a factor if all iterations are performed
- For multiple computations / multiple runs of Exudyn (parameter variation, optimization, compute sensitivities), you can use the processing sub module of Exudyn to parallelize computations and achieve speedups proporional to the number of cores/threads of your computer; specifically using the `multiThreading` option or even using a cluster (using `dispy`, see `ParameterVariation(...)` function)
- In case of multiprocessing and cluster computing, you may see a very high CPU usage of "Antimalware Service Executable", which is the Microsoft Defender Antivirus; you can turn off such problems by excluding `python.exe` from the defender (on your own risk!) in your settings: Settings $\ra$ Update & Security $\ra$ Windows Security $\ra$ Virus & threat protection settings $\ra$ Manage settings $\ra$ Exclusions $\ra$ Add or remove exclusions

**Possible speed ups for dynamic simulations**:

- for implicit integration, turn on **modified Newton**, which updates jacobians only if needed: `simulationSettings.timeIntegration.newton.useModifiedNewton = True`
- use **multi-threading**: `simulationSettings.parallel.numberOfThreads = ...`, depending on the number of cores (larger values usually do not help); improves greatly for contact problems, but also for some objects computed in parallel; will improve significantly in future
- decrease number of steps (`simulationSettings.timeIntegration.numberOfSteps = int(tEnd/h)`) by increasing the step size $h$ if not needed for accuracy reasons; not that in general, the solver will reduce steps in case of divergence, but not for accuracy reasons, which may still lead to divergence if step sizes are too large
- switch off measuring computation time, if not needed: `simulationSettings.show.computationTime = False`
- try to switch to **explicit solvers**, if problem has no constraints and if problem is not stiff
- try to have **constant mass matrices** (see according objects, which have constant mass matrices; e.g. rigid bodies using RotationVector Lie group node have constant mass matrix)
- for explicit integration, set `computeEndOfStepAccelerations = False`, if you do not need accurate evaluation of accelerations at end of time step (will then be taken from beginning)
- for explicit integration of large systems, use a **sparse solver** (`simulationSettings.linearSolver.solverType = exu.LinearSolverType.EigenSparse`): with the dense default every step multiplies with a dense mass matrix and costs $O(n^2)$ - 400 times slower than sparse for 2000 point masses; then set `timeIntegration.explicit.computeMassMatrixInversePerBody=True`, which avoids factorization and back substitution, which may speed up computations with many bodies / particles further (it has no effect with the dense solver)
- if you are sure that your mass matrix is constant, set:
- `simulationSettings.timeIntegration.reuseConstantMassMatrix = True`; check results!
- check that `simulationSettings.timeIntegration.realtime.active = False`; if set True, it breaks down simulation to real time
- do not record images, if not needed: `simulationSettings.solution.recordImagesInterval = -1`
- in case of bad convergence, decreasing the step size might also help; check also other flags for adaptive step size and for Newton
- use `simulationSettings.timeIntegration.verboseMode = 1`; larger values create lots of output which drastically slows down
- use `simulationSettings.timeIntegration.verboseModeFile = 0`, otherwise output written to file
- adjust `simulationSettings.solution.sensors.writePeriod` to avoid time spent on writing sensor files
- use `simulationSettings.solution.file.write = False`, otherwise much output may be written to file;
- if solution file is needed, adjust `simulationSettings.solution.file.writePeriod` to larger values and also adjust `simulationSettings.solution.precision`, e.g., to 6, in order to avoid larger files; also adjust `simulationSettings.solution.file.export.velocities = False` and `simulationSettings.solution.file.export.accelerations = False` to avoid large output files
