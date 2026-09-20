(sec-installation-gettingstarted)=
# Installation and Getting Started

The overview of what Exudyn is and what it can do is the page before this one, `README.rst` -
the landing page of the repository and of the PyPI package, which is part of this documentation
rather than a second copy of it (revision2026 step R7.1.5).


 Exudyn is hosted on [GitHub](https://github.com) [EXUDYNgit]:

- [https://github.com/jgerstmayr/EXUDYN](https://github.com/jgerstmayr/EXUDYN)

 Online documentation is available at:

- [https://jgerstmayr.github.io/EXUDYN](https://jgerstmayr.github.io/EXUDYN)
- [https://exudyn.readthedocs.io](https://exudyn.readthedocs.io)

For any comments, requests, issues, bug reports, send an email to:

- email: `reply.exudyn@gmail.com`

Thanks for your contribution!

## Getting started

This section will show:

1. What is Exudyn ?
2. Who is developing Exudyn ?
3. How to install Exudyn
4. How to use Exudyn in Python
5. Goals of Exudyn
6. Run a simple example in Python
7. FAQ -- Frequently asked questions

### What is Exudyn ?

Exudyn -- (fl**EX**ible m**U**ltibody **DYN**amics  -- **EX**tend yo**U**r **DYN**amics) \
 Exudyn is a C++ based Python library for efficient simulation of flexible multibody dynamics systems.
It is the follow up code of the previously developed multibody code HOTINT, which Johannes Gerstmayr started during his PhD-thesis.
It seemed that the previous code HOTINT reached limits of further (efficient) development and it seemed impossible to continue from this code as it was outdated regarding programming techniques and the numerical formulation at the time Exudyn was started.

Exudyn is designed to easily set up complex multibody models, consisting of rigid and flexible bodies with joints, loads and other components. It shall enable automatized model setup and parameter variations, which are often necessary for system design but also for analysis of technical problems. The broad usability of Python allows to couple a multibody simulation with environments such as optimization, statistics, data analysis, machine learning and others.

The multibody formulation is mainly based on redundant coordinates. This means that computational objects (rigid bodies, flexible bodies, ...) are added as independent bodies to the system. Hereafter, connectors (e.g., springs or constraints) are used to interconnect the bodies. The connectors are using Markers on the bodies as interfaces, in order to transfer forces and displacements.
For details on the interaction of nodes, objects, markers and loads see {ref}`sec-overview-items`. For a non-redundant formulation, see `ObjectKinematicTree` -- this allows to create tree-structures with minimal coordinates in Exudyn.

There are several journal papers of the developers which were using Exudyn (list is incomplete -- see google scholar):

- J. Gerstmayr. Exudyn -- a C++-based Python package for flexible multibody systems. Multibody System Dynamics (2023). \url{https://doi.org/10.1007/s11044-023-09937-1}
- J. Gerstmayr, P. Manzl, M. Pieber. Multibody Models Generated from Natural Language, Multibody System Dynamics (2024). \url{https://doi.org/10.1007/s11044-023-09962-0}
- P. Manzl, O. Rogov, J. Gerstmayr, A. Mikkola, G. Orzechowski. Reliability Evaluation of Reinforcement Learning Methods for Mechanical Systems with Increasing Complexity.  Preprint, Research Square, 2023.  \url{https://doi.org/10.21203/rs.3.rs-3066420/v1}
- S. Holzinger, M. Arnold, J. Gerstmayr. Evaluation and Implementation of Lie Group Integration Methods for Rigid Multibody Systems. Preprint, Research Square, 2023.  \url{https://doi.org/10.21203/rs.3.rs-2715112/v1}
- M. Sereinig, P. Manzl, and J. Gerstmayr. Task Dependent Comfort Zone, a Base Placement Strategy for Autonomous Mobile Manipulators using Manipulability Measures, Robotics and Autonomous Systems, submitted. [Sereinig2023comfortZone]
- R. Neurauter, J. Gerstmayr. A novel motion reconstruction method for inertial sensors with constraints, Multibody System Dynamics, 2022. [NeurauterGerstmayr2023]
- M. Pieber, K. Ntarladima, R. Winkler, J. Gerstmayr. A Hybrid ALE Formulation for the Investigation of the Stability of Pipes Conveying Fluid and Axially Moving Beams, ASME Journal of Computational and Nonlinear Dynamics, 2022.
- S. Holzinger, M. Schieferle, C. Gutmann, M. Hofer, J. Gerstmayr. Modeling and Parameter Identification for a Flexible Rotor with Impacts. Journal of Computational and Nonlinear Dynamics, 2022.
- S. Holzinger, J. Gerstmayr. Time integration of rigid bodies modelled with three rotation parameters, Multibody System Dynamics, Vol. 53(5), 2021.
- A. Zw{\"o}lfer, J. Gerstmayr. The nodal-based floating frame of reference formulation with modal reduction. Acta Mechanica, Vol. 232, pp.  835--851 (2021).
- A. Zw{\"o}lfer, J. Gerstmayr. A concise nodal-based derivation of the floating frame of reference formulation for displacement-based solid finite elements, Journal of Multibody System Dynamics, Vol. 49(3), pp. 291 -- 313, 2020.
- S. Holzinger, J. Sch{\"o}berl, J. Gerstmayr. The equations of motion for a rigid body using non-redundant unified local velocity coordinates. Multibody System Dynamics, Vol. 48, pp. 283 -- 309, 2020.

### Developers of Exudyn and thanks

Exudyn is developed at the University of Innsbruck.
In general, most of the Exudyn is written by Johannes Gerstmayr, implementing ideas that followed from the project HOTINT [GerstmayrEtAl2013], but also creating many new concepts and approaches. 15 years of development of HOTINT led to a lot of lessons learned and after 20 years, a code must be re-designed.

Some important tests for the coupling between C++ and Python have been written by Stefan Holzinger. Stefan also helped to set up the previous upload to GitLab and to test parallelization features.
For the interoperability between C++ and Python, we extensively use **Pybind11**[pybind11], originally written by Jakob Wenzel, see `https://github.com/pybind/pybind11`. Without Pybind11 we couldn't have made this project -- Thanks a lot!

Important discussions with researchers from the community were important for the design and development of Exudyn , where we like to mention Joachim Sch{\"o}berl from TU-Vienna who boosted the design of the code with great concepts.

The cooperation and funding within the EU H2020-MSCA-ITN project 'Joint Training on Numerical Modelling of Highly Flexible Structures for Industrial Applications' contributes to the development of the code.

The following people have contributed to Python and C++ library implementations, testing, examples or theory:

- Joachim Sch{\"o}berl, TU Vienna (Providing specialized NGsolve [Schoeberl1997; NGsolve2014; NGsolve2022] core library with `taskmanager` for **multi-threaded parallelization**, which is now replaced by a simplified internal version but closely following the original implementation; NGsolve mesh and FE-matrices import; highly efficient eigenvector computations)
- Stefan Holzinger, University of Innsbruck (Lie group module and solvers in Python, Lie group node; helped with Lie group solvers, geometrically exact beam; testing)
- Peter Manzl, University of Innsbruck (ConvexRoll Python and C++ implementation; revised artificialIntelligence, ParameterVariation, robotics and MPI parallelization; providing many figures for theDoc; pip install on linux, wsl with graphics; several other fixes; examples)
- Andreas Zw{\"o}lfer, Technical University Munich (theory and examples for FFRF, CMS formulation and ANCF 2D cable prototypes in MATLAB)
- Michael Pieber, University of Innsbruck (helped in several Python libraries; ComputeODE2Eigenvalues with constraints, FEM and CMS testing; Abaqus import and test files; ANCFCable2D+ALE theory improvements and equations check; examples); Exudyn graphical user interface
- Martin Sereinig, University of Innsbruck (special robotics functionality, mobile robots, manipulability measures, robot models)
- Sebastian Weyrer, University of Innsbruck (ObjectContactSphereSphere; FEM RigidBodyInertia; some fixes; examples)
- Grzegorz Orzechowski, Lappeenranta University of Technology (coupling with openAI gym and running machine learning algorithms)
- Zhaowei Zhang, Chinese Academy of Sciences (ANCFThinPlate; found several issues and bugs)
- Aaron Bacher, University of Innsbruck (helped to integrated OpenVR, connection with Franka Emika Panda)
- Martin Arnold, Martin-Luther-University of Halle-Wittenberg (support for explicit and implicit Lie group solvers, especially to theory / jacobians and automatic step size)
- Konstantina Ntarladima, University of Innsbruck (ANCFCable2D+ALE theory improvements and equations check)
- Alexander Humer, Johannes Kepler University Linz (initial discussions on structure and C++ code)
- Qasim Khadim, University of Oulu (suggestion for improved model of HydraulicsActuatorSimple with effective bulk modulus)
- Michael Gerbl, University of Innsbruck (figures in the documentation, taken from lecture notes)
- further examples provided by: Manuel Schieferle, Martin Knapp, Lukas March, Dominik Sponring, David Wibmer, Simon Scheiber and other Master students

-- thanks a lot! --

## Further notes

(sec-install-notes-goals)=
### Goals of Exudyn

After the first development phase (2019-2023), it

- is a moderately large  (wheels have only sizes of 2MB on Windows and 4MB on Linux, without fast Exudyn options) multibody library, which can be easily linked to other projects,
- contains basic multibody rigid bodies, flexible bodies, joints, contact, etc.,
- includes a large Python utility library for convenient building and post processing of models,
- allows to efficiently simulate small scale systems (compute $100\,000$s of time steps per second for systems with $n_{DOF}<10$),
- allows to efficiently simulate medium scaled systems for problems with $n_{DOF} < 1\,000\,000$,
- is a safe and widely accessible module for Python,
- allows to add user defined objects and solvers in C++,
- allows to add user defined objects and solvers in Python,
- allows multi-threaded parallel computing,
- includes Lie group integration,
- includes interfaces for robotics and ROS,
- includes interfaces for reinforcement learning (stable-baselines3), pytorch and artificial intelligence,
- includes kinematical trees with minimal coordinates.

Future goals (2024-2026) are:

- add specific and advanced connectors/constraints (extended wheels, contact, control connector)
- automatic step size selection for second order solvers (planned 2025),
- add 3D beams and plates (first attempts exist; planned 2024),
- export equations (planned, 2025),
- add GPU support (planned, 2025).

For solved issues (and new features), see section 'Issues and Bugs', {ref}`sec-issuetracker`.
For specific open issues, see `trackerlog.html` -- a document only intended for developers!

(sec-install-simpleexample)=
## Run a simple example in Python

After performing the steps of the previous section, this section shows a simplistic model which helps you to check if Exudyn runs on your computer.

The shortest check needs no file at all: `python -m exudyn demo` runs a built-in model, and
`python -m exudyn info` prints what is installed and where, see {ref}`sec-commandline`.

In order to start, run the Python interpreter Spyder (or any preferred Python environment).
In order to test the following example, which creates a {ref}`mbs <mbs>`, adds a node, an object, a marker and a load and simulates everything with default values,

- open `myFirstExample.py` from your `Examples` folder.

Hereafter, press the play button or `F5` in Spyder.

If successful, the IPython Console of Spyder will print something like:

```python
  runfile('C:/DATA/cpp/EXUDYN_git/python/Examples/myFirstExample.py',
    wdir='C:/DATA/cpp/EXUDYN_git/python/Examples')
  +++++++++++++++++++++++++++++++
  EXUDYN V1.2.9 solver: implicit second order time integration
  STEP100, t = 1 sec, timeToGo = 0 sec, Nit/step = 1
  solver finished after 0.0007824 seconds.
```

If you check your current directory (where `myFirstExample.py` lies), you will find a new file `coordinatesSolution.txt`, which contains the results of your computation (with default values for time integration).
The beginning and end of the file should look like: \

```python
  #Exudyn implicit second order time integration solver solution file
  #simulation started=2022-04-07,19:02:19
  #columns contain: time, ODE2 displacements, ODE2 velocities, ODE2 accelerations
  #number of system coordinates [nODE2, nODE1, nAlgebraic, nData] = [2,0,0,0]
  #number of written coordinates [nODE2, nVel2, nAcc2, nODE1, nVel1, nAlgebraic, nData] = [2,2,2,0,0,0,0]
  #total columns exported  (excl. time) = 6
  #number of time steps (planned) = 100
  #Exudyn version = 1.11.158.dev1; Python3.13.15; Windows x86_64 AVX2 FLOAT64
  #
  0,0,0,0,0,0.0001,0
  0.01,5e-09,0,1e-06,0,0.0001,0
  0.02,2e-08,0,2e-06,0,0.0001,0
  0.03,4.5e-08,0,3e-06,0,0.0001,0
  0.04,8e-08,0,4e-06,0,0.0001,0
  0.05,1.25e-07,0,5e-06,0,0.0001,0

  ...

  0.96,4.608e-05,0,9.6e-05,0,0.0001,0
  0.97,4.7045e-05,0,9.7e-05,0,0.0001,0
  0.98,4.802e-05,0,9.8e-05,0,0.0001,0
  0.99,4.9005e-05,0,9.9e-05,0,0.0001,0
  1,5e-05,0,0.0001,0,0.0001,0
  #simulation finished=2022-04-07,19:02:19
  #Solver Info: stepReductionFailed(or step failed)=0,discontinuousIterationSuccessful=1,newtonSolutionDiverged=0,massMatrixNotInvertible=1,total time steps=100,total Newton iterations=100,total Newton jacobians=100
```

Within this file, the first column shows the simulation time and the following columns provide coordinates, their derivatives and Lagrange multipliers on system level. For relation of local to global coordinates, see {ref}`sec-overview-ltgmapping`. As expected, the $x$-coordinate of the point mass has constant acceleration $a=f/m=0.001/10=0.0001$, the velocity grows up to $0.0001$ after 1 second and the point mass moves $0.00005$ along the $x$-axis.

Note that line 8 contains the Exudyn and Python versions (as well as some other specific information on the platform and compilation settings (which may help you identify with which computer, etc., you created results)) provided in the solution file are the versions at which Exudyn has been compiled with.
The Python micro version (last digit) may be different from the Python version from which you were running Exudyn.
This information is also provided in the sensor output files.

```{toctree}
:maxdepth: 2

gettingStartedInstall
gettingStartedFAQ
```
