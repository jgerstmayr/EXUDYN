

.. _sec-mainsystemextensions-solutionviewer:

Function: SolutionViewer
^^^^^^^^^^^^^^^^^^^^^^^^
`SolutionViewer <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L786>`__\ (\ ``solution = None``\ , \ ``rowIncrement = 1``\ , \ ``timeout = 0.04``\ , \ ``runOnStart = True``\ , \ ``runMode = 2``\ , \ ``fontSize = 12``\ , \ ``title = ''``\ , \ ``checkRenderEngineStopFlag = True``\ )

- | \ *function description*\ :
  | open interactive dialog and visulation (animate) solution loaded with LoadSolutionFile(...); Change slider 'Increment' to change the automatic increment of time frames; Change mode between continuous run, one cycle (fits perfect for animation recording) or 'Static' (to change Solution steps manually with the mouse); update period also lets you change the speed of animation; Press Run / Stop button to start/stop interactive mode (updating of grpahics)
  | - NOTE that this function is added to MainSystem via Python function SolutionViewer.
- | \ *input*\ :
  | \ ``solution``\ : solution dictionary previously loaded with exudyn.utilities.LoadSolutionFile(...); will be played from first to last row; if solution==None, it tries to load the file coordinatesSolutionFileName as stored in mbs.sys['simulationSettings'], which are the simulationSettings of the previous simulation
  | \ ``rowIncrement``\ : can be set larger than 1 in order to skip solution frames: e.g. rowIncrement=10 visualizes every 10th row (frame)
  | \ ``timeout``\ : in seconds is used between frames in order to limit the speed of animation; e.g. use timeout=0.04 to achieve approximately 25 frames per second
  | \ ``runOnStart``\ : immediately go into 'Run' mode
  | \ ``runMode``\ : 0=continuous run, 1=one cycle, 2=static (use slider/mouse to vary time steps)
  | \ ``fontSize``\ : define font size for labels in InteractiveDialog
  | \ ``title``\ : if empty, it uses default; otherwise define specific title
  | \ ``checkRenderEngineStopFlag``\ : if True, stopping renderer (pressing Q or Escape) also causes stopping the interactive dialog
- | \ *output*\ :
  | None; updates current visualization state, renders the scene continuously (after pressing button 'Run')
- | \ *example*\ :

.. code-block:: python

  #HERE, mbs must contain same model as solution stored in coordinatesSolution.txt
  #adjust autoFitScence, otherwise it may lead to unwanted fit to scene
  SC.visualizationSettings.general.autoFitScene = False
  from exudyn.interactive import SolutionViewer #import function
  sol = LoadSolutionFile('coordinatesSolution.txt') #load solution: adjust to your file name
  mbs.SolutionViewer(sol) #call via MainSystem


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `ANCFcableCantilevered.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcableCantilevered.py>`_\  (Ex), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Ex), \ `basicTutorial2024.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/basicTutorial2024.py>`_\  (Ex), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TM)



.. _sec-mainsystemextensions-plotsensor:

Function: PlotSensor
^^^^^^^^^^^^^^^^^^^^
`PlotSensor <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L230>`__\ (\ ``sensorNumbers = []``\ , \ ``components = 0``\ , \ ``xLabel = 'time (s)'``\ , \ ``yLabel = None``\ , \ ``labels = []``\ , \ ``colorCodeOffset = 0``\ , \ ``newFigure = True``\ , \ ``closeAll = False``\ , \ ``componentsX = []``\ , \ ``title = ''``\ , \ ``figureName = ''``\ , \ ``fontSize = 16``\ , \ ``colors = []``\ , \ ``lineStyles = []``\ , \ ``lineWidths = []``\ , \ ``markerStyles = []``\ , \ ``markerSizes = []``\ , \ ``markerDensity = 0.08``\ , \ ``rangeX = []``\ , \ ``rangeY = []``\ , \ ``majorTicksX = 10``\ , \ ``majorTicksY = 10``\ , \ ``offsets = []``\ , \ ``factors = []``\ , \ ``subPlot = []``\ , \ ``sizeInches = [6.4,4.8]``\ , \ ``fileName = ''``\ , \ ``useXYZcomponents = True``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | Helper function for direct and easy visualization of sensor outputs, without need for loading text files, etc.; PlotSensor can be used to simply plot, e.g., the measured x-Position over time in a figure. PlotSensor provides an interface to matplotlib (which needs to be installed). Default values of many function arguments can be changed using the exudyn.plot function PlotSensorDefaults(), see there for usage.
  | - NOTE that this function is added to MainSystem via Python function PlotSensor.
- | \ *input*\ :
  | \ ``sensorNumbers``\ : consists of one or a list of sensor numbers (type SensorIndex or int) as returned by the mbs function AddSensor(...); sensors need to set writeToFile=True and/or storeInternal=True for PlotSensor to work; alternatively, it may contain FILENAMES (incl. path) to stored sensor or solution files OR a numpy array instead of sensor numbers; the format of data (file or numpy array) must contain per row the time and according solution values in columns; if components is a list and sensorNumbers is a scalar, sensorNumbers is adjusted automatically to the components
  | \ ``components``\ : consists of one or a list of components according to the component of the sensor to be plotted at y-axis; if components is a list and sensorNumbers is a scalar, sensorNumbers is adjusted automatically to the components; as always, components are zero-based, meaning 0=X, 1=Y, etc.; for regular sensor files, time will be component=-1; to show the norm (e.g., of a force vector), use component=[plot.componentNorm] for according sensors; norm will consider all values of sensor except time (for 3D force, it will be \ :math:`\sqrt{f_0^2+f_1^2+f_2^2}`\ ); offsets and factors are mapped on norm (plot value=factor\*(norm(values) + offset) ), not on component values
  | \ ``componentsX``\ : default componentsX=[] uses time in files; otherwise provide componentsX as list of components (or scalar) representing x components of sensors in plotted curves; DON'T forget to change xLabel accordingly!
  | Using componentsX=[...] with a list of column indices specifies the respective columns used for the x-coordinates in all sensors; by default, values are plotted against the first column in the files, which is time; according to counting in PlotSensor, this represents componentX=-1;
  | plotting y over x in a position sensor thus reads: components=[1], componentsX=[0];
  | plotting time over x reads: components=[-1], componentsX=[0];
  | the default value reads componentsX=[-1,-1,...]
  | \ ``xLabel``\ : string for text at x-axis
  | \ ``yLabel``\ : string for text at y-axis (default: None==> label is automatically computed from sensor value types)
  | \ ``labels``\ : string (for one sensor) or list of strings (according to number of sensors resp. components) representing the labels used in legend; if labels=[], automatically generated legend is used
  | \ ``rangeX``\ : default rangeX=[]: computes range automatically; otherwise use rangeX to set range (limits) for x-axis provided as sorted list of two floats, e.g., rangeX=[0,4]
  | \ ``rangeY``\ : default rangeY=[]: computes range automatically; otherwise use rangeY to set range (limits) for y-axis provided as sorted list of two floats, e.g., rangeY=[-1,1]
  | \ ``figureName``\ : optional name for figure, if newFigure=True
  | \ ``fontSize``\ : change general fontsize of axis, labels, etc. (matplotlib default is 12, default in PlotSensor: 16)
  | \ ``title``\ : optional string representing plot title
  | \ ``offsets``\ : provide as scalar, list of scalars (per sensor) or list of 2D numpy.arrays (per sensor, having same rows/columns as sensor data; in this case it will also influence x-axis if componentsX is different from -1) to add offset to each sensor output; for an original value fOrig, the new value reads fNew = factor\*(fOrig+offset); for offset provided as numpy array (with same time values), the 'time' column is ignored in the offset computation; can be used to compute difference of sensors; if offsets=[], no offset is used
  | \ ``factors``\ : provide as scalar or list (per sensor) to add factor to each sensor output; for an original value fOrig, the new value reads fNew = factor\*(fOrig+offset); if factor=[], no factor is used
  | \ ``majorTicksX``\ : number of major ticks on x-axis; default: 10
  | \ ``majorTicksY``\ : number of major ticks on y-axis; default: 10
  | \ ``colorCodeOffset``\ : int offset for color code, color codes going from 0 to 27 (see PlotLineCode(...)); automatic line/color codes are used if no colors and lineStyles are used
  | \ ``colors``\ : color is automatically selected from colorCodeOffset if colors=[]; otherwise chose from 'b', 'g', 'r', 'c', 'm', 'y', 'k' and many other colors see https://matplotlib.org/stable/gallery/color/named_colors.html
  | \ ``lineStyles``\ : line style is automatically selected from colorCodeOffset if lineStyles=[]; otherwise define for all lines with string or with list of strings, chosing from '-', '--', '-.', ':', or ''
  | \ ``lineWidths``\ : float to define line width by float (default=1); either use single float for all sensors or list of floats with length >= number of sensors
  | \ ``markerStyles``\ : if different from [], marker styles are defined as list of marker style strings or single string for one sensor; chose from '.', 'o', 'x', '+' ... check listMarkerStylesFilled and listMarkerStyles in exudyn.plot and see https://matplotlib.org/stable/api/markers_api.html ; ADD a space to markers to make them empty (transparent), e.g. 'o ' will create an empty circle
  | \ ``markerSizes``\ : float to define marker size by float (default=6); either use single float for all sensors or list of floats with length >= number of sensors
  | \ ``markerDensity``\ : if int, it defines approx. the total number of markers used along each graph; if float, this defines the distance of markers relative to the diagonal of the plot (default=0.08); if None, it adds a marker to every data point if marker style is specified for sensor
  | \ ``newFigure``\ : if True, a new matplotlib.pyplot figure is created; otherwise, existing figures are overwritten
  | \ ``subPlot``\ : given as list [nx, ny, position] with nx, ny being the number of subplots in x and y direction (nx=cols, ny=rows), and position in [1,..., nx\*ny] gives the position in the subplots; use the same structure for first PlotSensor (with newFigure=True) and all subsequent PlotSensor calls with newFigure=False, which creates the according subplots; default=[](no subplots)
  | \ ``sizeInches``\ : given as list [sizeX, sizeY] with the sizes per (sub)plot given in inches; default: [6.4, 4.8]; in case of sub plots, the total size of the figure is computed from nx\*sizeInches[0] and ny\*sizeInches[1]
  | \ ``fileName``\ : if this string is non-empty, figure will be saved to given path and filename (use figName.pdf to safe as PDF or figName.png to save as PNG image); use matplotlib.use('Agg') in order not to open figures if you just want to save them
  | \ ``useXYZcomponents``\ : of True, it will use X, Y and Z for sensor components, e.g., measuring Position, Velocity, etc. wherever possible
  | \ ``closeAll``\ : if True, close all figures before opening new one (do this only in first PlotSensor command!)
  | \ ``[*kwargs]``\ :
  | \ ``minorTicksXon``\ : if True, turn minor ticks for x-axis on
  | \ ``minorTicksYon``\ : if True, turn minor ticks for y-axis on
  | \ ``logScaleX``\ : use log scale for x-axis
  | \ ``logScaleY``\ : use log scale for y-axis
  | \ ``fileCommentChar``\ : if exists, defines the comment character in files (\#, 
  | \ ``fileDelimiterChar``\ : if exists, defines the character indicating the columns for data (',', ' ', ';', ...)
- | \ *output*\ :
  | [Any, Any, Any, Any]; plots the sensor data; returns [plt, fig, ax, line] in which plt is matplotlib.pyplot, fig is the figure (or None), ax is the axis (or None) and line is the return value of plt.plot (or None) which could be changed hereafter
- | \ *notes*\ :
  | adjust default values by modifying the variables exudyn.plot.plotSensorDefault..., e.g., exudyn.plot.plotSensorDefaultFontSize
- | \ *example*\ :

.. code-block:: python

  #assume to have some position-based nodes 0 and 1:
  s0=mbs.AddSensor(SensorNode(nodeNumber=0, fileName='s0.txt',
                              outputVariableType=exu.OutputVariableType.Position))
  s1=mbs.AddSensor(SensorNode(nodeNumber=1, fileName='s1.txt',
                              outputVariableType=exu.OutputVariableType.Position))
  mbs.PlotSensor(s0, 0) #plot x-coordinate
  #plot x for s0 and z for s1:
  mbs.PlotSensor(sensorNumbers=[s0,s1], components=[0,2], yLabel='this is the position in meter')
  mbs.PlotSensor(sensorNumbers=s0, components=plot.componentNorm) #norm of position
  mbs.PlotSensor(sensorNumbers=s0, components=[0,1,2], factors=1000., title='Answers to the big questions')
  mbs.PlotSensor(sensorNumbers=s0, components=[0,1,2,3],
             yLabel='Coordantes with offset 1\nand scaled with $\\frac{1}{1000}$',
             factors=1e-3, offsets=1,fontSize=12, closeAll=True)
  #assume to have body sensor sBody, marker sensor sMarker:
  mbs.PlotSensor(sensorNumbers=[sBody]*3+[sMarker]*3, components=[0,1,2,0,1,2],
             colorCodeOffset=3, newFigure=False, fontSize=10,
             yLabel='Rotation $\\alpha, \\beta, \\gamma$ and\n Position $x,y,z$',
             title='compare marker and body sensor')
  #assume having file plotSensorNode.txt:
  mbs.PlotSensor(sensorNumbers=[s0]*3+ [filedir+'plotSensorNode.txt']*3,
             components=[0,1,2]*2)
  #plot y over x:
  mbs.PlotSensor(sensorNumbers=s0, componentsX=[0], components=[1], xLabel='x-Position', yLabel='y-Position')
  #for further examples, see also Examples/plotSensorExamples.py


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Ex), \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Ex), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Ex), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TM)



.. _sec-mainsystemextensions-solvestatic:

Function: SolveStatic
^^^^^^^^^^^^^^^^^^^^^
`SolveStatic <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L154>`__\ (\ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``updateInitialValues = False``\ , \ ``storeSolver = True``\ , \ ``showHints = False``\ , \ ``showCausingItems = True``\ )

- | \ *function description*\ :
  | solves the static mbs problem using simulationSettings; check theDoc.pdf for MainSolverStatic for further details of the static solver; this function is also available in exudyn (using exudyn.SolveStatic(...))
  | - NOTE that this function is added to MainSystem via Python function SolveStatic.
- | \ *input*\ :
  | \ ``simulationSettings``\ : specific simulation settings out of exu.SimulationSettings(), as described in Section :ref:`sec-solutionsettings`\ ; use options for newton, discontinuous settings, etc., from staticSolver sub-items
  | \ ``updateInitialValues``\ : if True, the results are written to initial values, such at a consecutive simulation uses the results of this simulation as the initial values of the next simulation
  | \ ``storeSolver``\ : if True, the staticSolver object is stored in the mbs.sys dictionary as mbs.sys['staticSolver'], and simulationSettings are stored as mbs.sys['simulationSettings']
- | \ *output*\ :
  | bool; returns True, if successful, False if fails; if storeSolver = True, mbs.sys contains staticSolver, which allows to investigate solver problems (check theDoc.pdf Section :ref:`sec-solversubstructures`\  and the items described in Section :ref:`sec-mainsolverstatic`\ )
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
  simulationSettings = exu.SimulationSettings()
  simulationSettings.timeIntegration.endTime = 10
  success = mbs.SolveStatic(simulationSettings, storeSolver = True)
  print("success =", success)
  print("iterations = ", mbs.sys['staticSolver'].it)
  print("pos=", mbs.GetObjectOutputBody(body,localPosition=[0,0,0],
        variableType=exu.OutputVariableType.Position))


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `3SpringsDistance.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/3SpringsDistance.py>`_\  (Ex), \ `ALEANCFpipe.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ALEANCFpipe.py>`_\  (Ex), \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Ex), \ `ANCFcantileverTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcantileverTest.py>`_\  (Ex), \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Ex), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TM), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFcontactCircleTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactCircleTest.py>`_\  (TM)



.. _sec-mainsystemextensions-solvedynamic:

Function: SolveDynamic
^^^^^^^^^^^^^^^^^^^^^^
`SolveDynamic <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L219>`__\ (\ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``solverType = exudyn.DynamicSolverType.GeneralizedAlpha``\ , \ ``updateInitialValues = False``\ , \ ``storeSolver = True``\ , \ ``showHints = False``\ , \ ``showCausingItems = True``\ )

- | \ *function description*\ :
  | solves the dynamic mbs problem using simulationSettings and solver type; check theDoc.pdf for MainSolverImplicitSecondOrder for further details of the dynamic solver; this function is also available in exudyn (using exudyn.SolveDynamic(...))
  | - NOTE that this function is added to MainSystem via Python function SolveDynamic.
- | \ *input*\ :
  | \ ``simulationSettings``\ : specific simulation settings out of exu.SimulationSettings(), as described in Section :ref:`sec-solutionsettings`\ ; use options for newton, discontinuous settings, etc., from timeIntegration; therein, implicit second order solvers use settings from generalizedAlpha and explict solvers from explicitIntegration; be careful with settings, as the influence accuracy (step size!), convergence and performance (see special Section :ref:`sec-overview-basics-speedup`\ )
  | \ ``solverType``\ : use exudyn.DynamicSolverType to set specific solver (default=generalized alpha)
  | \ ``updateInitialValues``\ : if True, the results are written to initial values, such at a consecutive simulation uses the results of this simulation as the initial values of the next simulation
  | \ ``storeSolver``\ : if True, the staticSolver object is stored in the mbs.sys dictionary as mbs.sys['staticSolver'], and simulationSettings are stored as mbs.sys['simulationSettings']
  | \ ``showHints``\ : show additional hints, if solver fails
  | \ ``showCausingItems``\ : if linear solver fails, this option helps to identify objects, etc. which are related to a singularity in the linearized system matrix
- | \ *output*\ :
  | bool; returns True, if successful, False if fails; if storeSolver = True, mbs.sys contains staticSolver, which allows to investigate solver problems (check theDoc.pdf Section :ref:`sec-solversubstructures`\  and the items described in Section :ref:`sec-mainsolverstatic`\ )
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
  #
  mbs.Assemble()
  simulationSettings = exu.SimulationSettings()
  simulationSettings.timeIntegration.endTime = 10
  success = mbs.SolveDynamic(simulationSettings, storeSolver = True)
  print("success =", success)
  print("iterations = ", mbs.sys['dynamicSolver'].it)
  print("pos=", mbs.GetObjectOutputBody(body,localPosition=[0,0,0],
        variableType=exu.OutputVariableType.Position))


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `3SpringsDistance.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/3SpringsDistance.py>`_\  (Ex), \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Ex), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Ex), \ `ALEANCFpipe.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ALEANCFpipe.py>`_\  (Ex), \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Ex), \ `abaqusImportTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/abaqusImportTest.py>`_\  (TM), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TM)



.. _sec-mainsystemextensions-computelinearizedsystem:

Function: ComputeLinearizedSystem
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeLinearizedSystem <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L369>`__\ (\ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``projectIntoConstraintNullspace = False``\ , \ ``singularValuesTolerance = 1e-12``\ , \ ``returnConstraintJacobian = False``\ , \ ``returnConstraintNullspace = False``\ )

- | \ *function description*\ :
  | compute linearized system of equations for ODE2 part of mbs, not considering the effects of algebraic constraints; for computation of eigenvalues and advanced computation with constrained systems, see ComputeODE2Eigenvalues; the current implementation is also able to project into the constrained space, however, this currently does not generally work with non-holonomic systems
  | - NOTE that this function is added to MainSystem via Python function ComputeLinearizedSystem.
- | \ *input*\ :
  | \ ``simulationSettings``\ : specific simulation settings used for computation of jacobian (e.g., sparse mode in static solver enables sparse computation)
  | \ ``projectIntoConstraintNullspace``\ : if False, algebraic equations (and constraint jacobian) are not considered for the linearized system; if True, the equations are projected into the nullspace of the constraints in the current configuration, using singular value decomposition; in the latter case, the returned list contains [M, K, D, C, N] where C is the constraint jacobian and N is the nullspace matrix (C and N may be an empty list, depending on the following flags)
  | \ ``singularValuesTolerance``\ : tolerance used to distinguish between zero and nonzero singular values for algebraic constraints projection
  | \ ``returnConstraintJacobian``\ : if True, the returned list contains [M, K, D, C, N] where C is the constraint jacobian and N is the nullspace matrix (may be empty)
  | \ ``returnConstraintNullspace``\ : if True, the returned list contains [M, K, D, C, N] where C is the constraint jacobian (may be empty) and N is the nullspace matrix
- | \ *output*\ :
  | [ArrayLike, ArrayLike, ArrayLike]; [M, K, D]; list containing numpy mass matrix M, stiffness matrix K and damping matrix D; for constraints, see options with arguments above, return values may change to [M, K, D, C, N]
- | \ *notes*\ :
  | consider paper of Agundez, Vallejo, Freire, Mikkola, "The dependent coordinates in the linearization of constrained multibody systems: Handling and elimination", https://www.sciencedirect.com/science/article/pii/S0020740324000791
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import *
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  #
  b0 = mbs.CreateMassPoint(referencePosition = [2,0,0],
                           initialVelocity = [2*0,5,0],
                           physicsMass = 1, gravity = [0,-9.81,0],
                           drawSize = 0.5, color=graphics.color.blue)
  #
  oGround = mbs.AddObject(ObjectGround())
  #add vertical spring
  oSD = mbs.CreateSpringDamper(bodyOrNodeList=[oGround, b0],
                               localPosition0=[2,1,0],
                               localPosition1=[0,0,0],
                               stiffness=1e4, damping=1e2,
                               drawSize=0.2)
  #
  mbs.Assemble()
  [M,K,D] = mbs.ComputeLinearizedSystem()
  exu.Print('M=\n',M,'\nK=\n',K,'\nD=\n',D)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TM), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TM), \ `geometricallyExactBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/geometricallyExactBeamTest.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-computeode2eigenvalues:

Function: ComputeODE2Eigenvalues
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeODE2Eigenvalues <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L502>`__\ (\ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``useSparseSolver = False``\ , \ ``numberOfEigenvalues = 0``\ , \ ``constrainedCoordinates = []``\ , \ ``convert2Frequencies = False``\ , \ ``useAbsoluteValues = True``\ , \ ``computeComplexEigenvalues = False``\ , \ ``ignoreAlgebraicEquations = False``\ , \ ``singularValuesTolerance = 1e-12``\ )

- | \ *function description*\ :
  | compute eigenvalues for unconstrained ODE2 part of mbs; the computation may include constraints in case that ignoreAlgebraicEquations=False (however, this currently does not generally work with non-holonomic systems); for algebraic constraints, however, a dense singular value decomposition of the constraint jacobian is used for the nullspace projection; the computation is done for the initial values of the mbs, independently of previous computations. If you would like to use the current state for the eigenvalue computation, you need to copy the current state to the initial state (using GetSystemState, SetSystemState, see Section :ref:`sec-mbs-systemdata`\ ); note that mass and stiffness matrices are computed in dense mode so far, while eigenvalues are computed according to useSparseSolver.
  | - NOTE that this function is added to MainSystem via Python function ComputeODE2Eigenvalues.
- | \ *input*\ :
  | \ ``simulationSettings``\ : specific simulation settings used for computation of jacobian (e.g., sparse mode in static solver enables sparse computation)
  | \ ``useSparseSolver``\ : if False (only for small systems), all eigenvalues are computed in dense mode (slow for large systems!); if True, only the numberOfEigenvalues are computed (numberOfEigenvalues must be set!); Currently, the matrices are exported only in DENSE MODE from mbs, which means that intermediate matrices may become huge for more than 5000 coordinates! NOTE that the sparsesolver accuracy is much less than the dense solver
  | \ ``numberOfEigenvalues``\ : number of eigenvalues and eivenvectors to be computed; if numberOfEigenvalues==0, all eigenvalues will be computed (may be impossible for larger or sparse problems!)
  | \ ``constrainedCoordinates``\ : if this list is non-empty (and there are no algebraic equations or ignoreAlgebraicEquations=True), the integer indices represent constrained coordinates of the system, which are fixed during eigenvalue/vector computation; according rows/columns of mass and stiffness matrices are erased; in this case, algebraic equations of the system are ignored
  | \ ``convert2Frequencies``\ : if True, the eigen values are converted into frequencies (Hz) and the output is [eigenFrequencies, eigenVectors]
  | \ ``useAbsoluteValues``\ : if True, abs(eigenvalues) is used, which avoids problems for small (close to zero) eigenvalues; needed, when converting to frequencies
  | \ ``computeComplexEigenvalues``\ : if True, the system is converted into a system of first order differential equations, including damping; for complex eigenvalues, set useAbsoluteValues=False (otherwise you will not get the complex values; values are unsorted, however!); only implemented for dense solver
  | \ ``ignoreAlgebraicEquations``\ : if True, algebraic equations (and constraint jacobian) are not considered for eigenvalue computation; otherwise, the solver tries to automatically project the system into the nullspace kernel of the constraint jacobian using a SVD; this gives eigenvalues of the constrained system; eigenvectors are not computed
  | \ ``singularValuesTolerance``\ : tolerance used to distinguish between zero and nonzero singular values for algebraic constraints projection
- | \ *output*\ :
  | [ArrayLike, ArrayLike]; [eigenValues, eigenVectors]; eigenValues being a numpy array of eigen values (\ :math:`\omega_i^2`\ , being the squared eigen frequencies in (\ :math:`\omega_i`\  in rad/s)!), eigenVectors a numpy array containing the eigenvectors in every column
- | \ *author*\ :
  | Johannes Gerstmayr, Michael Pieber
- | \ *example*\ :

.. code-block:: python

   #take any example from the Examples or TestModels folder, e.g., 'cartesianSpringDamper.py' and run it
   #specific example:
  import exudyn as exu
  from exudyn.utilities import *
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  #
  b0 = mbs.CreateMassPoint(referencePosition = [2,0,0],
                           physicsMass = 1, gravity = [0,-9.81,0],
                           drawSize = 0.5, color=graphics.color.blue)
  #
  oGround = mbs.AddObject(ObjectGround())
  #add vertical spring
  oSD = mbs.CreateSpringDamper(bodyOrNodeList=[oGround, b0],
                               localPosition0=[2,1,0],
                               localPosition1=[0,0,0],
                               stiffness=1e4, damping=1e2,
                               drawSize=0.2)
  #
  mbs.Assemble()
  #
  [eigenvalues, eigenvectors] = mbs.ComputeODE2Eigenvalues()
  #==>eigenvalues contain the eigenvalues of the ODE2 part of the system in the current configuration
  #
  #compute eigenfrequencies in Hz (analytical: 100/2/pi Hz for y-direction):
  [eigenvaluesHz, ev] = mbs.ComputeODE2Eigenvalues(convert2Frequencies=True)
  #
  #compute complex eigenvalues:
  [eigenvaluesComplex, ev] = mbs.ComputeODE2Eigenvalues(computeComplexEigenvalues=True,
                                                        useAbsoluteValues=False)


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `nMassOscillator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillator.py>`_\  (Ex), \ `nMassOscillatorEigenmodes.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorEigenmodes.py>`_\  (Ex), \ `nMassOscillatorInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorInteractive.py>`_\  (Ex), \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TM), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TM), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `complexEigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/complexEigenvaluesTest.py>`_\  (TM), \ `computeODE2AEeigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2AEeigenvaluesTest.py>`_\  (TM)



.. _sec-mainsystemextensions-computesystemdegreeoffreedom:

Function: ComputeSystemDegreeOfFreedom
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ComputeSystemDegreeOfFreedom <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/solver.py\#L719>`__\ (\ ``simulationSettings = exudyn.SimulationSettings()``\ , \ ``threshold = 1e-12``\ , \ ``verbose = False``\ , \ ``useSVD = False``\ )

- | \ *function description*\ :
  | compute system DOF numerically, considering Grübler-Kutzbach formula as well as redundant constraints; uses numpy matrix rank or singular value decomposition of scipy (useSVD=True)
  | - NOTE that this function is added to MainSystem via Python function ComputeSystemDegreeOfFreedom.
- | \ *input*\ :
  | \ ``simulationSettings``\ : used e.g. for settings regarding numerical differentiation; default settings may be used in most cases
  | \ ``threshold``\ : threshold factor for singular values which estimate the redundant constraints
  | \ ``useSVD``\ : use singular value decomposition directly, also showing SVD values if verbose=True
  | \ ``verbose``\ : if True, it will show the singular values and one may decide if the threshold shall be adapted
- | \ *output*\ :
  | dict; returns dictionary with key words 'degreeOfFreedom', 'redundantConstraints', 'nODE2', 'nODE1', 'nAE', 'nPureAE', where: degreeOfFreedom = the system degree of freedom computed numerically, redundantConstraints=the number of redundant constraints, nODE2=number of ODE2 coordinates, nODE1=number of ODE1 coordinates, nAE=total number of constraints, nPureAE=number of constraints on algebraic variables (e.g., lambda=0) that are not coupled to ODE2 coordinates
- | \ *notes*\ :
  | this approach could possibly fail with special constraints! Currently only works with dense matrices, thus it will be slow for larger systems
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import *
  import numpy as np
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  #
  b0 = mbs.CreateRigidBody(inertia = InertiaCuboid(density=5000,
                                                   sideLengths=[1,0.1,0.1]),
                           referencePosition = [6,0,0],
                           initialAngularVelocity = [0,8,0],
                           gravity = [0,-9.81,0],
                           graphicsDataList = [exu.graphics.Brick(size=[1,0.1,0.1],
                                                                        color=graphics.color.orange)])
  oGround = mbs.AddObject(ObjectGround())
  mbs.CreateGenericJoint(bodyNumbers=[oGround, b0], position=[5.5,0,0],
                         constrainedAxes=[1,1,1, 1,0,0],
                         rotationMatrixAxes=RotationMatrixX(0.125*pi), #tilt axes
                         useGlobalFrame=True, axesRadius=0.02, axesLength=0.2)
  #
  mbs.Assemble()
  dof = mbs.ComputeSystemDegreeOfFreedom(verbose=1)['degreeOfFreedom'] #print out details


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Ex), \ `bricardMechanism.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/bricardMechanism.py>`_\  (TM), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)



.. _sec-mainsystemextensions-createdistancesensorgeometry:

Function: CreateDistanceSensorGeometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateDistanceSensorGeometry <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L195>`__\ (\ ``meshPoints``\ , \ ``meshTrigs``\ , \ ``rigidBodyMarkerIndex``\ , \ ``searchTreeCellSize = [8,8,8]``\ )

- | \ *function description*\ :
  | Add geometry for distance sensor given by points and triangles (point indices) to mbs; use a rigid body marker where the geometry is put on;
  | Creates a GeneralContact for efficient search on background. If you have several sets of points and trigs, first merge them or add them manually to the contact
  | - NOTE that this function is added to MainSystem via Python function CreateDistanceSensorGeometry.
- | \ *input*\ :
  | \ ``meshPoints``\ : list of points (3D), as returned by graphics.ToPointsAndTrigs()
  | \ ``meshTrigs``\ : list of trigs (3 node indices each), as returned by graphics.ToPointsAndTrigs()
  | \ ``rigidBodyMarkerIndex``\ : rigid body marker to which the triangles are fixed on (ground or moving object)
  | \ ``searchTreeCellSize``\ : size of search tree (X,Y,Z); use larger values in directions where more triangles are located
- | \ *output*\ :
  | int; returns ngc, which is the number of GeneralContact in mbs, to be used in CreateDistanceSensor(...); keep the gContact as deletion may corrupt data
- | \ *notes*\ :
  | should be used by CreateDistanceSensor(...) and AddLidar(...) for simple initialization of GeneralContact; old name: DistanceSensorSetupGeometry(...)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `mobileMecanumWheelRobotWithLidar.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mobileMecanumWheelRobotWithLidar.py>`_\  (Ex), \ `laserScannerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/laserScannerTest.py>`_\  (TM)



.. _sec-mainsystemextensions-createdistancesensor:

Function: CreateDistanceSensor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateDistanceSensor <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L228>`__\ (\ ``generalContactIndex``\ , \ ``positionOrMarker``\ , \ ``dirSensor``\ , \ ``minDistance = -1e7``\ , \ ``maxDistance = 1e7``\ , \ ``cylinderRadius = 0``\ , \ ``selectedTypeIndex = exudyn.ContactTypeIndex.IndexEndOfEnumList``\ , \ ``storeInternal = False``\ , \ ``fileName = ''``\ , \ ``measureVelocity = False``\ , \ ``addGraphicsObject = False``\ , \ ``drawDisplaced = True``\ , \ ``color = exudyn.graphics.color.red``\ )

- | \ *function description*\ :
  | Function to create distance sensor based on GeneralContact in mbs; sensor can be either placed on absolute position or attached to rigid body marker; in case of marker, dirSensor is relative to the marker
  | - NOTE that this function is added to MainSystem via Python function CreateDistanceSensor.
- | \ *input*\ :
  | \ ``generalContactIndex``\ : the number of the GeneralContact object in mbs; the index of the GeneralContact object which has been added with last AddGeneralContact(...) command is generalContactIndex=mbs.NumberOfGeneralContacts()-1
  | \ ``positionOrMarker``\ : either a 3D position as list or np.array, or a MarkerIndex with according rigid body marker
  | \ ``dirSensor``\ : the direction (no need to normalize) along which the distance is measured (must not be normalized); in case of marker, the direction is relative to marker orientation if marker contains orientation (BodyRigid, NodeRigid)
  | \ ``minDistance``\ : the minimum distance which is accepted; smaller distance will be ignored
  | \ ``maxDistance``\ : the maximum distance which is accepted; items being at maxDistance or futher are ignored; if no items are found, the function returns maxDistance
  | \ ``cylinderRadius``\ : in case of spheres (selectedTypeIndex=ContactTypeIndex.IndexSpheresMarkerBased), a cylinder can be used which measures the shortest distance at a certain radius (geometrically interpreted as cylinder)
  | \ ``selectedTypeIndex``\ : either this type has default value, meaning that all items in GeneralContact are measured, or there is a specific type index, which is the only type that is considered during measurement
  | \ ``storeInternal``\ : like with any SensorUserFunction, setting to True stores sensor data internally
  | \ ``fileName``\ : if defined, recorded data of SensorUserFunction is written to specified file
  | \ ``measureVelocity``\ : if True, the sensor measures additionally the velocity (component 0=distance, component 1=velocity); velocity is the velocity in direction 'dirSensor' and does not account for changes in geometry, thus it may be different from the time derivative of the distance!
  | \ ``addGraphicsObject``\ : if True, the distance sensor is also visualized graphically in a simplified manner with a red line having the length of dirSensor; NOTE that updates are ONLY performed during computation, not in visualization; for this reason, solutionSettings.sensorsWritePeriod should be accordingly small
  | \ ``drawDisplaced``\ : if True, the red line is drawn backwards such that it moves along the measured surface; if False, the beam is fixed to marker or position
  | \ ``color``\ : optional color for 'laser beam' to be drawn
- | \ *output*\ :
  | SensorIndex; creates sensor and returns according sensor number of SensorUserFunction
- | \ *notes*\ :
  | use generalContactIndex = CreateDistanceSensorGeometry(...) before to create GeneralContact module containing geometry; old name: AddDistanceSensor(...)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `laserScannerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/laserScannerTest.py>`_\  (TM)



.. _sec-mainsystemextensions-drawsystemgraph:

Function: DrawSystemGraph
^^^^^^^^^^^^^^^^^^^^^^^^^
`DrawSystemGraph <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/utilities.py\#L883>`__\ (\ ``showLoads = True``\ , \ ``showSensors = True``\ , \ ``useItemNames = False``\ , \ ``useItemTypes = False``\ , \ ``addItemTypeNames = True``\ , \ ``multiLine = True``\ , \ ``fontSizeFactor = 1.``\ , \ ``layoutDistanceFactor = 3.``\ , \ ``layoutIterations = 100``\ , \ ``showLegend = True``\ , \ ``tightLayout = True``\ )

- | \ *function description*\ :
  | helper function which draws system graph of a MainSystem (mbs); several options let adjust the appearance of the graph; the graph visualization uses randomizer, which results in different graphs after every run!
  | - NOTE that this function is added to MainSystem via Python function DrawSystemGraph.
- | \ *input*\ :
  | \ ``showLoads``\ : toggle appearance of loads in mbs
  | \ ``showSensors``\ : toggle appearance of sensors in mbs
  | \ ``useItemNames``\ : if True, object names are shown instead of basic object types (Node, Load, ...)
  | \ ``useItemTypes``\ : if True, object type names (MassPoint, JointRevolute, ...) are shown instead of basic object types (Node, Load, ...); Note that Node, Object, is omitted at the beginning of itemName (as compared to theDoc.pdf); item classes become clear from the legend
  | \ ``addItemTypeNames``\ : if True, type nymes (Node, Load, etc.) are added
  | \ ``multiLine``\ : if True, labels are multiline, improving readability
  | \ ``fontSizeFactor``\ : use this factor to scale fonts, allowing to fit larger graphs on the screen with values < 1
  | \ ``showLegend``\ : shows legend for different item types
  | \ ``layoutDistanceFactor``\ : this factor influences the arrangement of labels; larger distance values lead to circle-like results
  | \ ``layoutIterations``\ : more iterations lead to better arrangement of the layout, but need more time for larger systems (use 1000-10000 to get good results)
  | \ ``tightLayout``\ : if True, uses matplotlib plt.tight_layout() which may raise warning
- | \ *output*\ :
  | [Any, Any, Any]; returns [networkx, G, items] with nx being networkx, G the graph and item what is returned by nx.draw_networkx_labels(...)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Ex), \ `rigidBodyTutorial3.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3.py>`_\  (Ex), \ `rigidBodyTutorial3withMarkers.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyTutorial3withMarkers.py>`_\  (Ex), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TM)

