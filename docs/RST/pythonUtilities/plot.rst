
.. _sec-module-plot:

Module: plot
============

Plot utility functions based on matplotlib, including plotting of sensors and FFT.

- Author:    Johannes Gerstmayr 
- Date:      2020-09-16 (created) 
- Notes: 	For a list of plot colors useful for matplotlib, see also utilities.PlotLineCode(...) 


.. _sec-plot-parseoutputfileheader:

Function: `ParseOutputFileHeader <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L57>`__\ (\ ``lines``\ )

- | \ *function description*\ :
  | parse header of output file (solution file, sensor file, genetic optimization output, ...) given in file.readlines() format
- | \ *output*\ :
  | return dictionary with 'type'=['sensor','solution','geneticOptimization','parameterVariation'], 'variableType' containing variable types, 'variableRanges' containing ranges for parameter variation


----

.. _sec-plot-plotsensordefaults:

Function: `PlotSensorDefaults <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L155>`__\ ()

- | \ *function description*\ :
  | returns structure with default values for PlotSensor which can be modified once to be set for all later calls of PlotSensor
- | \ *example*\ :

.. code-block:: python

  #change one parameter:
  plot.PlotSensorDefaults().fontSize = 12
  #==>now PlotSensor(...) will use fontSize=12
  #==>now PlotSensor(..., fontSize=10) will use fontSize=10
  #==>BUT PlotSensor(..., fontSize=16) will use fontSize=12, BECAUSE 16 is the original default value!!!
  #see which parameters are available:
  print(PlotSensorDefaults())


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `serialRobotFlexible.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotFlexible.py>`_\  (Ex), \ `coordinateVectorConstraintGenericODE2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateVectorConstraintGenericODE2.py>`_\  (TM)


----

.. _sec-plot-plotsensor:

Function: `PlotSensor <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L228>`__\ (\ ``mbs``\ , \ ``sensorNumbers = []``\ , \ ``components = 0``\ , \ ``xLabel = 'time (s)'``\ , \ ``yLabel = None``\ , \ ``labels = []``\ , \ ``colorCodeOffset = 0``\ , \ ``newFigure = True``\ , \ ``closeAll = False``\ , \ ``componentsX = []``\ , \ ``title = ''``\ , \ ``figureName = ''``\ , \ ``fontSize = 16``\ , \ ``colors = []``\ , \ ``lineStyles = []``\ , \ ``lineWidths = []``\ , \ ``markerStyles = []``\ , \ ``markerSizes = []``\ , \ ``markerDensity = 0.08``\ , \ ``rangeX = []``\ , \ ``rangeY = []``\ , \ ``majorTicksX = 10``\ , \ ``majorTicksY = 10``\ , \ ``offsets = []``\ , \ ``factors = []``\ , \ ``subPlot = []``\ , \ ``sizeInches = [6.4,4.8]``\ , \ ``fileName = ''``\ , \ ``useXYZcomponents = True``\ , \ ``**kwargs``\ )

- | \ *function description*\ :
  | Helper function for direct and easy visualization of sensor outputs, without need for loading text files, etc.; PlotSensor can be used to simply plot, e.g., the measured x-Position over time in a figure. PlotSensor provides an interface to matplotlib (which needs to be installed). Default values of many function arguments can be changed using the exudyn.plot function PlotSensorDefaults(), see there for usage.
- | \ *input*\ :
  | \ ``mbs``\ : must be a valid MainSystem (mbs)
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
  | \ ``fileCommentChar``\ : if exists, defines the comment character in files (\#, %, ...)
  | \ ``fileDelimiterChar``\ : if exists, defines the character indicating the columns for data (',', ' ', ';', ...)
- | \ *output*\ :
  | plots the sensor data; returns [plt, fig, ax, line] in which plt is matplotlib.pyplot, fig is the figure (or None), ax is the axis (or None) and line is the return value of plt.plot (or None) which could be changed hereafter
- | \ *notes*\ :
  | adjust default values by modifying the variables exudyn.plot.plotSensorDefault..., e.g., exudyn.plot.plotSensorDefaultFontSize
- | \ *example*\ :

.. code-block:: python

  #assume to have some position-based nodes 0 and 1:
  s0=mbs.AddSensor(SensorNode(nodeNumber=0, fileName='s0.txt',
  outputVariableType=exu.OutputVariableType.Position))
  s1=mbs.AddSensor(SensorNode(nodeNumber=1, fileName='s1.txt',
  outputVariableType=exu.OutputVariableType.Position))
  PlotSensor(mbs, s0, 0) #plot x-coordinate
  #plot x for s0 and z for s1:
  PlotSensor(mbs, sensorNumbers=[s0,s1], components=[0,2], yLabel='this is the position in meter')
  PlotSensor(mbs, sensorNumbers=s0, components=plot.componentNorm) #norm of position
  PlotSensor(mbs, sensorNumbers=s0, components=[0,1,2], factors=1000., title='Answers to the big questions')
  PlotSensor(mbs, sensorNumbers=s0, components=[0,1,2,3],
  yLabel='Coordantes with offset 1\nand scaled with $\\frac{1}{1000}$',
  factors=1e-3, offsets=1,fontSize=12, closeAll=True)
  #assume to have body sensor sBody, marker sensor sMarker:
  PlotSensor(mbs, sensorNumbers=[sBody]*3+[sMarker]*3, components=[0,1,2,0,1,2],
  colorCodeOffset=3, newFigure=False, fontSize=10,
  yLabel='Rotation $\\alpha, \\beta, \\gamma$ and\n Position $x,y,z$',
  title='compare marker and body sensor')
  #assume having file plotSensorNode.txt:
  PlotSensor(mbs, sensorNumbers=[s0]*3+ [filedir+'plotSensorNode.txt']*3,
  components=[0,1,2]*2)
  #plot y over x:
  PlotSensor(mbs, sensorNumbers=s0, componentsX=[0], components=[1], xLabel='x-Position', yLabel='y-Position')
  #for further examples, see also Examples/plotSensorExamples.py


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Ex), \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Ex), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Ex), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Ex), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TM), \ `ANCFgeneralContactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFgeneralContactCircle.py>`_\  (TM)


----

.. _sec-plot-plotfft:

Function: `PlotFFT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L669>`__\ (\ ``frequency``\ , \ ``data``\ , \ ``xLabel = 'frequency'``\ , \ ``yLabel = 'magnitude'``\ , \ ``label = ''``\ , \ ``freqStart = 0``\ , \ ``freqEnd = -1``\ , \ ``logScaleX = True``\ , \ ``logScaleY = True``\ , \ ``majorGrid = True``\ , \ ``minorGrid = True``\ )

- | \ *function description*\ :
  | plot fft spectrum of signal
- | \ *input*\ :
  | \ ``frequency``\ :  frequency vector (Hz, if time is in SECONDS)
  | \ ``data``\ :       magnitude or phase as returned by ComputeFFT() in exudyn.signalProcessing
  | \ ``xLabel``\ :     label for x-axis, default=frequency
  | \ ``yLabel``\ :     label for y-axis, default=magnitude
  | \ ``label``\ :      either empty string ('') or name used in legend
  | \ ``freqStart``\ :  starting range for frequency
  | \ ``freqEnd``\ :    end of range for frequency; if freqEnd==-1 (default), the total range is plotted
  | \ ``logScaleX``\ :  use log scale for x-axis
  | \ ``logScaleY``\ :  use log scale for y-axis
  | \ ``majorGrid``\ :  if True, plot major grid with solid line
  | \ ``minorGrid``\ :  if True, plot minor grid with dotted line
- | \ *output*\ :
  | creates plot and returns plot (plt) handle


----

.. _sec-plot-filestripspaces:

Function: `FileStripSpaces <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L719>`__\ (\ ``filename``\ , \ ``outputFilename``\ , \ ``fileCommentChar = ''``\ , \ ``removeDoubleChars = ''``\ )

- | \ *function description*\ :
  | strip spaces at beginning / end of lines; this may be sometimes necessary when reading solutions from files that are space-separated
- | \ *input*\ :
  | \ ``filename``\ : name of file to process
  | \ ``outputFilename``\ : name of file to which text without leading/trailing spaces is written
  | \ ``fileCommentChar``\ : if not equal '', lines starting with this character will not be processed
  | \ ``removeDoubleChars``\ : if not equal '', this double characters (especial multiple spaces) will be removed; '1.0   3.0' will be converted into '1.0 3.0'
- | \ *output*\ :
  | new file written


----

.. _sec-plot-dataarrayfromsensorlist:

Function: `DataArrayFromSensorList <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L746>`__\ (\ ``mbs``\ , \ ``sensorNumbers``\ , \ ``positionList = []``\ , \ ``time = ''``\ )

- | \ *function description*\ :
  | helper function to create data array from outputs defined by sensorNumbers list [+optional positionList which must have, e.g., local arc-length of beam according to sensor numbers]; if time=='', current sensor values will be used; if time!=[], evaluation will be based on loading values from file or sensor internal data and evaluate at that time
- | \ *input*\ :
  | \ ``mbs``\ : a MainSystem where the sensors are given
  | \ ``sensorNumbers``\ : a list of sensor numbers, which shall be evaluated
  | \ ``positionList``\ : an optional list of positions per sensor (e.g., axial positions at beam)
  | \ ``time``\ : optional time at which the sensor values are evaluated (currently not implemented)
- | \ *output*\ :
  | returns data as numpy array, containg per row the number or position (positionList) in the first column and all sensor values in the remaining columns

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Ex), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Ex)


----

.. _sec-plot-loadimage:

Function: `LoadImage <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L773>`__\ (\ ``fileName``\ , \ ``trianglesAsLines = True``\ , \ ``verbose = False``\ )

- | \ *function description*\ :
  | import image text file as exported from RedrawAndSaveImage() with exportImages.saveImageFormat='TXT'; triangles are converted to lines
- | \ *input*\ :
  | fileName includes directory
- | \ *output*\ :
  | returns dictionary with according structures

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex)


----

.. _sec-plot-plotimage:

Function: `PlotImage <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L846>`__\ (\ ``imageData``\ , \ ``HT = np.eye(4)``\ , \ ``axesEqual = True``\ , \ ``plot3D = False``\ , \ ``lineWidths = 1``\ , \ ``lineStyles = '-'``\ , \ ``triangleEdgeColors = 'black'``\ , \ ``triangleEdgeWidths = 0.5``\ , \ ``removeAxes = True``\ , \ ``orthogonalProjection = True``\ , \ ``title = ''``\ , \ ``figureName = ''``\ , \ ``fileName = ''``\ , \ ``fontSize = 16``\ , \ ``closeAll = False``\ , \ ``azim = 0.``\ , \ ``elev = 0.``\ )

- | \ *function description*\ :
  | plot image data as provided by LoadImage(...) using matplotlib; (currently) only plots lines; triangles are not processed
- | \ *input*\ :
  | \ ``imageData``\ : dictionary as provided by LoadImage(...)
  | \ ``HT``\ : homogeneous transformation, used to transform coordinates; lines are drawn in (x,y) plane
  | \ ``axesEqual``\ : for 2D mode, axis are set equal, otherwise model is distorted
  | \ ``plot3D``\ : in this mode, a 3D visualization is used; triangles are only be displayed in this mode!
  | \ ``lineWidths``\ : width of lines
  | \ ``lineStyles``\ : matplotlib codes for lines
  | \ ``triangleEdgeColors``\ : color for triangle edges as tuple of rgb colors or matplotlib color code strings 'black', 'r', ...
  | \ ``triangleEdgeWidths``\ : width of triangle edges; set to 0 if edges shall not be shown
  | \ ``removeAxes``\ : if True, all axes and background are removed for simpler export
  | \ ``orthogonalProjection``\ : if True, projection is orthogonal with no perspective view
  | \ ``title``\ : optional string representing plot title
  | \ ``figureName``\ : optional name for figure, if newFigure=True
  | \ ``fileName``\ : if this string is non-empty, figure will be saved to given path and filename (use figName.pdf to safe as PDF or figName.png to save as PNG image); use matplotlib.use('Agg') in order not to open figures if you just want to save them
  | \ ``fontSize``\ : change general fontsize of axis, labels, etc. (matplotlib default is 12, default in PlotSensor: 16)
  | \ ``closeAll``\ : if True, close all figures before opening new one (do this only in first PlotSensor command!)
  | azim, elev: for 3D plots: the initial angles for the 3D view in degrees

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex)

