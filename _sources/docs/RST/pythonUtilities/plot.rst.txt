
.. _sec-module-plot:

Module: plot
============

Plot utility functions based on matplotlib, including plotting of sensors and FFT.

- Author:    Johannes Gerstmayr 
- Date:      2020-09-16 (created) 
- Notes:     For a list of plot colors useful for matplotlib, see also advancedUtilities.PlotLineCode(...) 


.. _sec-plot-parseoutputfileheader:

Function: ParseOutputFileHeader
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ParseOutputFileHeader <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L56>`__\ (\ ``lines``\ )

- | \ *function description*\ :
  | parse header of output file (solution file, sensor file, genetic optimization output, ...) given in file.readlines() format
- | \ *output*\ :
  | return dictionary with 'type'=['sensor','solution','geneticOptimization','parameterVariation'], 'variableType' containing variable types, 'variableRanges' containing ranges for parameter variation



----


.. _sec-plot-plotsensordefaults:

Function: PlotSensorDefaults
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`PlotSensorDefaults <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L151>`__\ ()

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
  exudyn.Print(PlotSensorDefaults())


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `serialRobotFlexible.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotFlexible.py>`_\  (Ex), \ `coordinateVectorConstraintGenericODE2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateVectorConstraintGenericODE2.py>`_\  (TM)



----

Function: PlotSensor
^^^^^^^^^^^^^^^^^^^^
`PlotSensor <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L227>`__\ (\ ``mbs``\ , \ ``sensorNumbers = []``\ , \ ``components = 0``\ , \ ``xLabel = 'time (s)'``\ , \ ``yLabel = None``\ , \ ``labels = []``\ , \ ``colorCodeOffset = 0``\ , \ ``newFigure = True``\ , \ ``closeAll = False``\ , \ ``componentsX = []``\ , \ ``title = ''``\ , \ ``figureName = ''``\ , \ ``fontSize = 16``\ , \ ``colors = []``\ , \ ``lineStyles = []``\ , \ ``lineWidths = []``\ , \ ``markerStyles = []``\ , \ ``markerSizes = []``\ , \ ``markerDensity = 0.08``\ , \ ``rangeX = []``\ , \ ``rangeY = []``\ , \ ``majorTicksX = 10``\ , \ ``majorTicksY = 10``\ , \ ``offsets = []``\ , \ ``factors = []``\ , \ ``subPlot = []``\ , \ ``sizeInches = [6.4,4.8]``\ , \ ``fileName = ''``\ , \ ``useXYZcomponents = True``\ , \ ``**kwargs``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.PlotSensor(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-plotsensor`\ 



----


.. _sec-plot-plotfft:

Function: PlotFFT
^^^^^^^^^^^^^^^^^
`PlotFFT <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L685>`__\ (\ ``frequency``\ , \ ``data``\ , \ ``xLabel = 'frequency'``\ , \ ``yLabel = 'magnitude'``\ , \ ``label = ''``\ , \ ``freqStart = 0``\ , \ ``freqEnd = -1``\ , \ ``logScaleX = True``\ , \ ``logScaleY = True``\ , \ ``majorGrid = True``\ , \ ``minorGrid = True``\ )

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

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `nMassOscillatorEigenmodes.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorEigenmodes.py>`_\  (Ex)



----


.. _sec-plot-filestripspaces:

Function: FileStripSpaces
^^^^^^^^^^^^^^^^^^^^^^^^^
`FileStripSpaces <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L734>`__\ (\ ``filename``\ , \ ``outputFilename``\ , \ ``fileCommentChar = ''``\ , \ ``removeDoubleChars = ''``\ )

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

Function: DataArrayFromSensorList
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`DataArrayFromSensorList <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L761>`__\ (\ ``mbs``\ , \ ``sensorNumbers``\ , \ ``positionList = []``\ , \ ``time = ''``\ )

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

Function: LoadImage
^^^^^^^^^^^^^^^^^^^
`LoadImage <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L788>`__\ (\ ``fileName``\ , \ ``trianglesAsLines = True``\ , \ ``verbose = False``\ )

- | \ *function description*\ :
  | import image text file as exported from renderer.RedrawAndSaveImage() with exportImages.saveImageFormat='TXT'; triangles are converted to lines
- | \ *input*\ :
  | fileName includes directory
- | \ *output*\ :
  | returns dictionary with according structures

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex)



----


.. _sec-plot-plotimage:

Function: PlotImage
^^^^^^^^^^^^^^^^^^^
`PlotImage <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/plot.py\#L859>`__\ (\ ``imageData``\ , \ ``HT = np.eye(4)``\ , \ ``axesEqual = True``\ , \ ``plot3D = False``\ , \ ``lineWidths = 1``\ , \ ``lineStyles = '-'``\ , \ ``triangleEdgeColors = 'black'``\ , \ ``triangleEdgeWidths = 0.5``\ , \ ``removeAxes = True``\ , \ ``orthogonalProjection = True``\ , \ ``title = ''``\ , \ ``figureName = ''``\ , \ ``fileName = ''``\ , \ ``fontSize = 16``\ , \ ``closeAll = False``\ , \ ``azim = 0.``\ , \ ``elev = 0.``\ )

- | \ *function description*\ :
  | plot 2D or 3D vector image data as provided by LoadImage(...) using matplotlib
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

