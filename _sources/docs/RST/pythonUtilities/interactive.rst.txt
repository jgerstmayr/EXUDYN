
.. _sec-module-interactive:

Module: interactive
===================

Utilities for interactive simulation and results monitoring; NOTE: does not work on MacOS!

- Author:    Johannes Gerstmayr 
- Date:      2021-01-17 (created) 


.. _sec-interactive-animatemodes:

Function: AnimateModes
^^^^^^^^^^^^^^^^^^^^^^
`AnimateModes <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L591>`__\ (\ ``systemContainer``\ , \ ``mainSystem``\ , \ ``nodeNumber``\ , \ ``period = 0.04``\ , \ ``stepsPerPeriod = 30``\ , \ ``showTime = True``\ , \ ``renderWindowText = ''``\ , \ ``runOnStart = False``\ , \ ``runMode = 0``\ , \ ``scaleAmplitude = 1``\ , \ ``title = ''``\ , \ ``fontSize = 12``\ , \ ``checkRenderEngineStopFlag = True``\ , \ ``systemEigenVectors = None``\ )

- | \ *function description*\ :
  | animate modes of ObjectFFRFreducedOrder, of nodal coordinates (changes periodically one nodal coordinate) or of a list of system modes provided as list of lists; for creating snapshots, press 'Static' and 'Record frames' and press 'Run' to save one figure in the image subfolder; for creating animations for one mode, use the same procedure but use 'One Cycle'. Modes may be inverted by pressing according '+' and '-' buttons next to Amplitude.
- | \ *input*\ :
  | \ ``systemContainer``\ : system container (usually SC) of your model, containing visualization settings
  | \ ``mainSystem``\ : system (usually mbs) containing your model
  | \ ``nodeNumber``\ : node number of which the coordinates shall be animated. In case of ObjectFFRFreducedOrder, this is the generic node, e.g., 'nGenericODE2' in the dictionary returned by the function AddObjectFFRFreducedOrderWithUserFunctions(...); if nodeNumber=None, then the systemEigenVectors list is used
  | \ ``period``\ : delay for animation of every frame; the default of 0.04 results in approximately 25 frames per second
  | \ ``stepsPerPeriod``\ : number of steps into which the animation of one cycle of the mode is split into
  | \ ``showTime``\ : show a virtual time running from 0 to 2\*pi during one mode cycle
  | \ ``renderWindowText``\ : additional text written into renderwindow before 'Mode X' (use \ :math:`\backslash`\ n to add line breaks)
  | \ ``runOnStart``\ : immediately go into 'Run' mode
  | \ ``runMode``\ : 0=continuous run, 1=static continuous, 2=one cycle, 3=static (use slider/mouse to vary time steps)
  | \ ``scaleAmplitude``\ : additional scaling for amplitude if necessary
  | \ ``fontSize``\ : define font size for labels in InteractiveDialog
  | \ ``title``\ : if empty, it uses default; otherwise define specific title
  | \ ``checkRenderEngineStopFlag``\ : if True, stopping renderer (pressing Q or Escape) also causes stopping the interactive dialog
  | \ ``systemEigenVectors``\ : may be a list of lists of system eigenvectors for ODE2 (and possibly ODE1) coordinates or a eigenvector matrix containing mode vectors in columns; if nodeNumber=None, these eigenvectors are then used to be animated
- | \ *output*\ :
  | opens interactive dialog with further settings
- | \ *notes*\ :
  | Uses class InteractiveDialog in the background, which can be used to adjust animation creation. If meshes are large, animation artifacts may appear, which are resolved by using a larger update period.
  | Press 'Run' to start animation; Chose 'Mode shape', according component for contour plot; to record one cycle for animation, choose 'One cycle', run once to get the according range in the contour plot, press 'Record frames' and press 'Run', now images can be found in subfolder 'images' (for further info on animation creation see Section :ref:`sec-overview-basics-animations`\ ); now deactivate 'Record frames' by pressing 'Off' and chose another mode

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `CMSexampleCourse.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/CMSexampleCourse.py>`_\  (Ex), \ `netgenSTLtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/netgenSTLtest.py>`_\  (Ex), \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Ex), \ `NGsolveCraigBampton.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCraigBampton.py>`_\  (Ex), \ `NGsolveFFRF.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveFFRF.py>`_\  (Ex), \ `objectFFRFreducedOrderShowModes.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderShowModes.py>`_\  (TM), \ `runTestExamples.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/runTestExamples.py>`_\  (TM)



----

Function: SolutionViewer
^^^^^^^^^^^^^^^^^^^^^^^^
`SolutionViewer <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L785>`__\ (\ ``mainSystem``\ , \ ``solution = None``\ , \ ``rowIncrement = 1``\ , \ ``timeout = 0.04``\ , \ ``runOnStart = True``\ , \ ``runMode = 2``\ , \ ``fontSize = 12``\ , \ ``title = ''``\ , \ ``checkRenderEngineStopFlag = True``\ )


- | **NOTE**\ : this function is directly available in MainSystem (mbs); it should be directly called as mbs.SolutionViewer(...). For description of the interface, see the MainSystem Python extensions,  :ref:`sec-mainsystemextensions-solutionviewer`\ 



----


.. _sec-interactive-convertimages2video:

Function: ConvertImages2Video
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ConvertImages2Video <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L950>`__\ (\ ``workingDir = 'images'``\ , \ ``inputPattern = 'frame%05d.png'``\ , \ ``outputFile = 'animation.mp4'``\ , \ ``inputFrameRate = 25``\ , \ ``outputFrameRate = 25``\ , \ ``compressionCRF = 28``\ , \ ``startNumber = 0``\ , \ ``totalFrames = None``\ )

- | \ *function description*\ :
  | function to call ffmpeg in the background and convert images to video; requires ffmpeg-python to be installed
- | \ *input*\ :
  | \ ``workingDir``\ : directory where images are stored and where animation is written to
  | \ ``inputPattern``\ : pattern of images; 'frame' is the name used in visualizationSettings.exportImages.saveImageFileName; if saveImageFormat=PNG, then the ending is .png
  | \ ``outputFile``\ : filename and ending (.mp4 recommended) for generated video
  | \ ``inputFrameRate``\ : framerate for images relative to outputFrameRate: if inputFrameRate=50 and outputFrameRate=25, then only every second frame is used
  | \ ``outputFrameRate``\ : framerate for resulting video
  | \ ``compressionCRF``\ : compression rate of ffmpeg, where 0=uncompressed, 25 is medium compression, >30 is very low quality
  | \ ``startNumber``\ : start index of first frame chosen for animation
  | \ ``totalFrames``\ : total number of frames (keep field empty to select all frames after startNumber)
- | \ *output*\ :
  | None; writes animation when finished
- | \ *example*\ :

.. code-block:: python

  #after successful simulation, call:
  mbs.SolutionViewer() #click "Stop", "One Cycle" and "Record frames" => close window
  #if images are in folder 'images', then call this to create animation:
  ConvertImages2Video(workingDir='images', outputFile='test.mp4')




----


.. _sec-interactive-interactiveimages2video:

Function: InteractiveImages2Video
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`InteractiveImages2Video <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L988>`__\ (\ ``closeAfterCreation = False``\ , \ ``fontSize = 11``\ )

- | \ *function description*\ :
  | interactive dialog to convert generated images to videos using ffmpeg library; see also ConvertImages2Video() for meaning of values; requires ffmpeg-python to be installed


.. _sec-module-interactive-class-interactivedialog:

CLASS InteractiveDialog (in module interactive)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    create an interactive dialog, which allows to interact with simulations
    the dialog has a 'Run' button, which initiates the simulation and a 'Stop' button which stops/pauses simulation; 'Quit' closes the simulation model
    for examples, see \ ``simulateInteractively.py``\  and \ ``massSpringFrictionInteractive.py``\ 
    use __init__ method to setup this class with certain buttons, edit boxes and sliders

- | \ *example*\ :

.. code-block:: python

  #the following example is only demonstrating the structure of dialogItems and plots
  #dialogItems structure:
  #general items:
  #    'type' can be out of:
  #               'label' (simple text),
  #               'button' (button with callback function),
  #               'radio' (a radio button with several alternative options),
  #               'slider' (with an adjustable range to choose a value)
  #    'grid': (row, col, colspan) specifies the row, column and (optionally) the span of columns the item is placed at;
  #            exception in 'radio', where grid is a list of (row, col) for every choice
  #    'options': text options, where 'L' means flush left, 'R' means flush right
  #suboptions of 'label':
  #               'text': a text to be drawn
  #suboptions of 'button':
  #               'text': a text to be drawn on button
  #               'callFunction': function which is called on button-press
  #suboptions of 'radio':
  #               'textValueList': [('text1',0),('text2',1)] a list of texts with according values
  #               'value': default value (choice) of radio buttons
  #               'variable': according variable in mbs.variables (or mbs.sys), which is set to current radio button value
  #suboptions of 'slider':
  #               'range': (min, max) a tuple containing minimum and maximum value of slider
  #               'value': default value of slider
  #               'steps': number of steps in slider
  #               'variable': according variable in mbs.variables (or mbs.sys), which is set to current slider value
  #example:
  dialogItems = [{'type':'label', 'text':'Nonlinear oscillation simulator', 'grid':(0,0,2), 'options':['L']},
                 {'type':'button', 'text':'test button','callFunction':ButtonCall, 'grid':(1,0,2)},
                 {'type':'radio', 'textValueList':[('linear',0),('nonlinear',1)], 'value':0, 'variable':'mode', 'grid': [(2,0),(2,1)]},
                 {'type':'label', 'text':'excitation frequency (Hz):', 'grid':(5,0)},
                 {'type':'slider', 'range':(3*f1/800, 3*f1), 'value':omegaInit/(2*pi), 'steps':800, 'variable':'frequency', 'grid':(5,1)},
                 {'type':'label', 'text':'damping:', 'grid':(6,0)},
                 {'type':'slider', 'range': (0, 40), 'value':damper, 'steps':800, 'variable':'damping', 'grid':(6,1)},
                 {'type':'label', 'text':'stiffness:', 'grid':(7,0)},
                 {'type':'slider', 'range':(0, 10000), 'value':spring, 'steps':800, 'variable':'stiffness', 'grid':(7,1)}]
  #plots structure:
  plots={'nPoints':500,              #number of stored points in subplots (higher means slower drawing)
         'subplots':(2,1),           #(rows, columns) arrangement of subplots (for every sensor)
         #sensors defines per subplot (sensor, coordinate), xlabel and ylabel; if coordinate=0, time is used:
         'sensors':[[(sensPos,0),(sensPos,1),'time','mass position'],
                    [(sensFreq,0),(sensFreq,1),'time','excitation frequency']],
         'limitsX':[(0,2),(-5,5)],   #x-range per subplot; if not provided, autoscale is applied
         'limitsY':[(-5,5),(0,10),], #y-range per subplot; if not provided, autoscale is applied
         'fontSize':16,              #custom font size for figure
         'subplots':False,           #if not specified, subplots are created; if False, all plots go into one window
         'lineStyles':['r-','b-'],    #if not specified, uses default '-b', otherwise define list of line styles [string for matplotlib.pyplot.plot] per sensor
         'sizeInches':(12,12)}       #specific x and y size of figure in inches (using 100 dpi)



.. _sec-interactive-interactivedialog---init--:

Class function: __init__
^^^^^^^^^^^^^^^^^^^^^^^^
`__init__ <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L97>`__\ (\ ``self``\ , \ ``mbs``\ , \ ``simulationSettings``\ , \ ``simulationFunction``\ , \ ``dialogItems``\ , \ ``plots = None``\ , \ ``period = 0.04``\ , \ ``realtimeFactor = 1``\ , \ ``userStartSimulation = None``\ , \ ``title = ''``\ , \ ``showTime = False``\ , \ ``fontSize = 12``\ , \ ``doTimeIntegration = True``\ , \ ``runOnStart = False``\ , \ ``addLabelStringVariables = False``\ , \ ``addSliderVariables = False``\ , \ ``checkRenderEngineStopFlag = True``\ , \ ``userOnChange = None``\ , \ ``useSysVariables = False``\ )

- | \ *classFunction*\ :
  | initialize an InteractiveDialog
- | \ *input*\ :
  | \ ``mbs``\ : a multibody system to be simulated
  | \ ``simulationSettings``\ : exudyn.SimulationSettings() according to user settings
  | \ ``simulationFunction``\ : a user function(mbs, self) which is called before a simulation for the short period is started (e.g, assign special values, etc.); the arguments are the MainSystem mbs and the InteractiveDialog (self)
  | \ ``dialogItems``\ : a list of dictionaries, which describe the contents of the interactive items, where every dict has the structure {'type':[label, entry, button, slider, check] ... according to tkinter widgets, 'callFunction': a function to be called, if item is changed/button pressed, 'grid': (row,col) of item to be placed, 'rowSpan': number of rows to be used, 'columnSpan': number of columns to be used; for special item options see notes}
  | \ ``plots``\ : list of dictionaries to specify a sensor to be plotted live, see example; otherwise use default None
  | \ ``period``\ : a simulation time span in seconds which is simulated with the simulationFunction in every iteration
  | \ ``realtimeFactor``\ : if 1, the simulation is nearly performed in realtime (except for computation time); if > 1, it runs faster than realtime, if < 1, than it is slower
  | \ ``userStartSimulation``\ : a function F(flag) which is called every time after Run/Stop is pressed. The argument flag = False if button "Run" has been pressed, flag = True, if "Stop" has been pressed
  | \ ``title``\ : title text for interactive dialog
  | \ ``showTime``\ : shows current time in dialog
  | \ ``fontSize``\ : adjust font size for all dialog items
  | \ ``doTimeIntegration``\ : performs internal time integration with given parameters
  | \ ``runOnStart``\ : immediately activate 'Run' button on start
  | \ ``addLabelStringVariables``\ : True: adds a list labelStringVariables containing the (modifiable) list of string variables for label (text) widgets
  | \ ``addSliderVariables``\ : True: adds a list sliderVariables containing the (modifiable) list of variables for slider (=tkinter scale) widgets; this is not necessarily needed for changing slider values, as they can also be modified with dialog.widgets[..].set(...) method
  | \ ``checkRenderEngineStopFlag``\ : if True, stopping renderer (pressing Q or Escape) also causes stopping the interactive dialog
  | \ ``userOnChange``\ : a user function(mbs, self) which is called after period, if widget values are different from values stored in mbs.variables; this usually occurs if buttons are pressed or sliders are moved; the arguments are the MainSystem mbs and the InteractiveDialog (self)
  | \ ``useSysVariables``\ : for internal visualization functions: in this case, variables are written to mbs.sys instead of mbs.variables
- | \ *notes*\ :
  | detailed description of dialogItems and plots list/dictionary is given in commented the example below

----

.. _sec-interactive-interactivedialog-onquit:

Class function: OnQuit
^^^^^^^^^^^^^^^^^^^^^^
`OnQuit <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L350>`__\ (\ ``self``\ , \ ``event = None``\ )

- | \ *classFunction*\ :
  | function called when pressing escape or closing dialog

----

.. _sec-interactive-interactivedialog-startsimulation:

Class function: StartSimulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`StartSimulation <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L359>`__\ (\ ``self``\ , \ ``event = None``\ )

- | \ *classFunction*\ :
  | function called on button 'Run'

----

.. _sec-interactive-interactivedialog-processwidgetstates:

Class function: ProcessWidgetStates
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ProcessWidgetStates <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L372>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | assign current values of radio buttons and sliders to mbs.variables or mbs.sys

----

.. _sec-interactive-interactivedialog-continuousrunfunction:

Class function: ContinuousRunFunction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ContinuousRunFunction <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L387>`__\ (\ ``self``\ , \ ``event = None``\ )

- | \ *classFunction*\ :
  | function which is repeatedly called when button 'Run' is pressed

----

.. _sec-interactive-interactivedialog-initializeplots:

Class function: InitializePlots
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`InitializePlots <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L405>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | initialize figure and subplots for plots structure

----

.. _sec-interactive-interactivedialog-updateplots:

Class function: UpdatePlots
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`UpdatePlots <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L452>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | update all subplots with current sensor values

----

.. _sec-interactive-interactivedialog-initializesolver:

Class function: InitializeSolver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`InitializeSolver <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L508>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | function to initialize solver for repeated calls

----

.. _sec-interactive-interactivedialog-finalizesolver:

Class function: FinalizeSolver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`FinalizeSolver <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L514>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | stop solver (finalize correctly)

----

.. _sec-interactive-interactivedialog-runsimulationperiod:

Class function: RunSimulationPeriod
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`RunSimulationPeriod <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/interactive.py\#L520>`__\ (\ ``self``\ )

- | \ *classFunction*\ :
  | function which performs short simulation for given period

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `massSpringFrictionInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/massSpringFrictionInteractive.py>`_\  (Ex), \ `nMassOscillatorInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorInteractive.py>`_\  (Ex), \ `serialRobotInteractiveLimits.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInteractiveLimits.py>`_\  (Ex), \ `simulateInteractively.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/simulateInteractively.py>`_\  (Ex), \ `runTestExamples.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/runTestExamples.py>`_\  (TM)

