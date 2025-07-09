Mass-Spring-Damper tutorial
===========================

The Python source code of the first tutorial can be found in the file:

   \ ``main/pythonDev/Examples/springDamperTutorial.py``\ 

A similar version based on a simplified approach (using a 3D mass point) is available as, which uses simplified approaches:

   \ ``main/pythonDev/Examples/springDamperTutorialNew.py``\ 

The following tutorial will set up a mass point and a spring damper, dynamically compute the solution and evaluate the reference solution.


We import the exudyn library and the interface for all nodes, objects, markers, loads and sensors:

.. code-block:: python

  import exudyn as exu
  from exudyn.utilities import Point, NodePointGround, MassPoint, MarkerNodeCoordinate,\
                               CoordinateSpringDamper, LoadCoordinate, SensorObject
  import exudyn.graphics as graphics #only import if it does not conflict
  import numpy as np #for postprocessing


Instead of the named import of \ ``exudyn.utilities``\  functions and classes, you can use a star import,
which includes \ ``itemInterface``\ , \ ``rigidBodyUtilities``\  and some helper functions:

.. code-block:: python

  from exudyn.utilities import *


Next, we need a \ ``SystemContainer``\ , which contains all computable systems and add a new MainSystem \ ``mbs``\ .
Per default, you always should name your system 'mbs' (multibody system), in order to copy/paste code parts from other examples, tutorials and other projects:

.. code-block:: python

  SC = exu.SystemContainer()
  mbs = SC.AddSystem()


In order to check, which version you are using, you can printout the current Exudyn version. 
The version shown is in line with the issue tracker and marks the number of open/closed issues added to Exudyn .
Adding \ ``True``\  as argument will also print platform-specific information, which is helpful 
in case of reporting some compatibility issues:

.. code-block:: python

  print('EXUDYN version='+exu.config.Version(True))


Using the powerful Python language, we can define some variables for our problem, which will also be used for the analytical solution:

.. code-block:: python

  L=0.5               #reference position of mass
  mass = 1.6          #mass in kg
  spring = 4000       #stiffness of spring-damper in N/m
  damper = 8          #damping constant in N/(m/s)
  f =80               #force on mass


For the simple spring-mass-damper system, we need initial displacements and velocities:

.. code-block:: python

  u0=-0.08            #initial displacement
  v0=1                #initial velocity
  x0=f/spring         #static displacement
  print('resonance frequency = '+str(np.sqrt(spring/mass)))
  print('static displacement = '+str(x0))


We first need to add nodes, which provide the coordinates (and the degrees of freedom) to the system.
The following line adds a 3D node for 3D mass point\ (Note: Point is an abbreviation for NodePoint, defined in \ ``itemInterface.py``\ .):

.. code-block:: python

  n1=mbs.AddNode(Point(referenceCoordinates = [L,0,0], 
                       initialCoordinates = [u0,0,0], 
                       initialVelocities = [v0,0,0]))


Here, \ ``Point``\  (=\ ``NodePoint``\ ) is a Python class, which takes a number of arguments defined in the reference manual. The arguments here are \ ``referenceCoordinates``\ , which are the coordinates for which the system is defined. The initial configuration is given by \ ``referenceCoordinates + initialCoordinates``\ , while the initial state additionally gets \ ``initialVelocities``\ .
The command \ ``mbs.AddNode(...)``\  returns a \ ``NodeIndex n1``\ , which basically contains an integer, which can only be used as node number. This node number will be used lateron to use the node in the object or in the marker.

While \ ``Point``\  adds 3 unknown coordinates to the system, which need to be solved, we also can add ground nodes, which can be used similar to nodes, but they do not have unknown coordinates -- and therefore also have no initial displacements or velocities. The advantage of ground nodes (and ground bodies) is that no constraints are needed to fix these nodes.
Such a ground node is added via:

.. code-block:: python

  nGround=mbs.AddNode(NodePointGround(referenceCoordinates = [0,0,0]))


In the next step, we add an object\ (For the moment, we just need to know that objects either depend on one or more nodes, which are usually bodies and finite elements, or they can be connectors, which connect (the coordinates of) objects via markers, see Section :ref:`sec-overview-modulestructure`\ .), which provides equations for coordinates. The \ ``MassPoint``\  needs at least a mass (kg) and a node number to which the mass point is attached. Additionally, graphical objects could be attached:

.. code-block:: python

  massPoint = mbs.AddObject(MassPoint(physicsMass = mass, nodeNumber = n1))


Note that instead of adding a \ ``NodePoint``\  and a \ ``MassPoint``\  with \ ``mbs.AddNode(...)``\ 
and \ ``mbs.AddObject(...)``\ , there is also a convenient function \ ``mbs.CreateMassPoint(...)``\ , which can do everything at once including the option to add gravity.

In order to apply constraints and loads, we need markers. These markers are used as local positions (and frames), where we can attach a constraint lateron. In this example, we work on the coordinate level, both for forces as well as for constraints.
Markers are attached to the according ground and regular node number, additionally using a coordinate number (0 ... first coordinate):

.. code-block:: python

  groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, 
                                                  coordinate = 0))
  #marker for springDamper for first (x-)coordinate:
  nodeMarker = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= n1, 
                                                  coordinate = 0))


This means that constraints are be applied to the first coordinate of node \ ``n1``\  via marker with number \ ``nodeMarker``\ , which is in fact of type \ ``MarkerNodeCoordinate``\ .

Now we add a spring-damper to the markers with numbers \ ``groundMarker``\  and the \ ``nodeMarker``\ , providing stiffness and damping parameters:

.. code-block:: python

  nC = mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker], 
                                       stiffness = spring, 
                                       damping = damper)) 


A load is added to marker \ ``nodeMarker``\ , with a scalar load with value \ ``f``\ :

.. code-block:: python

  nLoad = mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, 
                                     load = f))


Again, instead of adding a \ ``MarkerNodeCoordinate``\  and a \ ``LoadCoordinate``\  with \ ``mbs.AddLoad(...)``\ ,
we could just use \ ``mbs.CreateForce(...)``\  to add a 3D force vector.
For specific joints, there are also \ ``mbs.Create...(...)``\  functions.

Finally, a sensor is added to the coordinate constraint object with number \ ``nC``\ , requesting the \ ``outputVariableType``\  \ ``Force``\ :

.. code-block:: python

  mbs.AddSensor(SensorObject(objectNumber=nC, fileName='groundForce.txt', 
                             outputVariableType=exu.OutputVariableType.Force))


Note that sensors can be attached, e.g., to nodes, bodies, objects (constraints) or loads.
As our system is fully set, we can print the overall information and assemble the system to make it ready for simulation:

.. code-block:: python

  print(mbs)     #show system properties
  mbs.Assemble() #prepare for simulation


We will use time integration and therefore define a number of steps (fixed step size; must be provided) and the total time span for the simulation:

.. code-block:: python

  tEnd = 1     #end time of simulation
  h = 0.001    #step size; leads to 1000 steps


All settings for simulation, see according reference section, can be provided in a structure given from \ ``exu.SimulationSettings()``\ . Note that this structure will contain all default values, and only non-default values need to be provided:

.. code-block:: python

  simulationSettings = exu.SimulationSettings()
  simulationSettings.solutionSettings.solutionWritePeriod = 5e-3 #output interval general
  simulationSettings.solutionSettings.sensorsWritePeriod = 5e-3  #output interval of sensors
  simulationSettings.timeIntegration.numberOfSteps = tEnd/h
  simulationSettings.timeIntegration.endTime = tEnd
  simulationSettings.displayComputationTime = True               #show how fast


In order to see some solver output, we must set \ ``verboseMode``\  to 1 (higher values gives detailed output per step).
Furthermore, we can show information on computation time (which may cost some overhead in computation!):

.. code-block:: python

  simulationSettings.timeIntegration.verboseMode = 1             #show some solver output
  simulationSettings.displayComputationTime = True               #show how fast


We are using a generalized alpha solver, where numerical damping is needed for index 3 constraints. As we have only spring-dampers, we can set the spectral radius to 1, meaning no numerical damping:

.. code-block:: python

  simulationSettings.timeIntegration.generalizedAlpha.spectralRadius = 1


In order to visualize the results online, a renderer can be started. As our computation will be very fast, it is a good idea to wait for the user to press SPACE, before starting the simulation (uncomment second line):

.. code-block:: python

  SC.renderer.Start()              #start graphics visualization
  #SC.renderer.DoIdleTasks()       #wait for SPACE bar or 'Q' to continue (in render window!)


As the simulation is still very fast, we will not see the motion of our node. Using a very small step size of, e.g., \ ``h=1e-7``\  in the lines above allows us to visualize the resulting oscillations in realtime.

Finally, we start the solver, by telling which system to be solved, solver type and the simulation settings:

.. code-block:: python

  exu.SolveDynamic(mbs, simulationSettings)



After simulation, our renderer needs to be stopped (otherwise it will stop unsafely as soon as the Python kernel is stopped or restarted). 
Sometimes you would like to wait until closing the render window, using \ ``WaitForRenderEngineStopFlag()``\ :

.. code-block:: python

  #SC.renderer.DoIdleTasks()       #wait for pressing 'Q' to quit
  SC.renderer.Stop()               #safely close rendering window!


If you run this code, e.g. in Spyder or Visual Studio Code, it may take a 1-2 seconds to complete. However, the time spent is only related to some overhead in the Python environment and for the visualization. The simulation itself will only take around 3-10 milliseconds, in which a large overhead is due to file writing.

There are several ways to evaluate results, see the reference pages. In the following we take the final value of node \ ``n1``\  and read its 3D position vector:

.. code-block:: python

  #evaluate final (=current) output values
  u = mbs.GetNodeOutput(n1, exu.OutputVariableType.Position)
  print('displacement=',u)


The following code generates a reference (exact) solution for our example:

.. code-block:: python

  import matplotlib.pyplot as plt
  import matplotlib.ticker as ticker

  omega0 = np.sqrt(spring/mass)          #eigen frequency of undamped system
  dRel = damper/(2*np.sqrt(spring*mass)) #dimensionless damping
  omega = omega0*np.sqrt(1-dRel**2)      #eigen freq of damped system
  C1 = u0-x0 #static solution needs to be considered!
  C2 = (v0+omega0*dRel*C1) / omega       #C1, C2 are coeffs for solution
  steps = int(tEnd/h)                    #use same steps for reference solution

  refSol = np.zeros((steps+1,2))
  for i in range(0,steps+1):
    t = tEnd*i/steps
    refSol[i,0] = t
    refSol[i,1] = np.exp(-omega0*dRel*t)*(C1*np.cos(omega*t)+C2*np.sin(omega*t))+x0

  plt.plot(refSol[:,0], refSol[:,1], 'r-', label='displacement (m); exact solution')


Now we can load our results from the default solution file \ ``coordinatesSolution.txt``\ , which is in the same
directory as your Python tutorial file. 
\ **Note**\  that the visualization of results can be simplified considerably using the \ ``PlotSensor(...)``\  utility function as shown in the \ **Rigid body and joints tutorial**\ !

For reading the file containing commented lines (this does not work in binary mode!), we use a numpy feature and finally plot the displacement of coordinate 0 or our mass point\ (\ ``data[:,0]``\  contains the simulation time, \ ``data[:,1]``\  contains displacement of (global) coordinate 0, \ ``data[:,2]``\  contains displacement of (global) coordinate 1, ...)):

.. code-block:: python

  data = np.loadtxt('coordinatesSolution.txt', comments='#', delimiter=',')
  plt.plot(data[:,0], data[:,1], 'b-', label='displacement (m); numerical solution') 


Note that the coordinates do not include the reference position (which is 0.5 in this case). For information on displacement and reference coordinates, see Section :ref:`sec-overview-items-coordinates`\ .

The sensor result can be loaded in the same way. The sensor output format contains time in the first column and sensor values in the remaining columns. The number of columns depends on the 
sensor and the output quantity (scalar, vector, ...):

.. code-block:: python

  data = np.loadtxt('groundForce.txt', comments='#', delimiter=',')
  plt.plot(data[:,0], data[:,1]*1e-3, 'g-', label='force (kN)')


In order to get a nice plot within Spyder, the following options can be used\ (note, in some environments you need finally the command \ ``plt.show()``\ ):

.. code-block:: python

  ax=plt.gca() # get current axes
  ax.grid(True, 'major', 'both')
  ax.xaxis.set_major_locator(ticker.MaxNLocator(10))
  ax.yaxis.set_major_locator(ticker.MaxNLocator(10))
  plt.legend() #show labels as legend
  plt.tight_layout()
  plt.show() 


The matplotlib output should look as shown in \ :numref:`fig-tutorial-springdamper`\ .


.. _fig-tutorial-springdamper:
.. figure:: ../theDoc/figures/plotSpringDamper.png
   :width: 400

   Output of spring-damper tutorial.






