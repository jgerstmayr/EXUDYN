

.. _sec-item-objectconnectorcoordinatespringdamper:

ObjectConnectorCoordinateSpringDamper
=====================================

A 1D (scalar) spring-damper element acting on single \ :ref:`ODE2 <ODE2>`\  coordinates; connects to coordinate-based markers; NOTE that the coordinate markers only measure the coordinate (=displacement), but the reference position is not included as compared to position-based markers!; the spring-damper can also act on rotational coordinates.

\ **Additional information for ObjectConnectorCoordinateSpringDamper**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``Coordinate``\ 
* | \ **Short name**\  for Python = \ ``CoordinateSpringDamper``\ 
* | \ **Short name**\  for Python visualization object = \ ``VCoordinateSpringDamper``\ 


The item \ **ObjectConnectorCoordinateSpringDamper**\  with type = 'ConnectorCoordinateSpringDamper' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **stiffness** [\ :math:`k`\ , type = Real, default = 0.]:
  | stiffness [SI:N/m] of spring; acts against relative value of coordinates
* | **damping** [\ :math:`d`\ , type = Real, default = 0.]:
  | damping [SI:N/(m s)] of damper; acts against relative velocity of coordinates
* | **offset** [\ :math:`l_\mathrm{off}`\ , type = Real, default = 0.]:
  | offset between two coordinates (reference length of springs), see equation
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **springForceUserFunction** [\ :math:`\mathrm{UF} \in \Rcal`\ , type = PyFunctionMbsScalarIndexScalar5, default =  0]:
  | A Python function which defines the spring force with 8 parameters, see equations section / see description below
* | **visualization** [type = VObjectConnectorCoordinateSpringDamper]:
  | parameters for visualization of item



The item VObjectConnectorCoordinateSpringDamper has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectconnectorcoordinatespringdamper:

DESCRIPTION of ObjectConnectorCoordinateSpringDamper
----------------------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Displacement``\ : \ :math:`\Delta q`\ 
  | relative scalar displacement of marker coordinates
* | ``Velocity``\ : \ :math:`\Delta v`\ 
  | difference of scalar marker velocity coordinates
* | ``Force``\ : \ :math:`f_{SD}`\ 
  | scalar force in connector



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | marker m0 coordinate
     - | \ :math:`q_{m0}`\ 
     - | current displacement coordinate which is provided by marker m0; does NOT include reference coordinate!
   * - | marker m1 coordinate
     - | \ :math:`q_{m1}`\ 
     - | 
   * - | marker m0 velocity coordinate
     - | \ :math:`v_{m0}`\ 
     - | current velocity coordinate which is provided by marker m0
   * - | marker m1 velocity coordinate
     - | \ :math:`v_{m1}`\ 
     - | 


Connector forces
----------------

Displacement between marker m0 to marker m1 coordinates (does NOT include reference coordinates),

.. math::

   \Delta q= q_{m1} - q_{m0}


and relative velocity,

.. math::

   \Delta v= v_{m1} - v_{m0}


If \ ``activeConnector = True``\ , the scalar spring force vector is computed as

.. math::

   f_{SD} = k \left( \Delta q - l_\mathrm{off} \right) + d \cdot \Delta v


If the springForceUserFunction \ :math:`\mathrm{UF}`\  is defined, \ :math:`{\mathbf{f}}_{SD}`\  instead becomes (\ :math:`t`\  is current time)

.. math::

   f_{SD} = \mathrm{UF}(mbs, t, i_N, \Delta q, \Delta v, k, d, l_\mathrm{off})


and \ ``iN``\  represents the itemNumber (=objectNumber).

If \ ``activeConnector = False``\ , \ :math:`f_{SD}`\  is set to zero.

NOTE  that until 2023-01-21 (exudyn V1.5.76), the CoordinateSpringDamper included the parameters dryFriction and dryFrictionProportionalZone.
These parameters have been removed and they are only available in CoordinateSpringDamperExt, HOWEVER, with different names.
In order to use CoordinateSpringDamperExt instead of the old CoordinateSpringDamper with the same friction behavior, we recoomend:

+  USE CoordinateSpringDamperExt.fDynamicFriction INSTEAD of CoordinateSpringDamper.dryFriction
+  USE CoordinateSpringDamperExt.frictionProportionalZone INSTEAD of CoordinateSpringDamper.dryFrictionProportionalZone
+  CoordinateSpringDamperExt.frictionProportionalZone has a different behavior in case that it is zero; 
        thus use 1e-16 in this case, to get as close as possible to previous behaviour
+  the variables stiffness, damping and offset have the same interpretation in both objects
+  keep every other friction, sticking and contact variables in CoordinateSpringDamperExt as default values
+  user functions obtained a new interface in CoordinateSpringDamperExt, which just needs to be adapted


--------

\ **Userfunction**\ : ``springForceUserFunction(mbs, t, itemNumber, displacement, velocity, stiffness, damping, offset, dryFriction, dryFrictionProportionalZone)`` 


A user function, which computes the scalar spring force depending on time, object variables (displacement, velocity) 
and object parameters .
The object variables are passed to the function using the current values of the CoordinateSpringDamper object.
Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
\ ``mbs.GetObjectParameter(itemNumber, ...)``\ , see the according description of \ ``GetObjectParameter``\ .

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments / return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs in which underlying item is defined
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs 
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number \ :math:`i_N`\  of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``displacement``\ 
     - | Real
     - | \ :math:`\Delta q`\ 
   * - | \ ``velocity``\ 
     - | Real
     - | \ :math:`\Delta v`\ 
   * - | \ ``stiffness``\ 
     - | Real
     - | copied from object
   * - | \ ``damping``\ 
     - | Real
     - | copied from object
   * - | \ ``offset``\ 
     - | Real
     - | copied from object
   * - | \returnValue
     - | Real
     - | scalar value of computed force


--------

\ **User function example**\ :



.. code-block:: python

    #see also mini example! NOTE changes above since 2023-01-23
    def UFforce(mbs, t, itemNumber, u, v, k, d, offset):
        return k*(u-offset) + d*v





.. _miniexample-objectconnectorcoordinatespringdamper:

MINI EXAMPLE for ObjectConnectorCoordinateSpringDamper
------------------------------------------------------


.. code-block:: python
   :linenos:

   #define user function:
   #NOTE: removed 2023-01-21: dryFriction, dryFrictionProportionalZone
   def springForce(mbs, t, itemNumber, u, v, k, d, offset): 
       return 0.1*k*u+k*u**3+v*d
   
   nMass=mbs.AddNode(Point(referenceCoordinates = [2,0,0]))
   massPoint = mbs.AddObject(MassPoint(physicsMass = 5, nodeNumber = nMass))
   
   groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
   nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nMass, coordinate = 0))
   
   #Spring-Damper between two marker coordinates
   mbs.AddObject(CoordinateSpringDamper(markerNumbers = [groundMarker, nodeMarker], 
                                        stiffness = 5000, damping = 80, 
                                        springForceUserFunction = springForce)) 
   loadCoord = mbs.AddLoad(LoadCoordinate(markerNumber = nodeMarker, load = 1)) #static linear solution:0.002
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveDynamic()
   
   #check result at default integration time
   exudynTestGlobals.testResult = mbs.GetNodeOutput(nMass, 
                                                exu.OutputVariableType.Displacement)[0]

Relevant Examples and TestModels with weblink:

    \ `slidercrankWithMassSpring.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/slidercrankWithMassSpring.py>`_\  (Examples/), \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Examples/), \ `ANCFswitchingSlidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFswitchingSlidingJoint2D.py>`_\  (Examples/), \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Examples/), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Examples/), \ `ComputeSensitivitiesExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ComputeSensitivitiesExample.py>`_\  (Examples/), \ `coordinateSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/coordinateSpringDamper.py>`_\  (Examples/), \ `finiteSegmentMethod.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/finiteSegmentMethod.py>`_\  (Examples/), \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Examples/), \ `involuteGearGraphics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/involuteGearGraphics.py>`_\  (Examples/), \ `lugreFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/lugreFrictionTest.py>`_\  (Examples/), \ `massSpringFrictionInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/massSpringFrictionInteractive.py>`_\  (Examples/), \ `scissorPrismaticRevolute2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/scissorPrismaticRevolute2D.py>`_\  (TestModels/), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TestModels/), \ `ANCFcontactFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactFrictionTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


