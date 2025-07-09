

.. _sec-item-objectconnectorcoordinate:

ObjectConnectorCoordinate
=========================

A coordinate constraint which constrains two (scalar) coordinates of Marker[Node|Body]Coordinates attached to nodes or bodies. The constraint acts directly on coordinates, but does not include reference values, e.g., of nodal values. This constraint is computationally efficient and should be used to constrain nodal coordinates.

\ **Additional information for ObjectConnectorCoordinate**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested \ ``Marker``\  type = \ ``Coordinate``\ 
* | \ **Short name**\  for Python = \ ``CoordinateConstraint``\ 
* | \ **Short name**\  for Python visualization object = \ ``VCoordinateConstraint``\ 


The item \ **ObjectConnectorCoordinate**\  with type = 'ConnectorCoordinate' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **offset** [\ :math:`l_\mathrm{off}`\ , type = Real, default = 0.]:
  | An offset between the two values
* | **factorValue1** [\ :math:`k_{m1}`\ , type = Real, default = 1.]:
  | An additional factor multiplied with value1 used in algebraic equation
* | **velocityLevel** [type = Bool, default = False]:
  | If true: connector constrains velocities (only works for \ :ref:`ODE2 <ODE2>`\  coordinates!); offset is used between velocities; in this case, the offsetUserFunction_t is considered and offsetUserFunction is ignored
* | **offsetUserFunction** [\ :math:`\mathrm{UF} \in \Rcal`\ , type = PyFunctionMbsScalarIndexScalar, default =  0]:
  | A Python function which defines the time-dependent offset; see description below
* | **offsetUserFunction_t** [\ :math:`\mathrm{UF}_t \in \Rcal`\ , type = PyFunctionMbsScalarIndexScalar, default =  0]:
  | time derivative of offsetUserFunction; needed for velocity level constraints; see description below
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectConnectorCoordinate]:
  | parameters for visualization of item



The item VObjectConnectorCoordinate has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = link size; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectconnectorcoordinate:

DESCRIPTION of ObjectConnectorCoordinate
----------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Displacement``\ : \ :math:`\Delta q`\ 
  | relative scalar displacement of marker coordinates, not including factorValue1
* | ``Velocity``\ : \ :math:`\Delta v`\ 
  | difference of scalar marker velocity coordinates, not including factorValue1
* | ``ConstraintEquation``\ : \ :math:`{\mathbf{c}}`\ 
  | (residuum of) constraint equation
* | ``Force``\ : \ :math:`\lambda_0`\ 
  | scalar constraint force (Lagrange multiplier)



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
   * - | difference of coordinates
     - | \ :math:`\Delta q = q_{m1} - q_{m0}`\ 
     - | Displacement between marker m0 to marker m1 coordinates (does NOT include reference coordinates)
   * - | difference of velocity coordinates
     - | \ :math:`\Delta v= v_{m1} - v_{m0}`\ 
     - | 


Connector constraint equations
------------------------------

If \ ``activeConnector = True``\ , the index 3 algebraic equation reads

.. math::

   {\mathbf{c}}(q_{m0}, q_{m1}) = k_{m1} \cdot q_{m1} - q_{m0} - l_\mathrm{off} = 0


If the offsetUserFunction \ :math:`\mathrm{UF}`\  is defined, \ :math:`{\mathbf{c}}`\  instead becomes (\ :math:`t`\  is current time)

.. math::

   {\mathbf{c}}(q_{m0}, q_{m1}) = k_{m1} \cdot q_{m1} - q_{m0} -  \mathrm{UF}(mbs, t, i_N, l_\mathrm{off}) = 0


The \ ``activeConnector = True``\ , index 2 (velocity level) algebraic equation reads

.. math::

   \dot {\mathbf{c}}(\dot q_{m0}, \dot q_{m1}) = k_{m1} \cdot \dot q_{m1} - \dot q_{m0} - d = 0


The factor \ :math:`d`\  in velocity level equations is zero, except if parameters.velocityLevel = True, then \ :math:`d=l_\mathrm{off}`\ .
If velocity level constraints are active and the velocity level offsetUserFunction_t \ :math:`\mathrm{UF}_t`\  is defined, \ :math:`\dot {\mathbf{c}}`\  instead becomes (\ :math:`t`\  is current time)

.. math::

   \dot {\mathbf{c}}(\dot q_{m0}, \dot q_{m1}) = k_{m1} \cdot \dot q_{m1} - \dot q_{m0} - \mathrm{UF}_t(mbs, t, i_N, l_\mathrm{off}) = 0


and \ ``iN``\  represents the itemNumber (=objectNumber).
Note that the index 2 equations are used, if the solver uses index 2 formulation OR if the flag parameters.velocityLevel = True (or both).
The user functions include dependency on time \ :math:`t`\ , but this time dependency is not respected in the computation of initial accelerations. Therefore,
it is recommended that \ :math:`\mathrm{UF}`\  and \ :math:`\mathrm{UF}_t`\  does not include initial accelerations.

If \ ``activeConnector = False``\ , the (index 1) algebraic equation reads for ALL cases:

.. math::

   {\mathbf{c}}(\lambda_0) = \lambda_0 = 0



--------

\ **Userfunction**\ : ``offsetUserFunction(mbs, t, itemNumber, lOffset)`` 


A user function, which computes scalar offset for the coordinate constraint, e.g., in order to move a node on a prescribed trajectory.
It is NECESSARY to use sufficiently smooth functions, having \ **initial offsets**\  consistent with \ **initial configuration**\  of bodies, 
either zero or compatible initial offset-velocity, and no initial accelerations.
The \ ``offsetUserFunction``\  is \ **ONLY used**\  in case of static computation or index3 (generalizedAlpha) time integration.
In order to be on the safe side, provide both  \ ``offsetUserFunction``\  and  \ ``offsetUserFunction_t``\ .

Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
\ ``mbs.GetObjectParameter(itemNumber, ...)``\ , see the according description of \ ``GetObjectParameter``\ .

The user function gets time and the offset parameter as an input and returns the computed offset:

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
   * - | \ ``lOffset``\ 
     - | Real
     - | \ :math:`l_\mathrm{off}`\ 
   * - | \returnValue
     - | Real
     - | computed offset for given time


--------

\ **Userfunction**\ : ``offsetUserFunction_t(mbs, t, itemNumber, lOffset)`` 


A user function, which computes scalar offset \ **velocity**\  for the coordinate constraint.
It is NECESSARY to use sufficiently smooth functions, having \ **initial offset velocities**\  consistent with \ **initial velocities**\  of bodies.
The \ ``offsetUserFunction_t``\  is used instead of \ ``offsetUserFunction``\  in case of \ ``velocityLevel = True``\ , 
or for index2 time integration and needed for computation of initial accelerations in second order implicit time integrators.

Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
\ ``mbs.GetObjectParameter(itemNumber, ...)``\ , see the according description of \ ``GetObjectParameter``\ .

The user function gets time and the offset parameter as an input and returns the computed offset velocity:

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
     - | integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``lOffset``\ 
     - | Real
     - | \ :math:`l_\mathrm{off}`\ 
   * - | \returnValue
     - | Real
     - | computed offset velocity for given time


--------

\ **User function example**\ :



.. code-block:: python

    #see also mini example!
    from math import sin, cos, pi
    def UFoffset(mbs, t, itemNumber, lOffset): 
        return 0.5*lOffset*(1-cos(0.5*pi*t))
    
    def UFoffset_t(mbs, t, itemNumber, lOffset): #time derivative of UFoffset
        return 0.5*lOffset*0.5*pi*sin(0.5*pi*t)

    nMass=mbs.AddNode(Point(referenceCoordinates = [2,0,0]))
    massPoint = mbs.AddObject(MassPoint(physicsMass = 5, nodeNumber = nMass))
    
    groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
    nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nMass, coordinate = 0))
    
    #Spring-Damper between two marker coordinates
    mbs.AddObject(CoordinateConstraint(markerNumbers = [groundMarker, nodeMarker], 
                                       offset = 0.1, 
                                       offsetUserFunction = UFoffset, 
                                       offsetUserFunction_t = UFoffset_t)) 





.. _miniexample-objectconnectorcoordinate:

MINI EXAMPLE for ObjectConnectorCoordinate
------------------------------------------


.. code-block:: python
   :linenos:

   def OffsetUF(mbs, t, itemNumber, lOffset): #gives 0.05 at t=1
       return 0.5*(1-np.cos(2*3.141592653589793*0.25*t))*lOffset
   
   nMass=mbs.AddNode(Point(referenceCoordinates = [2,0,0]))
   massPoint = mbs.AddObject(MassPoint(physicsMass = 5, nodeNumber = nMass))
   
   groundMarker=mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nGround, coordinate = 0))
   nodeMarker  =mbs.AddMarker(MarkerNodeCoordinate(nodeNumber= nMass, coordinate = 0))
   
   #Spring-Damper between two marker coordinates
   mbs.AddObject(CoordinateConstraint(markerNumbers = [groundMarker, nodeMarker], 
                                      offset = 0.1, offsetUserFunction = OffsetUF)) 
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveDynamic()
   
   #check result at default integration time
   exudynTestGlobals.testResult  = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Displacement)[0]

Relevant Examples and TestModels with weblink:

    \ `NGsolveModalAnalysis.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveModalAnalysis.py>`_\  (Examples/), \ `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_\  (Examples/), \ `ballBearningModel.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ballBearningModel.py>`_\  (Examples/), \ `camFollowerExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/camFollowerExample.py>`_\  (Examples/), \ `involuteGearGraphics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/involuteGearGraphics.py>`_\  (Examples/), \ `ALEANCFpipe.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ALEANCFpipe.py>`_\  (Examples/), \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Examples/), \ `ANCFcantileverTestDyn.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcantileverTestDyn.py>`_\  (Examples/), \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFmovingRigidbody.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFmovingRigidbody.py>`_\  (Examples/), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Examples/), \ `ballBearingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ballBearingTest.py>`_\  (TestModels/), \ `contactCurveExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/contactCurveExample.py>`_\  (TestModels/), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


