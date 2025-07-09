

.. _sec-item-objectconnectordistance:

ObjectConnectorDistance
=======================

Connector which enforces constant or prescribed distance between two bodies/nodes.

\ **Additional information for ObjectConnectorDistance**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``DistanceConstraint``\ 
* | \ **Short name**\  for Python visualization object = \ ``VDistanceConstraint``\ 


The item \ **ObjectConnectorDistance**\  with type = 'ConnectorDistance' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **distance** [\ :math:`d_0`\ , type = PReal, default = 0.]:
  | prescribed distance [SI:m] of the used markers; must by greater than zero
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectConnectorDistance]:
  | parameters for visualization of item



The item VObjectConnectorDistance has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = link size; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectconnectordistance:

DESCRIPTION of ObjectConnectorDistance
--------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Displacement``\ : \ :math:`\LU{0}{\Delta{\mathbf{p}}}`\ 
  | relative displacement in global coordinates
* | ``Velocity``\ : \ :math:`\LU{0}{\Delta{\mathbf{v}}}`\ 
  | relative translational velocity in global coordinates
* | ``Distance``\ : \ :math:`|\LU{0}{\Delta{\mathbf{p}}}|`\ 
  | distance between markers (should stay constant; shows constraint deviation)
* | ``Force``\ : \ :math:`\lambda_0`\ 
  | joint force (=scalar Lagrange multiplier)



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | marker m0 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
     - | current global position which is provided by marker m0
   * - | marker m1 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1}`\ 
     - | accordingly
   * - | marker m0 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity which is provided by marker m0
   * - | marker m1 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1}`\ 
     - | accordingly
   * - | relative displacement
     - | \ :math:`\LU{0}{\Delta{\mathbf{p}}}`\ 
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1} - \LU{0}{{\mathbf{p}}}_{m0}`\ 
   * - | relative velocity
     - | \ :math:`\LU{0}{\Delta{\mathbf{v}}}`\ 
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1} - \LU{0}{{\mathbf{v}}}_{m0}`\ 
   * - | algebraicVariable
     - | \ :math:`\lambda_0`\ 
     - | Lagrange multiplier = force in constraint



Connector forces constraint equations
-------------------------------------

If \ ``activeConnector = True``\ , the index 3 algebraic equation reads

.. math::

   \left|\LU{0}{\Delta{\mathbf{p}}}\right| - d_0 = 0


Due to the fact that the force direction is given by

.. math::

   \frac{1}{|\LU{0}{\Delta{\mathbf{p}}}|}\LU{0}{\Delta{\mathbf{p}}} ,


the prescribed distance \ :math:`d_0`\  may not be zero. This would, otherwise, result in a change of the number of constraints.
The index 2 (velocity level) algebraic equation reads

.. math::

   \left(\frac{\LU{0}{\Delta{\mathbf{p}}}}{\left|\LU{0}{\Delta{\mathbf{p}}}\right|}\right)\tp \Delta{\mathbf{v}} = 0


if \ ``activeConnector = False``\ , the algebraic equation reads

.. math::

   \lambda_0 = 0





.. _miniexample-objectconnectordistance:

MINI EXAMPLE for ObjectConnectorDistance
----------------------------------------


.. code-block:: python
   :linenos:

   #example with 1m pendulum, 50kg under gravity
   nMass = mbs.AddNode(NodePoint2D(referenceCoordinates=[1,0]))
   oMass = mbs.AddObject(MassPoint2D(physicsMass = 50, nodeNumber = nMass))
   
   mMass = mbs.AddMarker(MarkerNodePosition(nodeNumber=nMass))
   mGround = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition = [0,0,0]))
   oDistance = mbs.AddObject(DistanceConstraint(markerNumbers = [mGround, mMass], distance = 1))
   
   mbs.AddLoad(Force(markerNumber = mMass, loadVector = [0, -50*9.81, 0])) 
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   
   sims=exu.SimulationSettings()
   sims.timeIntegration.generalizedAlpha.spectralRadius=0.7
   mbs.SolveDynamic(sims)
   
   #check result at default integration time
   exudynTestGlobals.testResult = mbs.GetNodeOutput(nMass, exu.OutputVariableType.Position)[0]

Relevant Examples and TestModels with weblink:

    \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Examples/), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Examples/), \ `newtonsCradle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/newtonsCradle.py>`_\  (Examples/), \ `HydraulicActuatorStaticInitialization.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/HydraulicActuatorStaticInitialization.py>`_\  (Examples/), \ `pendulum2Dconstraint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulum2Dconstraint.py>`_\  (Examples/), \ `pendulumIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulumIftommBenchmark.py>`_\  (Examples/), \ `fourBarMechanismTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/fourBarMechanismTest.py>`_\  (TestModels/), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TestModels/), \ `deleteItemsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/deleteItemsTest.py>`_\  (TestModels/), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TestModels/), \ `taskmanagerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/taskmanagerTest.py>`_\  (TestModels/), \ `coordinateVectorConstraint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateVectorConstraint.py>`_\  (TestModels/), \ `coordinateVectorConstraintGenericODE2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateVectorConstraintGenericODE2.py>`_\  (TestModels/), \ `modelUnitTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/modelUnitTests.py>`_\  (TestModels/), \ `PARTS_ATEs_moving.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/PARTS_ATEs_moving.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


