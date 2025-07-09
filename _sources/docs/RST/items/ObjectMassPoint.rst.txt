

.. _sec-item-objectmasspoint:

ObjectMassPoint
===============

A 3D mass point which is attached to a position-based node, usually NodePoint.

\ **Additional information for ObjectMassPoint**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``SingleNoded``\ 
* | Requested \ ``Node``\  type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``MassPoint``\ 
* | \ **Short name**\  for Python visualization object = \ ``VMassPoint``\ 


The item \ **ObjectMassPoint**\  with type = 'MassPoint' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsMass** [\ :math:`m`\ , type = UReal, default = 0.]:
  | mass [SI:kg] of mass point
* | **nodeNumber** [\ :math:`n0`\ , type = NodeIndex, default = invalid (-1)]:
  | node number (type NodeIndex) for mass point
* | **visualization** [type = VObjectMassPoint]:
  | parameters for visualization of item



The item VObjectMassPoint has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **graphicsData** [type = BodyGraphicsData]:
  | Structure contains data for body visualization; data is defined in special list / dictionary structure


----------

.. _description-objectmasspoint:

DESCRIPTION of ObjectMassPoint
------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}\cConfig(\pLocB) = \LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef + \LU{0b}{\mathbf{I}_{3 \times 3}}\pLocB`\ 
  | global position vector of translated local position; local (body) coordinate system = global coordinate system
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}}\cConfig = [q_0,\;q_1,\;q_2]\cConfig\tp`\ 
  | global displacement vector of mass point
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}\cConfig = \LU{0}{\dot{\mathbf{u}}}\cConfig = [\dot q_0,\;\dot q_1,\;\dot q_2]\cConfig\tp`\ 
  | global velocity vector of mass point
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}}\cConfig = \LU{0}{\ddot{\mathbf{u}}}\cConfig = [\ddot q_0,\;\ddot q_1,\;\ddot q_2]\cConfig\tp`\ 
  | global acceleration vector of mass point
* | ``RotationMatrix``\ : 
  | identity matrix (only for completeness)
* | ``Rotation``\ : \ :math:`[0,0,0]`\ 
  | (only for completeness)
* | ``AngularVelocity``\ : \ :math:`[0,0,0]`\ 
  | (only for completeness)
* | ``AngularVelocityLocal``\ : \ :math:`[0,0,0]`\ 
  | (only for completeness)



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | node position
     - | \ :math:`\LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef = \LU{0}{{\mathbf{p}}}(n_0)\cConfig`\ 
     - | position of mass point which is provided by node \ :math:`n_0`\  in any configuration
   * - | node displacement
     - | \ :math:`\LU{0}{{\mathbf{u}}}\cConfig = \LU{0}{\pRef}\cConfig = [q_0,\;q_1,\;q_2]\cConfig\tp = \LU{0}{{\mathbf{u}}}(n_0)\cConfig`\ 
     - | displacement of mass point which is provided by node \ :math:`n_0`\  in any configuration
   * - | node velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}\cConfig = [\dot q_0,\;\dot q_1,\;\dot q_2]\cConfig\tp = \LU{0}{{\mathbf{v}}}(n_0)\cConfig`\ 
     - | velocity of mass point which is provided by node \ :math:`n_0`\  in any configuration
   * - | transformation matrix
     - | \ :math:`\LU{0b}{\Rot} = \mathbf{I}_{3 \times 3}`\ 
     - | transformation of local body (\ :math:`b`\ ) coordinates to global (0) coordinates; this is the constant unit matrix, because local = global coordinates for the mass point
   * - | residual forces
     - | \ :math:`\LU{0}{{\mathbf{f}}} = [f_0,\;f_1,\;f_2]\tp`\ 
     - | residual of all forces on mass point 
   * - | applied forces
     - | \ :math:`\LU{0}{{\mathbf{f}}}_a = [f_0,\;f_1,\;f_2]\tp`\ 
     - | applied forces (loads, connectors, joint reaction forces, ...)



Equations of motion
-------------------


.. math::

   \mr{m}{0}{0} {0}{m}{0} {0}{0}{m} \vr{\ddot q_0}{\ddot q_1}{\ddot q_2} = \vr{f_0}{f_1}{f_2}.


For example, a LoadCoordinate on coordinate 1 of the node would add a term in \ :math:`f_1`\  on the RHS.

Position-based markers can measure position \ :math:`{\mathbf{p}}\cConfig`\ . The \ **position jacobian**\   

.. math::

   {\mathbf{J}}_{pos} = \partial {\mathbf{p}}\cCur / \partial {\mathbf{c}}\cCur = \mr{1}{0}{0} {0}{1}{0} {0}{0}{0}


transforms the action of global applied forces \ :math:`\LU{0}{{\mathbf{f}}}_a`\  of position-based markers on the coordinates \ :math:`{\mathbf{c}}`\ 

.. math::

   {\mathbf{Q}} = {\mathbf{J}}_{pos} \LU{0}{{\mathbf{f}}}_a.





.. _miniexample-objectmasspoint:

MINI EXAMPLE for ObjectMassPoint
--------------------------------


.. code-block:: python
   :linenos:

   node = mbs.AddNode(NodePoint(referenceCoordinates = [1,1,0], 
                                initialCoordinates=[0.5,0,0],
                                initialVelocities=[0.5,0,0]))
   mbs.AddObject(MassPoint(nodeNumber = node, physicsMass=1))
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveDynamic()
   
   #check result
   exudynTestGlobals.testResult = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[0]
   #final x-coordinate of position shall be 2

Relevant Examples and TestModels with weblink:

    \ `interactiveTutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/interactiveTutorial.py>`_\  (Examples/), \ `ComputeSensitivitiesExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ComputeSensitivitiesExample.py>`_\  (Examples/), \ `coordinateSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/coordinateSpringDamper.py>`_\  (Examples/), \ `massSpringFrictionInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/massSpringFrictionInteractive.py>`_\  (Examples/), \ `minimizeExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/minimizeExample.py>`_\  (Examples/), \ `nMassOscillator.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillator.py>`_\  (Examples/), \ `nMassOscillatorInteractive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/nMassOscillatorInteractive.py>`_\  (Examples/), \ `parameterVariationExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/parameterVariationExample.py>`_\  (Examples/), \ `particleClusters.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particleClusters.py>`_\  (Examples/), \ `particlesSilo.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesSilo.py>`_\  (Examples/), \ `particlesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesTest.py>`_\  (Examples/), \ `particlesTest3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/particlesTest3D.py>`_\  (Examples/), \ `complexEigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/complexEigenvaluesTest.py>`_\  (TestModels/), \ `connectorGravityTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/connectorGravityTest.py>`_\  (TestModels/), \ `contactCoordinateTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/contactCoordinateTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


