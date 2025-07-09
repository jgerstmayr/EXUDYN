

.. _sec-item-objectmasspoint2d:

ObjectMassPoint2D
=================

A 2D mass point which is attached to a position-based 2D node.

\ **Additional information for ObjectMassPoint2D**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``SingleNoded``\ 
* | Requested \ ``Node``\  type = \ ``Position2D``\  + \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``MassPoint2D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VMassPoint2D``\ 


The item \ **ObjectMassPoint2D**\  with type = 'MassPoint2D' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsMass** [\ :math:`m`\ , type = UReal, default = 0.]:
  | mass [SI:kg] of mass point
* | **nodeNumber** [\ :math:`n0`\ , type = NodeIndex, default = invalid (-1)]:
  | node number (type NodeIndex) for mass point
* | **visualization** [type = VObjectMassPoint2D]:
  | parameters for visualization of item



The item VObjectMassPoint2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **graphicsData** [type = BodyGraphicsData]:
  | Structure contains data for body visualization; data is defined in special list / dictionary structure


----------

.. _description-objectmasspoint2d:

DESCRIPTION of ObjectMassPoint2D
--------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}\cConfig(\pLocB) = \LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef + \LU{0b}{\mathbf{I}_{2 \times 2}}\pLocB`\ 
  | global position vector of translated local position; local (body) coordinate system = global coordinate system
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}}\cConfig = [q_0,\;q_1,\;0]\cConfig\tp`\ 
  | global displacement vector of mass point
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}\cConfig = \LU{0}{\dot{\mathbf{u}}}\cConfig = [\dot q_0,\;\dot q_1,\;0]\cConfig\tp`\ 
  | global velocity vector of mass point
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}}\cConfig = \LU{0}{\ddot{\mathbf{u}}}\cConfig = [\ddot q_0,\;\ddot q_1,\;0]\cConfig\tp`\ 
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
     - | position of mass point which is provided by node \ :math:`n_0`\  in any configuration (except reference)
   * - | node displacement
     - | \ :math:`\LU{0}{{\mathbf{u}}}\cConfig = \LU{0}{\pRef}\cConfig = [q_0,\;q_1,\;0]\cConfig\tp = \LU{0}{{\mathbf{u}}}(n_0)\cConfig`\ 
     - | displacement of mass point which is provided by node \ :math:`n_0`\  in any configuration
   * - | node velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}\cConfig = [\dot q_0,\;\dot q_1,\;0]\cConfig\tp = \LU{0}{{\mathbf{v}}}(n_0)\cConfig`\ 
     - | velocity of mass point which is provided by node \ :math:`n_0`\  in any configuration
   * - | transformation matrix
     - | \ :math:`\LU{0b}{\Rot} = \mathbf{I}_{3 \times 3}`\ 
     - | transformation of local body (\ :math:`b`\ ) coordinates to global (0) coordinates; this is the constant unit matrix, because local = global coordinates for the mass point
   * - | residual forces
     - | \ :math:`\LU{0}{{\mathbf{f}}} = [f_0,\;f_1]\tp`\ 
     - | residual of all forces on mass point
   * - | applied forces
     - | \ :math:`\LU{0}{{\mathbf{f}}}_a = [f_0,\;f_1,\;f_2]\tp`\ 
     - | applied forces (loads, connectors, joint reaction forces, ...)


Equations of motion
-------------------


.. math::

   \mp{m}{0} {0}{m} \vp{\ddot q_0}{\ddot q_1} = \vp{f_0}{f_1}.


For example, a LoadCoordinate on coordinate 1 of the node would add a term in \ :math:`f_1`\  on the RHS.

Position-based markers can measure position \ :math:`{\mathbf{p}}\cConfig`\ . The \ **position jacobian**\   

.. math::

   {\mathbf{J}}_{pos} = \partial {\mathbf{p}}\cCur / \partial {\mathbf{c}}\cCur = \left[\!\! \begin{array}{ccc} 1 & 0 & 0 \\ 0 & 1 & 0 \end{array} \!\!\right]


transforms the action of global applied forces \ :math:`\LU{0}{{\mathbf{f}}}_a`\  of position-based markers on the coordinates \ :math:`{\mathbf{c}}`\ 

.. math::

   {\mathbf{Q}} = {\mathbf{J}}_{pos} \LU{0}{{\mathbf{f}}}_a.





.. _miniexample-objectmasspoint2d:

MINI EXAMPLE for ObjectMassPoint2D
----------------------------------


.. code-block:: python
   :linenos:

   node = mbs.AddNode(NodePoint2D(referenceCoordinates = [1,1], 
                                initialCoordinates=[0.5,0],
                                initialVelocities=[0.5,0]))
   mbs.AddObject(MassPoint2D(nodeNumber = node, physicsMass=1))
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveDynamic()
   
   #check result
   exudynTestGlobals.testResult = mbs.GetNodeOutput(node, exu.OutputVariableType.Position)[0]
   #final x-coordinate of position shall be 2

Relevant Examples and TestModels with weblink:

    \ `myFirstExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/myFirstExample.py>`_\  (Examples/), \ `reevingSystemOpen.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/reevingSystemOpen.py>`_\  (Examples/), \ `xExudynConfigSpecial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/xExudynConfigSpecial.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFslidingJoint2Drigid.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2Drigid.py>`_\  (Examples/), \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Examples/), \ `pendulum2Dconstraint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulum2Dconstraint.py>`_\  (Examples/), \ `pendulumIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulumIftommBenchmark.py>`_\  (Examples/), \ `SliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/SliderCrank.py>`_\  (Examples/), \ `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_\  (Examples/), \ `slidercrankWithMassSpring.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/slidercrankWithMassSpring.py>`_\  (Examples/), \ `SpringDamperMassUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/SpringDamperMassUserFunction.py>`_\  (Examples/), \ `modelUnitTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/modelUnitTests.py>`_\  (TestModels/), \ `coordinateVectorConstraint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateVectorConstraint.py>`_\  (TestModels/), \ `sliderCrankFloatingTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrankFloatingTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


