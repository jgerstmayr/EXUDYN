

.. _sec-item-nodepoint2d:

NodePoint2D
===========

A 2D point node for point masses or solid finite elements which has 2 displacement degrees of freedom for second order differential equations.

\ **Additional information for NodePoint2D**\ :

* | This \ ``Node``\  has/provides the following types = \ ``Position2D``\ , \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``Point2D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VPoint2D``\ 


The item \ **NodePoint2D**\  with type = 'Point2D' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [\ :math:`{\mathbf{q}}\cRef = [q_0,\,q_1]\tp\cRef = {\mathbf{p}}\cRef = [r_0,\,r_1]\tp`\ , type = Vector2D, size = 2, default = [0.,0.]]:
  | reference coordinates of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement
* | **initialCoordinates** [\ :math:`{\mathbf{q}}\cIni = [q_0,\,q_1]\cIni\tp = [u_0,\,u_1]\cIni\tp`\ , type = Vector2D, size = 2, default = [0.,0.]]:
  | initial displacement coordinate
* | **initialVelocities** [\ :math:`\dot{\mathbf{q}}\cIni = {\mathbf{v}}\cIni = [\dot q_0,\,\dot q_1]\cIni\tp`\ , type = Vector2D, size = 2, default = [0.,0.]]:
  | initial velocity coordinate
* | **visualization** [type = VNodePoint2D]:
  | parameters for visualization of item



The item VNodePoint2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used


----------

.. _description-nodepoint2d:

DESCRIPTION of NodePoint2D
--------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`{\mathbf{p}}\cConfig = [p_0,\,p_1,\,0]\cConfig\tp= {\mathbf{u}}\cConfig + {\mathbf{p}}\cRef`\ 
  | global 3D position vector of node; \ :math:`{\mathbf{u}}\cRef=0`\ 
* | ``Displacement``\ : \ :math:`{\mathbf{u}}\cConfig = [q_0,\,q_1,\,0]\cConfig\tp`\ 
  | global 3D displacement vector of node
* | ``Velocity``\ : \ :math:`{\mathbf{v}}\cConfig = [\dot q_0,\,\dot q_1,\,0]\cConfig\tp`\ 
  | global 3D velocity vector of node
* | ``Acceleration``\ : \ :math:`{\mathbf{a}}\cConfig = [\ddot q_0,\,\ddot q_1,\,0]\cConfig\tp`\ 
  | global 3D acceleration vector of node
* | ``CoordinatesTotal``\ : \ :math:`{\mathbf{c}}\cConfig = {\mathbf{u}}\cConfig + {\mathbf{p}}\cRef`\ 
  | displacement plus reference coordinates of node
* | ``Coordinates``\ : \ :math:`{\mathbf{c}}\cConfig = [q_0,\,q_1]\tp\cConfig`\ 
  | coordinate vector of node
* | ``Coordinates\_t``\ : \ :math:`\dot{\mathbf{c}}\cConfig = [\dot q_0,\,\dot q_1]\tp\cConfig`\ 
  | velocity coordinates vector of node
* | ``Coordinates\_tt``\ : \ :math:`\ddot{\mathbf{c}}\cConfig = {\mathbf{a}}\cConfig = [\ddot q_0,\,\ddot q_1]\tp\cConfig`\ 
  | acceleration coordinates vector of node
* | ``RotationMatrix``\ : 
  | identity matrix (only for completeness)
* | ``Rotation``\ : \ :math:`[0,0,0]`\ 
  | (only for completeness)
* | ``AngularVelocity``\ : \ :math:`[0,0,0]`\ 
  | (only for completeness)
* | ``AngularVelocityLocal``\ : \ :math:`[0,0,0]`\ 
  | (only for completeness)



\ **Detailed information:** 
The node provides \ :math:`n_c=2`\  displacement coordinates. Equations of motion need to be provided by an according object (e.g., MassPoint2D).
Coordinates are identical to the nodal displacements, except for the third coordinate \ :math:`u_2`\ , which is zero, because \ :math:`q_2`\  does not exist. 

Note that for this very simple node, coordinates are identical to the nodal displacements, same for time derivatives. This is not the case, e.g. for nodes with orientation. 


\ **Example**\  for NodePoint2D: see ObjectMassPoint2D, Section :ref:`sec-item-objectmasspoint2d`\ 


Relevant Examples and TestModels with weblink:

    \ `myFirstExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/myFirstExample.py>`_\  (Examples/), \ `pendulum2Dconstraint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulum2Dconstraint.py>`_\  (Examples/), \ `pendulumIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulumIftommBenchmark.py>`_\  (Examples/), \ `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_\  (Examples/), \ `SpringDamperMassUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/SpringDamperMassUserFunction.py>`_\  (Examples/), \ `xExudynConfigSpecial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/xExudynConfigSpecial.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFslidingJoint2Drigid.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2Drigid.py>`_\  (Examples/), \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Examples/), \ `SliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/SliderCrank.py>`_\  (Examples/), \ `slidercrankWithMassSpring.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/slidercrankWithMassSpring.py>`_\  (Examples/), \ `switchingConstraintsPendulum.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/switchingConstraintsPendulum.py>`_\  (Examples/), \ `modelUnitTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/modelUnitTests.py>`_\  (TestModels/), \ `sparseMatrixSpringDamperTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sparseMatrixSpringDamperTest.py>`_\  (TestModels/), \ `coordinateVectorConstraint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/coordinateVectorConstraint.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


