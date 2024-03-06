

.. _sec-item-markernodeposition:

MarkerNodePosition
==================

A node-Marker attached to a position-based node. It can be used for connectors, joints or loads where position is required. If connectors also require orientation information, use a MarkerNodeRigid.

\ **Additional information for MarkerNodePosition**\ :

* | This \ ``Marker``\  has/provides the following types = \ ``Node``\ , \ ``Position``\ 


The item \ **MarkerNodePosition**\  with type = 'NodePosition' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number to which marker is attached to
* | **visualization** [type = VMarkerNodePosition]:
  | parameters for visualization of item



The item VMarkerNodePosition has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-markernodeposition:

DESCRIPTION of MarkerNodePosition
---------------------------------
The node position marker provides an interface to a node which contains a position
(\ ``NodePoint``\ , \ ``NodePoint2D``\ , \ ``NodeRigidBodyEP``\ , \ ``NodePointSlope``\ , ...)
and accesses \ **position**\ , \ **velocity**\  and the \ **position jacobian**\ .
The position and velocity are computed according to the definition of output variables in the respective nodes.

The position jacobian represents the derivative of the node position \ :math:`{\mathbf{p}}_\mathrm{n}`\  with all nodal coordinates,

.. math::

   \LU{0}{{\mathbf{J}}_\mathrm{pos}} = \frac{\partial \LU{0}{{\mathbf{p}}_\mathrm{n}}}{\partial {\mathbf{q}}_\mathrm{n}}


For details, see the respective definition of the node and the C++ implementation.

In examplary case of a \ ``NodeRigidBody2D``\ ,  see Section :ref:`sec-item-noderigidbody2d`\ , its coordinates are 
\ :math:`{\mathbf{q}}_\mathrm{n}=[q_0,\;q_1,\;\psi_0,\;]\tp`\ , where \ :math:`q_0`\  represents the \ :math:`x`\ -displacement 
and \ :math:`q_1`\  represents the \ :math:`y`\ -displacement, such that the jacobian for the 3D position vector reads

.. math::

   \LU{0}{{\mathbf{J}}_\mathrm{pos}^{\mathrm{NodeRigidBody2D}}} = \mr{1}{0}{0} {0}{1}{0} {0}{0}{0}




Relevant Examples and TestModels with weblink:

    \ `ALEANCFpipe.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ALEANCFpipe.py>`_\  (Examples/), \ `ANCFcantileverTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcantileverTest.py>`_\  (Examples/), \ `ANCFcantileverTestDyn.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcantileverTestDyn.py>`_\  (Examples/), \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFslidingJoint2Drigid.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2Drigid.py>`_\  (Examples/), \ `ANCFtestHalfcircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFtestHalfcircle.py>`_\  (Examples/), \ `ANCFtests2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFtests2.py>`_\  (Examples/), \ `doublePendulum2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/doublePendulum2D.py>`_\  (Examples/), \ `flexibleRotor3Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/flexibleRotor3Dtest.py>`_\  (Examples/), \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Examples/), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TestModels/), \ `ANCFcontactCircleTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactCircleTest.py>`_\  (TestModels/), \ `ANCFcontactFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactFrictionTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


