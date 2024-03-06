

.. _sec-item-markerbodyposition:

MarkerBodyPosition
==================

A position body-marker attached to a local (body-fixed) position \ :math:`\pLocB = [b_0,\; b_1,\; b_2]`\  (\ :math:`x`\ , \ :math:`y`\ , and \ :math:`z`\  coordinates) of the body. It provides position information as well as the according derivatives (=velocity and derivative of position w.r.t. body coordinates). It can be used for connectors, joints or loads where position is required. If connectors also require orientation information, use a MarkerBodyRigid.

\ **Additional information for MarkerBodyPosition**\ :

* | This \ ``Marker``\  has/provides the following types = \ ``Object``\ , \ ``Body``\ , \ ``Position``\ 


The item \ **MarkerBodyPosition**\  with type = 'BodyPosition' has the following parameters:

* | **name** [type = String, default = '']:
  | marker's unique name
* | **bodyNumber** [type = ObjectIndex, default = invalid (-1)]:
  | body number to which marker is attached to
* | **localPosition** [\ :math:`\pLocB`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local body position of marker; e.g. local (body-fixed) position where force is applied to
* | **visualization** [type = VMarkerBodyPosition]:
  | parameters for visualization of item



The item VMarkerBodyPosition has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown


----------

.. _description-markerbodyposition:

DESCRIPTION of MarkerBodyPosition
---------------------------------
The body position marker provides an interface to a object of type body 
(\ ``ObjectGround``\ , \ ``ObjectMassPoint``\ , \ ``ObjectRigidBody``\ , ...)
and provides access to kinematic quantities such as \ **position**\  and \ **velocity**\  
and to the \ **position jacobian**\ , using a \ ``localPosition``\  \ :math:`\pLocB`\  which is defined within the 
local coordinates of the body (\ :math:`b`\ ).
The kinematic quantities are computed according to the definition of output variables in the respective bodies.

The position jacobian represents the derivative of the node position \ :math:`{\mathbf{p}}_\mathrm{n}`\  with all nodal coordinates,

.. math::

   \LU{0}{{\mathbf{J}}_\mathrm{pos}} = \frac{\partial \LU{0}{{\mathbf{p}}_\mathrm{n}}}{\partial {\mathbf{q}}_\mathrm{n}}


and it is usually computed as the derivative of the (global) translational velocity w.r.t.\ velocity coordinates,

.. math::

   \LU{0}{{\mathbf{J}}_\mathrm{pos}} = \frac{\partial \LU{0}{{\mathbf{v}}_\mathrm{n}}}{\partial \dot {\mathbf{q}}_\mathrm{n}}



As an example of the \ ``ObjectRigidBody2D``\ , see Section :ref:`sec-item-objectrigidbody2d`\ , the position and velocity are computed as

.. math::

   \LU{0}{{\mathbf{p}}}\cConfig(\pLocB) = \LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef + \LU{0b}{\Rot}\pLocB ,



.. math::

   \LU{0}{{\mathbf{v}}}\cConfig(\pLocB) = \LU{0}{\dot{\mathbf{u}}}\cConfig + \LU{0b}{\Rot}(\LU{b}{\tomega} \times \pLocB\cConfig) .


Thus, the position jacobian for \ ``ObjectRigidBody2D``\  reads

.. math::

   \LU{0}{{\mathbf{J}}_\mathrm{pos}^{\mathrm{NodeRigidBody2D}}} = \mr{1}{0}{-\sin\theta_0 \LU{b}{b_0} - \cos\theta_0 \LU{b}{b_1}} {0}{1}{\cos\theta_0 \LU{b}{b_0} - \sin\theta_0 \LU{b}{b_1}} {0}{0}{0}


For details, see the respective definition of the body and the C++ implementation.


Relevant Examples and TestModels with weblink:

    \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFmovingRigidbody.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFmovingRigidbody.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFslidingJoint2Drigid.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2Drigid.py>`_\  (Examples/), \ `ANCFswitchingSlidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFswitchingSlidingJoint2D.py>`_\  (Examples/), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Examples/), \ `coordinateSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/coordinateSpringDamper.py>`_\  (Examples/), \ `finiteSegmentMethod.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/finiteSegmentMethod.py>`_\  (Examples/), \ `flexibleRotor3Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/flexibleRotor3Dtest.py>`_\  (Examples/), \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Examples/), \ `HydraulicActuator2Arms.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/HydraulicActuator2Arms.py>`_\  (Examples/), \ `ANCFcontactCircleTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactCircleTest.py>`_\  (TestModels/), \ `ANCFcontactFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactFrictionTest.py>`_\  (TestModels/), \ `ANCFmovingRigidBodyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFmovingRigidBodyTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


