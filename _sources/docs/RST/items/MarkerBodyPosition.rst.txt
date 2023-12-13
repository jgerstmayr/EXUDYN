

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

Relevant Examples and TestModels with weblink:

    \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFmovingRigidbody.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFmovingRigidbody.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFslidingJoint2Drigid.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2Drigid.py>`_\  (Examples/), \ `ANCFswitchingSlidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFswitchingSlidingJoint2D.py>`_\  (Examples/), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Examples/), \ `coordinateSpringDamper.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/coordinateSpringDamper.py>`_\  (Examples/), \ `finiteSegmentMethod.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/finiteSegmentMethod.py>`_\  (Examples/), \ `flexibleRotor3Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/flexibleRotor3Dtest.py>`_\  (Examples/), \ `geneticOptimizationSliderCrank.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/geneticOptimizationSliderCrank.py>`_\  (Examples/), \ `HydraulicActuator2Arms.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/HydraulicActuator2Arms.py>`_\  (Examples/), \ `ANCFcontactCircleTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactCircleTest.py>`_\  (TestModels/), \ `ANCFcontactFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactFrictionTest.py>`_\  (TestModels/), \ `ANCFmovingRigidBodyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFmovingRigidBodyTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


