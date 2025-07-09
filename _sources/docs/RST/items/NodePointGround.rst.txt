

.. _sec-item-nodepointground:

NodePointGround
===============

A 3D point node fixed to ground. The node can be used as NodePoint, but it does not generate coordinates. Applied or reaction forces do not have any effect. This node can be used for 'blind' or 'dummy' \ :ref:`ODE2 <ODE2>`\  and \ :ref:`ODE1 <ODE1>`\  coordinates to which CoordinateSpringDamper or CoordinateConstraint objects are attached to.

\ **Additional information for NodePointGround**\ :

* | This \ ``Node``\  has/provides the following types = \ ``Ground``\ , \ ``Position2D``\ , \ ``Position``\ , \ ``Orientation``\ , \ ``GenericODE2``\ 
* | \ **Short name**\  for Python = \ ``PointGround``\ 
* | \ **Short name**\  for Python visualization object = \ ``VPointGround``\ 


The item \ **NodePointGround**\  with type = 'PointGround' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [\ :math:`{\mathbf{q}}\cRef = [q_0,\,q_1,\,q_2]\tp\cRef = {\mathbf{p}}\cRef = [r_0,\,r_1,\,r_2]\tp`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | reference coordinates of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement
* | **visualization** [type = VNodePointGround]:
  | parameters for visualization of item



The item VNodePointGround has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used


----------

.. _description-nodepointground:

DESCRIPTION of NodePointGround
------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`{\mathbf{p}}\cConfig = [p_0,\,p_1,\,p_2]\cConfig\tp = {\mathbf{p}}\cRef`\ 
  | global 3D position vector of node (=reference position)
* | ``Displacement``\ : \ :math:`{\mathbf{u}}\cConfig = [0,\,0,\,0]\cConfig\tp`\ 
  | zero 3D vector
* | ``Velocity``\ : \ :math:`{\mathbf{v}}\cConfig = [0,\,0,\,0]\cConfig\tp`\ 
  | zero 3D vector
* | ``CoordinatesTotal``\ : \ :math:`{\mathbf{c}}\cConfig =[]`\ 
  | vector of length zero
* | ``Coordinates``\ : \ :math:`{\mathbf{c}}\cConfig =[]`\ 
  | vector of length zero
* | ``Coordinates\_t``\ : \ :math:`\dot{\mathbf{c}}\cConfig =[]`\ 
  | vector of length zero
* | ``RotationMatrix``\ : 
  | identity matrix (only for completeness)
* | ``Rotation``\ : \ :math:`[0,0,0]`\ 
  | (only for completeness)
* | ``AngularVelocity``\ : \ :math:`[0,0,0]`\ 
  | (only for completeness)
* | ``AngularVelocityLocal``\ : \ :math:`[0,0,0]`\ 
  | (only for completeness)



Relevant Examples and TestModels with weblink:

    \ `ALEANCFpipe.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ALEANCFpipe.py>`_\  (Examples/), \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Examples/), \ `ANCFcableCantilevered.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcableCantilevered.py>`_\  (Examples/), \ `ANCFcantileverTestDyn.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcantileverTestDyn.py>`_\  (Examples/), \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFmovingRigidbody.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFmovingRigidbody.py>`_\  (Examples/), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFslidingJoint2Drigid.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2Drigid.py>`_\  (Examples/), \ `ANCFswitchingSlidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFswitchingSlidingJoint2D.py>`_\  (Examples/), \ `ANCFtestHalfcircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFtestHalfcircle.py>`_\  (Examples/), \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TestModels/), \ `ANCFBeamTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamTest.py>`_\  (TestModels/), \ `ANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFbeltDrive.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


