

.. _sec-item-objectjointrevolutez:

ObjectJointRevoluteZ
====================

A revolute joint in 3D; constrains the position of two rigid body markers and the rotation about two axes, while the joint \ :math:`z`\ -rotation axis (defined in local coordinates of marker 0 / joint J0 coordinates) can freely rotate. An additional local rotation (rotationMarker) can be used to transform the markers' coordinate systems into the joint coordinate system. For easier definition of the joint, use the exudyn.rigidbodyUtilities function AddRevoluteJoint(...), Section :ref:`sec-rigidbodyutilities-addrevolutejoint`\ , for two rigid bodies (or ground). \addExampleImageRevoluteJointZ

\ **Additional information for ObjectJointRevoluteZ**\ :

* | The Object has the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested marker type = \ ``Position``\  + \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``RevoluteJointZ``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRevoluteJointZ``\ 


The item \ **ObjectJointRevoluteZ**\  with type = 'JointRevoluteZ' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, size =  2, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **rotationMarker0** [\ :math:`\LU{m0,J0}{\Rot}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker \ :math:`m0`\ ; translation and rotation axes for marker \ :math:`m0`\  are defined in the local body coordinate system and additionally transformed by rotationMarker0
* | **rotationMarker1** [\ :math:`\LU{m1,J1}{\Rot}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker \ :math:`m1`\ ; translation and rotation axes for marker \ :math:`m1`\  are defined in the local body coordinate system and additionally transformed by rotationMarker1
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectJointRevoluteZ]:
  | parameters for visualization of item



The item VObjectJointRevoluteZ has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **axisRadius** [type = float, default = 0.1]:
  | radius of joint axis to draw
* | **axisLength** [type = float, default = 0.4]:
  | length of joint axis to draw
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
  | current global position of position marker \ :math:`m0`\ 
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
  | current global velocity of position marker \ :math:`m0`\ 
* | ``DisplacementLocal``\ : \ :math:`\LU{J0}{\Delta{\mathbf{p}}}`\ 
  | relative displacement in local joint0 coordinates; uses local J0 coordinates even for spherical joint configuration
* | ``VelocityLocal``\ : \ :math:`\LU{J0}{\Delta{\mathbf{v}}}`\ 
  | relative translational velocity in local joint0 coordinates
* | ``Rotation``\ : \ :math:`\LU{J0}{\ttheta}= [\theta_0,\theta_1,\theta_2]\tp`\ 
  | relative rotation parameters (Tait Bryan Rxyz); Z component represents rotation of joint, other components represent constraint drift
* | ``AngularVelocityLocal``\ : \ :math:`\LU{J0}{\Delta\tomega}`\ 
  | relative angular velocity in joint J0 coordinates, giving a vector with Z-component only
* | ``ForceLocal``\ : \ :math:`\LU{J0}{{\mathbf{f}}}`\ 
  | joint force in local \ :math:`J0`\  coordinates
* | ``TorqueLocal``\ : \ :math:`\LU{J0}{{\mathbf{m}}}`\ 
  | joint torques in local \ :math:`J0`\  coordinates; torque around Z is zero




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


