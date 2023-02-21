

.. _sec-item-objectjointgeneric:

ObjectJointGeneric
==================

A generic joint in 3D; constrains components of the absolute position and rotations of two points given by PointMarkers or RigidMarkers. An additional local rotation (rotationMarker) can be used to adjust the three rotation axes and/or sliding axes.

\ **Additional information for ObjectJointGeneric**\ :

* | The Object has the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested marker type = \ ``Position``\  + \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``GenericJoint``\ 
* | \ **Short name**\  for Python visualization object = \ ``VGenericJoint``\ 


The item \ **ObjectJointGeneric**\  with type = 'JointGeneric' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, size =  2, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **constrainedAxes** [\ :math:`{\mathbf{j}}=[j_0,\,\ldots,\,j_5]`\ , type = ArrayIndex, size = 6, default = [1,1,1,1,1,1]]:
  | flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; for \ :math:`j_i`\ , two values are possible: 0=free axis, 1=constrained axis
* | **rotationMarker0** [\ :math:`\LU{m0,J0}{\Rot}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker \ :math:`m0`\ ; translation and rotation axes for marker \ :math:`m0`\  are defined in the local body coordinate system and additionally transformed by rotationMarker0
* | **rotationMarker1** [\ :math:`\LU{m1,J1}{\Rot}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker \ :math:`m1`\ ; translation and rotation axes for marker \ :math:`m1`\  are defined in the local body coordinate system and additionally transformed by rotationMarker1
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **offsetUserFunctionParameters** [\ :math:`{\mathbf{p}}_{par}`\ , type = Vector6D, default = [0.,0.,0.,0.,0.,0.]]:
  | vector of 6 parameters for joint's offsetUserFunction
* | **offsetUserFunction** [\ :math:`\mathrm{UF} \in \Rcal^6`\ , type = PyFunctionVector6DmbsScalarIndexVector6D, default =  0]:
  | A Python function which defines the time-dependent (fixed) offset of translation (indices 0,1,2) and rotation (indices 3,4,5) joint coordinates with parameters (mbs, t, offsetUserFunctionParameters)
* | **offsetUserFunction_t** [\ :math:`\mathrm{UF} \in \Rcal^6`\ , type = PyFunctionVector6DmbsScalarIndexVector6D, default =  0]:
  | (NOT IMPLEMENTED YET)time derivative of offsetUserFunction using the same parameters
* | **visualization** [type = VObjectJointGeneric]:
  | parameters for visualization of item



The item VObjectJointGeneric has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **axesRadius** [type = float, default = 0.1]:
  | radius of joint axes to draw
* | **axesLength** [type = float, default = 0.4]:
  | length of joint axes to draw
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
  | relative rotation parameters (Tait Bryan Rxyz); if all axes are fixed, this output represents the rotational drift; for a revolute joint with free Z-axis, it contains the rotation in the Z-component
* | ``AngularVelocityLocal``\ : \ :math:`\LU{J0}{\Delta\tomega}`\ 
  | relative angular velocity in local joint0 coordinates; if all axes are fixed, this output represents the angular velocity constraint error; for a revolute joint, it contains the angular velocity of this axis
* | ``ForceLocal``\ : \ :math:`\LU{J0}{{\mathbf{f}}}`\ 
  | joint force in local \ :math:`J0`\  coordinates
* | ``TorqueLocal``\ : \ :math:`\LU{J0}{{\mathbf{m}}}`\ 
  | joint torque in local \ :math:`J0`\  coordinates; depending on joint configuration, the result may not be the according torque vector




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


