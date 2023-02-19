

.. _sec-item-objectjointgeneric:

ObjectJointGeneric
==================

A generic joint in 3D; constrains components of the absolute position and rotations of two points given by PointMarkers or RigidMarkers. An additional local rotation (rotationMarker) can be used to adjust the three rotation axes and/or sliding axes.
 



The item \ **ObjectJointGeneric**\  with type = 'JointGeneric' has the following parameters:

 

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ], size =  2]:
  | list of markers used in connector
* | **constrainedAxes** [type = ArrayIndex, default = [1,1,1,1,1,1], size = 6]:
  | flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; for \ :math:`j_i`\ , two values are possible: 0=free axis, 1=constrained axis
* | **rotationMarker0** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker \ :math:`m0`\ ; translation and rotation axes for marker \ :math:`m0`\  are defined in the local body coordinate system and additionally transformed by rotationMarker0
* | **rotationMarker1** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker \ :math:`m1`\ ; translation and rotation axes for marker \ :math:`m1`\  are defined in the local body coordinate system and additionally transformed by rotationMarker1
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **offsetUserFunctionParameters** [type = Vector6D, default = [0.,0.,0.,0.,0.,0.]]:
  | vector of 6 parameters for joint's offsetUserFunction
* | **offsetUserFunction** [type = PyFunctionVector6DmbsScalarIndexVector6D, default =  0]:
  | A Python function which defines the time-dependent (fixed) offset of translation (indices 0,1,2) and rotation (indices 3,4,5) joint coordinates with parameters (mbs, t, offsetUserFunctionParameters)
* | **offsetUserFunction_t** [type = PyFunctionVector6DmbsScalarIndexVector6D, default =  0]:
  | (NOT IMPLEMENTED YET)time derivative of offsetUserFunction using the same parameters



The item VObjectJointGeneric has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **axesRadius** [type = float, default = 0.1]:
  | radius of joint axes to draw
* | **axesLength** [type = float, default = 0.4]:
  | length of joint axes to draw
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


