

.. _sec-item-objectjointprismaticx:

ObjectJointPrismaticX
=====================

A prismatic joint in 3D; constrains the relative rotation of two rigid body markers and relative motion w.r.t. the joint \ :math:`y`\  and \ :math:`z`\  axes, allowing a relative motion along the joint \ :math:`x`\  axis (defined in local coordinates of marker 0 / joint J0 coordinates). An additional local rotation (rotationMarker) can be used to transform the markers' coordinate systems into the joint coordinate system. For easier definition of the joint, use the exudyn.rigidbodyUtilities function AddPrismaticJoint(...), Section :ref:`sec-rigidbodyutilities-addprismaticjoint`\ , for two rigid bodies (or ground). \addExampleImagePrismaticJointX
 



The item \ **ObjectJointPrismaticX**\  with type = 'JointPrismaticX' has the following parameters:

 

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ], size =  2]:
  | list of markers used in connector
* | **rotationMarker0** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker \ :math:`m0`\ ; translation and rotation axes for marker \ :math:`m0`\  are defined in the local body coordinate system and additionally transformed by rotationMarker0
* | **rotationMarker1** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker \ :math:`m1`\ ; translation and rotation axes for marker \ :math:`m1`\  are defined in the local body coordinate system and additionally transformed by rotationMarker1
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint



The item VObjectJointPrismaticX has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **axisRadius** [type = float, default = 0.1]:
  | radius of joint axis to draw
* | **axisLength** [type = float, default = 0.4]:
  | length of joint axis to draw
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


