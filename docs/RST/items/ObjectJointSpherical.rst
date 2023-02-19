

.. _sec-item-objectjointspherical:

ObjectJointSpherical
====================

A spherical joint, which constrains the relative translation between two position based markers.
 



The item \ **ObjectJointSpherical**\  with type = 'JointSpherical' has the following parameters:

 

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ], size =  2]:
  | list of markers used in connector; \ :math:`m1`\  is the moving coin rigid body and \ :math:`m0`\  is the marker for the ground body, which use the localPosition=[0,0,0] for this marker!
* | **constrainedAxes** [type = ArrayIndex, default = [1,1,1], size = 3]:
  | flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; for \ :math:`j_i`\ , two values are possible: 0=free axis, 1=constrained axis
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint



The item VObjectJointSpherical has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **jointRadius** [type = float, default = 0.1]:
  | radius of joint to draw
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


