

.. _sec-item-objectjointrevolute2d:

ObjectJointRevolute2D
=====================

A revolute joint in 2D; constrains the absolute 2D position of two points given by PointMarkers or RigidMarkers

\ **Additional information for ObjectJointRevolute2D**\ :

* | The Object has the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested marker type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``RevoluteJoint2D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRevoluteJoint2D``\ 


The item \ **ObjectJointRevolute2D**\  with type = 'JointRevolute2D' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectJointRevolute2D]:
  | parameters for visualization of item



The item VObjectJointRevolute2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = radius of revolute joint; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


