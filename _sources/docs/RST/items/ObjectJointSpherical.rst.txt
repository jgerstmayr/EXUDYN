

.. _sec-item-objectjointspherical:

ObjectJointSpherical
====================

A spherical joint, which constrains the relative translation between two position based markers.

\ **Additional information for ObjectJointSpherical**\ :

* | The Object has the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested marker type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``SphericalJoint``\ 
* | \ **Short name**\  for Python visualization object = \ ``VSphericalJoint``\ 


The item \ **ObjectJointSpherical**\  with type = 'JointSpherical' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, size =  2, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector; \ :math:`m1`\  is the moving coin rigid body and \ :math:`m0`\  is the marker for the ground body, which use the localPosition=[0,0,0] for this marker!
* | **constrainedAxes** [\ :math:`{\mathbf{j}}=[j_0,\,\ldots,\,j_2]`\ , type = ArrayIndex, size = 3, default = [1,1,1]]:
  | flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; for \ :math:`j_i`\ , two values are possible: 0=free axis, 1=constrained axis
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectJointSpherical]:
  | parameters for visualization of item



The item VObjectJointSpherical has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **jointRadius** [type = float, default = 0.1]:
  | radius of joint to draw
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
  | current global position of position marker \ :math:`m0`\ 
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
  | current global velocity of position marker \ :math:`m0`\ 
* | ``Displacement``\ : \ :math:`\LU{0}{\Delta{\mathbf{p}}}=\LU{0}{{\mathbf{p}}}_{m1} - \LU{0}{{\mathbf{p}}}_{m0}`\ 
  | constraint drift or relative motion, if not all axes fixed
* | ``Force``\ : \ :math:`\LU{0}{{\mathbf{f}}}`\ 
  | joint force in global coordinates




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


