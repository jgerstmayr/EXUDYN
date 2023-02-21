

.. _sec-item-objectground:

ObjectGround
============

A ground object behaving like a rigid body, but having no degrees of freedom; used to attach body-connectors without an action. For examples see spring dampers and joints.

\ **Additional information for ObjectGround**\ :

* | The Object has the following types = \ ``Ground``\ , \ ``Body``\ 


The item \ **ObjectGround**\  with type = 'Ground' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **referencePosition** [\ :math:`\pRefG`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | reference point = reference position for ground object; local position is added on top of reference position for a ground object
* | **referenceRotation** [\ :math:`\LU{0b}{\Rot} \in \Rcal^{3 \times 3}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | the constant ground rotation matrix, which transforms body-fixed (b) to global (0) coordinates
* | **visualization** [type = VObjectGround]:
  | parameters for visualization of item



The item VObjectGround has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **graphicsDataUserFunction** [type = PyFunctionGraphicsData, default =  0]:
  | A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGB node color; if R==-1, use default color
* | **graphicsData** [type = BodyGraphicsData]:
  | Structure contains data for body visualization; data is defined in special list / dictionary structure



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}} = \pRefG + \LU{0b}{\Rot} \pLocB`\ 
  | global position vector of translated local position
* | ``Displacement``\ : \ :math:`\Null`\ 
  | global displacement vector of local position
* | ``Velocity``\ : \ :math:`\Null`\ 
  | global velocity vector of local position
* | ``AngularVelocity``\ : \ :math:`\Null`\ 
  | angular velocity of body
* | ``RotationMatrix``\ : \ :math:`\LU{0b}{\Rot}`\ 
  | rotation matrix in vector form (stored in row-major order)




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


