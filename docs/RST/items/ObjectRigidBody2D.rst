

.. _sec-item-objectrigidbody2d:

ObjectRigidBody2D
=================

A 2D rigid body which is attached to a rigid body 2D node. The body obtains coordinates, position, velocity, etc. from the underlying 2D node

\ **Additional information for ObjectRigidBody2D**\ :

* | The Object has the following types = \ ``Body``\ , \ ``SingleNoded``\ 
* | Requested node type = \ ``Position2D``\  + \ ``Orientation2D``\  + \ ``Position``\  + \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``RigidBody2D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRigidBody2D``\ 


The item \ **ObjectRigidBody2D**\  with type = 'RigidBody2D' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsMass** [\ :math:`m`\ , type = UReal, default = 0.]:
  | mass [SI:kg] of rigid body
* | **physicsInertia** [\ :math:`J`\ , type = UReal, default = 0.]:
  | inertia [SI:kgm\ :math:`^2`\ ] of rigid body w.r.t. center of mass
* | **nodeNumber** [\ :math:`n_0`\ , type = NodeIndex, default = invalid (-1)]:
  | node number (type NodeIndex) for 2D rigid body node
* | **visualization** [type = VObjectRigidBody2D]:
  | parameters for visualization of item



The item VObjectRigidBody2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **graphicsDataUserFunction** [type = PyFunctionGraphicsData, default =  0]:
  | A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates
* | **graphicsData** [type = BodyGraphicsData]:
  | Structure contains data for body visualization; data is defined in special list / dictionary structure



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}\cConfig(\pLocB) = \LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef + \LU{0b}{\Rot}\pLocB`\ 
  | global position vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}}\cConfig + \LU{0b}{\Rot}\pLocB`\ 
  | global displacement vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}\cConfig(\pLocB) = \LU{0}{\dot{\mathbf{u}}}\cConfig + \LU{0b}{\Rot}(\LU{b}{\tomega} \times \pLocB\cConfig)`\ 
  | global velocity vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``VelocityLocal``\ : \ :math:`\LU{b}{{\mathbf{v}}}\cConfig(\pLocB) = \LU{b0}{\Rot} \LU{0}{{\mathbf{v}}}\cConfig(\pLocB)`\ 
  | local (body-fixed) velocity vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``RotationMatrix``\ : \ :math:`\mathrm{vec}(\LU{0b}{\Rot})=[A_{00},\,A_{01},\,A_{02},\,A_{10},\,\ldots,\,A_{21},\,A_{22}]\cConfig\tp`\ 
  | vector with 9 components of the rotation matrix (row-major format)
* | ``Rotation``\ : \ :math:`\theta_{0\mathrm{config}}`\ 
  | scalar rotation angle of body
* | ``AngularVelocity``\ : \ :math:`\LU{0}{\tomega}\cConfig`\ 
  | angular velocity of body
* | ``AngularVelocityLocal``\ : \ :math:`\LU{b}{\tomega}\cConfig`\ 
  | local (body-fixed) 3D velocity vector of node
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}}\cConfig(\pLocB) = \LU{0}{\ddot{\mathbf{u}}} + \LU{0}{\talpha} \times (\LU{0b}{\Rot} \pLocB) +  \LU{0}{\tomega} \times ( \LU{0}{\tomega} \times(\LU{0b}{\Rot} \pLocB))`\ 
  | global acceleration vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``AccelerationLocal``\ : \ :math:`\LU{b}{{\mathbf{a}}}\cConfig(\pLocB) = \LU{b0}{\Rot} \LU{0}{{\mathbf{a}}}\cConfig(\pLocB)`\ 
  | local (body-fixed) acceleration vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``AngularAcceleration``\ : \ :math:`\LU{0}{\talpha}\cConfig`\ 
  | angular acceleration vector of body
* | ``AngularAccelerationLocal``\ : \ :math:`\LU{b}{\talpha}\cConfig = \LU{b0}{\Rot} \LU{0}{\talpha}\cConfig`\ 
  | local angular acceleration vector of body




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


