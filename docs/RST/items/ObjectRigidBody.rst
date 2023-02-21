

.. _sec-item-objectrigidbody:

ObjectRigidBody
===============

A 3D rigid body which is attached to a 3D rigid body node. The rotation parametrization of the rigid body follows the rotation parametrization of the node. Use Euler parameters in the general case (no singularities) in combination with implicit solvers (GeneralizedAlpha or TrapezoidalIndex2), Tait-Bryan angles for special cases, e.g., rotors where no singularities occur if you rotate about \ :math:`x`\  or \ :math:`z`\  axis, or use Lie-group formulation with rotation vector together with explicit solvers. REMARK: Use the class \ ``RigidBodyInertia``\ , see Section :ref:`sec-rigidbodyutilities-rigidbodyinertia---init--`\  and \ ``AddRigidBody(...)``\ , see Section :ref:`sec-rigidbodyutilities-addrigidbody`\ , of \ ``exudyn.rigidBodyUtilities``\  to handle inertia, COM and mass. \addExampleImageObjectRigidBody

\ **Additional information for ObjectRigidBody**\ :

* | The Object has the following types = \ ``Body``\ , \ ``SingleNoded``\ 
* | Requested node type = \ ``Position``\  + \ ``Orientation``\  + \ ``RigidBody``\ 
* | \ **Short name**\  for Python = \ ``RigidBody``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRigidBody``\ 


The item \ **ObjectRigidBody**\  with type = 'RigidBody' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsMass** [\ :math:`m`\ , type = UReal, default = 0.]:
  | mass [SI:kg] of rigid body
* | **physicsInertia** [\ :math:`\LU{b}{{\mathbf{j}}_6}`\ , type = Vector6D, default = [0.,0.,0., 0.,0.,0.]]:
  | inertia components [SI:kgm\ :math:`^2`\ ]: \ :math:`[J_{xx}, J_{yy}, J_{zz}, J_{yz}, J_{xz}, J_{xy}]`\  in body-fixed coordinate system and w.r.t. to the reference point of the body, NOT necessarily w.r.t. to COM; use the class RigidBodyInertia and AddRigidBody(...) of exudynRigidBodyUtilities.py to handle inertia, COM and mass
* | **physicsCenterOfMass** [\ :math:`\LU{b}{\bv_{COM}}`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local position of COM relative to the body's reference point; if the vector of the COM is [0,0,0], the computation will not consider additional terms for the COM and it is faster
* | **nodeNumber** [\ :math:`n0`\ , type = NodeIndex, default = invalid (-1)]:
  | node number (type NodeIndex) for rigid body node
* | **visualization** [type = VObjectRigidBody]:
  | parameters for visualization of item



The item VObjectRigidBody has the following parameters:

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
* | ``Rotation``\ : 
  | vector with 3 components of the Euler angles in xyz-sequence (R=Rx*Ry*Rz), recomputed from rotation matrix
* | ``AngularVelocity``\ : \ :math:`\LU{0}{\tomega}\cConfig`\ 
  | angular velocity of body
* | ``AngularVelocityLocal``\ : \ :math:`\LU{b}{\tomega}\cConfig`\ 
  | local (body-fixed) 3D velocity vector of node
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}}\cConfig(\pLocB) = \LU{0}{\ddot{\mathbf{u}}} + \LU{0}{\talpha} \times (\LU{0b}{\Rot} \pLocB) +  \LU{0}{\tomega} \times ( \LU{0}{\tomega} \times(\LU{0b}{\Rot} \pLocB))`\ 
  | global acceleration vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``AccelerationLocal``\ : \ :math:`\LU{b}{{\mathbf{a}}}\cConfig(\pLocB) = (\LU{b0}{\Rot} \LU{0}{{\mathbf{a}}}\cConfig(\pLocB)`\ 
  | local (body-fixed) acceleration vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``AngularAcceleration``\ : \ :math:`\LU{0}{\talpha}\cConfig`\ 
  | angular acceleration vector of body
* | ``AngularAccelerationLocal``\ : \ :math:`\LU{b}{\talpha}\cConfig = (\LU{b0}{\Rot} \LU{0}{\talpha}\cConfig`\ 
  | local angular acceleration vector of body




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


