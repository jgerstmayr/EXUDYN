

.. _sec-item-objectrigidbody:

ObjectRigidBody
===============

A 3D rigid body which is attached to a 3D rigid body node. The rotation parametrization of the rigid body follows the rotation parametrization of the node. Use Euler parameters in the general case (no singularities) in combination with implicit solvers (GeneralizedAlpha or TrapezoidalIndex2), Tait-Bryan angles for special cases, e.g., rotors where no singularities occur if you rotate about \ :math:`x`\  or \ :math:`z`\  axis, or use Lie-group formulation with rotation vector together with explicit solvers. REMARK: Use the class \ ``RigidBodyInertia``\ , see Section :ref:`sec-rigidbodyutilities-rigidbodyinertia---init--`\  and \ ``AddRigidBody(...)``\ , see Section :ref:`sec-rigidbodyutilities-addrigidbody`\ , of \ ``exudyn.rigidBodyUtilities``\  to handle inertia, COM and mass. \addExampleImageObjectRigidBody
 



The item \ **ObjectRigidBody**\  with type = 'RigidBody' has the following parameters:

 

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsMass** [type = UReal, default = 0.]:
  | mass [SI:kg] of rigid body
* | **physicsInertia** [type = Vector6D, default = [0.,0.,0., 0.,0.,0.]]:
  | inertia components [SI:kgm\ :math:`^2`\ ]: \ :math:`[J_xx, J_yy, J_zz, J_yz, J_xz, J_xy]`\  in body-fixed coordinate system and w.r.t. to the reference point of the body, NOT necessarily w.r.t. to COM; use the class RigidBodyInertia and AddRigidBody(...) of exudynRigidBodyUtilities.py to handle inertia, COM and mass
* | **physicsCenterOfMass** [type = Vector3D, default = [0.,0.,0.], size = 3]:
  | local position of COM relative to the body's reference point; if the vector of the COM is [0,0,0], the computation will not consider additional terms for the COM and it is faster
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number (type NodeIndex) for rigid body node



The item VObjectRigidBody has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **graphicsDataUserFunction** [type = PyFunctionGraphicsData, default =  0]:
  | A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates
* | **graphicsData** [type = BodyGraphicsData]:
  | Structure contains data for body visualization; data is defined in special list / dictionary structure




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


