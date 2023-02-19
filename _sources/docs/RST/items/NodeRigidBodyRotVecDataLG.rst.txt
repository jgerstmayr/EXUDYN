

.. _sec-item-noderigidbodyrotvecdatalg:

NodeRigidBodyRotVecDataLG
=========================

A 3D rigid body node based on rotation vector and Lie group methods for rigid bodies or beams; the node has 3 displacement coordinates and three rotation coordinates (rotation vector) and additionally data coordinates for configuration at start of a computation step. External operations (initial coordinates, graphics, ...) operate on data coordinates, which represent the global frame. Internally, ODE2 coordinates represent the local frame and they must be updated using (implicit) Lie group methods. UNDER CONSTRUCTION, DO NOT USE!!!
 



The item \ **NodeRigidBodyRotVecDataLG**\  with type = 'RigidBodyRotVecDataLG' has the following parameters:

 

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector6D, default = [0.,0.,0., 0.,0.,0.], size = 3]:
  | reference coordinates (position and rotation vector \ :math:`\tnu`\ ) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints)
* | **initialCoordinates** [type = Vector6D, default = [0.,0.,0., 0.,0.,0.], size = 3]:
  | initial displacement coordinates \ :math:`{\mathbf{u}}`\  and rotation vector \ :math:`\tnu`\  relative to reference coordinates; these coordinates are mapped to the data coordiantes!
* | **initialVelocities** [type = Vector6D, default = [0.,0.,0., 0.,0.,0.], size = 3]:
  | initial velocity coordinate: time derivatives of displacement and angular velocity vector; these coordinates are mapped to the ODE2 velocity coordinates!



The item VNodeRigidBodyRotVecDataLG has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.], size = 4]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


