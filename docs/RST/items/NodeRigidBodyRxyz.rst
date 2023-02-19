

.. _sec-item-noderigidbodyrxyz:

NodeRigidBodyRxyz
=================

A 3D rigid body node based on Euler / Tait-Bryan angles for rigid bodies or beams; all coordinates lead to second order differential equations; NOTE that this node has a singularity if the second rotation parameter reaches \ :math:`\psi_1 = (2k-1) \pi/2`\ , with \ :math:`k \in \Ncal`\  or \ :math:`-k \in \Ncal`\ .
 



The item \ **NodeRigidBodyRxyz**\  with type = 'RigidBodyRxyz' has the following parameters:

 

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector6D, default = [0.,0.,0., 0.,0.,0.], size = 6]:
  | reference coordinates (3 position and 3 xyz Euler angles) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints)
* | **initialCoordinates** [type = Vector6D, default = [0.,0.,0., 0.,0.,0.], size = 6]:
  | initial displacement coordinates: ux,uy,uz and 3 Euler angles (xyz) relative to reference coordinates
* | **initialVelocities** [type = Vector6D, default = [0.,0.,0., 0.,0.,0.], size = 6]:
  | initial velocity coordinate: time derivatives of ux,uy,uz and of 3 Euler angles (xyz)



The item VNodeRigidBodyRxyz has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.], size = 4]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


