

.. _sec-item-noderigidbodyep:

NodeRigidBodyEP
===============

A 3D rigid body node based on Euler parameters for rigid bodies or beams; the node has 3 displacement coordinates (representing displacement of reference point \ :math:`\LU0{\mathbf{r}}`\ ) and four rotation coordinates (Euler parameters = unit quaternions).
 



The item \ **NodeRigidBodyEP**\  with type = 'RigidBodyEP' has the following parameters:

 

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [type = Vector7D, default = [0.,0.,0., 0.,0.,0.,0.], size = 7]:
  | reference coordinates (3 position coordinates and 4 Euler parameters) of node ==> e.g. ref. coordinates for finite elements or reference position of rigid body (e.g. for definition of joints)
* | **initialCoordinates** [type = Vector7D, default = [0.,0.,0., 0.,0.,0.,0.], size = 7]:
  | initial displacement coordinates and 4 Euler parameters relative to reference coordinates
* | **initialVelocities** [type = Vector7D, default = [0.,0.,0., 0.,0.,0.,0.], size = 7]:
  | initial velocity coordinates: time derivatives of initial displacements and Euler parameters
* | **addConstraintEquation** [type = Bool, default = True]:
  | True: automatically add Euler parameter constraint for node; False: Euler parameter constraint is not added, must be done manually (e.g., with CoordinateVectorConstraint)



The item VNodeRigidBodyEP has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.], size = 4]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


