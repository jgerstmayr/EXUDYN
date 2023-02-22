

.. _sec-item-nodepointground:

NodePointGround
===============

A 3D point node fixed to ground. The node can be used as NodePoint, but it does not generate coordinates. Applied or reaction forces do not have any effect. This node can be used for 'blind' or 'dummy' ODE2 and ODE1 coordinates to which CoordinateSpringDamper or CoordinateConstraint objects are attached to.

\ **Additional information for NodePointGround**\ :

* | The Node has the following types = \ ``Ground``\ , \ ``Position2D``\ , \ ``Position``\ , \ ``Orientation``\ , \ ``GenericODE2``\ 
* | \ **Short name**\  for Python = \ ``PointGround``\ 
* | \ **Short name**\  for Python visualization object = \ ``VPointGround``\ 


The item \ **NodePointGround**\  with type = 'PointGround' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [\ :math:`{\mathbf{q}}\cRef = [q_0,\,q_1,\,q_2]\tp\cRef = {\mathbf{p}}\cRef = [r_0,\,r_1,\,r_2]\tp`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | reference coordinates of node ==> e.g. ref. coordinates for finite elements; global position of node without displacement
* | **visualization** [type = VNodePointGround]:
  | parameters for visualization of item



The item VNodePointGround has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used


----------

.. _description-nodepointground:

DESCRIPTION of NodePointGround
------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`{\mathbf{p}}\cConfig = [p_0,\,p_1,\,p_2]\cConfig\tp = {\mathbf{p}}\cRef`\ 
  | global 3D position vector of node (=reference position)
* | ``Displacement``\ : \ :math:`{\mathbf{u}}\cConfig = [0,\,0,\,0]\cConfig\tp`\ 
  | zero 3D vector
* | ``Velocity``\ : \ :math:`{\mathbf{v}}\cConfig = [0,\,0,\,0]\cConfig\tp`\ 
  | zero 3D vector
* | ``Coordinates``\ : \ :math:`{\mathbf{c}}\cConfig =[]`\ 
  | vector of length zero
* | ``Coordinates\_t``\ : \ :math:`\dot{\mathbf{c}}\cConfig =[]`\ 
  | vector of length zero




\ **The web version may not be complete. For details, always consider the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


