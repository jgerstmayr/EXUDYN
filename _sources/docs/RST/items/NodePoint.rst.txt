

.. _sec-item-nodepoint:

NodePoint
=========

A 3D point node for point masses or solid finite elements which has 3 displacement degrees of freedom for \ :ref:`ODE2 <ODE2>`\ .

\ **Additional information for NodePoint**\ :

* | The Node has the following types = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``Point``\ 
* | \ **Short name**\  for Python visualization object = \ ``VPoint``\ 


The item \ **NodePoint**\  with type = 'Point' has the following parameters:

* | **name** [type = String, default = '']:
  | node's unique name
* | **referenceCoordinates** [\ :math:`{\mathbf{q}}\cRef = [q_0,\,q_1,\,q_2]\tp\cRef = {\mathbf{p}}\cRef = [r_0,\,r_1,\,r_2]\tp`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | reference coordinates of node, e.g. ref. coordinates for finite elements; global position of node without displacement
* | **initialCoordinates** [\ :math:`{\mathbf{q}}\cIni = [q_0,\,q_1,\,q_2]\cIni\tp = {\mathbf{u}}\cIni = [u_0,\,u_1,\,u_2]\cIni\tp`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | initial displacement coordinate
* | **initialVelocities** [\ :math:`\dot{\mathbf{q}}\cIni = {\mathbf{v}}\cIni = [\dot q_0,\,\dot q_1,\,\dot q_2]\cIni\tp`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | initial velocity coordinate
* | **visualization** [type = VNodePoint]:
  | parameters for visualization of item



The item VNodePoint has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size (diameter, dimensions of underlying cube, etc.)  for item; size == -1.f means that default size is used
* | **color** [type = Float4, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | Default RGBA color for nodes; 4th value is alpha-transparency; R=-1.f means, that default color is used


----------

.. _description-nodepoint:

DESCRIPTION of NodePoint
------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`{\mathbf{p}}\cConfig = [p_0,\,p_1,\,p_2]\cConfig\tp= {\mathbf{u}}\cConfig + {\mathbf{p}}\cRef`\ 
  | global 3D position vector of node; \ :math:`{\mathbf{u}}\cRef=0`\ 
* | ``Displacement``\ : \ :math:`{\mathbf{u}}\cConfig = [q_0,\,q_1,\,q_2]\cConfig\tp`\ 
  | global 3D displacement vector of node
* | ``Velocity``\ : \ :math:`{\mathbf{v}}\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2]\cConfig\tp`\ 
  | global 3D velocity vector of node
* | ``Acceleration``\ : \ :math:`{\mathbf{a}}\cConfig = \ddot {\mathbf{q}}\cConfig = [\ddot q_0,\,\ddot q_1,\,\ddot q_2]\cConfig\tp`\ 
  | global 3D acceleration vector of node
* | ``Coordinates``\ : \ :math:`{\mathbf{c}}\cConfig = {\mathbf{u}}\cConfig = [q_0,\,q_1,\,q_2]\tp\cConfig`\ 
  | coordinate vector of node
* | ``Coordinates\_t``\ : \ :math:`\dot{\mathbf{c}}\cConfig = {\mathbf{v}}\cConfig = [\dot q_0,\,\dot q_1,\,\dot q_2]\tp\cConfig`\ 
  | velocity coordinates vector of node
* | ``Coordinates\_tt``\ : \ :math:`\ddot{\mathbf{c}}\cConfig = {\mathbf{a}}\cConfig = [\ddot q_0,\,\ddot q_1,\,\ddot q_2]\tp\cConfig`\ 
  | acceleration coordinates vector of node


\paragraphDetailed information:
The node provides \ :math:`n_c=3`\  displacement coordinates. Equations of motion need to be provided by an according object (e.g., MassPoint, finite elements, ...).
Usually, the nodal coordinates are provided in the global frame. However, the coordinate system is defined by the object (e.g. MassPoint uses global coordinates, but floating frame of reference objects use local frames).
Note that for this very simple node, coordinates are identical to the nodal displacements, same for time derivatives. This is not the case, e.g. for nodes with orientation. 


\ **Example**\  for NodePoint: see ObjectMassPoint, Section :ref:`sec-item-objectmasspoint`\ 



\ **The web version may not be complete. For details, always consider the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


