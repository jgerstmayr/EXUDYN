

.. _sec-item-objectmasspoint:

ObjectMassPoint
===============

A 3D mass point which is attached to a position-based node, usually NodePoint.

\ **Additional information for ObjectMassPoint**\ :

* | The Object has the following types = \ ``Body``\ , \ ``SingleNoded``\ 
* | Requested node type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``MassPoint``\ 
* | \ **Short name**\  for Python visualization object = \ ``VMassPoint``\ 


The item \ **ObjectMassPoint**\  with type = 'MassPoint' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsMass** [\ :math:`m`\ , type = UReal, default = 0.]:
  | mass [SI:kg] of mass point
* | **nodeNumber** [\ :math:`n0`\ , type = NodeIndex, default = invalid (-1)]:
  | node number (type NodeIndex) for mass point
* | **visualization** [type = VObjectMassPoint]:
  | parameters for visualization of item



The item VObjectMassPoint has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **graphicsData** [type = BodyGraphicsData]:
  | Structure contains data for body visualization; data is defined in special list / dictionary structure



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}\cConfig(\pLocB) = \LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef + \LU{0b}{{\mathbf{I}}Three}\pLocB`\ 
  | global position vector of translated local position; local (body) coordinate system = global coordinate system
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}}\cConfig = [q_0,\;q_1,\;q_2]\cConfig\tp`\ 
  | global displacement vector of mass point
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}\cConfig = \LU{0}{\dot{\mathbf{u}}}\cConfig = [\dot q_0,\;\dot q_1,\;\dot q_2]\cConfig\tp`\ 
  | global velocity vector of mass point
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}}\cConfig = \LU{0}{\ddot{\mathbf{u}}}\cConfig = [\ddot q_0,\;\ddot q_1,\;\ddot q_2]\cConfig\tp`\ 
  | global acceleration vector of mass point




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


