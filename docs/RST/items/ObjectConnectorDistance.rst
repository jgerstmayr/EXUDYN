

.. _sec-item-objectconnectordistance:

ObjectConnectorDistance
=======================

Connector which enforces constant or prescribed distance between two bodies/nodes.

\ **Additional information for ObjectConnectorDistance**\ :

* | The Object has the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested marker type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``DistanceConstraint``\ 
* | \ **Short name**\  for Python visualization object = \ ``VDistanceConstraint``\ 


The item \ **ObjectConnectorDistance**\  with type = 'ConnectorDistance' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **distance** [\ :math:`d_0`\ , type = UReal, default = 0.]:
  | prescribed distance [SI:m] of the used markers
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectConnectorDistance]:
  | parameters for visualization of item



The item VObjectConnectorDistance has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = link size; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Displacement``\ : \ :math:`\LU{0}{\Delta{\mathbf{p}}}`\ 
  | relative displacement in global coordinates
* | ``Velocity``\ : \ :math:`\LU{0}{\Delta{\mathbf{v}}}`\ 
  | relative translational velocity in global coordinates
* | ``Distance``\ : \ :math:`|\LU{0}{\Delta{\mathbf{p}}}|`\ 
  | distance between markers (should stay constant; shows constraint deviation)
* | ``Force``\ : \ :math:`\lambda_0`\ 
  | joint force (=scalar Lagrange multiplier)




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


