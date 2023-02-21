

.. _sec-item-objectconnectorgravity:

ObjectConnectorGravity
======================

A connector for additing forces due to gravitational fields beween two bodies, which can be used for aerospace and small-scale astronomical problems; DO NOT USE this connector for adding gravitational forces (loads), which should be using LoadMassProportional, which is acting global and always in the same direction.

\ **Additional information for ObjectConnectorGravity**\ :

* | The Object has the following types = \ ``Connector``\ 
* | Requested marker type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``ConnectorGravity``\ 
* | \ **Short name**\  for Python visualization object = \ ``VConnectorGravity``\ 


The item \ **ObjectConnectorGravity**\  with type = 'ConnectorGravity' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **gravitationalConstant** [\ :math:`G`\ , type = Real, default = 6.67430e-11]:
  | gravitational constant [SI:m\ :math:`^3`\ kg\ :math:`^{-1}`\ s\ :math:`^{-2}`\ )]; while not recommended, a negative constant gan represent a repulsive force
* | **mass0** [\ :math:`mass_0`\ , type = UReal, default = 0.]:
  | mass [SI:kg] of object attached to marker \ :math:`m0`\ 
* | **mass1** [\ :math:`mass_1`\ , type = UReal, default = 0.]:
  | mass [SI:kg] of object attached to marker \ :math:`m1`\ 
* | **minDistanceRegularization** [\ :math:`d_{min}`\ , type = UReal, default = 0.]:
  | distance [SI:m] at which a regularization is added in order to avoid singularities, if objects come close
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectConnectorGravity]:
  | parameters for visualization of item



The item VObjectConnectorGravity has the following parameters:

* | **show** [type = Bool, default = False]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Distance``\ : \ :math:`L`\ 
  | distance between both points
* | ``Displacement``\ : \ :math:`\Delta\! \LU{0}{{\mathbf{p}}}`\ 
  | relative displacement between both points
* | ``Force``\ : \ :math:`{\mathbf{f}}`\ 
  | gravity force vector, pointing from marker \ :math:`m0`\  to marker \ :math:`m1`\ 




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


