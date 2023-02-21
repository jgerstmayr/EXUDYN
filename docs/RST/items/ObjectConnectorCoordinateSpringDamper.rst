

.. _sec-item-objectconnectorcoordinatespringdamper:

ObjectConnectorCoordinateSpringDamper
=====================================

A 1D (scalar) spring-damper element acting on single ODE2 coordinates; connects to coordinate-based markers; NOTE that the coordinate markers only measure the coordinate (=displacement), but the reference position is not included as compared to position-based markers!; the spring-damper can also act on rotational coordinates.

\ **Additional information for ObjectConnectorCoordinateSpringDamper**\ :

* | The Object has the following types = \ ``Connector``\ 
* | Requested marker type = \ ``Coordinate``\ 
* | \ **Short name**\  for Python = \ ``CoordinateSpringDamper``\ 
* | \ **Short name**\  for Python visualization object = \ ``VCoordinateSpringDamper``\ 


The item \ **ObjectConnectorCoordinateSpringDamper**\  with type = 'ConnectorCoordinateSpringDamper' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **stiffness** [\ :math:`k`\ , type = Real, default = 0.]:
  | stiffness [SI:N/m] of spring; acts against relative value of coordinates
* | **damping** [\ :math:`d`\ , type = Real, default = 0.]:
  | damping [SI:N/(m s)] of damper; acts against relative velocity of coordinates
* | **offset** [\ :math:`l_\mathrm{off}`\ , type = Real, default = 0.]:
  | offset between two coordinates (reference length of springs), see equation
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **springForceUserFunction** [\ :math:`\mathrm{UF} \in \Rcal`\ , type = PyFunctionMbsScalarIndexScalar5, default =  0]:
  | A Python function which defines the spring force with 8 parameters, see equations section / see description below
* | **visualization** [type = VObjectConnectorCoordinateSpringDamper]:
  | parameters for visualization of item



The item VObjectConnectorCoordinateSpringDamper has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Displacement``\ : \ :math:`\Delta q`\ 
  | relative scalar displacement of marker coordinates
* | ``Velocity``\ : \ :math:`\Delta v`\ 
  | difference of scalar marker velocity coordinates
* | ``Force``\ : \ :math:`f_{SD}`\ 
  | scalar force in connector




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 

