

.. _sec-item-objectconnectorcoordinate:

ObjectConnectorCoordinate
=========================

A coordinate constraint which constrains two (scalar) coordinates of Marker[Node|Body]Coordinates attached to nodes or bodies. The constraint acts directly on coordinates, but does not include reference values, e.g., of nodal values. This constraint is computationally efficient and should be used to constrain nodal coordinates.
 



The item \ **ObjectConnectorCoordinate**\  with type = 'ConnectorCoordinate' has the following parameters:

 

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **offset** [type = Real, default = 0.]:
  | An offset between the two values
* | **factorValue1** [type = Real, default = 1.]:
  | An additional factor multiplied with value1 used in algebraic equation
* | **velocityLevel** [type = Bool, default = False]:
  | If true: connector constrains velocities (only works for ODE2 coordinates!); offset is used between velocities; in this case, the offsetUserFunction_t is considered and offsetUserFunction is ignored
* | **offsetUserFunction** [type = PyFunctionMbsScalarIndexScalar, default =  0]:
  | A Python function which defines the time-dependent offset; see description below
* | **offsetUserFunction_t** [type = PyFunctionMbsScalarIndexScalar, default =  0]:
  | time derivative of offsetUserFunction; needed for velocity level constraints; see description below
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint



The item VObjectConnectorCoordinate has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = link size; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


