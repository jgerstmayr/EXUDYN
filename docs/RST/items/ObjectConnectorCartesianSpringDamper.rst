

.. _sec-item-objectconnectorcartesianspringdamper:

ObjectConnectorCartesianSpringDamper
====================================

An 3D spring-damper element, providing springs and dampers in three (global) directions (x,y,z); the connector can be attached to position-based markers.

\ **Additional information for ObjectConnectorCartesianSpringDamper**\ :

* | The Object has the following types = \ ``Connector``\ 
* | Requested marker type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``CartesianSpringDamper``\ 
* | \ **Short name**\  for Python visualization object = \ ``VCartesianSpringDamper``\ 


The item \ **ObjectConnectorCartesianSpringDamper**\  with type = 'ConnectorCartesianSpringDamper' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **stiffness** [\ :math:`{\mathbf{k}}`\ , type = Vector3D, default = [0.,0.,0.]]:
  | stiffness [SI:N/m] of springs; act against relative displacements in 0, 1, and 2-direction
* | **damping** [\ :math:`{\mathbf{d}}`\ , type = Vector3D, default = [0.,0.,0.]]:
  | damping [SI:N/(m s)] of dampers; act against relative velocities in 0, 1, and 2-direction
* | **offset** [\ :math:`{\mathbf{v}}_{\mathrm{off}}`\ , type = Vector3D, default = [0.,0.,0.]]:
  | offset between two springs
* | **springForceUserFunction** [\ :math:`\mathrm{UF} \in \Rcal^3`\ , type = PyFunctionVector3DmbsScalarIndexScalar4Vector3D, default =  0]:
  | A Python function which computes the 3D force vector between the two marker points, if activeConnector=True; see description below
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectConnectorCartesianSpringDamper]:
  | parameters for visualization of item



The item VObjectConnectorCartesianSpringDamper has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Displacement``\ : \ :math:`\Delta\! \LU{0}{{\mathbf{p}}} = \LU{0}{{\mathbf{p}}}_{m1} - \LU{0}{{\mathbf{p}}}_{m0}`\ 
  | relative displacement in global coordinates
* | ``Distance``\ : \ :math:`L=|\Delta\! \LU{0}{{\mathbf{p}}}|`\ 
  | scalar distance between both marker points
* | ``Velocity``\ : \ :math:`\Delta\! \LU{0}{{\mathbf{v}}} = \LU{0}{{\mathbf{v}}}_{m1} - \LU{0}{{\mathbf{v}}}_{m0}`\ 
  | relative translational velocity in global coordinates
* | ``Force``\ : \ :math:`{\mathbf{f}}_{SD}`\ 
  | joint force in global coordinates, see equations




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


