

.. _sec-item-objectconnectorreevingsystemsprings:

ObjectConnectorReevingSystemSprings
===================================

A rD reeving system defined by a list of torque-free and friction-free sheaves or points that are connected with one rope (modelled as massless spring). The force is assumed to be constant all over the rope. The sheaves or connection points are defined by \ :math:`nr`\  rigid body markers \ :math:`[m_0, \, m_1, \, \ldots, \, m_{nr-1}]`\ . At both ends of the rope there may be a prescribed motion coupled to a coordinate marker each, given by \ :math:`m_{c0}`\  and \ :math:`m_{c1}`\  .

\ **Additional information for ObjectConnectorReevingSystemSprings**\ :

* | The Object has the following types = \ ``Connector``\ 
* | Requested marker type = \ ``_None``\ 
* | \ **Short name**\  for Python = \ ``ReevingSystemSprings``\ 
* | \ **Short name**\  for Python visualization object = \ ``VReevingSystemSprings``\ 


The item \ **ObjectConnectorReevingSystemSprings**\  with type = 'ConnectorReevingSystemSprings' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [\ :math:`[m_0, \, m_1, \, \ldots, \, m_{nr-1},\, m_{c0}, \, m_{c1}]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of position or rigid body markers used in reeving system and optional two coordinate markers (\ :math:`m_{c0}, \, m_{c1}`\ ); the first marker \ :math:`m_0`\  and the last rigid body marker \ :math:`m_{nr-1}`\  represent the ends of the rope and are directly connected to a position; the markers \ :math:`m_1, \, \ldots, \, m_{nr-2}`\  can be connected to sheaves, for which a radius and an axis can be prescribed. The coordinate markers are optional and represent prescribed length at the rope ends (marker \ :math:`m_{c0}`\  is added length at start, marker \ :math:`m_{c1}`\  is added length at end of the rope in the reeving system)
* | **hasCoordinateMarkers** [type = Bool, default = False]:
  | flag, which determines, the list of markers (markerNumbers) contains two coordinate markers at the end of the list, representing the prescribed change of length at both ends
* | **coordinateFactors** [\ :math:`[f_0,\, f_1]\tp`\ , type = Vector2D, default = [1,1]]:
  | factors which are multiplied with the values of coordinate markers; this can be used, e.g., to change directions or to transform rotations (revolutions of a sheave) into change of length
* | **stiffnessPerLength** [\ :math:`EA`\ , type = UReal, default = 0.]:
  | stiffness per length [SI:N/m/m] of rope; in case of cross section \ :math:`A`\  and Young's modulus \ :math:`E`\ , this parameter results in \ :math:`E\cdot A`\ ; the effective stiffness of the reeving system is computed as \ :math:`EA/L`\  in which \ :math:`L`\  is the current length of the rope
* | **dampingPerLength** [\ :math:`DA`\ , type = UReal, default = 0.]:
  | axial damping per length [SI:N/(m/s)/m] of rope; the effective damping coefficient of the reeving system is computed as \ :math:`DA/L`\  in which \ :math:`L`\  is the current length of the rope
* | **dampingTorsional** [\ :math:`DT`\ , type = UReal, default = 0.]:
  | torsional damping [SI:Nms] between sheaves; this effect can damp rotations around the rope axis, pairwise between sheaves; this parameter is experimental
* | **dampingShear** [\ :math:`DS`\ , type = UReal, default = 0.]:
  | damping of shear motion [SI:Ns] between sheaves; this effect can damp motion perpendicular to the rope between each pair of sheaves; this parameter is experimental
* | **referenceLength** [\ :math:`L_{ref}`\ , type = Real, default = 0.]:
  | reference length for computation of roped force
* | **sheavesAxes** [\ :math:`{\mathbf{l}}_a = [\LU{m0}{{\mathbf{a}}_0},\, \LU{m1}{{\mathbf{a}}_1},\, \ldots ] in [\Rcal^{3}, ...]`\ , type = Vector3DList, default = []]:
  | list of local vectors axes of sheaves; vectors refer to rigid body markers given in list of markerNumbers; first and last axes are ignored, as they represent the attachment of the rope ends
* | **sheavesRadii** [\ :math:`{\mathbf{l}}_r = [r_0,\, r_1,\, \ldots]\tp \in \Rcal^{n}`\ , type = Vector, default = []]:
  | radius for each sheave, related to list of markerNumbers and list of sheaveAxes; first and last radii must always be zero.
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectConnectorReevingSystemSprings]:
  | parameters for visualization of item



The item VObjectConnectorReevingSystemSprings has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **ropeRadius** [type = float, default = 0.001]:
  | radius of rope
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Distance``\ : \ :math:`L`\ 
  | current total length of rope
* | ``VelocityLocal``\ : \ :math:`\dot L`\ 
  | scalar time derivative of current total length of rope
* | ``ForceLocal``\ : \ :math:`F`\ 
  | scalar force in reeving system (constant over length of rope)




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 

