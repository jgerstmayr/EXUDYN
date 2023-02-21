

.. _sec-item-objectconnectortorsionalspringdamper:

ObjectConnectorTorsionalSpringDamper
====================================

An torsional spring-damper element acting on relative rotations around Z-axis of local joint0 coordinate system; connects to orientation-based markers; if other rotation axis than the local joint0 Z axis shall be used, the joint rotationMarker0 / rotationMarker1 may be used. The joint perfectly extends a RevoluteJoint with a spring-damper, which can also be used to represent feedback control in an elegant and efficient way, by chosing appropriate user functions. It also allows to measure continuous / infinite rotations by making use of a NodeGeneric which compensates \ :math:`\pm \pi`\  jumps in the measured rotation (OutputVariableType.Rotation).

\ **Additional information for ObjectConnectorTorsionalSpringDamper**\ :

* | The Object has the following types = \ ``Connector``\ 
* | Requested marker type = \ ``Orientation``\ 
* | Requested node type = \ ``GenericData``\ 
* | \ **Short name**\  for Python = \ ``TorsionalSpringDamper``\ 
* | \ **Short name**\  for Python visualization object = \ ``VTorsionalSpringDamper``\ 


The item \ **ObjectConnectorTorsionalSpringDamper**\  with type = 'ConnectorTorsionalSpringDamper' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **nodeNumber** [\ :math:`n_d`\ , type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData with 1 dataCoordinate for continuous rotation reconstruction; if this node is left to invalid index, it will not be used
* | **stiffness** [\ :math:`k`\ , type = Real, default = 0.]:
  | torsional stiffness [SI:Nm/rad] against relative rotation
* | **damping** [\ :math:`d`\ , type = Real, default = 0.]:
  | torsional damping [SI:Nm/(rad/s)]
* | **rotationMarker0** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker 0; transforms joint into marker coordinates
* | **rotationMarker1** [type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker 1; transforms joint into marker coordinates
* | **offset** [\ :math:`v_\mathrm{off}`\ , type = Real, default = 0.]:
  | rotational offset considered in the spring torque calculation (this can be used as rotation control input!)
* | **velocityOffset** [\ :math:`\dot v_\mathrm{off}`\ , type = Real, default = 0.]:
  | angular velocity offset considered in the damper torque calculation (this can be used as angular velocity control input!)
* | **torque** [\ :math:`\tau_c`\ , type = Real, default = 0.]:
  | additional constant torque [SI:Nm] added to spring-damper; this can be used to prescribe a torque between the two attached bodies (e.g., for actuation and control)
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **springTorqueUserFunction** [\ :math:`\mathrm{UF} \in \Rcal`\ , type = PyFunctionMbsScalarIndexScalar5, default =  0]:
  | A Python function which computes the scalar torque between the two rigid body markers in local joint0 coordinates, if activeConnector=True; see description below
* | **visualization** [type = VObjectConnectorTorsionalSpringDamper]:
  | parameters for visualization of item



The item VObjectConnectorTorsionalSpringDamper has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Rotation``\ : \ :math:`\Delta\theta`\ 
  | relative rotation around the spring-damper Z-coordinate, enhanced to a continuous rotation (infinite rotations \ :math:`>+\pi`\  and \ :math:`<-\pi`\ ) if a NodeGeneric with 1 coordinate as added
* | ``AngularVelocityLocal``\ : \ :math:`\Delta\omega`\ 
  | scalar relative angular velocity around joint0 Z-axis
* | ``TorqueLocal``\ : \ :math:`\tau_{SD}`\ 
  | scalar spring-damper torque around the local joint0 Z-axis




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


