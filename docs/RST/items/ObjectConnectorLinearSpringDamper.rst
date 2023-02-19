

.. _sec-item-objectconnectorlinearspringdamper:

ObjectConnectorLinearSpringDamper
=================================

An linear spring-damper element acting on relative translations along given axis of local joint0 coordinate system; connects to position and orientation-based markers; the linear spring-damper is intended to act within prismatic joints or in situations where only one translational axis is free; if the two markers rotate relative to each other, the spring-damper will always act in the local joint0 coordinate system.
 



The item \ **ObjectConnectorLinearSpringDamper**\  with type = 'ConnectorLinearSpringDamper' has the following parameters:

 

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **stiffness** [type = Real, default = 0.]:
  | torsional stiffness [SI:Nm/rad] against relative rotation
* | **damping** [type = Real, default = 0.]:
  | torsional damping [SI:Nm/(rad/s)]
* | **axisMarker0** [type = Vector3D, default = [1,0,0]]:
  | local axis of spring-damper in marker 0 coordinates; this axis will co-move with marker \ :math:`m0`\ ; if marker m0 is attached to ground, the spring-damper represents linear equations
* | **offset** [type = Real, default = 0.]:
  | translational offset considered in the spring force calculation (this can be used as position control input!)
* | **velocityOffset** [type = Real, default = 0.]:
  | velocity offset considered in the damper force calculation (this can be used as velocity control input!)
* | **force** [type = Real, default = 0.]:
  | additional constant force [SI:Nm] added to spring-damper; this can be used to prescribe a force between the two attached bodies (e.g., for actuation and control)
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **springForceUserFunction** [type = PyFunctionMbsScalarIndexScalar5, default =  0]:
  | A Python function which computes the scalar force between the two rigid body markers along axisMarker0 in \ :math:`m0`\  coordinates, if activeConnector=True; see description below



The item VObjectConnectorLinearSpringDamper has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **drawAsCylinder** [type = Bool, default = False]:
  | if this flag is True, the spring-damper is represented as cylinder; this may fit better if the spring-damper represents an actuator
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


