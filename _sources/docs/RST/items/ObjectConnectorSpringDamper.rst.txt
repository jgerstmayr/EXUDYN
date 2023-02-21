

.. _sec-item-objectconnectorspringdamper:

ObjectConnectorSpringDamper
===========================

An simple spring-damper element with additional force; connects to position-based markers.

\ **Additional information for ObjectConnectorSpringDamper**\ :

* | The Object has the following types = \ ``Connector``\ 
* | Requested marker type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``SpringDamper``\ 
* | \ **Short name**\  for Python visualization object = \ ``VSpringDamper``\ 


The item \ **ObjectConnectorSpringDamper**\  with type = 'ConnectorSpringDamper' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **referenceLength** [\ :math:`L_0`\ , type = PReal, default = 0.]:
  | reference length [SI:m] of spring
* | **stiffness** [\ :math:`k`\ , type = UReal, default = 0.]:
  | stiffness [SI:N/m] of spring; acts against (length-initialLength)
* | **damping** [\ :math:`d`\ , type = UReal, default = 0.]:
  | damping [SI:N/(m s)] of damper; acts against d/dt(length)
* | **force** [\ :math:`f_{a}`\ , type = Real, default = 0.]:
  | added constant force [SI:N] of spring; scalar force; f=1 is equivalent to reducing initialLength by 1/stiffness; f > 0: tension; f < 0: compression; can be used to model actuator force
* | **velocityOffset** [\ :math:`\dot L_0`\ , type = Real, default = 0.]:
  | velocity offset [SI:m/s] of damper, being equivalent to time change of reference length
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **springForceUserFunction** [\ :math:`\mathrm{UF} \in \Rcal`\ , type = PyFunctionMbsScalarIndexScalar5, default =  0]:
  | A Python function which defines the spring force with parameters; the Python function will only be evaluated, if activeConnector is true, otherwise the SpringDamper is inactive; see description below
* | **visualization** [type = VObjectConnectorSpringDamper]:
  | parameters for visualization of item



The item VObjectConnectorSpringDamper has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Distance``\ : 
  | distance between both points
* | ``Displacement``\ : 
  | relative displacement between both points
* | ``Velocity``\ : 
  | relative velocity between both points
* | ``Force``\ : \ :math:`{\mathbf{f}}`\ 
  | 3D spring-damper force vector
* | ``ForceLocal``\ : \ :math:`f_{SD}`\ 
  | scalar spring-damper force




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


