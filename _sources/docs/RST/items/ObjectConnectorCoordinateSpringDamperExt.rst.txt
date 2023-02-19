

.. _sec-item-objectconnectorcoordinatespringdamperext:

ObjectConnectorCoordinateSpringDamperExt
========================================

A 1D (scalar) spring-damper element acting on single ODE2 coordinates; same as ObjectConnectorCoordinateSpringDamper but with extended features, such as limit stop and improved friction; has different user function interface and additional data node as compared to ObjectConnectorCoordinateSpringDamper, but otherwise behaves very similar. The CoordinateSpringDamperExt is very useful for a single axis of a robot or similar machine modelled with a KinematicTree, as it can add friction and limits based on physical properties. It is highly recommended, to use the bristle model for friction with frictionProportionalZone=0 in case of implicit integrators (GeneralizedAlpha) as it converges better.
 



The item \ **ObjectConnectorCoordinateSpringDamperExt**\  with type = 'ConnectorCoordinateSpringDamperExt' has the following parameters:

 

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData for 3 data coordinates (friction mode, last sticking position, limit stop state), see description for details; must exist in case of bristle friction model or limit stops
* | **stiffness** [type = Real, default = 0.]:
  | stiffness [SI:N/m] of spring; acts against relative value of coordinates
* | **damping** [type = Real, default = 0.]:
  | damping [SI:N/(m s)] of damper; acts against relative velocity of coordinates
* | **offset** [type = Real, default = 0.]:
  | offset between two coordinates (reference length of springs), see equation; it can be used to represent the pre-scribed drive coordinate
* | **velocityOffset** [type = Real, default = 0.]:
  | offset between two coordinates; used to model D-control of a drive, where damping is not acting against prescribed velocity
* | **factor0** [type = Real, default = 1.]:
  | marker 0 coordinate is multiplied with factor0
* | **factor1** [type = Real, default = 1.]:
  | marker 1 coordinate is multiplied with factor1
* | **fDynamicFriction** [type = UReal, default = 0.]:
  | dynamic (viscous) friction force [SI:N] against relative velocity when sliding; assuming a normal force \ :math:`f_N`\ , the friction force can be interpreted as \ :math:`f_\mu = \mu f_N`\ 
* | **fStaticFrictionOffset** [type = UReal, default = 0.]:
  | static (dry) friction offset force [SI:N]; assuming a normal force \ :math:`f_N`\ , the friction force is limited by \ :math:`f_\mu \le (\mu_so + \mu_d) f_N = f_\mu_d + f_\mu_so`\ 
* | **stickingStiffness** [type = UReal, default = 0.]:
  | stiffness of bristles in sticking case  [SI:N/m]
* | **stickingDamping** [type = UReal, default = 0.]:
  | damping of bristles in sticking case  [SI:N/(m/s)]
* | **exponentialDecayStatic** [type = PReal, default = 1.e-3]:
  | relative velocity for exponential decay of static friction offset force [SI:m/s] against relative velocity; at \ :math:`\Delta v = v_\mathrmexp`\ , the static friction offset force is reduced to 36.8\%
* | **fViscousFriction** [type = Real, default = 0.]:
  | viscous friction force part [SI:N/(m s)], acting against relative velocity in sliding case
* | **frictionProportionalZone** [type = UReal, default = 0.]:
  | if non-zero, a regularized Stribeck model is used, regularizing friction force around zero velocity - leading to zero friction force in case of zero velocity; this does not require a data node at all; if zero, the bristle model is used, which requires a data node which contains previous friction state and last sticking position
* | **limitStopsUpper** [type = Real, default = 0.]:
  | upper (maximum) value [SI:m] of coordinate before limit is activated; defined relative to the two marker coordinates
* | **limitStopsLower** [type = Real, default = 0.]:
  | lower (minimum) value [SI:m] of coordinate before limit is activated; defined relative to the two marker coordinates
* | **limitStopsStiffness** [type = UReal, default = 0.]:
  | stiffness [SI:N/m] of limit stop (contact stiffness); following a linear contact model
* | **limitStopsDamping** [type = UReal, default = 0.]:
  | damping [SI:N/(m/s)] of limit stop (contact damping); following a linear contact model
* | **useLimitStops** [type = bool, default = False]:
  | if True, limit stops are considered and parameters must be set accordingly; furthermore, the NodeGenericData must have 3 data coordinates
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **springForceUserFunction** [type = PyFunctionMbsScalarIndexScalar11, default =  0]:
  | A Python function which defines the spring force with 8 parameters, see equations section / see description below



The item VObjectConnectorCoordinateSpringDamperExt has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


