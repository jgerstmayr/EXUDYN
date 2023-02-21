

.. _sec-item-objectjointalemoving2d:

ObjectJointALEMoving2D
======================

A specialized axially moving joint (without rotation) in 2D between a ALE Cable2D (marker1) and a position-based marker (marker0); ALE=Arbitrary Lagrangian Eulerian; the data coordinate x[0] provides the current index in slidingMarkerNumbers, and the ODE2 coordinate q[0] provides the (given) moving coordinate in the cable element.

\ **Additional information for ObjectJointALEMoving2D**\ :

* | The Object has the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested marker type = \ ``_None``\ 
* | Requested node type: read detailed information of item
* | \ **Short name**\  for Python = \ ``ALEMovingJoint2D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VALEMovingJoint2D``\ 


The item \ **ObjectJointALEMoving2D**\  with type = 'JointALEMoving2D' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,\,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | marker m0: position-marker of mass point or rigid body; marker m1: updated marker to ANCF Cable2D element, where the sliding joint currently is attached to; must be initialized with an appropriate (global) marker number according to the starting position of the sliding object; this marker changes with time (PostNewtonStep)
* | **slidingMarkerNumbers** [\ :math:`[m_{s0}, \ldots, m_{sn}]\tp`\ , type = ArrayMarkerIndex, default = []]:
  | a list of sn (global) marker numbers which are are used to update marker1
* | **slidingMarkerOffsets** [\ :math:`[d_{s0}, \ldots, d_{sn}]`\ , type = Vector, default = []]:
  | this list contains the offsets of every sliding object (given by slidingMarkerNumbers) w.r.t. to the initial position (0): marker0: offset=0, marker1: offset=Length(cable0), marker2: offset=Length(cable0)+Length(cable1), ...
* | **slidingOffset** [\ :math:`s_{off}`\ , type = Real, default = 0.]:
  | sliding offset list [SI:m]: a list of sn scalar offsets, which represent the (reference arc) length of all previous sliding cable elements
* | **nodeNumbers** [\ :math:`[n_{GD}, n_{ALE}]`\ , type = ArrayNodeIndex, default = [ invalid [-1], invalid [-1] ]]:
  | node number of NodeGenericData (GD) with one data coordinate and of NodeGenericODE2 (ALE) with one ODE2 coordinate
* | **usePenaltyFormulation** [type = Bool, default = False]:
  | flag, which determines, if the connector is formulated with penalty, but still using algebraic equations (IsPenaltyConnector() still false)
* | **penaltyStiffness** [\ :math:`k`\ , type = Real, default = 0.]:
  | penalty stiffness [SI:N/m] used if usePenaltyFormulation=True
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectJointALEMoving2D]:
  | parameters for visualization of item



The item VObjectJointALEMoving2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = radius of revolute joint; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
  | current global position of position marker \ :math:`m0`\ 
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
  | current global velocity of position marker \ :math:`m0`\ 
* | ``SlidingCoordinate``\ : \ :math:`s_g = q_{ALE} + s_{off}`\ 
  | current value of the global sliding ALE coordinate, including offset; note that reference coordinate of \ :math:`q_{ALE}`\  is ignored!
* | ``Coordinates``\ : \ :math:`[x_{data0},\,q_{ALE}]\tp`\ 
  | provides two values: [0] = current sliding marker index, [1] = ALE sliding coordinate
* | ``Coordinates\_t``\ : \ :math:`[\dot q_{ALE}]\tp`\ 
  | provides ALE sliding velocity
* | ``Force``\ : \ :math:`{\mathbf{f}}`\ 
  | joint force vector (3D)




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


