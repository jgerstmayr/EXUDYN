

.. _sec-item-objectjointalemoving2d:

ObjectJointALEMoving2D
======================

A specialized axially moving joint (without rotation) in 2D between a ALE Cable2D (marker1) and a position-based marker (marker0); ALE=Arbitrary Lagrangian Eulerian; the data coordinate x[0] provides the current index in slidingMarkerNumbers, and the \ :ref:`ODE2 <ODE2>`\  coordinate q[0] provides the (given) moving coordinate in the cable element.

\ **Additional information for ObjectJointALEMoving2D**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested \ ``Marker``\  type = \ ``_None``\ 
* | Requested \ ``Node``\  type: read detailed information of item
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
* | **slidingOffset** [\ :math:`s_\mathrm{off}`\ , type = Real, default = 0.]:
  | sliding offset [SI:m]: a scalar offset, which represents the (reference arc) length of all previous sliding cable elements
* | **nodeNumbers** [\ :math:`[n_{GD}, n_{ALE}]`\ , type = ArrayNodeIndex, default = [ invalid [-1], invalid [-1] ]]:
  | node number of NodeGenericData (GD) with one data coordinate and of NodeGenericODE2 (ALE) with one \ :ref:`ODE2 <ODE2>`\  coordinate
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


----------

.. _description-objectjointalemoving2d:

DESCRIPTION of ObjectJointALEMoving2D
-------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
  | current global position of position marker \ :math:`m0`\ 
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
  | current global velocity of position marker \ :math:`m0`\ 
* | ``SlidingCoordinate``\ : \ :math:`s_g = q_{ALE} + s_\mathrm{off}`\ 
  | current value of the global sliding ALE coordinate, including offset; note that reference coordinate of \ :math:`q_{ALE}`\  is ignored!
* | ``Coordinates``\ : \ :math:`[x_{data0},\,q_{ALE}]\tp`\ 
  | provides two values: [0] = current sliding marker index, [1] = ALE sliding coordinate
* | ``Coordinates\_t``\ : \ :math:`[\dot q_{ALE}]\tp`\ 
  | provides ALE sliding velocity
* | ``Force``\ : \ :math:`{\mathbf{f}}`\ 
  | joint force vector (3D)



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | generic data node
     - | \ :math:`{\mathbf{x}}=[x_{data0}]\tp`\ 
     - | coordinates of node with node number \ :math:`n_{GD}`\ 
   * - | generic \ :ref:`ODE2 <ODE2>`\  node
     - | \ :math:`{\mathbf{q}}=[q_{0}]\tp`\ 
     - | coordinates of node with node number \ :math:`n_{ALE}`\ , which is shared with all ALE-ANCF and ALE sliding joint objects
   * - | data coordinate
     - | \ :math:`x_{data0}`\ 
     - | the current index in slidingMarkerNumbers
   * - | ALE coordinate
     - | \ :math:`q_{ALE} = q_{0}`\ 
     - | current ALE coordinate (in fact this is the Eulerian coordinate in the ALE formulation); note that reference coordinate of \ :math:`q_{ALE}`\  is ignored!
   * - | marker m0 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
     - | current global position which is provided by marker m0
   * - | marker m0 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity which is provided by marker m0
   * - | cable coordinates
     - | \ :math:`{\mathbf{q}}_{ANCF,m1}`\ 
     - | current coordiantes of the ANCF cable element with the current marker \ :math:`m1`\  is referring to
   * - | sliding position
     - | \ :math:`\LUR{0}{{\mathbf{r}}}{ANCF} = {\mathbf{S}}(s_{el}){\mathbf{q}}_{ANCF,m1}`\ 
     - | current global position at the ANCF cable element, evaluated at local sliding position \ :math:`s_{el}`\ 
   * - | sliding position slope
     - | \ :math:`\LURU{0}{{\mathbf{r}}}{ANCF}{\prime} = {\mathbf{S}}^\prime(s_{el}){\mathbf{q}}_{ANCF,m1}`\ 
     - | current global slope vector of the ANCF cable element, evaluated at local sliding position \ :math:`s_{el}`\ 
   * - | sliding velocity
     - | \ :math:`\LUR{0}{{\mathbf{v}}}{ANCF} = {\mathbf{S}}(s_{el})\dot{\mathbf{q}}_{ANCF,m1} + \dot q_{ALE} \LURU{0}{{\mathbf{r}}}{ANCF}{\prime}`\ 
     - | current global velocity at the ANCF cable element, evaluated at local sliding position \ :math:`s_{el}`\ , including convective term
   * - | sliding normal vector
     - | \ :math:`\LU{0}{{\mathbf{n}}} = [-r^\prime_1,\,r^\prime_0]`\ 
     - | 2D normal vector computed from slope \ :math:`{\mathbf{r}}^\prime=\LURU{0}{{\mathbf{r}}}{ANCF}{\prime}`\ 
   * - | algebraic variables
     - | \ :math:`{\mathbf{z}}=[\lambda_0,\,\lambda_1]\tp`\ 
     - | algebraic variables (Lagrange multipliers) according to the algebraic equations 



Geometric relations
-------------------

The element sliding coordinate (in the local coordinates of the current sliding element) is computed from the ALE coordinate

.. math::

   s_{el} = q_{ALE} + s_\mathrm{off} - d_{m1} = s_g - d_{m1}.


For the description of the according quantities, see the description above. The distance \ :math:`d_{m1}`\  is obtained from the \ ``slidingMarkerOffsets``\  list, using the current (local) index \ :math:`x_{data0}`\ .
The vector (=difference; error) between the marker \ :math:`m0`\  and the marker \ :math:`m1`\  (=\ :math:`{\mathbf{r}}_{ANCF}`\ ) positions reads

.. math::

   \LU{0}{\Delta{\mathbf{p}}} = \LUR{0}{{\mathbf{r}}}{ANCF} - \LU{0}{{\mathbf{p}}}_{m0}


Note that \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\  represents the current position of the marker \ :math:`m0`\ , which could represent the midpoint of a mass sliding along the beam.
The position \ :math:`\LUR{0}{{\mathbf{r}}}{ANCF}`\  is computed from the beam represented by marker \ :math:`m1`\ , using the local beam coordinate \ :math:`x=s_{el}`\ . 
The marker and the according beam finite element changes during movement using the list \ ``slidingMarkerNumbers``\  and the index is updated in the PostNewtonStep.
The vector (=difference; error) between the marker \ :math:`m0`\  and the marker \ :math:`m1`\  velocities reads

.. math::

   \LU{0}{\Delta{\mathbf{v}}} = \LUR{0}{{\mathbf{v}}}{ANCF} - \LU{0}{{\mathbf{v}}}_{m0}




Connector constraint equations
------------------------------

The 2D sliding joint is implemented having 2 equations, using the Lagrange multipliers \ :math:`{\mathbf{z}}`\ . 
The algebraic (index 3) equations read

.. math::

   \LU{0}{\Delta{\mathbf{p}}} = 0


Note that the Lagrange multipliers \ :math:`[\lambda_0,\,\lambda_1]\tp`\ are the global forces in the joint.
In the index 2 case the algebraic equations read

.. math::

   \LU{0}{\Delta{\mathbf{v}}} = 0


If \ ``usePenalty = True``\ , the algebraic equations are changed to:

.. math::

   \LU{0}{\Delta {\mathbf{p}}} - \frac 1 k {\mathbf{z}} = 0.



If \ ``activeConnector = False``\ , the algebraic equations are changed to:

.. math::

   \lambda_0 &=& 0,   \\
   \lambda_1 &=& 0.

   

Post Newton Step
----------------

After the Newton solver has converged, a PostNewtonStep is performed for the element, which
updates the marker \ :math:`m1`\  index if necessary.

.. math::

   s_{el} < 0 \quad \ra \quad x_{data0} \;-\!\!=1 \nonumber\\
   s_{el} > L \quad \ra \quad x_{data0} \;+\!\!=1


Furthermore, it is checked, if \ :math:`x_{data0}`\  becomes smaller than zero, which raises a warning and keeps \ :math:`x_{data0}=0`\ .
The same results if \ :math:`x_{data0}\ge sn`\ , then \ :math:`x_{data0} = sn`\ .
Finally, the data coordinate is updated in order to provide the starting value for the next step,

.. math::

   x_{data1} \;+\!\!= s.




Relevant Examples and TestModels with weblink:

    \ `ANCFmovingRigidbody.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFmovingRigidbody.py>`_\  (Examples/), \ `ANCFmovingRigidBodyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFmovingRigidBodyTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


