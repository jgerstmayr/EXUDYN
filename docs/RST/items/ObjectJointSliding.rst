

.. _sec-item-objectjointsliding:

ObjectJointSliding
==================

A specialized 3D sliding joint between a list of beam elements (updated marker1) and a position-based marker (marker0); the data coordinate x[0] provides the current index in slidingMarkerNumbers, and x[1] the local position in the cable element at the beginning of the timestep.

\ **Additional information for ObjectJointSliding**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested \ ``Marker``\  type = \ ``_None``\ 
* | Requested \ ``Node``\  type = \ ``GenericData``\ 
* | \ **Short name**\  for Python = \ ``SlidingJoint``\ 
* | \ **Short name**\  for Python visualization object = \ ``VSlidingJoint``\ 


The item \ **ObjectJointSliding**\  with type = 'JointSliding' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | marker m0: position or rigid body marker of mass point or rigid body; marker m1: updated marker to Cable2D element, where the sliding joint currently is attached to; must be initialized with an appropriate (global) marker number according to the starting position of the sliding object; this marker changes with time (PostNewtonStep)
* | **slidingMarkerNumbers** [\ :math:`[m_{s0}, \ldots, m_{sn}]\tp`\ , type = ArrayMarkerIndex, default = []]:
  | these markers are used to update marker m1, if the sliding position exceeds the current cable's range; the markers must be sorted such that marker \ :math:`m_{si}`\  at x=cable(i).length is equal to marker(i+1) at x=0 of cable(i+1)
* | **slidingMarkerOffsets** [\ :math:`[d_{s0}, \ldots, d_{sn}]`\ , type = Vector, default = []]:
  | this list contains the offsets of every sliding object (given by slidingMarkerNumbers) w.r.t. to the initial position (0): marker m0: offset=0, marker m1: offset=Length(cable0), marker m2: offset=Length(cable0)+Length(cable1), ...
* | **nodeNumber** [\ :math:`n_{GD}`\ , type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData for 1 dataCoordinate showing the according marker number which is currently active and the start-of-step (global) sliding position
* | **constrainRotations** [type = ArrayIndex, size = 3, default = [1,1,1]]:
  | flags for constrained rotation about x, y and z-axis: if flag=1, add constraint on rotation of marker m0 relative to respective axis; flag=0: sliding body can rotate freely about this axis; for ANCFCable, rotation about x-axis cannot be constrained
* | **constrainTranslations** [type = ArrayIndex, size = 3, default = [1,1,1]]:
  | flags for constrained translation in x, y and z-direction: if flag=1, add constraint on translation of marker m0 relative to respective axis; flag=0: sliding body can translate freely about this axis; along x-axis this should be usually 0, except for driven motion
* | **axialForce** [\ :math:`f_\mathrm{ax}`\ , type = Real, default = 0]:
  | ONLY APPLIES if classicalFormulation==True; axialForce represents an additional sliding force acting between beam and marker m0 body in axial (beam) direction; this force can be used to drive a body on a beam, but can only be changed with user functions.
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectJointSliding]:
  | parameters for visualization of item



The item VObjectJointSliding has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawSize** [type = float, default = -1.]:
  | drawing size = radius of revolute joint; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectjointsliding:

DESCRIPTION of ObjectJointSliding
---------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : 
  | position vector of joint given by marker0
* | ``Velocity``\ : 
  | velocity vector of joint given by marker0
* | ``SlidingCoordinate``\ : 
  | global sliding coordinate along all elements; the maximum sliding coordinate is equivalent to the reference lengths of all sliding elements
* | ``Force``\ : 
  | joint force vector (3D)



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | data node
     - | \ :math:`{\mathbf{x}}=[x_{data0},\,x_{data1}]\tp`\ 
     - | coordinates of node with node number \ :math:`n_{GD}`\ 
   * - | data coordinate 0
     - | \ :math:`x_{data0}`\ 
     - | the current index in slidingMarkerNumbers
   * - | data coordinate 1
     - | \ :math:`x_{data1}`\ 
     - | the global sliding coordinate (ranging from 0 to the total length of all sliding elements) at \ **start-of-step**\  - beginning of the timestep
   * - | marker m0 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
     - | current global position which is provided by marker m0
   * - | marker m0 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity which is provided by marker m0
   * - | marker m0 orientation
     - | \ :math:`\LU{0,m0}{\Rot}`\ 
     - | current rotation matrix provided by marker m0 (assumed to be rigid body)
   * - | marker m0 angular velocity
     - | \ :math:`\LU{0}{\tomega}_{m0}`\ 
     - | current angular velocity vector provided by marker m0 (assumed to be rigid body)
   * - | sliding position
     - | \ :math:`\LUR{0}{{\mathbf{r}}}{ANCF} = {\mathbf{S}}(s_{el}){\mathbf{q}}_{ANCF,m1}`\ 
     - | current global position at the ANCF cable element, evaluated at local sliding position \ :math:`s_{el}`\ 
   * - | sliding position slope
     - | \ :math:`\LURU{0}{{\mathbf{r}}}{ANCF}{\prime} = {\mathbf{S}}^\prime(s_{el}){\mathbf{q}}_{ANCF,m1} = [r^\prime_0,\,r^\prime_1,\,r^\prime_2]\tp`\ 
     - | current global slope vector of the ANCF cable element, evaluated at local sliding position \ :math:`s_{el}`\ 
   * - | sliding velocity
     - | \ :math:`\LUR{0}{{\mathbf{v}}}{ANCF} = {\mathbf{S}}(s_{el})\dot{\mathbf{q}}_{ANCF,m1}`\ 
     - | current global velocity at the ANCF cable element, evaluated at local sliding position \ :math:`s_{el}`\  (\ :math:`s_{el}`\  not differentiated!!!)
   * - | sliding velocity slope
     - | \ :math:`\LURU{0}{{\mathbf{v}}}{ANCF}{\prime} = {\mathbf{S}}^\prime(s_{el})\dot{\mathbf{q}}_{ANCF,m1}`\ 
     - | current global slope velocity vector of the ANCF cable element, evaluated at local sliding position \ :math:`s_{el}`\ 
   * - | algebraic coordinates
     - | \ :math:`{\mathbf{z}}=[\lambda_0,\,\ldots,\,\lambda_5,\, s]\tp`\ 
     - | algebraic coordinates composed of 3 Lagrange multipliers for forces \ :math:`\lambda_{0..2}`\ , 3 multipliers for torques \ :math:`\lambda_{3..5}`\  and the current sliding coordinate \ :math:`s`\ , which is local in the current cable element. 
   * - | local sliding coordinate
     - | \ :math:`s`\ 
     - | local incremental sliding coordinate \ :math:`s`\ : the (algebraic) sliding coordinate \ **relative to the start-of-step value**\ . Thus, \ :math:`s`\  only contains small local increments.


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | output variables
     - | symbol
     - | formula
   * - | Position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
     - | current global position of position marker \ :math:`m0`\ 
   * - | Velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity of position marker \ :math:`m0`\ 
   * - | SlidingCoordinate
     - | \ :math:`s_g = s + x_{data1}`\ 
     - | current value of the global sliding coordinate
   * - | Force
     - | \ :math:`{\mathbf{f}}`\ 
     - | see below



Geometric relations
-------------------

Assume we have given the sliding coordinate \ :math:`s`\  (e.g., as a guess of the Newton method or beginning of the time step). 
The element sliding coordinate (in the local coordinates of the current sliding element) is computed as

.. math::

   s_{el} = s + x_{data1} - d_{m1} = s_g - d_{m1}.


The vector (=difference; error) between the marker \ :math:`m0`\  and the marker \ :math:`m1`\  (=\ :math:`{\mathbf{r}}_{ANCF}`\ ) positions reads

.. math::

   \LU{0}{\Delta{\mathbf{p}}} = \LUR{0}{{\mathbf{r}}}{ANCF} - \LU{0}{{\mathbf{p}}}_{m0}


The vector (=difference; error) between the marker \ :math:`m0`\  and the marker \ :math:`m1`\  velocities reads

.. math::

   \LU{0}{\Delta{\mathbf{v}}} = \LUR{0}{\dot{\mathbf{r}}}{ANCF} - \LU{0}{{\mathbf{v}}}_{m0}



Connector constraint equations
------------------------------
 
The 3D sliding joint is implemented having 7 equations, using the special algebraic coordinates \ :math:`{\mathbf{z}}`\ .
The algebraic equations read

.. math::

   \LU{0}{\Delta{\mathbf{p}}} \!&=&\! \Null, \quad \mbox{... three index 3 eqs $\ra$ sliding body stays on cable}\\
   \vr{\lambda_1}{\lambda_2}{\lambda_3} \cdot  \LURU{0}{{\mathbf{r}}}{ANCF}{\prime} - |\LURU{0}{{\mathbf{r}}}{ANCF}{\prime}| \cdot f_\mathrm{ax} \!&=&\! 0, \quad \mbox{... three index 1 equ. $\ra$ force in sliding dir.=$f_\mathrm{ax}$}  \\



No index 2 case exists, because no time derivative exists for \ :math:`s_{el}`\ . The jacobian matrices for algebraic and \ :ref:`ODE2 <ODE2>`\  coordinates read
if \ ``activeConnector = False``\ , the algebraic equations are changed to:

.. math::

   \lambda_0 &=& 0,   \\
   \ldots && ,   \\
   \lambda_5 &=& 0,   \\
   s &=& 0


In case that \ ``constrainRotations=[0,0,0]``\ , the Lagrange multipliers for rotations are set

.. math::

   \left[\lambda_3,\lambda_4,\lambda_5\right] = \Null


In case that any flag in \ ``constrainRotations``\  is equal to 1, the constraints read 
for \ ``constrainRotations[0] = 1``\ :

.. math::

   \LURU{0}{{\mathbf{r}}}{ANCF,y}{\mathrm{T}} \LU{0,m0}{\Rot} \vr{0}{0}{1} = 0


for \ ``constrainRotations[1] = 1``\ :

.. math::

   \LURU{0}{{\mathbf{r}}}{ANCF}{\prime\mathrm{T}} \LU{0,m0}{\Rot} \vr{0}{0}{1} = 0


for \ ``constrainRotations[2] = 0``\ :

.. math::

   \LURU{0}{{\mathbf{r}}}{ANCF}{\prime\mathrm{T}} \LU{0,m0}{\Rot} \vr{0}{1}{0} = 0




Post Newton Step
----------------

After the Newton solver has converged, a PostNewtonStep is performed for the element, which
updates the marker \ :math:`m1`\  index if necessary.

.. math::

   s_{el} < 0 \quad \ra \quad x_{data0}\;-\!\!=1 \nonumber\\
   s_{el} > L \quad \ra \quad x_{data0}\;+\!\!=1


Furthermore, it is checked, if \ :math:`x_{data0}`\  becomes smaller than zero, which raises a warning and keeps \ :math:`x_{data0}=0`\ .
The same results if \ :math:`x_{data0}\ge sn`\ , then \ :math:`x_{data0} = sn`\ .
Finally, the data coordinate is updated in order to provide the starting value for the next step,

.. math::

   x_{data1} \;+\!\!= s.




Relevant Examples and TestModels with weblink:

    \ `ANCFslidingJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint.py>`_\  (Examples/), \ `NGsolveFFRFSlidingJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveFFRFSlidingJoint.py>`_\  (Examples/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


