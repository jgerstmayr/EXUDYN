

.. _sec-item-objectconnectorreevingsystemsprings:

ObjectConnectorReevingSystemSprings
===================================

A rD reeving system defined by a list of torque-free and friction-free sheaves or points that are connected with one rope (modelled as massless spring). NOTE that the spring can undergo tension AND compression (in order to avoid compression, use a PreStepUserFunction to turn off stiffness and damping in this case!). The force is assumed to be constant all over the rope. The sheaves or connection points are defined by \ :math:`nr`\  rigid body markers \ :math:`[m_0, \, m_1, \, \ldots, \, m_{nr-1}]`\ . At both ends of the rope there may be a prescribed motion coupled to a coordinate marker each, given by \ :math:`m_{c0}`\  and \ :math:`m_{c1}`\  .

\ **Additional information for ObjectConnectorReevingSystemSprings**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``_None``\ 
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
* | **regularizationForce** [\ :math:`F_{reg}`\ , type = Real, default = 0.1]:
  | small regularization force [SI:N] in order to avoid large compressive forces; this regularization force can either be \ :math:`<0`\  (using a linear tension/compression spring model) or \ :math:`>0`\ , which restricts forces in the rope to be always \ :math:`\ge -F_{reg}`\ . Note that smaller forces lead to problems in implicit integrators and smaller time steps. For explicit integrators, this force can be chosen close to zero.
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


----------

.. _description-objectconnectorreevingsystemsprings:

DESCRIPTION of ObjectConnectorReevingSystemSprings
--------------------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Distance``\ : \ :math:`L`\ 
  | current total length of rope
* | ``VelocityLocal``\ : \ :math:`\dot L`\ 
  | scalar time derivative of current total length of rope
* | ``ForceLocal``\ : \ :math:`F`\ 
  | scalar force in reeving system (constant over length of rope)



General model assumptions
-------------------------

The \ ``ConnectorReevingSystemSprings``\  model is based on a linear elastic, visco-elastic, and mass-less spring which
is tangent to a list of rolls. The contact with the rolls is friction-less, causing no torque w.r.t.\ the rolling axis of the sheave.
The force in the rope results from the difference of the total length \ :math:`L`\  compared to the reference length or the rope, which 
may be changed by adding or subtracting rope length at the end points. All geometric operations are performed in 3D, allowing to model
simple reeving systems in 3D.



.. _fig-reevingsystemsprings-tangents:
.. figure:: ../../theDoc/figures/CommonTangents3D.png
   :width: 500

   Geometry of common tangent for two spatial circles defined by radii \ :math:`R_A`\  and \ :math:`R_B`\  as well as by the normalized axis vectors \ :math:`{\mathbf{a}}_A`\  and \ :math:`{\mathbf{a}}_B`\ . The tangent is undefined, if one of the axis vectors is parallel to the vector \ :math:`{\mathbf{c}}`\ , which connects the two center points. The positive rotation sense is indicated by means of the angular velocities \ :math:`\omega_A`\  and \ :math:`\omega_B`\ .


Common tangent of two circles in 3D
-----------------------------------

In order to compute the total length of the rope of the reeving system, the tangent of two arbitrary circles in space needs to be computed.
Considering \ :numref:`fig-reevingsystemsprings-tangents`\ , the relations are based on the
center points of the circles \ :math:`{\mathbf{p}}_A`\  and \ :math:`{\mathbf{p}}_B`\ , the radii \ :math:`R_A`\  and \ :math:`R_B`\  as well as
the axis vectors \ :math:`{\mathbf{a}}_A`\  and \ :math:`{\mathbf{a}}_B`\ , the latter vectors also defining the side at which the tangent contacts.
For the definition of the tangent, the vectors \ :math:`{\mathbf{r}}_A`\  and \ :math:`{\mathbf{r}}_B`\  need to be computed.

For the special case of \ :math:`R_A=R_B=0`\ , it follows that \ :math:`{\mathbf{r}}_A={\mathbf{p}}_A`\  and \ :math:`{\mathbf{r}}_B={\mathbf{p}}_B`\ .
Otherwise, we first compute the vector between circle centers,

.. math::

   {\mathbf{c}} = {\mathbf{p}}_B - {\mathbf{p}}_A, \quad \mathrm{and} \quad {\mathbf{c}}_0 = \frac{{\mathbf{c}}}{|{\mathbf{c}}|} ,


and obtain the tangent vectors

.. math::

   {\mathbf{t}}_A = {\mathbf{t}}_B = {\mathbf{c}}_0 ,


as well as the normal vectors

.. math::

   {\mathbf{n}}_A = {\mathbf{a}}_A \times {\mathbf{c}}_0, \quad \mathrm{and} \quad {\mathbf{n}}_B = {\mathbf{a}}_B \times {\mathbf{c}}_0 .


Note that the orientation of the axis vectors \ :math:`{\mathbf{a}}_A`\  and \ :math:`{\mathbf{a}}_B`\  defines the orientation of the normals.
By definition, we assume the following conditions,

.. math::

   {\mathbf{n}}_A\tp {\mathbf{r}}_A < 0, \quad \mathrm{and} \quad {\mathbf{n}}_B\tp {\mathbf{r}}_B < 0 .


For two circles with equal radius and axes orientations, the angles result in \ :math:`\varphi_A=\varphi_B=\pi`\ .
In general, the unknown vectors \ :math:`{\mathbf{r}}_A`\  and \ :math:`{\mathbf{r}}_B`\  are computed by means of Newton's method.
The unknown tangent vector is given as 

.. math::

   {\mathbf{t}}_c = {\mathbf{p}}_B + {\mathbf{r}}_B - {\mathbf{p}}_A - {\mathbf{r}}_A = {\mathbf{c}} + {\mathbf{r}}_B - {\mathbf{r}}_A .


We now parameterize the two unknown vectors by means of unknown angles \ :math:`\varphi_A`\  and \ :math:`\varphi_B`\ ,

.. math::

   {\mathbf{r}}_A = -R_A \left( \cos(\varphi_A) {\mathbf{t}}_A - \sin(\varphi_A) {\mathbf{n}}_A \right), \quad \mathrm{and} \quad {\mathbf{r}}_B = -R_B \left( \cos(\varphi_B) {\mathbf{t}}_B - \sin(\varphi_B) {\mathbf{n}}_B \right) .


As vectors \ :math:`{\mathbf{r}}_A`\  and \ :math:`{\mathbf{r}}_B`\  must be perpendicular to \ :math:`{\mathbf{t}}_c`\ , it follows that

.. math::

   {\mathbf{r}}_A\tp ({\mathbf{c}} + {\mathbf{r}}_B - {\mathbf{r}}_A) = 0, \quad \mathrm{and} \quad {\mathbf{r}}_B\tp ({\mathbf{c}} + {\mathbf{r}}_B - {\mathbf{r}}_A) = 0,


or

.. math::
   :label: eq-reevingsystemsprings-newton

   {\mathbf{r}}_A\tp {\mathbf{c}} + {\mathbf{r}}_A\tp {\mathbf{r}}_B - R_A^2 = 0, \quad \mathrm{and} \quad {\mathbf{r}}_B\tp {\mathbf{c}} - {\mathbf{r}}_B\tp{\mathbf{r}}_A + R_B^2 = 0 .


The relations Eq. :eq:`eq-reevingsystemsprings-newton`\  reduce to only one equation, if either \ :math:`R_A=0`\  or \ :math:`R_B = 0`\ .
The equations can be solved by Newton's method by computing the jacobian of \ :math:`{\mathbf{J}}_{CT}`\  of Eq. :eq:`eq-reevingsystemsprings-newton`\  w.r.t.\ the 
unknown angles \ :math:`\varphi_A`\  and \ :math:`\varphi_B`\ . The iterations are started with

.. math::

   \varphi_A = \pi \quad \mathrm{and} \quad \varphi_B = \pi,


and iterate until the error is below a certain tolerance, for details see the implementation in \ ``Geometry.h``\ .


Connector forces
----------------

The current rope length results from the configuration of sheaves, including start and end position:

.. math::

   L = d_{m_0-m_1} + C_{m_1} + d_{m_1-m_2} + C_{m_2} + \ldots  + d_{m_{nr-2}-m_{nr-1}}


in which \ :math:`d_{...}`\  represents the free spans between two sheaves as computed from the common tangent in the previous section,
and \ :math:`C_{...}`\  represents the length along the circumference of the according marker if the according radius \ :math:`r`\  is non-zero.
The quantity \ :math:`C_{...}`\  can be computed easily as soon as the radius vectors to the tangents \ :math:`{\mathbf{r}}_A`\  and \ :math:`{\mathbf{r}}_B`\ 
are known. Within a series of tangents, the previous to the current tangent will always enclose an angle between \ :math:`0`\  and \ :math:`2\cdot \pi`\ .

In case that \ ``hasCoordinateMarkers=True``\ , the total reference length and its derivative result as

.. math::

   L_0 = L_{ref} + f_0 \cdot q_{m_{c0}} + f_1 \cdot q_{m_{c1}}, \quad \dot L_0 = f_0 \cdot \dot q_{m_{c0}} + f_1 \cdot \dot q_{m_{c1}}, \quad


while we set \ :math:`L_0 = L_{ref}`\  and \ :math:`\dot L_0=0`\  otherwise.
The linear force in the reeving system (assumed to be constant all over the rope) is computed as

.. math::

   F_{lin} = (L-L_{0}) \frac{EA}{L_0} + (\dot L - \dot L_0)\frac{DA}{L_0}


The rope force is computed from

.. math::

   F =   \begin{cases} F_{lin} \quad \mathrm{if} \quad F_{lin} > 0 \\ F_{reg} \cdot \mathrm{tanh}(F_{lin}/F_{reg})\quad \mathrm{else} \end{cases}


Which allows small compressive forces \ :math:`F_{reg}`\ .
In case that \ :math:`F_{reg} < 0`\ , compressive forces are not regularized (linear spring).
The case \ :math:`F_{reg} = 0`\  will be used in future only in combination with a data node, 
which allows switching similar as in friction and contact elements.

Note that in case of \ :math:`L_0=0`\ , the term \ :math:`\frac{1}{L_0}`\  is replaced by \ :math:`1000`\ .
However, this case must be avoided by the user by choosing appropriate parameters for the system.

Additional damping may be added via the parameters \ :math:`DT`\  and \ :math:`DS`\ , which have to be treated carefully. The shearing parameter may
be helpful to damp undesired oscillatory shearing motion, however, it may also damp rigid body motion of the overall mechanism.

Further details are given in the implementation and examples are provided in the \ ``Examples``\  and \ ``TestModels``\  folders.


Relevant Examples and TestModels with weblink:

    \ `craneReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/craneReevingSystem.py>`_\  (Examples/), \ `reevingSystemSpringsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/reevingSystemSpringsTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


