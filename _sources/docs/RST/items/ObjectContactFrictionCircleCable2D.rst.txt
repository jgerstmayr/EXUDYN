

.. _sec-item-objectcontactfrictioncirclecable2d:

ObjectContactFrictionCircleCable2D
==================================

A very specialized penalty-based contact/friction condition between a 2D circle in the local x/y plane (=marker0, a RigidBody Marker, from node or object) on a body and an ANCFCable2DShape (=marker1, Marker: BodyCable2DShape), in xy-plane; a node NodeGenericData is required with 3\ :math:`\times`\ (number of contact segments) -- containing per segment: [contact gap, stick/slip (stick=0, slip=+-1, undefined=-2), last friction position]. The connector works with Cable2D and ALECable2D, HOWEVER, due to conceptual differences the (tangential) frictionStiffness cannot be used with ALECable2D; if using, it gives wrong tangential stresses, even though it may work in general.

\ **Additional information for ObjectContactFrictionCircleCable2D**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``_None``\ 
* | Requested \ ``Node``\  type = \ ``GenericData``\ 


The item \ **ObjectContactFrictionCircleCable2D**\  with type = 'ContactFrictionCircleCable2D' has the following parameters:

* | **name** [type = String, default = '']:
  | connector's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ]]:
  | a marker \ :math:`m0`\  with position and orientation and a marker \ :math:`m1`\  of type BodyCable2DShape; together defining the contact geometry
* | **nodeNumber** [\ :math:`n_g`\ , type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData with 3 \ :math:`\times n_{cs}`\   dataCoordinates (used for active set strategy \ :math:`\ra`\  hold the gap of the last discontinuous iteration, friction state (+-1=slip, 0=stick, -2=undefined) and the last sticking position; initialize coordinates with list [0.1]*\ :math:`n_{cs}`\ +[-2]*\ :math:`n_{cs}`\ +[0.]*\ :math:`n_{cs}`\ , meaning that there is no initial contact with undefined slip/stick
* | **numberOfContactSegments** [\ :math:`n_{cs}`\ , type = PInt, default = 3]:
  | number of linear contact segments to determine contact; each segment is a line and is associated to a data (history) variable; must be same as in according marker
* | **contactStiffness** [\ :math:`k_c`\ , type = UReal, default = 0.]:
  | contact (penalty) stiffness [SI:N/m/(contact segment)]; the stiffness is per contact segment; specific contact forces (per length) \ :math:`f_n`\  act in contact normal direction only upon penetration
* | **contactDamping** [\ :math:`d_c`\ , type = UReal, default = 0.]:
  | contact damping [SI:N/(m s)/(contact segment)]; the damping is per contact segment; acts in contact normal direction only upon penetration
* | **frictionVelocityPenalty** [\ :math:`\mu_v`\ , type = UReal, default = 0.]:
  | tangential velocity dependent penalty coefficient for friction [SI:N/(m s)/(contact segment)]; the coefficient causes tangential (contact) forces against relative tangential velocities in the contact area
* | **frictionStiffness** [\ :math:`\mu_k`\ , type = UReal, default = 0.]:
  | tangential displacement dependent penalty/stiffness coefficient for friction [SI:N/m/(contact segment)]; the coefficient causes tangential (contact) forces against relative tangential displacements in the contact area
* | **frictionCoefficient** [\ :math:`\mu`\ , type = UReal, default = 0.]:
  | friction coefficient [SI: 1]; tangential specific friction forces (per length) \ :math:`f_t`\  must fulfill the condition \ :math:`f_t \le \mu f_n`\ 
* | **circleRadius** [\ :math:`r`\ , type = UReal, default = 0.]:
  | radius [SI:m] of contact circle
* | **useSegmentNormals** [type = Bool, default = True]:
  | True: use normal and tangent according to linear segment; this is appropriate for very long (compared to circle) segments; False: use normals at segment points according to vector to circle center; this is more consistent for short segments, as forces are only applied in beam tangent and normal direction
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectContactFrictionCircleCable2D]:
  | parameters for visualization of item



The item VObjectContactFrictionCircleCable2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set True, if item is shown in visualization and false if it is not shown; note that only normal contact forces can be  drawn, which are approximated by \ :math:`k_c \cdot g`\  (neglecting damping term)
* | **showContactCircle** [type = Bool, default = True]:
  | if True and show=True, the underlying contact circle is shown; uses circleTiling*4 for tiling (from VisualizationSettings.general)
* | **drawSize** [type = float, default = -1.]:
  | drawing size = diameter of spring; size == -1.f means that default connector size is used
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectcontactfrictioncirclecable2d:

DESCRIPTION of ObjectContactFrictionCircleCable2D
-------------------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Coordinates``\ : \ :math:`[u_{t,0},\, g_0,\, u_{t,1},\, g_1,\, \ldots,\, u_{t,n_{cs}},\, g_{n_{cs}}]\tp`\ 
  | local (relative) displacement in tangential (\ :math:`{\mathbf{t}}`\ ) and normal (\ :math:`{\mathbf{n}}`\ ) direction per segment (\ :math:`n_{cs}`\ ); values are only provided in case of contact, otherwise zero; tangential displacement is only non-zero in case of sticking!
* | ``Coordinates\_t``\ : \ :math:`[v_{t,0},\, v_{n,0},\, v_{t,1},\, v_{n,1},\, \ldots,\, v_{t,n_{cs}},\, v_{n,n_{cs}}]\tp`\ 
  | local (relative) velocity in tangential (\ :math:`{\mathbf{t}}`\ ) and normal (\ :math:`{\mathbf{n}}`\ ) direction per segment (\ :math:`n_{cs}`\ ); values are only provided in case of contact, otherwise zero
* | ``ForceLocal``\ : \ :math:`[f_{t,0},\, f_{n,0},\, f_{t,1},\, f_{n,1},\, \ldots,\, f_{t,n_{cs}},\, f_{n,n_{cs}}]\tp`\ 
  | local contact forces in tangential (\ :math:`{\mathbf{t}}`\ ) and normal (\ :math:`{\mathbf{n}}`\ ) direction per segment (\ :math:`n_{cs}`\ )



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | marker m0 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
     - | represents current global position of the circle's centerpoint
   * - | marker m0 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity which is provided by marker m0
   * - | marker m1
     - | 
     - | represents the 2D ANCF cable
   * - | data node
     - | \ :math:`{\mathbf{x}}=[x_{i},\; \ldots,\; x_{3 n_{cs} -1}]\tp`\ 
     - | coordinates of node with node number \ :math:`n_{GD}`\ 
   * - | data coordinates for segment \ :math:`i`\ 
     - | \ :math:`[x_i,\, x_{n_{cs}+ i},\, x_{2\cdot n_{cs}+ i}]\tp = [x_{gap},\, x_{isSlipStick},\, x_{lastStick}]\tp`\ , with \ :math:`i \in [0,n_{cs}-1]`\ 
     - | 
          The data coordinates include the gap \ :math:`x_{gap}`\ , the stick-slip state \ :math:`x_{isSlipStick}`\  and the previous sticking position \ :math:`x_{lastStick}`\  as computed in the PostNewtonStep, see description below. 
   * - | shortest distance to segment \ :math:`s_i`\ 
     - | \ :math:`{\mathbf{d}}_{g,i}`\ 
     - | shortest distance of center of circle to contact segment, considering the endpoint of the segment



.. _fig-objectcontactfrictioncirclecable2d-sketch:
.. figure:: ../../theDoc/figures/ContactFrictionCircleCable2D.png
   :width: 600

   Sketch of cable, contact segments and circle; showing case without contact, \ :math:`|\mathbf{d}_{g1}| > r`\ , while contact occurs with \ :math:`|\mathbf{d}_{g1}| \le r`\ ; the shortest distance vector \ :math:`\mathbf{d}_{g1}`\  is related to segment \ :math:`s_1`\  (which is perpendicular to the the segment line) and \ :math:`\mathbf{d}_{g2}`\  is the shortest distance to the end point of segment \ :math:`s_2`\ , not being perpendicular


Connector forces: contact geometry
----------------------------------

The connector represents a force element between a 'circle' (or cylinder) represented by a marker \ :math:`m0`\ , which has position and orientation,
and an \ ``ANCFCable2D``\  beam element (denoted as 'cable') represented by a \ ``MarkerBodyCable2DShape``\  \ :math:`m1`\ .
The cable with reference length \ :math:`L`\  is discretized by splitting into \ :math:`n_{cs}`\  straight segments \ :math:`s_i`\ , located between points \ :math:`p_i`\  and \ :math:`p_{i+1}`\ .
Note that these points can be placed with an offset from the cable centerline, see \ ``verticalOffset``\  defined in \ ``MarkerBodyCable2DShape``\ .
In order to compute the gap function for a line segment, the shortest distance of one line segment with
points \ :math:`{\mathbf{p}}_i`\ , \ :math:`{\mathbf{p}}_{i+1}`\  to the circle's centerpoint given by the marker \ :math:`{\mathbf{p}}_{m0}`\  is computed. 
All computations here are performed in the global coordinates system (0), 
including edge points of every segment.

With the intermediate quantities (all of them related to segment \ :math:`s_i`\ )\ (we omit \ :math:`s_i`\  in some terms for brevity!),

.. math::

   {\mathbf{v}}_s = {\mathbf{p}}_{i+1} - {\mathbf{p}}_i, \quad {\mathbf{v}}_p = {\mathbf{p}}_{m0} - {\mathbf{p}}_i, \quad n = {\mathbf{v}}_s\tp {\mathbf{v}}_p, \quad d = {\mathbf{v}}_s\tp {\mathbf{v}}_s


and assuming that \ :math:`d \neq 0`\  (otherwise the two segment points would be identical and
the shortest distance would be \ :math:`d_g = |{\mathbf{v}}_p|`\ ),
we find the relative position \ :math:`\rho`\  of the shortest (projected) point on the 
segment, which runs from 0 to 1 if lying on the segment, as

.. math::

   \rho = \frac{n}{d}


We distinguish 3 cases (see also \ :numref:`fig-objectcontactfrictioncirclecable2d-sketch`\  for cases 1 and 2):
    
+  If \ :math:`\rho \le 0`\ , the shortest distance would be the distance to point \ :math:`{\mathbf{p}}_p={\mathbf{p}}_i`\ ,
    reading 
    
.. math::

   d_g = |{\mathbf{p}}_{m0} - {\mathbf{p}}_i| \quad (\rho \le 0)


+  If \ :math:`\rho \ge 1`\ , the shortest distance would be the distance to point \ :math:`{\mathbf{p}}_p={\mathbf{p}}_{i+1}`\ ,
    reading 
    
.. math::

   d_g = |{\mathbf{p}}_{m0} - {\mathbf{p}}_{i+1}| \quad (\rho \ge 1)


+  Finally, if \ :math:`0 < \rho < 1`\ , then the shortest distance has a projected point somewhere
    on the segment with the point (projected on the segment)
    
.. math::

   {\mathbf{p}}_p = {\mathbf{p}}_i + \rho \cdot {\mathbf{v}}_s


    and the distance
    
.. math::

   d_g = |{\mathbf{d}}_g| = \sqrt{{\mathbf{v}}_p\tp {\mathbf{v}}_p - (n^2)/d}



Here, the shortest distance vector for every segment results from the projected point \ :math:`{\mathbf{p}}_p`\  
of the above mentioned cases, see also \ :numref:`fig-objectcontactfrictioncirclecable2d-sketch`\ ,
with the relation

.. math::

   {\mathbf{d}}_g = {\mathbf{d}}_{g,s_i}= {\mathbf{p}}_{m0} - {\mathbf{p}}_p .


The contact gap for a specific point for segment \ :math:`s_i`\  is in general defined as

.. math::
   :label: objectcontactfrictioncirclecable2d-gap

   g = g_{s_i} = d_g - r .


using \ :math:`d_g = |{\mathbf{d}}_g|`\ .


Contact frame and relative motion
---------------------------------

Irrespective of the choice of \ ``useSegmentNormals``\ , the contact normal vector \ :math:`{\mathbf{n}}_{s_i}`\  and tangential vector \ :math:`{\mathbf{t}}_{s_i}`\  are defined per segment as

.. math::

   {\mathbf{n}}_{s_i} = {\mathbf{n}} = [n_0, n_1]\tp = \frac{1}{|{\mathbf{d}}_{g,s_i}|} {\mathbf{d}}_{g,s_i}, \quad {\mathbf{t}}_{s_i} = {\mathbf{t}} = [-n_1, n_0]\tp


The vectors \ :math:`{\mathbf{t}}_{s_i}`\  and \ :math:`{\mathbf{n}}_{s_i}`\  define the local (contact) frame for further computations.

The velocity at the closest point of the segment \ :math:`s_i`\  is interpolated using \ :math:`\rho`\  and computed as

.. math::

   \dot {\mathbf{p}}_p = (1-\rho) \cdot {\mathbf{v}}_i + \rho \cdot {\mathbf{v}}_{i+1}


Alternatively, \ :math:`\dot {\mathbf{p}}_p`\  could be computed from the cable element by evaluating the velocity at the contact points, but we feel that
this choice is more consistent with the computations at position level.

The gap velocity \ :math:`v_n`\  (\ :math:`\neq \dot g`\ ) thus reads

.. math::

   v_n = \left( \dot {\mathbf{p}}_p - \dot {\mathbf{p}}_{m0} \right) {\mathbf{n}}


In a similar, the tangential velocity reads

.. math::
   :label: objectcontactfrictioncirclecable2d-vtangent

   v_t = \left( \dot {\mathbf{p}}_p - \dot {\mathbf{p}}_{m0} \right) {\mathbf{t}}


In case of \ ``frictionStiffness != 0``\ , we continuously track the sticking position at which the cable element (or segment) and the circle 
previously sticked together, similar as proposed by Lugr{\'i}s et al.~. 
The difference here to the latter reference, is that we explicitly exclude switching from Newton's method and that Lugr{\'i}s et al.~used
contact points, while we use linear segments.
For a simple 1D example using this position based approach for friction, see \ ``Examples/lugreFrictionText.py``\ , 
which compares the traditional LuGre friction model  with the position based model with tangential stiffness. 


.. _fig-objectcontactfrictioncirclecable2d-stickingpos:
.. figure:: ../../theDoc/figures/ContactFrictionCircleCable2DstickingPos.png
   :width: 600

   Calculation of last sticking position; blue parts mark the sticking position calculated as \ :math:`x^*_{curStick}`\ .


Because there is the chance to wind/unwind relative to the (last) sticking position without slipping,
the following strategy is used.
In case of sliding (which could be the last time sliding before sticking), 
we compute the \ **current sticking position**\ , see \ :numref:`fig-objectcontactfrictioncirclecable2d-stickingpos`\ , as the sum of the relative position at the segment \ :math:`s`\ 

.. math::

   x_{s,curStick} = \rho \cdot L_{seg}


in which \ :math:`\rho \in [0,1]`\  denotes the relative position of contact at the segment with reference length \ :math:`L_{seg}=\frac{L}{n_{cs}}`\ .
The relative position at the circle \ :math:`c`\  is

.. math::

   x_{c,curStick} = \alpha \cdot r


We immediately see, that under pure rolling\ (neglecting the effects of small penetration, usually much smaller than shown for visibility in \ :numref:`fig-objectcontactfrictioncirclecable2d-stickingpos`\ .),

.. math::

   x_{s,curStick} + x_{c,curStick}  = \mathrm{const}.


Note that the \ ``verticalOffset``\  from the cable center line, as defined in the related \ ``MarkerBodyCable2DShape``\ ,
influences the behavior significantly, which is why we recommend to use \ ``verticalOffset=0``\  whenever this is an 
appropriate assumption.
Thus, the current sticking position \ :math:`x_{curStick}`\  is computed per segment as

.. math::
   :label: objectcontactfrictioncirclecable2d-lastcurstick

   x^*_{curStick} = x_{s,curStick} + x_{c,curStick}, \quad


Due to the possibility of switching of \ :math:`\alpha+\phi`\  between \ :math:`-\pi`\  and \ :math:`\pi`\ , the result is normalized to

.. math::
   :label: objectcontactfrictioncirclecable2d-curstick

   x_{curStick} = x^*_{curStick} - \mathrm{floor}\left(\frac{x^*_{curStick} }{2 \pi \cdot r} + \frac{1}{2}\right) \cdot 2 \pi \cdot r, \quad


which gives \ :math:`\bar x_{curStick} \in [-\pi \cdot r,\pi \cdot r]`\ , which is stored in the 3rd data variable (per segment).
The function floor() is a standardized version of rounding, available in C and Python programming languages.
In the \ ``PostNewtonStep``\ , the last sticking position is computed, \ :math:`x_{lastStick} = x_{curStick}`\ , and it is also available in the \ ``startOfStep``\  state.


Contact forces: definition
--------------------------

The contact force \ :math:`f_n`\  is zero for \ :math:`g > 0`\  and otherwise computed from 

.. math::
   :label: objectcontactfrictioncirclecable2d-contactforce

   f_n = k_c \cdot g + d_c \cdot v_n


NOTE that currently, there is only a linear spring-damper model available, assuming that the impact dynamics 
is not dominating (such as in belt drives or reeving systems).

Friction forces are primarily based on relative (tangential) velocity at each segment.
The 'linear' friction force, based on the velocity penalty parameter \ :math:`\mu_v`\  reads

.. math::

   f_t^{(lin)} = \mu_v \cdot v_t ,

    

PostNewtonStep
--------------

In general, see the solver flow chart for the \ ``DiscontinuousIteration``\ , see \ :numref:`fig-solver-discontinuous-iteration`\ , should be considered when reading this description. Every step is started with values \ ``startOfStep``\ , while current values are iterated and updated in the Newton or \ ``DiscontinuousIteration``\ .

The \ ``PostNewtonStep``\  computes 3 values per segment, which are used for computation of contact forces, irrespectively of the 
current geometryof the contact. 
The \ ``PostNewtonStep``\  is called after every full Newton method and evaluates the current state w.r.t. the assumed data variables.
If the assumptions do not fit, new data variables are computed.
This is necessary in order to avoid discontinuities in the equations, while otherwise the Newton iterations would not 
(or only slowly) converge.

The data variables per segment are

.. math::

   [x_{gap},\, x_{isSlipStick},\, x_{lastStick}]


Here, \ :math:`x_{gap}`\  contains the gap of the segment (\ :math:`\le 0`\  means contact), \ :math:`x_{lastStick}`\  is described in 
Eq. :eq:`objectcontactfrictioncirclecable2d-curstick`\ , and 
\ :math:`x_{isSlipStick}`\  defines the stick or slip case,

+  \ :math:`x_{isSlipStick} = -2`\ : undefined, used for initialization
+  \ :math:`x_{isSlipStick} = 0`\ : sticking
+  \ :math:`x_{isSlipStick} = \pm 1`\ : slipping, sign defines slipping direction


The basic algorithm in the \ ``PostNewtonStep``\ , with all operations given for any segment \ :math:`s_i`\ , can be summarized as follows:

+  [I.] Evaluate gap per segment \ :math:`g`\  using Eq. :eq:`objectcontactfrictioncirclecable2d-gap`\  and store in data variable: 
        \ :math:`x_{gap} = g`\ 
+  [II.] If \ :math:`x_{gap} < 0`\  and (\ :math:`\mu_v \neq 0`\  or  \ :math:`\mu_k \neq 0`\ ):
  
+  Compute contact force \ :math:`f_n`\  according to Eq. :eq:`objectcontactfrictioncirclecable2d-contactforce`\ 
+  Compute current sticking position \ :math:`x_{curStick}`\  according to Eq. :eq:`objectcontactfrictioncirclecable2d-lastcurstick`\ \ (terms are only evaluated if \ :math:`\mu_k \neq 0`\ )
+  Retrieve \ ``startOfStep``\  sticking position\ (Importantly, the \ ``PostNewtonStep``\  always refers to the \ ``startOfStep``\  state in the sticking position, because in the discontinuous iterations, the algorithm could switch to slipping in between and override the last sticking position in the current step) in \ :math:`x^{startOfStep}_{lastStick}`\  and compute and normalize
    difference in sticking position\ (in case that \ :math:`x_{isSlipStick} = -2`\ , meaning that there is no stored sticking position, we set \ :math:`\Delta x_{stick} = 0`\ ):
    
.. math::

   \Delta x^*_{stick} = x_{curStick} - x^{startOfStep}_{lastStick}, \quad \Delta x_{stick} = \Delta x^*_{stick} - \mathrm{floor}\left(\frac{\Delta x^*_{stick} }{2 \pi \cdot r} + \frac{1}{2}\right) \cdot 2 \pi \cdot r


+  Compute linear tangential force for friction stiffness and velocity penalty: 
      
.. math::

   f_{t,lin} = \mu_v \cdot v_t + \mu_k \Delta x_{stick}


+  Compute tangential force according to Coulomb friction model \ (note that the sign of \ :math:`\Delta x_{stick}`\  is used here, but
    alternatively we may also use the sign of \ :math:`f_{t,lin}`\ ):
    
.. math::

   f_t = \begin{cases} f_t^{(lin)}, \quad \quad \quad \quad \quad \quad \quad \mathrm{if} \quad |f_t^{(lin)}| \le \mu \cdot |f_n| \\ \mu \cdot |f_n| \cdot \mathrm{Sign}(\Delta x_{stick}), \quad \mathrm{else} \end{cases}


+  In the case of slipping, given by \ :math:`|f_t^{(lin)}| > \mu \cdot |f_n|`\ , we update the last sticking position in the data variable, 
    such that the spring is pre-tensioned already,
    
.. math::

   x_{lastStick} = x_{curStick} - \mathrm{Sign}(\Delta x_{stick}) \frac{\mu \cdot |f_n|}{\mu_k}, \quad x_{isSlipStick} = \mathrm{Sign}(\Delta x_{stick})


+  In the case of sticking, given by \ :math:`|f_t^{(lin)}| \le \mu \cdot |f_n|`\ : Set \ :math:`x_{isSlipStick} = 0`\  and, 
    if \ :math:`x^{startOfStep}_{isSlipStick} = -2`\  (undefined), we update \ :math:`x_{lastStick} = x_{curStick}`\ , while otherwise, \ :math:`x_{lastStick}`\  is unchanged.
  
+  [III. ] If \ :math:`x_{gap} > 0`\  or (\ :math:`\mu_v == 0`\  and \ :math:`\mu_k == 0`\ ), we set \ :math:`x_{isSlipStick} = -2`\  (undefined); this means that in the next step (if this step is accepted), there is no stored sticking position.
+  [IV.] Compute an error \ :math:`\varepsilon_{PNS} = \varepsilon^n_{PNS}+\varepsilon^t_{PNS}`\ ,
              with physical units forces (per segment point), for \ ``PostNewtonStep``\ :
  
+  if gap \ :math:`x_{gap,lastPNS}`\  of previous \ ``PostNewtonStep``\  had different sign to current gap, set
    
.. math::

   \varepsilon^n_{PNS} = k_c \cdot \Vert x_{gap} - x_{gap,lastPNS}\Vert


while otherwise \ :math:`\varepsilon^n_{PNS}=0`\ .
+  if stick-slip-state \ :math:`x_{isSlipStick,lastPNS}`\  of previous \ ``PostNewtonStep``\  is different from current \ :math:`x_{isSlipStick}`\ , set
    
.. math::

   \varepsilon^t_{PNS} = \Vert \left(\Vert f_t^{(lin)} \Vert  - \mu \cdot |f_n| \right)\Vert


while otherwise \ :math:`\varepsilon^t_{PNS}=0`\ .
  

Note that the \ ``PostNewtonStep``\  is iterated and the data variables are updated continuously until convergence, or until a max.\ number of iterations is reached. If \ ``ignoreMaxIterations``\  == 0, computation will continue even if no convergence is reached after the given number of iterations. This will lead so larger errors in such steps, but may have less influence on the overall solution if such cases are rare. 


Computation of connector forces in Newton
-----------------------------------------

The computation of LHS terms, the action of forces produced by the contact-friction element, is done during Newton iterations and may not have
discontinuous behavior, thus relating computations to data variables computed in the \ ``PostNewtonStep``\ .
For efficiency, the LHS computation is only performed, if the \ ``PostNewtonStep``\  determined contact in any segment.

The operations are similar to the \ ``PostNewtonStep``\ , but without switching. The following operations are performed for each segment \ :math:`s_i`\ , if 
\ :math:`x_{gap, s_i} <= 0`\ :

+ [I.] Compute contact force \ :math:`f_n`\ , Eq. :eq:`objectcontactfrictioncirclecable2d-contactforce`\ .
+ [II.] In case of sticking (\ :math:`|x_{isSlipStick}|\neq 1`\ ):
  
+  [II.1] the current sticking position \ :math:`x_{curStick}`\  is computed from Eq. :eq:`objectcontactfrictioncirclecable2d-lastcurstick`\ , and the difference of current and last sticking position reads\ (see the difference to the \ ``PostNewtonStep``\ : we use \ :math:`x_{lastStick}`\  here, not the \ ``startOfStep``\  variant.):
    
.. math::

   \Delta x^*_{stick} = x_{curStick} - x_{lastStick}, \quad \Delta x_{stick} = x^*_{stick} - \mathrm{floor}\left(\frac{\Delta x^*_{stick} }{2 \pi \cdot r} + \frac{1}{2}\right) \cdot 2 \pi \cdot r


+  [II.2] if the friction stiffness is \ :math:`\mu_k==0`\  or if \ :math:`x_{isSlipStick} == -2`\ , we set \ :math:`\Delta x_{stick}=0`\ 
+  [II.3] using the tangential velocity from Eq. :eq:`objectcontactfrictioncirclecable2d-vtangent`\ , the tangent force follows as (even if it is larger than the sticking limit)
    
.. math::

   f_t = \mu_v \cdot v_t + \mu_k \Delta x_{stick}



+  [III.] In case of slipping (\ :math:`|x_{isSlipStick}|=1`\ ), the tangential firction force is set  as\ (see again difference to \ ``PostNewtonStep``\ !),
  
.. math::

   f_t = \mu \cdot |f_n| \cdot x_{isSlipStick}, \quad \mathrm{else}

 

Note that in the Newton method, the tangential force may be inconsistent with the Kuhn-Tucker conditions. However,
the \ ``PostNewtonStep``\  resolves this inconsistency.

Computation of LHS terms for circle and ANCF cable element
----------------------------------------------------------

If \ ``activeConnector = True``\ , 
contact forces \ :math:`{\mathbf{f}}_i`\  with \ :math:`i \in [0,n_{cs}]`\  -- these are \ :math:`(n_{cs}+1)`\  forces -- are applied at the points \ :math:`p_i`\ , and they are computed for every contact segments (i.e., two segments may contribute to contact forces of one point).
For every contact computation, first all contact forces at segment points are set to zero. 
We distinguish two cases SN and PWN. If \ ``useSegmentNormals==True``\ , we use the SN case, while otherwise the PWN case is used, 
compare \ :numref:`fig-objectcontactfrictioncirclecable2d-normals`\ .


.. _fig-objectcontactfrictioncirclecable2d-normals:
.. figure:: ../../theDoc/figures/ContactFrictionCircleCable2Dnormals.png
   :width: 700

   Choice of normals and tangent vectors for calculation of normal contact forces and tangential (friction) forces; note that the \ ``useSegmentNormals=False``\  is not appropriate for this setup and would produce highly erroneous forces.


Segment normals (=SN) lead to always good approximations for normal directions, irrespectively of short or extremely long segments as compared to the circle. However, in case of segments that are short as compared to the circle radius, normals computed from the center of the circle to the segment points (=PWN) are more consistent and produce tangents only in circumferential direction, which may improve behavior in some applications. The equations for the two cases read:

   \ **CASE SN**\ : use \ **S**\ egment \ **N**\ ormals

If there is contact in a segment \ :math:`s_i`\ , i.e., gap state \ :math:`x_{gap} \le 0`\ , see \ :numref:`fig-objectcontactfrictioncirclecable2d-sketch`\ (right), contact forces \ :math:`{\mathbf{f}}_{s_i}`\  are computed per segment,

.. math::

   {\mathbf{f}}_{s_i} = f_n \cdot {\mathbf{n}}_{s_i} + f_t {\mathbf{t}}_{s_i}


and added to every force at segment points according to
  
.. math::

   {\mathbf{f}}_i &\pluseq& (1-\rho) \cdot {\mathbf{f}}_{s_i}      \\
   \nonumber {\mathbf{f}}_{i+1} &\pluseq& \rho \cdot {\mathbf{f}}_{s_i}


while in case \ :math:`x_{gap}  > 0`\  nothing is added.
   \ **CASE PWN**\ : use \ **P**\ oint \ **W**\ ise \ **N**\ ormals (at segment points)

If there is contact in a segment \ :math:`s_i`\ , i.e., gap \ :math:`x_{gap} \le 0`\ , 
see \ :numref:`fig-objectcontactfrictioncirclecable2d-sketch`\ (right), 
intermediate contact forces \ :math:`{\mathbf{f}}^{l,r}_{i}`\  are computed per segment point,
  
.. math::

   {\mathbf{f}}^l = f_n \cdot {\mathbf{n}}_{l,s_i} + f_t {\mathbf{t}}_{l,s_i}, \quad {\mathbf{f}}^r = f_n \cdot {\mathbf{n}}_{r,s_i} + f_t {\mathbf{t}}_{r,s_i}


  in which \ :math:`{\mathbf{n}}_{l,s_i}`\  is the vector from circle center to the left point (\ :math:`i`\ ) of the segment \ :math:`s_i`\ ,
  and \ :math:`{\mathbf{n}}_{l,s_i}`\  to the right point (\ :math:`i+1`\ ). The tangent vectors are perpendicular to the normals.
  The forces are then applied to the contact forces \ :math:`{\mathbf{f}}_i`\  using the parameter \ :math:`\rho`\ , which takes into account the distance of contact to the left or right side of the segment,
  
.. math::

   {\mathbf{f}}_i &\pluseq& (1-\rho) \cdot {\mathbf{f}}^l      \\
   \nonumber {\mathbf{f}}_{i+1} &\pluseq& \rho \cdot {\mathbf{f}}^r


while in case \ :math:`x_{gap}  > 0`\  nothing is added.

The forces \ :math:`{\mathbf{f}}_i`\  are then applied through the marker to the \ ``ObjectANCFCable2D``\  element as point loads via a position jacobian
(using the according access function), for details see the C++ implementation.

The forces on the circle marker \ :math:`m0`\  are computed as the total sum of all
segment contact forces, 

.. math::

   {\mathbf{f}}_{m0} = -\sum_{s_i} {\mathbf{f}}_{s_i}


and additional torques on the circle's rotation simply follow from

.. math::

   \tau_{m0} = -\sum_{s_i} r \cdot f_{t_{s_i}} .


During Newton iterations, the contact forces for segment \ :math:`s_i`\  are considered only, if 
\ :math:`x_i <= 0`\ . The dataCoordinate \ :math:`x_i`\  is not modified during Newton iterations, but computed
during the DiscontinuousIteration, see \ :numref:`fig-solver-discontinuous-iteration`\  in the solver description. 


If \ ``activeConnector = False``\ , all contact and friction forces on the cable and the force and torque on the 
circle's marker are set to zero.


Relevant Examples and TestModels with weblink:

    \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Examples/), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Examples/), \ `beltDrivesComparison.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDrivesComparison.py>`_\  (Examples/), \ `sliderCrank3DwithANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive.py>`_\  (Examples/), \ `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_\  (Examples/), \ `ANCFcontactFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactFrictionTest.py>`_\  (TestModels/), \ `ANCFmovingRigidBodyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFmovingRigidBodyTest.py>`_\  (TestModels/), \ `ANCFslidingAndALEjointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFslidingAndALEjointTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


