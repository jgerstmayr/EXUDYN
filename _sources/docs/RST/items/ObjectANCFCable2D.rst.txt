

.. _sec-item-objectancfcable2d:

ObjectANCFCable2D
=================

A 2D cable finite element using 2 nodes of type NodePoint2DSlope1. The localPosition of the beam with length \ :math:`L`\ =physicsLength and height \ :math:`h`\  ranges in \ :math:`X`\ -direction in range \ :math:`[0, L]`\  and in \ :math:`Y`\ -direction in range \ :math:`[-h/2,h/2]`\  (which is in fact not needed in the \ :ref:`EOM <EOM>`\ ).

\ **Additional information for ObjectANCFCable2D**\ :

* | Requested \ ``Node``\  type = \ ``Position2D``\  + \ ``Orientation2D``\  + \ ``Point2DSlope1``\  + \ ``Position``\  + \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``Cable2D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VCable2D``\ 


The item \ **ObjectANCFCable2D**\  with type = 'ANCFCable2D' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsLength** [\ :math:`L`\ , type = UReal, default = 0.]:
  | [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives \ :math:`\rho A L`\ ; must be positive
* | **physicsMassPerLength** [\ :math:`\rho A`\ , type = UReal, default = 0.]:
  | [SI:kg/m] mass per length of beam
* | **physicsBendingStiffness** [\ :math:`EI`\ , type = UReal, default = 0.]:
  | [SI:Nm\ :math:`^2`\ ] bending stiffness of beam; the bending moment is \ :math:`m = EI (\kappa - \kappa_0)`\ , in which \ :math:`\kappa`\  is the material measure of curvature
* | **physicsAxialStiffness** [\ :math:`EA`\ , type = UReal, default = 0.]:
  | [SI:N] axial stiffness of beam; the axial force is \ :math:`f_{ax} = EA (\varepsilon -\varepsilon_0)`\ , in which \ :math:`\varepsilon = |{\mathbf{r}}^\prime|-1`\  is the axial strain
* | **physicsBendingDamping** [\ :math:`d_{K}`\ , type = UReal, default = 0.]:
  | [SI:Nm\ :math:`^2`\ /s] bending damping of beam ; the additional virtual work due to damping is \ :math:`\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx`\ 
* | **physicsAxialDamping** [\ :math:`d_{\varepsilon}`\ , type = UReal, default = 0.]:
  | [SI:N/s] axial damping of beam; the additional virtual work due to damping is \ :math:`\delta W_{\dot\varepsilon} = \int_0^L \dot \varepsilon \delta \varepsilon dx`\ 
* | **physicsReferenceAxialStrain** [\ :math:`\varepsilon_0`\ , type = Real, default = 0.]:
  | [SI:1] reference axial strain of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference axial strain value
* | **physicsReferenceCurvature** [\ :math:`\kappa_0`\ , type = Real, default = 0.]:
  | [SI:1/m] reference curvature of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference curvature value
* | **strainIsRelativeToReference** [\ :math:`f\cRef`\ , type = Real, default = 0.]:
  | if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration plus the values of \ :math:`\varepsilon_0`\  and \ :math:`\kappa_0`\  serve as a reference geometry; allows also values between 0. and 1.
* | **nodeNumbers** [type = NodeIndex2, default = [invalid [-1], invalid [-1]]]:
  | two node numbers ANCF cable element
* | **useReducedOrderIntegration** [type = Index, default = 0]:
  | 0/false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; 1/true: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments
* | **axialForceUserFunction** [\ :math:`\mathrm{UF} \in \Rcal`\ , type = PyFunctionMbsScalarIndexScalar9, default =  0]:
  | A Python function which defines the (nonlinear relations) of local strains (including axial strain and bending strain) as well as time derivatives to the local axial force; see description below
* | **bendingMomentUserFunction** [\ :math:`\mathrm{UF} \in \Rcal`\ , type = PyFunctionMbsScalarIndexScalar9, default =  0]:
  | A Python function which defines the (nonlinear relations) of local strains (including axial strain and bending strain) as well as time derivatives to the local bending moment; see description below
* | **visualization** [type = VObjectANCFCable2D]:
  | parameters for visualization of item



The item VObjectANCFCable2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawHeight** [type = float, default = 0.]:
  | if beam is drawn with rectangular shape, this is the drawing height
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color of the object; if R==-1, use default color


----------

.. _description-objectancfcable2d:

DESCRIPTION of ObjectANCFCable2D
--------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}\cConfig(x,y,0)} = {\mathbf{r}}\cConfig(x) + y\cdot {\mathbf{n}}\cConfig(x)`\ 
  | global position vector of local position \ :math:`[x,y,0]`\ 
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}\cConfig(x,y,0)} = \LU{0}{{\mathbf{p}}\cConfig(x,y,0)} - \LU{0}{{\mathbf{p}}\cRef(x,y,0)}`\ 
  | global displacement vector of local position
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}(x,y,0)} = \LU{0}{\dot {\mathbf{r}}(x)} - y \cdot \omega_2 \cdot\LU{0}{{\mathbf{t}}(x)}`\ 
  | global velocity vector of local position
* | ``VelocityLocal``\ : \ :math:`\LU{b}{{\mathbf{v}}(x,y,0)} = \LU{b0}{\Rot}\LU{0}{{\mathbf{v}}(x,y,0)}`\ 
  | local velocity vector of local position
* | ``Rotation``\ : \ :math:`\varphi = \mathrm{atan2}(r'_y, r'_x)`\ 
  | (scalar) rotation angle of axial slope vector (relative to global \ :math:`x`\ -axis)
* | ``Director1``\ : \ :math:`{\mathbf{r}}'(x)`\ 
  | (axial) slope vector of local axis position (at \ :math:`y`\ =0)
* | ``StrainLocal``\ : \ :math:`\varepsilon`\ 
  | axial strain (scalar) of local axis position (at Y=0)
* | ``CurvatureLocal``\ : \ :math:`K`\ 
  | axial strain (scalar)
* | ``ForceLocal``\ : \ :math:`N`\ 
  | (local) section normal force (scalar, including reference strains) (at \ :math:`y`\ =0); note that strains are highly inaccurate when coupled to bending, thus consider useReducedOrderIntegration=2 and evaluate axial strain at nodes or at midpoint
* | ``TorqueLocal``\ : \ :math:`M`\ 
  | (local) bending moment (scalar) (at \ :math:`y`\ =0)
* | ``AngularVelocity``\ : \ :math:`\tomega = [0,\, ,0,\, \omega_2]`\ 
  | angular velocity of local axis position (at \ :math:`y`\ =0)
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}(x,y,0)} = \LU{0}{\ddot {\mathbf{r}}(x)} - y \cdot \dot\omega_2 \cdot\LU{0}{{\mathbf{t}}(x)}- y \cdot \omega_2 \cdot\LU{0}{\dot{\mathbf{t}}(x)}`\ 
  | global acceleration vector of local position
* | ``AngularAcceleration``\ : \ :math:`\talpha = [0,\, ,0,\, \dot\omega_2]`\ 
  | angular acceleration of local axis position



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | beam height
     - | \ :math:`h`\ 
     - | beam height used in several definitions, but effectively undefined. The geometry of the cross section has no influence except for drawing or contact.
   * - | local beam position
     - | \ :math:`\pLocB=[x,\, y,\, 0]\tp`\ 
     - | local position at axial coordinate \ :math:`x \in [0,L]`\  and cross section coordinate \ :math:`y \in [-h/2, h/2]`\ . 
   * - | beam axis position
     - | \ :math:`\LU{0}{{\mathbf{r}}(x)} = {\mathbf{r}}(x)`\ 
     - | 
   * - | beam axis slope
     - | \ :math:`\LU{0}{{\mathbf{r}}'(x)} = {\mathbf{r}}'(x)`\ 
     - | 
   * - | beam axis tangent
     - | \ :math:`\LU{0}{{\mathbf{t}}(x)} = \frac{{\mathbf{r}}'(x)}{\Vert {\mathbf{r}}(x)'\Vert}`\ 
     - | this (normalized) vector is normal to cross section
   * - | beam axis normal
     - | \ :math:`\LU{0}{{\mathbf{n}}(x)} = [n_x,\, n_y]\tp = [-t_y,\, t_x]\tp`\ 
     - | this (normalized) vector lies within the cross section and defines positive \ :math:`y`\ -direction.
   * - | angular velocity
     - | \ :math:`\omega_2 = (-r'_y \cdot \dot r'_x + r'_x \cdot \dot r'_y) / \Vert {\mathbf{r}}(x)'\Vert^2`\ 
     - | 
   * - | rotation matrix
     - | \ :math:`\LU{0b}{\Rot}`\ 
     - | 


The Bernoulli-Euler beam is capable of large axial and bendig deformation as it employs the material measure of curvature for the bending.

Kinematics and interpolation
----------------------------

Note that in this section, expressions are written in 2D, while output variables are in general 3D quantities, adding a zero for the \ :math:`z`\ -coordinate.
ANCF elements follow the original concept proposed by Shabana .
The present 2D element is based on the interpolation used by Berzeri and Shabana , but the formulation (especially of the elastic forces) is according to
Gerstmayr and Irschik .
Slight improvements for the integration of elastic forces and additional terms for off-axis forces and constraints are mentioned here.

The current position of an arbitrary element at local axial position \ :math:`x \in [0,L]`\ , where \ :math:`L`\  is the beam length, reads

.. math::

   {\mathbf{r}}={\mathbf{r}}(x, t),


The derivative of the position w.r.t.\ the axial reference coordinate is denoted as slope vector,

.. math::

   {\mathbf{r}}'= \frac{\partial {\mathbf{r}}(x, t)}{\partial x}


The interpolation is based on cubic (spline) interpolation of position, displacements and velocities.
The generalized coordinates \ :math:`{\mathbf{q}} \in \Rcal^8`\  of the beam element is defined by

.. math::

   {\mathbf{q}}= \left[\, {\mathbf{r}}_0^{T}\;\;{\mathbf{r}}_0^{' T}\;\; {\mathbf{r}}_1^{T}\;\; {\mathbf{r}}_1^{' T}\, \right]^{T}.


in which \ :math:`{\mathbf{r}}_0`\  is the position of node 0 and \ :math:`{\mathbf{r}}_1`\  is the position of node 1,
\ :math:`{\mathbf{r}}'_0`\  the slope at node 0 and \ :math:`{\mathbf{r}}'_1`\  the slope at node 1.
Note that ANCF coordinates in the present notation are computed as sum of reference and current coordinates

.. math::

   {\mathbf{q}} = {\mathbf{q}}\cCur + {\mathbf{q}}\cRef


which is used throughout here. For time derivatives, it follows that \ :math:`\dot {\mathbf{q}} = \dot {\mathbf{q}}\cCur`\ .

Position and slope are interpolated with shape functions.
The position and slope along the beam are interpolated by means of 

.. math::

   {\mathbf{r}} = {\mathbf{S}} {\mathbf{q}} \qquad \mathrm{and} \qquad {\mathbf{r}}'={\mathbf{S}}' {\mathbf{q}}.


in which \ :math:`{\mathbf{S}}`\  is the shape function matrix,

.. math::

   {\mathbf{S}}(x)= \left[\, S_1(x)\,\mathbf{I}_{2 \times 2}\;\; S_2(x)\,\mathbf{I}_{2 \times 2}\;\; S_3(x)\,\mathbf{I}_{2 \times 2}\;\; S_4(x)\,\mathbf{I}_{2 \times 2}\, \right].


with identity matrix \ :math:`\mathbf{I}_{2 \times 2} \in \Rcal^{2 \times 2}`\  and the shape functions

.. math::
   :label: eq-cable2d-shapefunctions

   S_1(x) &=& 1-3\frac{x^2}{L^2}+2\frac{x^3}{L^3}, \quad S_2(x) = x-2\frac{x^2}{L}+\frac{x^3}{L^2}\nonumber\\
   S_3(x) &=& 3\frac{x^2}{L^2}-2\frac{x^3}{L^3}, \; \; \; \; \; \;  \quad S_4(x) = -\frac{x^2}{L}+\frac{x^3}{L^2}


Velocity simply follows as 

.. math::

   \frac{\partial {\mathbf{r}}}{\partial t} = \dot {\mathbf{r}} = {\mathbf{S}} \dot {\mathbf{q}}.



Mass matrix
-----------

The mass matrix is constant and therefore precomputed at the first time it is needed (e.g., during computation of initial accelerations).
The analytical form of the mass matrix reads

.. math::

   {\mathbf{M}}_{analytic} = \int_0^L \rho A {\mathbf{S}}(x)^T {\mathbf{S}}(x) dx


which is approximated using

.. math::

   {\mathbf{M}} = \sum_{ip = 0}^{n_{ip}-1} w(x_{ip}) \frac{L}{2} \rho A {\mathbf{S}}(x_{ip})^T {\mathbf{S}}(x_{ip})


with integration weights \ :math:`w(x_{ip})`\ , \ :math:`\sum w(x_{ip})=2`\ , and integration points \ :math:`x_{ip}`\ , given as,

.. math::
   :label: eq-ancfcable-iptransform

   x_{ip} = \frac{L}{2}\xi_{ip} + \frac{L}{2} .


Here, we use the Gauss integration rule with order 7, having \ :math:`n_{ip}=4`\  Gauss points, see Section :ref:`sec-integrationpoints`\ . 
Due to the third order polynomials, the integration is exact up to round-off errors.
        

Elastic forces
--------------

The elastic forces \ :math:`{\mathbf{Q}}_e`\  are implicitly defined by the relation to the 
virtual work of elastic forces, \ :math:`\delta W_e`\ , of applied forces, \ :math:`\delta W_a`\  and of viscous forces, \ :math:`\delta W_v`\ , 

.. math::
   :label: eq-cable2d-elasticforces

   {\mathbf{Q}}_e^T \delta {\mathbf{q}} = \delta W_e + \delta W_a + \delta W_v.


The virtual work of elastic forces reads ,

.. math::

   \delta W_e = \int_0^L (N \delta \varepsilon + M \delta K) \,dx,


in which the axial strain is defined as 

.. math::

   \varepsilon=\Vert {\mathbf{r}}'\Vert-1.

 
and the material measure of curvature (bending strain) is given as

.. math::

   K={\mathbf{e}}_3^T \frac{ {\mathbf{r}}'\times {\mathbf{r}}'' }{\Vert {\mathbf{r}}'\Vert^2} .


in which \ :math:`{\mathbf{e}}_3`\  is the unit vector which is perpendicular to the plane of the planar beam element.

By derivation, we obtain the variation of axial strain

.. math::
   :label: eq-cable2d-deltaepsilon

   \delta \varepsilon =\frac{\partial \varepsilon}{\partial q_i}\delta q_i =\frac{1}{\Vert {\mathbf{r}}'\Vert}{\mathbf{r}}'^{T}{\mathbf{S}}'_i \delta q_i.


and the variation of \ :math:`K`\ 

.. math::
   :label: eq-cable2d-deltakappa

   \delta K &=& \frac{\partial}{\partial q_i} \left( \frac{({\mathbf{r}}'^{T}\times {\mathbf{r}}'' )^{T}{\mathbf{e}}_{3}}{\Vert {\mathbf{r}}' \Vert^2 }\right) \delta q_i\nonumber\\
   &=& \frac{1}{\Vert {\mathbf{r}}' \Vert^4} \left[ \Vert {\mathbf{r}}' \Vert^2 ({\mathbf{S}}'_i  \times {\mathbf{r}}'' +{\mathbf{r}}' \times {\mathbf{S}}''_i) -2 ({\mathbf{r}}' \times {\mathbf{r}}'') ({\mathbf{r}}'^{T} {\mathbf{S}}'_i) \right]^{T} {\mathbf{e}}_3 \delta q_i


The normal force (axial force) \ :math:`N`\  in the beam is defined as function of the current strain \ :math:`\varepsilon`\ ,

.. math::
   :label: eq-n

   N = EA \, (\varepsilon - \varepsilon_0 - f\cRef \cdot \varepsilon\cRef).


in which \ :math:`\varepsilon_0`\  includes the (pre-)stretch of the beam, e.g., due to temperature or plastic deformation and 
\ :math:`\varepsilon\cRef`\  includes the strain of the reference configuration.
As can be seen, the reference strain is only considered, if \ :math:`f\cRef=1`\ , which allows to consider the reference configuration to be
completely stress-free (but the default value is \ :math:`f\cRef=0`\  !).
Note that -- due to the inherent nonlinearity of \ :math:`\varepsilon`\  -- a combination of \ :math:`\varepsilon_0`\  and \ :math:`f\cRef=1`\  is physically only meaningful for small strains.
A factor \ :math:`f\cRef<1`\  allows to realize a smooth transition between deformed and straight reference configuration, e.g. for initial configurations.

The bending moment \ :math:`M`\  in the beam is defined as function of the current material measure of curvature \ :math:`K`\ ,

.. math::
   :label: eq-m

   M = EI \, (K - K_0 - f\cRef \cdot K\cRef).


in which \ :math:`K_0`\  includes the (pre-)curvature of the undeformed beam and
\ :math:`K\cRef`\  includes the curvature of the reference configuration, multiplied with the factor \ :math:`f\cRef=1`\ , see the axial strain above.

Using the latter definitions, the elastic forces follow from Eq. :eq:`eq-cable2d-elasticforces`\ .

The virtual work of viscous damping forces, assuming viscous effects proportial to axial streching and bending, is defined as

.. math::

   \delta W_v = \int_0^L \left( d_\varepsilon \dot \varepsilon \delta \varepsilon + d_K \dot K \delta K \right) \,d x.


with material coefficients \ :math:`d_\varepsilon`\  and \ :math:`d_K`\ .
The time derivatives of axial strain \ :math:`\dot \varepsilon_p`\  follows by elementary differentiation

.. math::

   \dot \varepsilon =  \frac{\partial }{\partial t}\left(\Vert {\mathbf{r}}'\Vert-1 \right) = \frac{1}{\Vert {\mathbf{r}}'\Vert} {\mathbf{r}}^{\prime T} {\mathbf{S}}' \dot {\mathbf{q}}


as well as the derivative of the curvature,

.. math::

   \dot K & = &  \frac{\partial }{\partial t}\left({\mathbf{e}}_3^T\frac{ {\mathbf{r}}'\times {\mathbf{r}}'' }{\Vert {\mathbf{r}}'\Vert^2}\right) \nonumber\\
   & = &\frac{{\mathbf{e}}_3^T}{({\mathbf{r}}'^T {\mathbf{r}}')^2} \left( ({\mathbf{r}}'^T {\mathbf{r}}')   \frac{\partial \left( {\mathbf{r}}' \times {\mathbf{r}}'' \right)^T }{\partial t} -\left( {\mathbf{r}}' \times {\mathbf{r}}'' \right)^T  \frac{\partial  ({\mathbf{r}}'^T {\mathbf{r}}')}{\partial t} \right)\nonumber\\
   & = &  \frac{{\mathbf{e}}_3^T}{({\mathbf{r}}'^T {\mathbf{r}}')^2}\left(({\mathbf{r}}'^T {\mathbf{r}}')\left(({\mathbf{S}}' \dot {\mathbf{q}}) \times {\mathbf{r}}'' + ({\mathbf{S}}'' \dot {\mathbf{q}}) \times {\mathbf{r}}'\right)-\left( {\mathbf{r}}' \times {\mathbf{r}}'' \right) (2{\mathbf{r}}'^T ({\mathbf{S}}' \dot {\mathbf{q}})) \right) .



The virtual work of applied forces reads

.. math::
   :label: eq-applied

   \delta W_a = \sum_i {\mathbf{f}}_i^T \delta {\mathbf{r}}_i(x_f) + \int_0^L {\mathbf{b}}^T \delta {\mathbf{r}}(x) \,d x ,


in which \ :math:`{\mathbf{f}}_i`\  are forces applied to a certain position \ :math:`x_f`\  at the beam centerline.
The second term contains a load per length \ :math:`{\mathbf{b}}`\ , which is case of gravity vector \ :math:`{\mathbf{g}}`\  reads

.. math::

   {\mathbf{b}} = \rho {\mathbf{g}}.


Note that the variation of \ :math:`{\mathbf{r}}`\  simply follows as

.. math::

   \delta {\mathbf{r}}= {\mathbf{S}}\, \delta {\mathbf{q}}




Numerical integration of Elastic Forces
---------------------------------------

The numerical integration of elastic forces \ :math:`{\mathbf{Q}}_e`\  is split into terms due to \ :math:`\delta \varepsilon`\  and \ :math:`\delta K`\ ,

.. math::

   {\mathbf{Q}}_e = \int_0^L \left(\bullet(x) \frac{\partial \delta \varepsilon}{\partial \delta {\mathbf{q}}} + \bullet(x) \frac{\partial \delta K}{\partial \delta {\mathbf{q}}} \right) \,dx


using different integration rules

.. math::

   {\mathbf{Q}}_e \approx  \sum_{ip = 0}^{n_{ip}^\varepsilon-1}  \left(\frac{L}{2}  \bullet(x_{ip}) \frac{\partial \delta \varepsilon}{\partial \delta {\mathbf{q}}} \right) + \sum_{ip = 0}^{n_{ip}^K-1} \left( \frac{L}{2}\bullet(x_{ip}) \frac{\partial \delta K}{\partial \delta {\mathbf{q}}} \right) \,dx


with the integration points \ :math:`x_{ip}`\  as defined in Eq. :eq:`eq-ancfcable-iptransform`\  and integration rules from Section :ref:`sec-integrationpoints`\ .
There are 3 different options for integration rules depending on the flag \ ``useReducedOrderIntegration``\ :

+  \ ``useReducedOrderIntegration``\  = 0: \ :math:`n_{ip}^\varepsilon = 5`\  (Gauss order 9), \ :math:`n_{ip}^K = 3`\  (Gauss order 5) -- this is considered as full integration, leading to very small approximations; certainly, due to the high nonlinearity of expressions, this is only an approximation.
+  \ ``useReducedOrderIntegration``\  = 1: \ :math:`n_{ip}^\varepsilon = 4`\  (Gauss order 7), \ :math:`n_{ip}^K = 2`\  (Gauss order 3) -- this is considered as reduced integration, which is usually sufficiently accurate but leads to slightly less computational efforts, especially for bending terms.
+  \ ``useReducedOrderIntegration``\  = 2: \ :math:`n_{ip}^\varepsilon = 3`\  (Lobatto order 3), \ :math:`n_{ip}^K = 2`\  (Gauss order 3) -- this is a further reduced integration, with the exceptional property that axial strain and bending strain terms are computed at completely disjointed locations: axial strain terms are evaluated at \ :math:`0`\ , \ :math:`L/2`\  and \ :math:`L`\ , while bending terms are evaluated at \ :math:`\frac{L}{2} \pm \frac{L}{2}\sqrt{1/3}`\ . This allows axial strains to freely follow the bending terms at \ :math:`\frac{L}{2} \pm \frac{L}{2}\sqrt{1/3}`\ , while axial strains are almost independent from bending terms at \ :math:`0`\ , \ :math:`L/2`\  and \ :math:`L`\ . However, due to the highly reduced integration, spurious (hourglass) modes may occur in certain applications!

Note that the Jacobian of elastic forces is computed using automatic differentiation.


Access functions
----------------

For application of forces and constraints at any local beam position \ :math:`\pLocB=[x,\, y,\, 0]\tp`\ , the position / velocity Jacobian reads

.. math::

   \frac{\partial \LU{0}{{\mathbf{v}}(x)}}{\dot {\mathbf{q}}} = {\mathbf{S}}(x) + \left[ -y \cdot n_x S'_1(x) \frac{1}{\Vert {\mathbf{r}}'\Vert} \LU{0}{{\mathbf{t}}}, \,\, -y \cdot n_y S'_1(x) \frac{1}{\Vert {\mathbf{r}}'\Vert} \LU{0}{{\mathbf{t}}}, \,\, -y \cdot n_x S'_2(x) \frac{1}{\Vert {\mathbf{r}}'\Vert} \LU{0}{{\mathbf{t}}}, \,\,\ldots \right]


with the normalized beam axis normal \ :math:`\LU{0}{{\mathbf{n}}} = [n_x,\, n_y]\tp`\ , see table above.

For application of torques at any axis point \ :math:`x`\ , the rotation / angular velocity Jacobian \ :math:`\frac{\partial \LU{0}{\omega(x)}}{\dot {\mathbf{q}}} \in \Rcal^{3 \times 8}`\  reads

.. math::

   \frac{\partial \LU{0}{\omega(x)}}{\dot {\mathbf{q}}} = \left[\!\! \begin{array}{ccccc} 0 & 0 & 0 & \cdots & 0 \\ 0 & 0 & 0 & \cdots & 0 \\ -r'_y \cdot S'_1(x) \frac{1}{{\mathbf{r}}^{\prime 2}} & r'_x \cdot S'_1(x) \frac{1}{{\mathbf{r}}^{\prime 2}} & -r'_y \cdot S'_2(x) \frac{1}{{\mathbf{r}}^{\prime 2}} & \cdots & r'_x \cdot S'_4(x) \frac{1}{{\mathbf{r}}^{\prime 2}}  \end{array} \!\!\right]



--------

\ **Userfunction**\ : ``axialForceUserFunction(mbs, t, itemNumber, axialPositionNormalized, axialStrain, axialStrain_t, axialStrainRef, physicsAxialStiffness, physicsAxialDamping, curvature, curvature_t, curvatureRef)`` 


A user function, which computes the axial force depending on time, strains and curvatures and 
object parameters (stiffness, damping).
The object variables are provided to the function using the current values of the ANCFCable2D object.
Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
\ ``mbs.GetObjectParameter(itemNumber, ...)``\ , see the according description of \ ``GetObjectParameter``\ .
\ **NOTE:**\  this function has a different interface as compared to the bending moment function.

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments /  return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs to which object belongs
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number \ :math:`i_N`\  of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``axialPositionNormalized``\ 
     - | Real
     - | axial position at the cable where the user function is evaluated; range is [0,1]
   * - | \ ``axialStrain``\ 
     - | Real
     - | \ :math:`\varepsilon`\ 
   * - | \ ``axialStrain_t``\ 
     - | Real
     - | \ :math:`\varepsilon_t`\ 
   * - | \ ``axialStrainRef``\ 
     - | Real
     - | \ :math:`\varepsilon_0 + f\cRef \cdot \varepsilon\cRef`\ 
   * - | \ ``physicsAxialStiffness``\ 
     - | Real
     - | as given in object parameters
   * - | \ ``physicsAxialDamping``\ 
     - | Real
     - | as given in object parameters
   * - | \ ``curvature``\ 
     - | Real
     - | \ :math:`K`\ 
   * - | \ ``curvature_t``\ 
     - | Real
     - | \ :math:`\dot K`\ 
   * - | \ ``curvatureRef``\ 
     - | Real
     - | \ :math:`K_0 + f\cRef \cdot K\cRef`\ 
   * - | \returnValue
     - | Real
     - | scalar value of computed axial force


--------

\ **Userfunction**\ : ``bendingMomentUserFunction(mbs, t, itemNumber, axialPositionNormalized, curvature, curvature_t, curvatureRef, physicsBendingStiffness, physicsBendingDamping, axialStrain, axialStrain_t, axialStrainRef)`` 


A user function, which computes the bending moment depending on time, strains and curvatures and 
object parameters (stiffness, damping).
The object variables are provided to the function using the current values of the ANCFCable2D object.
Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
\ ``mbs.GetObjectParameter(itemNumber, ...)``\ , see the according description of \ ``GetObjectParameter``\ .
\ **NOTE:**\  this function has a different interface as compared to the axial force function.

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments /  return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs to which object belongs
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number \ :math:`i_N`\  of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``axialPositionNormalized``\ 
     - | Real
     - | axial position at the cable where the user function is evaluated; range is [0,1]
   * - | \ ``curvature``\ 
     - | Real
     - | \ :math:`K`\ 
   * - | \ ``curvature_t``\ 
     - | Real
     - | \ :math:`\dot K`\ 
   * - | \ ``curvatureRef``\ 
     - | Real
     - | \ :math:`K_0 + f\cRef \cdot K\cRef`\ 
   * - | \ ``physicsBendingStiffness``\ 
     - | Real
     - | as given in object parameters
   * - | \ ``physicsBendingDamping``\ 
     - | Real
     - | as given in object parameters
   * - | \ ``axialStrain``\ 
     - | Real
     - | \ :math:`\varepsilon`\ 
   * - | \ ``axialStrain_t``\ 
     - | Real
     - | \ :math:`\varepsilon_t`\ 
   * - | \ ``axialStrainRef``\ 
     - | Real
     - | \ :math:`\varepsilon_0 + f\cRef \cdot \varepsilon\cRef`\ 
   * - | \returnValue
     - | Real
     - | scalar value of computed bending moment


--------

\ **User function example**\ :



.. code-block:: python

    #define some material parameters
    rhoA = 100.
    EA =   1e7.
    EI =   1e5

    #example of bending moment user function
    def bendingMomentUserFunction(mbs, t, itemNumber, axialPositionNormalized, 
               curvature, curvature_t, curvatureRef, physicsBendingStiffness, 
               physicsBendingDamping, axialStrain, axialStrain_t, axialStrainRef):
        fact = min(1,t) #runs from 0 to 1
        #change reference curvature of beam over time:
        kappa=(curvature-curvatureRef*fact) 
        return physicsBendingStiffness*(kappa) + physicsBendingDamping*curvature_t

    def axialForceUserFunction(mbs, t, itemNumber, axialPositionNormalized, 
               axialStrain, axialStrain_t, axialStrainRef, physicsAxialStiffness, 
               physicsAxialDamping, curvature, curvature_t, curvatureRef):
        fact = min(1,t) #runs from 0 to 1
        return (physicsAxialStiffness*(axialStrain-fact*axialStrainRef) + 
                physicsAxialDamping*axialStrain_t)

    cable = ObjectANCFCable2D(physicsMassPerLength=rhoA, 
                    physicsBendingStiffness=EI, 
                    physicsBendingDamping = EI*0.1,
                    physicsAxialStiffness=EA,
                    physicsAxialDamping=EA*0.05,
                    physicsReferenceAxialStrain=0.1, #10
                    physicsReferenceCurvature=1,     #radius=1
                    bendingMomentUserFunction=bendingMomentUserFunction,
                    axialForceUserFunction=axialForceUserFunction,
                    )
    #use  cable with GenerateStraightLineANCFCable(...)

 



.. _miniexample-objectancfcable2d:

MINI EXAMPLE for ObjectANCFCable2D
----------------------------------


.. code-block:: python
   :linenos:

   rhoA = 78.
   EA = 1000000.
   EI = 833.3333333333333
   cable = Cable2D(physicsMassPerLength=rhoA, 
                   physicsBendingStiffness=EI, 
                   physicsAxialStiffness=EA, 
                   )
   
   ancf=GenerateStraightLineANCFCable2D(mbs=mbs,
                   positionOfNode0=[0,0,0], positionOfNode1=[2,0,0],
                   numberOfElements=32, #converged to 4 digits
                   cableTemplate=cable, #this defines the beam element properties
                   massProportionalLoad = [0,-9.81,0],
                   fixedConstraintsNode0 = [1,1,0,1], #add constraints for pos and rot (r'_y)
                   )
   lastNode = ancf[0][-1]
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveStatic()
   
   #check result
   exudynTestGlobals.testResult = mbs.GetNodeOutput(lastNode, exu.OutputVariableType.Displacement)[0]
   #ux=-0.5013058140308901

Relevant Examples and TestModels with weblink:

    \ `sliderCrank3DwithANCFbeltDrive2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive2.py>`_\  (Examples/), \ `ALEANCFpipe.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ALEANCFpipe.py>`_\  (Examples/), \ `ANCFcantileverTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcantileverTest.py>`_\  (Examples/), \ `ANCFcantileverTestDyn.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcantileverTestDyn.py>`_\  (Examples/), \ `ANCFcontactCircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle.py>`_\  (Examples/), \ `ANCFcontactCircle2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFcontactCircle2.py>`_\  (Examples/), \ `ANCFmovingRigidbody.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFmovingRigidbody.py>`_\  (Examples/), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Examples/), \ `ANCFslidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2D.py>`_\  (Examples/), \ `ANCFslidingJoint2Drigid.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFslidingJoint2Drigid.py>`_\  (Examples/), \ `ANCFswitchingSlidingJoint2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFswitchingSlidingJoint2D.py>`_\  (Examples/), \ `ANCFtestHalfcircle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFtestHalfcircle.py>`_\  (Examples/), \ `ANCFcontactCircleTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactCircleTest.py>`_\  (TestModels/), \ `ANCFcontactFrictionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFcontactFrictionTest.py>`_\  (TestModels/), \ `computeODE2EigenvaluesTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/computeODE2EigenvaluesTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


