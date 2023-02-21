

.. _sec-item-objectancfcable2d:

ObjectANCFCable2D
=================

A 2D cable finite element using 2 nodes of type NodePoint2DSlope1. The localPosition of the beam with length \ :math:`L`\ =physicsLength and height \ :math:`h`\  ranges in \ :math:`X`\ -direction in range \ :math:`[0, L]`\  and in \ :math:`Y`\ -direction in range \ :math:`[-h/2,h/2]`\  (which is in fact not needed in the EOM).

\ **Additional information for ObjectANCFCable2D**\ :

* | Requested node type = \ ``Position2D``\  + \ ``Orientation2D``\  + \ ``Point2DSlope1``\  + \ ``Position``\  + \ ``Orientation``\ 
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
* | **visualization** [type = VObjectANCFCable2D]:
  | parameters for visualization of item



The item VObjectANCFCable2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawHeight** [type = float, default = 0.]:
  | if beam is drawn with rectangular shape, this is the drawing height
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color of the object; if R==-1, use default color



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




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


