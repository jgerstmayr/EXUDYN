

.. _sec-item-objectancfcable2d:

ObjectANCFCable2D
=================

A 2D cable finite element using 2 nodes of type NodePoint2DSlope1. The localPosition of the beam with length \ :math:`L`\ =physicsLength and height \ :math:`h`\  ranges in \ :math:`X`\ -direction in range \ :math:`[0, L]`\  and in \ :math:`Y`\ -direction in range \ :math:`[-h/2,h/2]`\  (which is in fact not needed in the EOM).
 



The item \ **ObjectANCFCable2D**\  with type = 'ANCFCable2D' has the following parameters:

 

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsLength** [type = UReal, default = 0.]:
  | [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives \ :math:`\rho A L`\ ; must be positive
* | **physicsMassPerLength** [type = UReal, default = 0.]:
  | [SI:kg/m] mass per length of beam
* | **physicsBendingStiffness** [type = UReal, default = 0.]:
  | [SI:Nm\ :math:`^2`\ ] bending stiffness of beam; the bending moment is \ :math:`m = EI (\kappa - \kappa_0)`\ , in which \ :math:`\kappa`\  is the material measure of curvature
* | **physicsAxialStiffness** [type = UReal, default = 0.]:
  | [SI:N] axial stiffness of beam; the axial force is \ :math:`f_ax = EA (\varepsilon -\varepsilon_0)`\ , in which \ :math:`\varepsilon = |{\mathbf{r}}^\prime|-1`\  is the axial strain
* | **physicsBendingDamping** [type = UReal, default = 0.]:
  | [SI:Nm\ :math:`^2`\ /s] bending damping of beam ; the additional virtual work due to damping is \ :math:`\delta W_\dot \kappa = \int_0^L \dot \kappa \delta \kappa dx`\ 
* | **physicsAxialDamping** [type = UReal, default = 0.]:
  | [SI:N/s] axial damping of beam; the additional virtual work due to damping is \ :math:`\delta W_\dot\varepsilon = \int_0^L \dot \varepsilon \delta \varepsilon dx`\ 
* | **physicsReferenceAxialStrain** [type = Real, default = 0.]:
  | [SI:1] reference axial strain of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference axial strain value
* | **physicsReferenceCurvature** [type = Real, default = 0.]:
  | [SI:1/m] reference curvature of beam (pre-deformation) of beam; without external loading the beam will statically keep the reference curvature value
* | **strainIsRelativeToReference** [type = Real, default = 0.]:
  | if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration plus the values of \ :math:`\varepsilon_0`\  and \ :math:`\kappa_0`\  serve as a reference geometry; allows also values between 0. and 1.
* | **nodeNumbers** [type = NodeIndex2, default = [invalid [-1], invalid [-1]]]:
  | two node numbers ANCF cable element
* | **useReducedOrderIntegration** [type = Index, default = 0]:
  | 0/false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; 1/true: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments



The item VObjectANCFCable2D has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawHeight** [type = float, default = 0.]:
  | if beam is drawn with rectangular shape, this is the drawing height
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color of the object; if R==-1, use default color




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


