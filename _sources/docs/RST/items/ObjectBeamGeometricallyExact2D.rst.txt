

.. _sec-item-objectbeamgeometricallyexact2d:

ObjectBeamGeometricallyExact2D
==============================

A 2D geometrically exact beam finite element, currently using 2 nodes of type NodeRigidBody2D; FURTHER TESTS REQUIRED. Note that the orientation of the nodes need to follow the cross section orientation; e.g., an angle 0 represents the cross section pointing in \ :math:`y`\ -direction, while and angle \ :math:`\pi`\  means that the cross section points in negative \ :math:`x`\ -direction and the axis shows in positive \ :math:`y`\ -direction. The localPosition of the beam with length \ :math:`L`\ =physicsLength and height \ :math:`h`\  ranges in \ :math:`X`\ -direction in range \ :math:`[-L/2, L/2]`\  and in \ :math:`Y`\ -direction in range \ :math:`[-h/2,h/2]`\  (which is in fact not needed in the EOM).
 



The item \ **ObjectBeamGeometricallyExact2D**\  with type = 'BeamGeometricallyExact2D' has the following parameters:

 

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumbers** [type = NodeIndex2, default = [invalid [-1], invalid [-1]]]:
  | two node numbers for beam element
* | **physicsLength** [type = UReal, default = 0.]:
  | [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives \ :math:`\rho A L`\ ; must be positive
* | **physicsMassPerLength** [type = UReal, default = 0.]:
  | [SI:kg/m] mass per length of beam
* | **physicsCrossSectionInertia** [type = UReal, default = 0.]:
  | [SI:kg m] cross section mass moment of inertia; inertia acting against rotation of cross section
* | **physicsBendingStiffness** [type = UReal, default = 0.]:
  | [SI:Nm\ :math:`^2`\ ] bending stiffness of beam; the bending moment is \ :math:`m = EI (\kappa - \kappa_0)`\ , in which \ :math:`\kappa`\  is the material measure of curvature
* | **physicsAxialStiffness** [type = UReal, default = 0.]:
  | [SI:N] axial stiffness of beam; the axial force is \ :math:`f_ax = EA (\varepsilon -\varepsilon_0)`\ , in which \ :math:`\varepsilon`\  is the axial strain
* | **physicsShearStiffness** [type = UReal, default = 0.]:
  | [SI:N] effective shear stiffness of beam, including stiffness correction
* | **physicsBendingDamping** [type = UReal, default = 0.]:
  | [SI:Nm\ :math:`^2`\ /s] viscous damping of bending deformation; the additional virtual work due to damping is \ :math:`\delta W_\dot \kappa = \int_0^L \dot \kappa \delta \kappa dx`\ 
* | **physicsAxialDamping** [type = UReal, default = 0.]:
  | [SI:N/s] viscous damping of axial deformation
* | **physicsShearDamping** [type = UReal, default = 0.]:
  | [SI:N/s] viscous damping of shear deformation
* | **physicsReferenceCurvature** [type = Real, default = 0.]:
  | [SI:1/m] reference curvature of beam (pre-deformation) of beam
* | **includeReferenceRotations** [type = bool, default = False]:
  | if True, the computation of bending strains considers reference rotations in nodes; otherwise, the strains are relative to reference values (which allows to consider pre-curved geometries naturally)



The item VObjectBeamGeometricallyExact2D has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawHeight** [type = float, default = 0.]:
  | if beam is drawn with rectangular shape, this is the drawing height
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color of the object; if R==-1, use default color




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


