

.. _sec-item-objectbeamgeometricallyexact2d:

ObjectBeamGeometricallyExact2D
==============================

A 2D geometrically exact beam finite element, currently using 2 nodes of type NodeRigidBody2D; FURTHER TESTS REQUIRED. Note that the orientation of the nodes need to follow the cross section orientation in case that includeReferenceRotations=True; e.g., an angle 0 represents the cross section aligned with the \ :math:`y`\ -axis, while and angle \ :math:`\pi/2`\  means that the cross section points in negative \ :math:`x`\ -direction. Pre-curvature can be included with physicsReferenceCurvature and axial pre-stress can be considered by using a physicsLength different from the reference configuration of the nodes. The localPosition of the beam with length \ :math:`L`\ =physicsLength and height \ :math:`h`\  ranges in \ :math:`X`\ -direction in range \ :math:`[-L/2, L/2]`\  and in \ :math:`Y`\ -direction in range \ :math:`[-h/2,h/2]`\  (which is in fact not needed in the \ :ref:`EOM <EOM>`\ ).

\ **Additional information for ObjectBeamGeometricallyExact2D**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``MultiNoded``\ 
* | Requested \ ``Node``\  type = \ ``Position2D``\  + \ ``Orientation2D``\  + \ ``Position``\  + \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``Beam2D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VBeam2D``\ 


The item \ **ObjectBeamGeometricallyExact2D**\  with type = 'BeamGeometricallyExact2D' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **nodeNumbers** [type = NodeIndex2, default = [invalid [-1], invalid [-1]]]:
  | two node numbers for beam element
* | **physicsLength** [\ :math:`L`\ , type = UReal, default = 0.]:
  | [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives \ :math:`\rho A L`\ ; must be positive
* | **physicsMassPerLength** [\ :math:`\rho A`\ , type = UReal, default = 0.]:
  | [SI:kg/m] mass per length of beam
* | **physicsCrossSectionInertia** [\ :math:`\rho J`\ , type = UReal, default = 0.]:
  | [SI:kg m] cross section mass moment of inertia; inertia acting against rotation of cross section
* | **physicsBendingStiffness** [\ :math:`EI`\ , type = UReal, default = 0.]:
  | [SI:Nm\ :math:`^2`\ ] bending stiffness of beam; the bending moment is \ :math:`m = EI (\kappa - \kappa_0)`\ , in which \ :math:`\kappa`\  is the material measure of curvature
* | **physicsAxialStiffness** [\ :math:`EA`\ , type = UReal, default = 0.]:
  | [SI:N] axial stiffness of beam; the axial force is \ :math:`f_{ax} = EA (\varepsilon -\varepsilon_0)`\ , in which \ :math:`\varepsilon`\  is the axial strain
* | **physicsShearStiffness** [\ :math:`GA`\ , type = UReal, default = 0.]:
  | [SI:N] effective shear stiffness of beam, including stiffness correction
* | **physicsBendingDamping** [\ :math:`d_{K}`\ , type = UReal, default = 0.]:
  | [SI:Nm\ :math:`^2`\ /s] viscous damping of bending deformation; the additional virtual work due to damping is \ :math:`\delta W_{\dot \kappa} = \int_0^L \dot \kappa \delta \kappa dx`\ 
* | **physicsAxialDamping** [\ :math:`d_{\varepsilon}`\ , type = UReal, default = 0.]:
  | [SI:N/s] viscous damping of axial deformation
* | **physicsShearDamping** [\ :math:`d_{\gamma}`\ , type = UReal, default = 0.]:
  | [SI:N/s] viscous damping of shear deformation
* | **physicsReferenceCurvature** [\ :math:`\kappa_0`\ , type = Real, default = 0.]:
  | [SI:1/m] reference curvature of beam (pre-deformation) of beam
* | **includeReferenceRotations** [type = bool, default = False]:
  | if True, rotations at nodes consider reference rotations, which are used for the computation of bending strains (this means that a pre-curved beam is stress-free); if False, the reference rotation of the cross section is orthogonal to the direction between the reference position of the end nodes. This allows to easily share nodes among several beams with different cross section orientation.
* | **visualization** [type = VObjectBeamGeometricallyExact2D]:
  | parameters for visualization of item



The item VObjectBeamGeometricallyExact2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawHeight** [type = float, default = 0.]:
  | if beam is drawn with rectangular shape, this is the drawing height
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color of the object; if R==-1, use default color


----------

.. _description-objectbeamgeometricallyexact2d:

DESCRIPTION of ObjectBeamGeometricallyExact2D
---------------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : 
  | global position vector of local axis (X) and cross section (Y) position
* | ``Displacement``\ : 
  | global displacement vector of local axis (X) and cross section (Y) position
* | ``Velocity``\ : 
  | global velocity vector of local axis (X) and cross section (Y) position
* | ``Rotation``\ : 
  | 3D Tait-Bryan rotation components, containing rotation around \ :math:`Z`\ -axis only
* | ``StrainLocal``\ : 
  | 6 (local) strain components, containing only axial (\ :math:`XX`\ , index 0) and shear strain (\ :math:`XY`\ , index 5); evaluated at beam axis ONLY
* | ``CurvatureLocal``\ : 
  | 3D vector of (local) curvature, only \ :math:`Z`\  component is non-zero
* | ``ForceLocal``\ : 
  | 3D vector of (local) section normal force, containing axial (X) and shear force (Y)
* | ``TorqueLocal``\ : 
  | 3D vector of (local) torques, containing only bending moment (Z)


See paper of Simo and Vu-Quoc (1986).
Detailed description coming later.


Relevant Examples and TestModels with weblink:

    \ `pendulumGeomExactBeam2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/pendulumGeomExactBeam2D.py>`_\  (Examples/), \ `ANCFBeamEigTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFBeamEigTest.py>`_\  (TestModels/), \ `geometricallyExactBeam2Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/geometricallyExactBeam2Dtest.py>`_\  (TestModels/), \ `gridGeomExactBeam2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/gridGeomExactBeam2D.py>`_\  (TestModels/), \ `LShapeGeomExactBeam2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/LShapeGeomExactBeam2D.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


