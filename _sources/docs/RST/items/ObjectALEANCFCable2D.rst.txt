

.. _sec-item-objectaleancfcable2d:

ObjectALEANCFCable2D
====================

A 2D cable finite element using 2 nodes of type NodePoint2DSlope1 and a axially moving coordinate of type NodeGenericODE2, which adds additional (redundant) motion in axial direction of the beam. This allows modeling pipes but also axially moving beams. The localPosition of the beam with length \ :math:`L`\ =physicsLength and height \ :math:`h`\  ranges in \ :math:`X`\ -direction in range \ :math:`[0, L]`\  and in \ :math:`Y`\ -direction in range \ :math:`[-h/2,h/2]`\  (which is in fact not needed in the \ :ref:`EOM <EOM>`\ ).

\ **Additional information for ObjectALEANCFCable2D**\ :

* | Requested \ ``Node``\  type: read detailed information of item
* | \ **Short name**\  for Python = \ ``ALECable2D``\ 
* | \ **Short name**\  for Python visualization object = \ ``VALECable2D``\ 


The item \ **ObjectALEANCFCable2D**\  with type = 'ALEANCFCable2D' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsLength** [\ :math:`L`\ , type = UReal, default = 0.]:
  | [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives \ :math:`\rho A L`\ ; must be positive
* | **physicsMassPerLength** [\ :math:`\rho A`\ , type = UReal, default = 0.]:
  | [SI:kg/m] total mass per length of beam (including axially moving parts / fluid)
* | **physicsMovingMassFactor** [type = UReal, default = 1.]:
  | this factor denotes the amount of \ :math:`\rho A`\  which is moving; physicsMovingMassFactor=1 means, that all mass is moving; physicsMovingMassFactor=0 means, that no mass is moving; factor can be used to simulate e.g. pipe conveying fluid, in which \ :math:`\rho A`\  is the mass of the pipe+fluid, while \ :math:`physicsMovingMassFactor \cdot \rho A`\  is the mass per unit length of the fluid
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
* | **physicsUseCouplingTerms** [type = Bool, default = True]:
  | true: correct case, where all coupling terms due to moving mass are respected; false: only include constant mass for ALE node coordinate, but deactivate other coupling terms (behaves like ANCFCable2D then)
* | **physicsAddALEvariation** [type = Bool, default = True]:
  | true: correct case, where additional terms related to variation of strain and curvature are added
* | **nodeNumbers** [type = NodeIndex3, default = [invalid [-1], invalid [-1], invalid [-1]]]:
  | two node numbers ANCF cable element, third node=ALE GenericODE2 node
* | **useReducedOrderIntegration** [type = Index, default = 0]:
  | 0/false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; 1/true: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments
* | **strainIsRelativeToReference** [\ :math:`f\cRef`\ , type = Real, default = 0.]:
  | if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration plus the values of \ :math:`\varepsilon_0`\  and \ :math:`\kappa_0`\  serve as a reference geometry; allows also values between 0. and 1.
* | **visualization** [type = VObjectALEANCFCable2D]:
  | parameters for visualization of item



The item VObjectALEANCFCable2D has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **drawHeight** [type = float, default = 0.]:
  | if beam is drawn with rectangular shape, this is the drawing height
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color of the object; if R==-1, use default color


----------

.. _description-objectaleancfcable2d:

DESCRIPTION of ObjectALEANCFCable2D
-----------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : 
  | global position vector of local position (in X/Y beam coordinates)
* | ``Displacement``\ : 
  | global displacement vector of local position
* | ``Velocity``\ : 
  | global velocity vector of local position
* | ``VelocityLocal``\ : 
  | local velocity vector of local position
* | ``Rotation``\ : 
  | (scalar) rotation angle of axial slope vector (relative to global x-axis)
* | ``Director1``\ : 
  | (axial) slope vector of local axis position (at Y=0)
* | ``StrainLocal``\ : \ :math:`\varepsilon`\ 
  | axial strain (scalar) of local axis position (at Y=0)
* | ``CurvatureLocal``\ : \ :math:`K`\ 
  | axial strain (scalar)
* | ``ForceLocal``\ : \ :math:`N`\ 
  | (local) section normal force (scalar, including reference strains) (at Y=0); note that strains are highly inaccurate when coupled to bending, thus consider useReducedOrderIntegration=2 and evaluate axial strain at nodes or at midpoint
* | ``TorqueLocal``\ : \ :math:`M`\ 
  | (local) bending moment (scalar) (at Y=0)


A 2D cable finite element using 2 nodes of type NodePoint2DSlope1 and an axially moving coordinate of type NodeGenericODE2.
The element has 8+1 coordinates and uses cubic polynomials for position interpolation.
In addition to ANCFCable2D the element adds an Eulerian axial velocity by the GenericODE2 coordiante.
The parameter \ ``physicsMovingMassFactor``\  allows to control the amount of mass, which moves with
the Eulerian velocity (e.g., the fluid), and which is not moving (the pipe). 
A factor of \ ``physicsMovingMassFactor=1``\  gives an axially moving beam.

The Bernoulli-Euler beam is capable of large deformation as it employs the material measure of curvature for the bending.
Note that damping (physicsBendingDamping, physicsAxialDamping) only acts on the non-moving part of the beam, as it is the case for the pipe.

Note that most functions act on the underlying cable finite element, which is not co-moving axially. E.g., if you apply constraints
to the nodal coordinates, the cable can be fixed, while still the axial component is freely moving.
If you apply a LoadForce using a MarkerPosition, the force is acting on the beam finite element, but not on the axially moving coordinate.
In contrast to the latter, the ObjectJointALEMoving2D and the MarkerBodyMass are acting on the moving coordinate as well.

A detailed paper on this element is yet under submission, but a similar formulation can be found in  and 
the underlying beam element is identical to ObjectANCFCable2D.


Relevant Examples and TestModels with weblink:

    \ `ALEANCFpipe.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ALEANCFpipe.py>`_\  (Examples/), \ `ANCFALEtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFALEtest.py>`_\  (Examples/), \ `ANCFmovingRigidbody.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFmovingRigidbody.py>`_\  (Examples/), \ `flexiblePendulumANCF.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/flexiblePendulumANCF.py>`_\  (Examples/), \ `ANCFoutputTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ANCFoutputTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


