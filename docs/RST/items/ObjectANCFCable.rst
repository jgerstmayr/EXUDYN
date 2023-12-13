

.. _sec-item-objectancfcable:

ObjectANCFCable
===============

A 3D cable finite element using 2 nodes of type NodePointSlope1. The localPosition of the beam with length \ :math:`L`\ =physicsLength and height \ :math:`h`\  ranges in \ :math:`X`\ -direction in range \ :math:`[0, L]`\  and in \ :math:`Y`\ -direction in range \ :math:`[-h/2,h/2]`\  (which is in fact not needed in the \ :ref:`EOM <EOM>`\ ). For description see ObjectANCFCable2D, which is almost identical to 3D case. Note that this element does not include torsion, therfore a torque cannot be applied along the local x-axis.

\ **Additional information for ObjectANCFCable**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``MultiNoded``\ 
* | Requested \ ``Node``\  type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``Cable``\ 
* | \ **Short name**\  for Python visualization object = \ ``VCable``\ 


The item \ **ObjectANCFCable**\  with type = 'ANCFCable' has the following parameters:

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
* | **strainIsRelativeToReference** [\ :math:`f\cRef`\ , type = Real, default = 0.]:
  | if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration plus the values of \ :math:`\varepsilon_0`\  and \ :math:`\kappa_0`\  serve as a reference geometry; allows also values between 0. and 1.
* | **nodeNumbers** [type = NodeIndex2, default = [invalid [-1], invalid [-1]]]:
  | two node numbers ANCF cable element
* | **useReducedOrderIntegration** [type = Index, default = 0]:
  | 0/false: use Gauss order 9 integration for virtual work of axial forces, order 5 for virtual work of bending moments; 1/true: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments
* | **visualization** [type = VObjectANCFCable]:
  | parameters for visualization of item



The item VObjectANCFCable has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown; note that all quantities are computed at the beam centerline, even if drawn on surface of cylinder of beam; this effects, e.g., Displacement or Velocity, which is drawn constant over cross section
* | **radius** [type = float, default = 0.]:
  | if radius==0, only the centerline is drawn; else, a cylinder with radius is drawn; circumferential tiling follows general.cylinderTiling and beam axis tiling follows bodies.beams.axialTiling
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color of the object; if R==-1, use default color


----------

.. _description-objectancfcable:

DESCRIPTION of ObjectANCFCable
------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}\cConfig(x,0,0)} = {\mathbf{r}}\cConfig(x) + y\cdot {\mathbf{n}}\cConfig(x)`\ 
  | global position vector of local position \ :math:`[x,0,0]`\ 
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}\cConfig(x,0,0)} = \LU{0}{{\mathbf{p}}\cConfig(x,0,0)} - \LU{0}{{\mathbf{p}}\cRef(x,0,0)}`\ 
  | global displacement vector of local position
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}(x,0,0)} = \LU{0}{\dot {\mathbf{r}}(x)}`\ 
  | global velocity vector of local position
* | ``Director1``\ : \ :math:`{\mathbf{r}}'(x)`\ 
  | (axial) slope vector of local axis position (at \ :math:`y`\ =0)
* | ``StrainLocal``\ : \ :math:`\varepsilon`\ 
  | axial strain (scalar) of local axis position (at Y=Z=0)
* | ``CurvatureLocal``\ : \ :math:`[K_x, K_y, K_z]\tp`\ 
  | local curvature vector
* | ``ForceLocal``\ : \ :math:`N`\ 
  | (local) section normal force (scalar, including reference strains) (at \ :math:`y`\ =\ :math:`z`\ =0); note that strains are highly inaccurate when coupled to bending, thus consider useReducedOrderIntegration=2 and evaluate axial strain at nodes or at midpoint
* | ``TorqueLocal``\ : \ :math:`M`\ 
  | (local) bending moment (scalar) (at \ :math:`y`\ =\ :math:`z`\ =0), which are bending moments as there is no torque
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}(x,0,0)} = \LU{0}{\ddot {\mathbf{r}}(x)}`\ 
  | global acceleration vector of local position





.. _miniexample-objectancfcable:

MINI EXAMPLE for ObjectANCFCable
--------------------------------


.. code-block:: python
   :linenos:

   from exudyn.beams import GenerateStraightLineANCFCable
   rhoA = 78.
   EA = 1000000.
   EI = 833.3333333333333
   cable = Cable(physicsMassPerLength=rhoA, 
                 physicsBendingStiffness=EI, 
                 physicsAxialStiffness=EA, 
                 )
   
   ancf=GenerateStraightLineANCFCable(mbs=mbs,
                 positionOfNode0=[0,0,0], positionOfNode1=[2,0,0],
                 numberOfElements=32, #converged to 4 digits
                 cableTemplate=cable, #this defines the beam element properties
                 massProportionalLoad = [0,-9.81,0],
                 fixedConstraintsNode0 = [1,1,1, 0,1,1], #add constraints for pos and rot (r'_y,r'_z)
                 )
   lastNode = ancf[0][-1]
   
   #assemble and solve system for default parameters
   mbs.Assemble()
   mbs.SolveStatic()
   
   #check result
   exudynTestGlobals.testResult = mbs.GetNodeOutput(lastNode, exu.OutputVariableType.Displacement)[0]
   #ux=-0.5013058140308901


\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


