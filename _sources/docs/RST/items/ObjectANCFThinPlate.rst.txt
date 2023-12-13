

.. _sec-item-objectancfthinplate:

ObjectANCFThinPlate
===================

A 3D thin Kirchhoff plate finite element based on the absolute nodal coordinate formulation, using 4 nodes of type NodePointSlope12. The geometry as well as (deformed and distorted) reference configuration is given by the nodes. The localPosition follows unit-coordinates in the range [-1,1] for X, Y and Z coordinates; the thickness of the plate is h; This element is under construction.

\ **Additional information for ObjectANCFThinPlate**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``MultiNoded``\ 
* | Requested \ ``Node``\  type = \ ``Position``\ 


The item \ **ObjectANCFThinPlate**\  with type = 'ANCFThinPlate' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsHeight** [\ :math:`L`\ , type = UReal, default = 0.]:
  | [SI:m] reference length of beam; such that the total volume (e.g. for volume load) gives \ :math:`\rho A L`\ ; must be positive
* | **physicsStrainCoefficients** [\ :math:`{\mathbf{D}}_\varepsilon`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | [SI:N/m] stiffness coefficients related to inplane normal and shear strains, integrated over height of the plate
* | **physicsCurvatureCoefficients** [\ :math:`{\mathbf{D}}_\kappa`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | [SI:Nm] stiffness coefficients related to curvatures, integrated over height of the plate
* | **strainIsRelativeToReference** [\ :math:`f\cRef`\ , type = Real, default = 0.]:
  | if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration serves as a reference geometry; allows also values between 0. and 1. to perform a transition during static computation
* | **nodeNumbers** [type = NodeIndex4, default = [invalid [-1], invalid [-1], invalid [-1], invalid [-1]]]:
  | 4 NodePointSlope12 node numbers
* | **useReducedOrderIntegration** [type = Index, default = 0]:
  | 0/false: use highest Gauss integration for virtual work of strains, order 5 for bending moments; 1/true: use Gauss order 7 integration for virtual work of axial forces, order 3 for virtual work of bending moments
* | **visualization** [type = VObjectANCFThinPlate]:
  | parameters for visualization of item



The item VObjectANCFThinPlate has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown; note that all quantities are computed at the beam centerline, even if drawn on surface of cylinder of beam; this effects, e.g., Displacement or Velocity, which is drawn constant over cross section
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA color of the object; if R==-1, use default color


----------

.. _description-objectancfthinplate:

DESCRIPTION of ObjectANCFThinPlate
----------------------------------

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





.. _miniexample-objectancfthinplate:

MINI EXAMPLE for ObjectANCFThinPlate
------------------------------------


.. code-block:: python
   :linenos:

   #NEEDS to be revised
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


