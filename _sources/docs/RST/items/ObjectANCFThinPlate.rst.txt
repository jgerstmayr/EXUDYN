

.. _sec-item-objectancfthinplate:

ObjectANCFThinPlate
===================

OBJECT UNDER CONSTRUCTION: A 3D thin Kirchhoff plate finite element based on the absolute nodal coordinate formulation, using 4 nodes of type NodePointSlope12. The geometry as well as (deformed and distorted) reference configuration is given by the nodes. The localPosition follows unit-coordinates in the range [-1,1] for X, Y and Z coordinates; the thickness of the plate is h; This element is under construction.

\ **Additional information for ObjectANCFThinPlate**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``MultiNoded``\ 
* | Requested \ ``Node``\  type = \ ``Position``\ 


The item \ **ObjectANCFThinPlate**\  with type = 'ANCFThinPlate' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsThickness** [\ :math:`h`\ , type = NumpyVector, default = []]:
  | [SI:m] thickness of plate either provided as scalar or as vector (4 values, same order as local element node numbers) values that are linearly interpolated from nodal values; dimensionality must agree between thickness, strainCoefficients and curvatureCoefficients
* | **physicsDensity** [\ :math:`\rho`\ , type = UReal, default = 0.]:
  | [SI:kg/m\ :math:`^3`\ ] density of the plate, possibly averaged over thickness
* | **physicsMassProportionalDamping** [type = Real, default = 0.]:
  | mass-proportional damping coefficient \ :math:`\alpha`\  [SI:1/s]; adds massmatrix proportional damping forces \ :math:`{\mathbf{f}}_d = \alpha {\mathbf{M}} \dot{{\mathbf{q}}}`\ 
* | **physicsStrainCoefficients** [\ :math:`{\mathbf{D}}_\varepsilon`\ , type = Matrix3DList, default = []]:
  | [SI:N/m] stiffness coefficients related to inplane normal and shear strains, integrated over height of the plate; either given as 3D Matrix (numpy array), or a list of 3D matrices at each nodal point, see thickness; dimensionality must agree between thickness, strainCoefficients and curvatureCoefficients
* | **physicsCurvatureCoefficients** [\ :math:`{\mathbf{D}}_\kappa`\ , type = Matrix3DList, default = []]:
  | [SI:Nm] stiffness coefficients related to curvatures, integrated over height of the plate; either given as 3D Matrix (numpy array), or a list of 3D matrices at each nodal point, see thickness; dimensionality must agree between thickness, strainCoefficients and curvatureCoefficients
* | **strainIsRelativeToReference** [\ :math:`f\cRef`\ , type = Real, default = 1.]:
  | if set to 1., a pre-deformed reference configuration is considered as the stressless state; if set to 0., the straight configuration serves as a reference geometry; allows also values between 0. and 1. to perform a transition during static computation
* | **slopesScalingX** [type = Vector4D, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | scaling of x-slopes at each element node; flat elements: half of the side length of the element; curved: optimal values such that curved geometry is best approximated; if negative (default) values are used, length is computed from node distances.
* | **slopesScalingY** [type = Vector4D, size = 4, default = [-1.,-1.,-1.,-1.]]:
  | scaling of y-slopes at each element node; flat elements: half of the side length of the element; curved: optimal values such that curved geometry is best approximated; if negative (default) values are used, length is computed from node distances.
* | **nodeNumbers** [type = NodeIndex4, default = [invalid [-1], invalid [-1], invalid [-1], invalid [-1]]]:
  | 4 NodePointSlope12 node numbers, with local (xi,eta) coordinates as [(-1,-1),(1,-1),(1,1),(-1,1)]
* | **useReducedOrderIntegration** [type = Index, default = 0]:
  | 0/false: use highest Gauss integration for virtual work of strains
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

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}\cConfig(x,y,z)}`\ 
  | global position vector of local position \ :math:`[x,y,z]`\ 
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}\cConfig(x,y,z)} = \LU{0}{{\mathbf{p}}\cConfig(x,y,z)} - \LU{0}{{\mathbf{p}}\cRef(x,y,z)}`\ 
  | global displacement vector of local position
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}(x,y,z)} = \LU{0}{\dot {\mathbf{r}}(x,y,z)}`\ 
  | global velocity vector of local position
* | ``Director1``\ : \ :math:`{\mathbf{r}}_x(x,y,z)`\ 
  | (axial) slope vector of local position (at \ :math:`z`\ =0)
* | ``Director2``\ : \ :math:`{\mathbf{r}}_y(x,y,z)`\ 
  | (axial) slope vector of local position (at \ :math:`z`\ =0)
* | ``StrainLocal``\ : \ :math:`\varepsilon`\ 
  | axial strain (scalar) of local axis position (at Z=0)
* | ``CurvatureLocal``\ : \ :math:`[K_x, K_y, K_z]\tp`\ 
  | local curvature vector
* | ``ForceLocal``\ : \ :math:`N`\ 
  | (local) section normal force per length (scalar, including reference strains) (at \ :math:`z`\ =0)
* | ``TorqueLocal``\ : \ :math:`M`\ 
  | (local) bending moment per length (scalar) (at \ :math:`z`\ =0), which are bending moments as there is no torque
* | ``StressLocal``\ : 
  | local inplane stress components
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}(x,y,z)} = \LU{0}{\ddot {\mathbf{r}}(x,y,z)}`\ 
  | global acceleration vector of local position


Note: For output variables, the localPosition is defined in \ :math:`[-1,-1,-1] ... [1,1,1]`\ , where \ :math:`[-1,-1,0]`\  is the position of node 0.



.. _miniexample-objectancfthinplate:

MINI EXAMPLE for ObjectANCFThinPlate
------------------------------------


.. code-block:: python

   #to be done
   
   #check result
   exudynTestGlobals.testResult = 0


\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


