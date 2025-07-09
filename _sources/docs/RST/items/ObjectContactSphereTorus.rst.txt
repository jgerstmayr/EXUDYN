

.. _sec-item-objectcontactspheretorus:

ObjectContactSphereTorus
========================

[UNDER DEVELOPMENT] A simple contact connector between a sphere (marker0) and a torus (marker1). The sphere is assumed to be placed inside of the torus (outer contact of sphere with torus currently not implemented!).

Author: Gerstmayr Johannes

\ **Additional information for ObjectContactSphereTorus**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\  + \ ``Orientation``\ 
* | Requested \ ``Node``\  type = \ ``GenericData``\ 


The item \ **ObjectContactSphereTorus**\  with type = 'ContactSphereTorus' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, size =  2, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers representing centers of sphere (marker 0) and center of torus (marker 1)
* | **nodeNumber** [\ :math:`n_d`\ , type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData with numberOfDataCoordinates = 4 dataCoordinates, needed for discontinuous iteration (friction and contact); data variables contain values from last PostNewton iteration: data[0] is the  gap, data[1] is the norm of the tangential velocity (and thus contains information if it is stick or slip); data[2] is the impact velocity; data[3] is unused.
* | **radiusSphere** [\ :math:`r_S`\ , type = PReal, default = 0.]:
  | radius of sphere [SI:m]
* | **torusMajorRadius** [\ :math:`r_{M}`\ , type = PReal, default = 0.]:
  | major radius of torus [SI:m], representing center of rotated circle
* | **torusMinorRadius** [\ :math:`r_{m}`\ , type = PReal, default = 0.]:
  | minor radius of torus [SI:m], representing radius of circle of ring
* | **torusAxis** [\ :math:`{\mathbf{v}}_{axis}`\ , type = Vector3D, size = 3, default = [0,0,0]]:
  | Vector containing rotation axis of torus; must be a unit vector.
* | **dynamicFriction** [\ :math:`\mu_d`\ , type = UReal, default = 0.]:
  | dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section :ref:`sec-module-physics`\ 
* | **frictionProportionalZone** [\ :math:`v_{reg}`\ , type = UReal, default = 1e-3]:
  | limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section :ref:`sec-module-physics`\ 
* | **contactStiffness** [\ :math:`k_c`\ , type = UReal, default = 0.]:
  | normal contact stiffness [SI:N/m] (units in case that \ :math:`n_\mathrm{exp}=1`\ )
* | **contactDamping** [\ :math:`d_c`\ , type = UReal, default = 0.]:
  | linear normal contact damping [SI:N/(m s)]; this damping should be used (!=0) if the restitution coefficient is < 1, as it changes its behavior.
* | **contactStiffnessExponent** [\ :math:`n_\mathrm{exp}`\ , type = PReal, default = 1.]:
  | exponent in normal contact model [SI:1]
* | **restitutionCoefficient** [\ :math:`e_\mathrm{res}`\ , type = PReal, default = 1.]:
  | coefficient of restitution [SI:1]; used in particular for impact mechanics; different models available within parameter impactModel; the coefficient must be > 0, but can become arbitrarily small to emulate plastic impact (however very small values may lead to numerical problems)
* | **minimumImpactVelocity** [\ :math:`\dot\delta_\mathrm{-,min}`\ , type = UReal, default = 0.]:
  | minimal impact velocity for coefficient of restitution [SI:1]; this value adds a lower bound for impact velocities for calculation of viscous impact force; it can be used to apply a larger damping behavior for low impact velocities (or permanent contact)
* | **impactModel** [\ :math:`m_\mathrm{impact}`\ , type = UInt, default = 0]:
  | number of impact model: 0) linear model (only linear damping is used); 1) Hunt-Crossley model; 2) Gonthier/EtAl-Carvalho/Martins mixed model; model 2 is much more accurate regarding the coefficient of restitution, in the full range [0,1] except for 0; NOTE: in all models, the linear contactDamping is added, if not set to zero!
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectContactSphereTorus]:
  | parameters for visualization of item



The item VObjectContactSphereTorus has the following parameters:

* | **show** [type = Bool, default = False]:
  | set true, if item is shown in visualization and false if it is not shown; draws spheres by given radii
* | **color** [type = Float4, default = [0.7,0.7,0.7,1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectcontactspheretorus:

DESCRIPTION of ObjectContactSphereTorus
---------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : 
  | contact center point (also given for positive gap, when no contact occurs)
* | ``Displacement``\ : 
  | global displacement vector between the two spheres midpoints
* | ``DisplacementLocal``\ : 
  | 1D Vector, containing only gap
* | ``Director1``\ : 
  | normalized vector from marker 0 to marker 1
* | ``Director2``\ : 
  | the normalized vector from marker 0 to marker 1 projected into the plane of the torus major circle
* | ``Director3``\ : 
  | normalized vector from the projected point on the major circle (center of the minor circle) to marker 1, being in direction of the contact and normal to the surface
* | ``Force``\ : 
  | global contact force vector
* | ``Torque``\ : 
  | global torque due to friction on marker 0



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
     - | global position of torus 0 center as provided by marker m0
   * - | marker m0 orientation
     - | \ :math:`\LU{0,m0}{\Rot}`\ 
     - | current rotation matrix provided by marker m0
   * - | marker m1 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1}`\ 
     - | global position of sphere 1 center as provided by marker m1
   * - | marker m1 orientation
     - | \ :math:`\LU{0,m1}{\Rot}`\ 
     - | current rotation matrix provided by marker m1
   * - | data coordinates
     - | \ :math:`{\mathbf{x}}=[x_0,\,x_1,\,x_2,\,x_3]\tp`\ 
     - | hold the current gap (0), the (norm of the) tangential velocity (1), the impact velocity (2), and (3) which is undefined 
   * - | marker m0 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity which is provided by marker m0
   * - | marker m1 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1}`\ 
     - | current global velocity which is provided by marker m1
   * - | marker m0 angular velocity
     - | \ :math:`\LU{0}{\tomega}_{m0}`\ 
     - | current angular velocity vector provided by marker m0
   * - | marker m1 angular velocity
     - | \ :math:`\LU{0}{\tomega}_{m1}`\ 
     - | current angular velocity vector provided by marker m1


Connector Forces
----------------

TBD



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


