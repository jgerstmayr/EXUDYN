

.. _sec-item-objectcontactconvexroll:

ObjectContactConvexRoll
=======================

A contact connector representing a convex roll (marker 1) on a flat surface (marker 0, ground body, not moving) in global \ :math:`x`\ -\ :math:`y`\  plane. The connector is similar to ObjectConnectorRollingDiscPenalty, but includes a (strictly) convex shape of the roll defined by a polynomial. It is based on a penalty formulation and adds friction and slipping. The formulation is still under development and needs further testing. Note that the rolling body must have the reference point at the center of the disc.

Author: Manzl Peter

\ **Additional information for ObjectContactConvexRoll**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\  + \ ``Orientation``\ 
* | Requested \ ``Node``\  type = \ ``GenericData``\ 


The item \ **ObjectContactConvexRoll**\  with type = 'ContactConvexRoll' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, size =  2, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector; \ :math:`m0`\  represents the ground, which can undergo translations but not rotations, and \ :math:`m1`\  represents the rolling body, which has its reference point (=local position [0,0,0]) at the roll's center point
* | **nodeNumber** [\ :math:`n_d`\ , type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData (size=3) for 3 dataCoordinates, needed for discontinuous iteration (friction and contact)
* | **contactStiffness** [\ :math:`k_c`\ , type = Real, default = 0.]:
  | normal contact stiffness [SI:N/m]
* | **contactDamping** [\ :math:`d_c`\ , type = Real, default = 0.]:
  | normal contact damping [SI:N/(m s)]
* | **dynamicFriction** [\ :math:`\mu_d`\ , type = UReal, default = 0.]:
  | dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section :ref:`sec-module-physics`\ 
* | **staticFrictionOffset** [\ :math:`\mu_{s_off}`\ , type = UReal, default = 0.]:
  | static friction offset for friction model (static friction = dynamic friction + static offset), see StribeckFunction in exudyn.physics, Section :ref:`sec-module-physics`\ 
* | **viscousFriction** [\ :math:`\mu_v`\ , type = UReal, default = 0.]:
  | viscous friction coefficient (velocity dependent part) for friction model, see StribeckFunction in exudyn.physics, Section :ref:`sec-module-physics`\ 
* | **exponentialDecayStatic** [\ :math:`v_{exp}`\ , type = PReal, default = 1e-3]:
  | exponential decay of static friction offset (must not be zero!), see StribeckFunction in exudyn.physics (named expVel there!), Section :ref:`sec-module-physics`\ 
* | **frictionProportionalZone** [\ :math:`v_{reg}`\ , type = UReal, default = 1e-3]:
  | limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section :ref:`sec-module-physics`\ 
* | **rollLength** [\ :math:`L`\ , type = UReal, default = 0.]:
  | roll length [m], symmetric w.r.t.\ centerpoint
* | **coefficientsHull** [\ :math:`{\mathbf{k}} \in \Rcal^{n_p}`\ , type = NumpyVector, default =  []]:
  | a vector of polynomial coefficients, which provides the polynomial of the CONVEX hull of the roll; \ :math:`\mathrm{hull}(x) = k_0 x^{n_p-1} + k x^{n_p-2} + \ldots + k_{n_p-2} x  + k_{n_p-1}`\ 
* | **coefficientsHullDerivative** [\ :math:`{\mathbf{k}}^\prime \in \Rcal^{n_p}`\ , type = NumpyVector, default = []]:
  | polynomial coefficients of the polynomial \ :math:`\mathrm{hull}^\prime(x)`\ 
* | **coefficientsHullDDerivative** [type = NumpyVector, default = []]:
  | second derivative of the hull polynomial.
* | **rBoundingSphere** [type = UReal, default = 0]:
  | The  radius of the bounding sphere for the contact pre-check, calculated from the polynomial coefficients of the hull
* | **pContact** [type = Vector3D, default = [0,0,0]]:
  | The  current potential contact point. Contact occures if pContact[2] < 0. 
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectContactConvexRoll]:
  | parameters for visualization of item



The item VObjectContactConvexRoll has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectcontactconvexroll:

DESCRIPTION of ObjectContactConvexRoll
--------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}_{C}`\ 
  | current global position of contact point between roller and ground
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}_{C}`\ 
  | current velocity of the trail (contact) point in global coordinates; this is the velocity with which the contact moves over the ground plane
* | ``Force``\ : \ :math:`\LU{0}{{\mathbf{f}}}`\ 
  | Roll-ground force in ground coordinates
* | ``Torque``\ : \ :math:`\LU{0}{{\mathbf{m}}}`\ 
  | Roll-ground torque in ground coordinates



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
     - | current global position which is provided by marker m0, any ground reference point; currently unused
   * - | marker m0 orientation
     - | \ :math:`\LU{0,m0}{\Rot}`\ 
     - | current rotation matrix provided by marker m0; currently unused
   * - | marker m1 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1}`\ 
     - | center of roll
   * - | Contact position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{C}`\ 
     - | Position of the Contact point C in the global frame 0
   * - | Position marker m1 to contact
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{\mathrm{m1, C}}`\ 
     - | Position of the contact point C relative to the marker m1 in global frame
   * - | marker m1 orientation
     - | \ :math:`\LU{0,m1}{\Rot}`\ 
     - | current rotation matrix provided by marker m1
   * - | data coordinates
     - | \ :math:`{\mathbf{x}}=[x_0,\,x_1,\,x_2]\tp`\ 
     - | data coordinates for \ :math:`[x_0,\,x_1]`\ : hold the sliding velocity in lateral and longitudinal direction of last discontinuous iteration; \ :math:`x_2`\ : represents gap of last discontinuous iteration (in contact normal direction)
   * - | marker m1 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1}`\ 
     - | current global velocity which is provided by marker m1
   * - | marker m1 angular velocity
     - | \ :math:`\LU{0}{\tomega}_{m1}`\ 
     - | current angular velocity vector provided by marker m1
   * - | ground normal vector
     - | \ :math:`\LU{0}{{\mathbf{n}}}`\ 
     - | normalized normal vector to the (moving, but not rotating) ground, by default [0,0,1]



Geometric relations
-------------------

The geometrical setup is shown in \ :numref:`fig-objectcontactconvexroll-sketch`\ . To calculate the contact point of the convex body of revolution the contact (ground) plane is rotated into the local frame of the body. In this local frame in which the generatrix of the body of revolution is described by the polynomial function

.. math::
   :label: eq-connectorconvexrolling-polynomial

   \mathrm{r}(^bx) = \sum_{i=0}^n k_i \; x^{n-i}


with the coefficients of the hull \ :math:`a_i`\ . As a pre-Check for the contact two spheres are put into both ends of the object with the maximum radius and only if one of these is in contact. The contact point \ :math:`^{\mathrm{b}}{\mathbf{p}}_{\mathrm{m1,C}}`\  is calculated relative to the bodies marker \ ``m1``\  in the bodies local frame and transformed accordingly. 
The contact point C can for be calculated convex bodies by matching the derivative of the polynomial \ :math:`r(^bx)`\  with the gradient of the contact plane, shown in \ :numref:`fig-objectcontactconvexroll-sketch`\ , explained in detail in . 
At the contact point a normal force \ :math:`{\mathbf{f}}_{\mathrm{N}} = \begin{bmatrix} 0 & 0 & \mathrm{f}_{\mathrm{N}} \end{bmatrix}^T`\   with 

.. math::
   :label: eq-fpencontact

   \mathrm{f}_{\mathrm{N}} = \begin{cases} - (k_c \, z_{\mathrm{pen}} + d_c \,  \dot{z}_{\mathrm{pen}})  &\text{$z_{\mathrm{pen}}>0$} \\ 0 &\text{else} \end{cases}


acts against the penetration of the ground. The penetration depth \ :math:`z_{\mathrm{pen}}`\  is the z-component of the position vector of the contact point relative to the ground frame \ :math:`{^0{\mathbf{p}}_{\mathrm{C}}}`\ . 


.. _fig-objectcontactconvexroll-sketch:
.. figure:: ../../theDoc/figures/ConvexRolling.png
   :width: 600

   Sketch of the roller Dimensions. The rollers radius \ :math:`r({^bx})`\  is described by the polynomial \ ``coefficientsHull``\ .



The revolution results in a velocity of 

.. math::

   ^{0}{\mathbf{v}}_{C} ={^{0}{\tomega_{\mathrm{m1}}}} \times {^{0}{{\mathbf{p}}_{\mathrm{m1,\,C}}}}


in the contact point, while the tangential component of the velocity of the body itself with the normal Vector to the contact plane \ :math:`{\mathbf{n}}`\  follows to

.. math::

   \LURU{0}{{\mathbf{v}}}{\mathrm{m1,\,t}}{} = \LU{0}{{\mathbf{v}}_{\mathrm{m1}}} - {^0{\mathbf{n}}} \, \left({^0{\mathbf{n}}}^T \, \LU{0}{{\mathbf{v}}_{\mathrm{m1}}}\right).

 
Therefore the slip velocity of the body can be calculated with

.. math::

   \LURU{0}{{\mathbf{v}}}{\mathrm{s}}{} = \LURU{0}{{\mathbf{v}}}{C}{} - {^0{\mathbf{v}}_{\mathrm{m1,\,t}}}


and points in the direction 

.. math::

   \LURU{0}{{\mathbf{r}}}{s}{} = \frac{1}{\left\lVert \LURU{0}{{\mathbf{v}}}{\mathrm{s}}{}\right\rVert} {^0{{\mathbf{v}}}_{\mathrm{s}}}.


The slip force is then calculated

.. math::

   ^0{\mathbf{f}}_{\mathrm{s}} = \mu(\left\lVert\LU{}{^0{\mathbf{v}}_{\mathrm{s}}}\right\rVert)  \, \mathrm{f}_{\mathrm{N}} \, {^0{\mathbf{r}}_\mathrm{s}}


and uses for the friction coefficient \ :math:`\mu`\  the regularized friction approach from the StribeckFunction, see Section :ref:`sec-module-physics`\ . 
The torque 

.. math::

   ^0\ttau = {^0{\mathbf{p}}_{\mathrm{m1,\,C}}} \times (^0{\mathbf{f}}_{\mathrm{N}} + {^0{\mathbf{f}}_{\mathrm{s}}})


acts onto the body, resulting from the slip force acting not in the bodies center. 


Relevant Examples and TestModels with weblink:

    \ `ConvexContactTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ConvexContactTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


