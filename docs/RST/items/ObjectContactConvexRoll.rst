

.. _sec-item-objectcontactconvexroll:

ObjectContactConvexRoll
=======================

A contact connector representing a convex roll (marker 1) on a flat surface (marker 0, ground body, not moving) in global \ :math:`x`\ -\ :math:`y`\  plane. The connector is similar to ObjectConnectorRollingDiscPenalty, but includes a (strictly) convex shape of the roll defined by a polynomial. It is based on a penalty formulation and adds friction and slipping. The formulation is still under development and needs further testing. Note that the rolling body must have the reference point at the center of the disc.

Author: Manzl Peter

\ **Additional information for ObjectContactConvexRoll**\ :

* | The Object has the following types = \ ``Connector``\ 
* | Requested marker type = \ ``Position``\  + \ ``Orientation``\ 
* | Requested node type = \ ``GenericData``\ 


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



\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}_{C}`\ 
  | current global position of contact point between roller and ground
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}_{C}`\ 
  | current velocity of the trail (contact) point in global coordinates; this is the velocity with which the contact moves over the ground plane
* | ``Force``\ : \ :math:`\LU{0}{{\mathbf{f}}}`\ 
  | Roll-ground force in ground coordinates
* | ``Torque``\ : \ :math:`\LU{0}{{\mathbf{m}}}`\ 
  | Roll-ground torque in ground coordinates




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


