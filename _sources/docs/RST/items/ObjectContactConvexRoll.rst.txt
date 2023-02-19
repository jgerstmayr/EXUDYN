

.. _sec-item-objectcontactconvexroll:

ObjectContactConvexRoll
=======================

A contact connector representing a convex roll (marker 1) on a flat surface (marker 0, ground body, not moving) in global \ :math:`x`\ -\ :math:`y`\  plane. The connector is similar to ObjectConnectorRollingDiscPenalty, but includes a (strictly) convex shape of the roll defined by a polynomial. It is based on a penalty formulation and adds friction and slipping. The formulation is still under development and needs further testing. Note that the rolling body must have the reference point at the center of the disc.
 



The item \ **ObjectContactConvexRoll**\  with type = 'ContactConvexRoll' has the following parameters:

 

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [type = ArrayMarkerIndex, default = [ invalid [-1], invalid [-1] ], size =  2]:
  | list of markers used in connector; \ :math:`m0`\  represents the ground, which can undergo translations but not rotations, and \ :math:`m1`\  represents the rolling body, which has its reference point (=local position [0,0,0]) at the roll's center point
* | **nodeNumber** [type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData (size=3) for 3 dataCoordinates, needed for discontinuous iteration (friction and contact)
* | **contactStiffness** [type = Real, default = 0.]:
  | normal contact stiffness [SI:N/m]
* | **contactDamping** [type = Real, default = 0.]:
  | normal contact damping [SI:N/(m s)]
* | **dynamicFriction** [type = UReal, default = 0.]:
  | dynamic friction coefficient for friction model, see StribeckFunction in exudyn.physics, Section :ref:`sec-module-physics`\ 
* | **staticFrictionOffset** [type = UReal, default = 0.]:
  | static friction offset for friction model (static friction = dynamic friction + static offset), see StribeckFunction in exudyn.physics, Section :ref:`sec-module-physics`\ 
* | **viscousFriction** [type = UReal, default = 0.]:
  | viscous friction coefficient (velocity dependent part) for friction model, see StribeckFunction in exudyn.physics, Section :ref:`sec-module-physics`\ 
* | **exponentialDecayStatic** [type = PReal, default = 1e-3]:
  | exponential decay of static friction offset (must not be zero!), see StribeckFunction in exudyn.physics (named expVel there!), Section :ref:`sec-module-physics`\ 
* | **frictionProportionalZone** [type = UReal, default = 1e-3]:
  | limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations), see StribeckFunction in exudyn.physics (named regVel there!), Section :ref:`sec-module-physics`\ 
* | **rollLength** [type = UReal, default = 0.]:
  | roll length [m], symmetric w.r.t.\ centerpoint
* | **coefficientsHull** [type = NumpyVector, default =  []]:
  | a vector of polynomial coefficients, which provides the polynomial of the CONVEX hull of the roll; \ :math:`\mathrmhull(x) = k_0 x^n_p-1 + k x^n_p-2 + \ldots + k_n_p-2 x  + k_n_p-1`\ 
* | **coefficientsHullDerivative** [type = NumpyVector, default = []]:
  | polynomial coefficients of the polynomial \ :math:`\mathrmhull^\prime(x)`\ 
* | **coefficientsHullDDerivative** [type = NumpyVector, default = []]:
  | second derivative of the hull polynomial.
* | **rBoundingSphere** [type = UReal, default = 0]:
  | The  radius of the bounding sphere for the contact pre-check, calculated from the polynomial coefficients of the hull
* | **pContact** [type = Vector3D, default = [0,0,0]]:
  | The  current potential contact point. Contact occures if pContact[2] < 0. 
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint



The item VObjectContactConvexRoll has the following parameters:

 

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color




\ **This is only a small part of information on this item. For details see the Exudyn documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


