

.. _sec-item-objectjointrollingdisc:

ObjectJointRollingDisc
======================

A joint representing a rolling rigid disc (marker 1) on a flat surface (marker 0, ground body) in global \ :math:`x`\ -\ :math:`y`\  plane. The contraint is based on an idealized rolling formulation with no slip. The contraints works for discs as long as the disc axis and the plane normal vector are not parallel. It must be assured that the disc has contact to ground in the initial configuration (adjust z-position of body accordingly). The ground body can be a rigid body which is moving. In this case, the flat surface is assumed to be in the \ :math:`x`\ -\ :math:`y`\ -plane at \ :math:`z=0`\ . Note that the rolling body must have the reference point at the center of the disc. NOTE: the cases of normal other than \ :math:`z`\ -direction, wheel axis other than \ :math:`x`\ -axis and moving ground body needs to be tested further, check your results!

\ **Additional information for ObjectJointRollingDisc**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\  + \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``RollingDiscJoint``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRollingDiscJoint``\ 


The item \ **ObjectJointRollingDisc**\  with type = 'JointRollingDisc' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, size =  2, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector; \ :math:`m0`\  represents the ground and \ :math:`m1`\  represents the rolling body, which has its reference point (=local position [0,0,0]) at the disc center point
* | **constrainedAxes** [\ :math:`{\mathbf{j}}=[j_0,\,\ldots,\,j_2]`\ , type = ArrayIndex, size = 3, default = [1,1,1]]:
  | flags, which determine which constraints are active, in which \ :math:`j_0`\  represents lateral motion, \ :math:`j_1`\  longitudinal (forward/backward) motion and \ :math:`j_2`\  represents the normal (contact) direction
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **discRadius** [type = PReal, default = 0]:
  | defines the disc radius
* | **discAxis** [\ :math:`\LU{m1}{{\mathbf{w}}_{1}}, \;\; |\LU{m1}{{\mathbf{w}}_{1}}| = 1`\ , type = Vector3D, default = [1,0,0]]:
  | axis of disc defined in marker \ :math:`m1`\  frame
* | **planeNormal** [\ :math:`\LU{m0}{{\mathbf{v}}_{PN}}`\ , type = Vector3D, default = [0,0,1]]:
  | normal to the contact / rolling plane defined in marker \ :math:`m0`\  coordinates
* | **visualization** [type = VObjectJointRollingDisc]:
  | parameters for visualization of item



The item VObjectJointRollingDisc has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **discWidth** [type = float, default = 0.1]:
  | width of disc for drawing
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectjointrollingdisc:

DESCRIPTION of ObjectJointRollingDisc
-------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}_{G}`\ 
  | current global position of contact point between rolling disc and ground
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}_{trail}`\ 
  | current velocity of the trail (according to motion of the contact point along the trail!) in global coordinates; this is not the velocity of the contact point; needs further testing for general case of relative moving bodies
* | ``ForceLocal``\ : \ :math:`\LU{J1}{{\mathbf{f}}} = \LU{0}{[f_0,\, f_1,\, f_2]\tp}= [-{\mathbf{z}}^T \LU{0}{{\mathbf{w}}_{lat}}, \, -{\mathbf{z}}^T \LU{0}{{\mathbf{w}}_2}, \, -{\mathbf{z}}^T \LU{0}{{\mathbf{v}}_{PN}}]\tp`\ 
  | contact forces acting on disc, in special \ :math:`J1`\  joint coordinates, \ :math:`f_0`\  being the lateral force (parallel to ground plane), \ :math:`f_1`\  being the longitudinal force and \ :math:`f_2`\  being the normal force
* | ``RotationMatrix``\ : \ :math:`\LU{0,J1}{{\mathbf{A}}} = [\LU{0}{{\mathbf{w}}_{lat}},\, \LU{0}{{\mathbf{w}}}_2,\, \LU{0}{{\mathbf{v}}_{PN}}]`\ 
  | transformation matrix of special joint coordinates \ :math:`J1`\  to global coordinates



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
     - | current global position of marker \ :math:`m0`\ ; needed only if body \ :math:`m0`\  is not a ground body
   * - | marker m0 orientation
     - | \ :math:`\LU{0,m0}{\Rot}`\ 
     - | current rotation matrix provided by marker m0 (assumed to be rigid body)
   * - | marker m0 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity which is provided by marker m0 (assumed to be rigid body)
   * - | marker m0 angular velocity
     - | \ :math:`\LU{0}{\tomega}_{m0}`\ 
     - | current angular velocity vector provided by marker m0 (assumed to be rigid body)
   * - | marker m1 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1}`\ 
     - | center of disc
   * - | marker m1 orientation
     - | \ :math:`\LU{0,m1}{\Rot}`\ 
     - | current rotation matrix provided by marker m1
   * - | marker m1 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1}`\ 
     - | accordingly
   * - | marker m1 angular velocity
     - | \ :math:`\LU{0}{\tomega}_{m1}`\ 
     - | current angular velocity vector provided by marker m1
   * - | ground normal vector
     - | \ :math:`\LU{0}{{\mathbf{v}}_{PN}} = \LU{0,m0}{{\mathbf{A}}} \LU{m0}{{\mathbf{v}}_{PN}}`\ 
     - | normalized normal vector to the ground plane, moving with marker \ :math:`m0`\ 
   * - | ground position B
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{B}`\ 
     - | disc center point projected on ground in plane normal (\ :math:`z`\ -direction, \ :math:`z=0`\ )
   * - | ground position C
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{C}`\ 
     - | contact point of disc with ground in global coordinates
   * - | ground velocity C
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{Cm1}`\ 
     - | velocity of disc (marker 1) at ground contact point (must be zero if ground does not move)
   * - | ground velocity C
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{Cm2}`\ 
     - | velocity of ground (marker 0) at ground contact point (is always zero if ground does not move)
   * - | wheel axis vector
     - | \ :math:`\LU{0}{{\mathbf{w}}_1} =\LU{0,m1}{\Rot} \LU{m1}{{\mathbf{w}}_{1}}`\ 
     - | normalized disc axis vector
   * - | longitudinal vector
     - | \ :math:`\LU{0}{{\mathbf{w}}_2}`\ 
     - | vector in longitudinal (motion) direction
   * - | lateral vector
     - | \ :math:`\LU{0}{{\mathbf{w}}_{lat}} = \LU{0}{{\mathbf{v}}_{PN}} \times \LU{0}{{\mathbf{w}}}_2`\ 
     - | vector in lateral direction, parallel to ground plane
   * - | contact point vector
     - | \ :math:`\LU{0}{{\mathbf{w}}_3}`\ 
     - | normalized vector from disc center point in direction of contact point C
   * - | \ :math:`D1`\  transformation matrix
     - | \ :math:`\LU{0,D1}{{\mathbf{A}}} = [\LU{0}{{\mathbf{w}}_1},\, \LU{0}{{\mathbf{w}}_2},\, \LU{0}{{\mathbf{w}}_3}]`\ 
     - | transformation of special disc coordinates \ :math:`D1`\  to global coordinates
   * - | \ :math:`J1`\  transformation matrix
     - | \ :math:`\LU{0,J1}{{\mathbf{A}}} = [\LU{0}{{\mathbf{w}}_{lat}},\, \LU{0}{{\mathbf{w}}}_2,\, \LU{0}{{\mathbf{v}}_{PN}}]`\ 
     - | transformation of special joint \ :math:`J1`\  coordinates to global coordinates
   * - | algebraic variables
     - | \ :math:`{\mathbf{z}}=[\lambda_0,\,\lambda_1,\,\lambda_2]\tp`\ 
     - | vector of algebraic variables (Lagrange multipliers) according to the algebraic equations


Geometric relations
-------------------

The main geometrical setup is shown in the following figure:

First, the contact point \ :math:`\LU{0}{{\mathbf{p}}}_{C}`\  must be computed.
With the helper vector,

.. math::

   \LU{0}{{\mathbf{x}}} = \LU{0}{{\mathbf{w}}}_1 \times \LU{0}{{\mathbf{v}}_{PN}}


we obtain a disc coordinate system, representing the longitudinal direction,

.. math::

   \LU{0}{{\mathbf{w}}}_2 = \frac{1}{|\LU{0}{{\mathbf{x}}}|} \LU{0}{{\mathbf{x}}}


and the vector to the contact point,

.. math::

   \LU{0}{{\mathbf{w}}}_3 = \LU{0}{{\mathbf{w}}}_1 \times \LU{0}{{\mathbf{w}}}_2


The contact point \ :math:`C`\  can be computed from

.. math::

   \LU{0}{{\mathbf{p}}}_{C} = \LU{0}{{\mathbf{p}}}_{m1} + r \cdot \LU{0}{{\mathbf{w}}}_3


The velocity of the contact point at the disc is computed from,

.. math::

   \LU{0}{{\mathbf{v}}}_{Cm1} = \LU{0}{{\mathbf{v}}}_{m1} + \LU{0}{\tomega}_{m1} \times (r\cdot \LU{0}{{\mathbf{w}}}_3)


If marker 0 body is (moving) rigid body instead of a ground body, the contact point \ :math:`C`\  is reconstructed in 
body of marker 0,

.. math::

   \LU{m0}{{\mathbf{p}}}_{C} = \LU{m0,0}{\Rot} (\LU{0}{{\mathbf{p}}}_{C} - \LU{0}{{\mathbf{p}}}_{m0})


The velocity of the contact point at the marker 0 body reads

.. math::

   \LU{0}{{\mathbf{v}}}_{Cm0} = \LU{0}{{\mathbf{v}}}_{m0} + \LU{0}{\tomega}_{m0} \times \left( \LU{0,m0}{\Rot} \LU{m0}{{\mathbf{p}}}_{C} \right)



Connector constraint equations
------------------------------

Constraints for \ ``activeConnector = True``\ :

The non-holonomic, index 2 constraints for the tangential and normal contact follow from (an index 3 formulation would be possible, but is not implemented yet because of mixing different jacobians)

.. math::

   \LU{J1,0}{{\mathbf{A}}} \left(\vr{\LU{0}{{\mathbf{v}}}_{Cm1,x}}{\LU{0}{{\mathbf{v}}}_{Cm1,y}}{\LU{0}{{\mathbf{v}}}_{Cm1,z}} - \vr{\LU{0}{{\mathbf{v}}}_{Cm0,x}}{\LU{0}{{\mathbf{v}}}_{Cm0,y}}{\LU{0}{{\mathbf{v}}}_{Cm0,z}} \right) = \Null


In case that \ ``activeConnector = False``\ , the Lagrange multipliers are set to zero:

.. math::

   {\mathbf{z}} = \Null


Note that since version 1.8.27 the constraints can be turned on/off separately with \ ``constrainedAxes=[b0,b1,b2]``\ , in which
\ ``b0``\  represents the flag for lateral motion, \ ``b1``\  switches the constraint for forward motion and \ ``b2``\  for motion in plane normal direction.


Relevant Examples and TestModels with weblink:

    \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Examples/), \ `reinforcementLearningRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/reinforcementLearningRobot.py>`_\  (Examples/), \ `rollingCoinTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rollingCoinTest.py>`_\  (TestModels/), \ `rollingDiscTangentialForces.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rollingDiscTangentialForces.py>`_\  (TestModels/), \ `rotatingTableTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rotatingTableTest.py>`_\  (TestModels/), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TestModels/), \ `createRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createRollingDiscTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


