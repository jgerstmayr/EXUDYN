

.. _sec-item-objectconnectorrollingdiscpenalty:

ObjectConnectorRollingDiscPenalty
=================================

A (flexible) connector representing a rolling rigid disc (marker 1) on a flat surface (marker 0, ground body, not moving) in global \ :math:`x`\ -\ :math:`y`\  plane. The connector is based on a penalty formulation and adds friction and slipping. The contraints works for discs as long as the disc axis and the plane normal vector are not parallel. Parameters may need to be adjusted for better convergence (e.g., dryFrictionProportionalZone). The formulation for the arbitrary disc axis is still under development and needs further testing. Note that the rolling body must have the reference point at the center of the disc.

\ **Additional information for ObjectConnectorRollingDiscPenalty**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\  + \ ``Orientation``\ 
* | Requested \ ``Node``\  type = \ ``GenericData``\ 
* | \ **Short name**\  for Python = \ ``RollingDiscPenalty``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRollingDiscPenalty``\ 


The item \ **ObjectConnectorRollingDiscPenalty**\  with type = 'ConnectorRollingDiscPenalty' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, size =  2, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector; \ :math:`m0`\  represents a point at the plane surface (normal of surface plane defined by planeNormal); the ground can also be a moving rigid body; \ :math:`m1`\  represents the rolling body, which has its reference point (=local position [0,0,0]) at the disc center point
* | **nodeNumber** [\ :math:`n_d`\ , type = NodeIndex, default = invalid (-1)]:
  | node number of a NodeGenericData (size=3) for 3 dataCoordinates, needed for discontinuous iteration (friction and contact)
* | **discRadius** [type = PReal, default = 0.]:
  | defines the disc radius
* | **discAxis** [\ :math:`\LU{m1}{{\mathbf{w}}_{1}}, \;\; |\LU{m1}{{\mathbf{w}}_{1}}| = 1`\ , type = Vector3D, default = [1,0,0]]:
  | axis of disc defined in marker \ :math:`m1`\  frame
* | **planeNormal** [\ :math:`\LU{m0}{{\mathbf{v}}_{PN}}, \;\; |\LU{m0}{{\mathbf{v}}_{PN}}| = 1`\ , type = Vector3D, default = [0,0,1]]:
  | normal to the contact / rolling plane (ground); note that the plane reference point can be arbitrarily chosen by the location of the marker \ :math:`m0`\ 
* | **dryFrictionAngle** [\ :math:`\alpha_t`\ , type = Real, default = 0.]:
  | angle [SI:1 (rad)] which defines a rotation of the local tangential coordinates dry friction; this allows to model Mecanum wheels with specified roll angle
* | **contactStiffness** [\ :math:`k_c`\ , type = UReal, default = 0.]:
  | normal contact stiffness [SI:N/m]
* | **contactDamping** [\ :math:`d_c`\ , type = UReal, default = 0.]:
  | normal contact damping [SI:N/(m s)]
* | **dryFriction** [\ :math:`[\mu_x,\mu_y]\tp`\ , type = Vector2D, default = [0,0]]:
  | dry friction coefficients [SI:1] in local marker 1 joint \ :math:`J1`\  coordinates; if \ :math:`\alpha_t==0`\ , lateral direction \ :math:`l=x`\  and forward direction \ :math:`f=y`\ ; assuming a normal force \ :math:`f_n`\ , the local friction force can be computed as \ :math:`\LU{J1}{\vp{f_{t,x}}{f_{t,y}}} = \vp{\mu_x f_n}{\mu_y f_n}`\ 
* | **dryFrictionProportionalZone** [\ :math:`v_\mu`\ , type = Real, default = 0.]:
  | limit velocity [m/s] up to which the friction is proportional to velocity (for regularization / avoid numerical oscillations)
* | **viscousFriction** [\ :math:`[d_x, d_y]\tp`\ , type = Vector2D, default = [0,0]]:
  | viscous friction coefficients [SI:1/(m/s)] in local marker 1 joint \ :math:`J1`\  coordinates; proportional to slipping velocity, leading to increasing slipping friction force for increasing slipping velocity
* | **rollingFrictionViscous** [\ :math:`\mu_r`\ , type = Real, default = 0.]:
  | rolling friction [SI:1], which acts against the velocity of the trail on ground and leads to a force proportional to the contact normal force; currently, only implemented for disc axis parallel to ground!
* | **useLinearProportionalZone** [type = Bool, default = False]:
  | if True, a linear proportional zone is used; the linear zone performs better in implicit time integration as the Jacobian has a constant tangent in the sticking case
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectConnectorRollingDiscPenalty]:
  | parameters for visualization of item



The item VObjectConnectorRollingDiscPenalty has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **discWidth** [type = float, default = 0.1]:
  | width of disc for drawing
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectconnectorrollingdiscpenalty:

DESCRIPTION of ObjectConnectorRollingDiscPenalty
------------------------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}_{G}`\ 
  | current global position of contact point between rolling disc and ground
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}_{trail}`\ 
  | current velocity of the trail (according to motion of the contact point along the trail!) in global coordinates; this is not the velocity of the contact point!
* | ``VelocityLocal``\ : \ :math:`\LU{J1}{{\mathbf{v}}}`\ 
  | relative slip velocity at contact point in special \ :math:`J1`\  joint coordinates
* | ``ForceLocal``\ : \ :math:`\LU{J1}{{\mathbf{f}}} = \LU{0}{[f_{t,x},\, f_{t,y},\, f_{n}]\tp}`\ 
  | contact forces acting on disc, in special \ :math:`J1`\  joint coordinates, see section Connector Forces, \ :math:`f_{t,x}`\  being the lateral force (parallel to ground plane), \ :math:`f_{t,y}`\  being the longitudinal force and \ :math:`f_{n}`\  being the contact normal force
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
     - | current global position which is provided by marker m0, any ground reference point; currently unused
   * - | marker m0 orientation
     - | \ :math:`\LU{0,m0}{\Rot}`\ 
     - | current rotation matrix provided by marker m0; currently unused
   * - | marker m1 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1}`\ 
     - | center of disc
   * - | marker m1 orientation
     - | \ :math:`\LU{0,m1}{\Rot}`\ 
     - | current rotation matrix provided by marker m1
   * - | data coordinates
     - | \ :math:`{\mathbf{x}}=[x_0,\,x_1,\,x_2]\tp`\ 
     - | data coordinates for \ :math:`[x_0,\,x_1]`\ : hold the sliding velocity in lateral and longitudinal direction of last discontinuous iteration; \ :math:`x_2`\ : represents gap of last discontinuous iteration (in contact normal direction)
   * - | marker m1 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1}`\ 
     - | accordingly
   * - | marker m1 angular velocity
     - | \ :math:`\LU{0}{\tomega}_{m1}`\ 
     - | current angular velocity vector provided by marker m1
   * - | ground normal vector
     - | \ :math:`\LU{0}{{\mathbf{v}}_{PN}} = \LU{0,m0}{{\mathbf{A}}} \LU{m0}{{\mathbf{v}}_{PN}}`\ 
     - | normalized normal vector to the ground body (rotates with marker \ :math:`m0`\  if not fixed to ground)
   * - | ground position B
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{B}`\ 
     - | disc center point projected on ground (normal projection)
   * - | ground position C
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{C}`\ 
     - | contact point of disc with ground
   * - | ground velocity C
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{C}`\ 
     - | velocity of disc at ground contact point (must be zero at end of iteration)
   * - | wheel axis vector
     - | \ :math:`\LU{0}{{\mathbf{w}}_1} =\LU{0,m1}{\Rot} \LU{m1}{{\mathbf{w}}_{1}}`\ 
     - | normalized disc axis vector in global coordinates
   * - | longitudinal vector
     - | \ :math:`\LU{0}{{\mathbf{w}}_2}`\ 
     - | vector in longitudinal (motion) direction
   * - | contact point vector
     - | \ :math:`\LU{0}{{\mathbf{w}}_3}`\ 
     - | normalized vector from disc center point in direction of contact point C
   * - | lateral vector
     - | \ :math:`\LU{0}{{\mathbf{w}}_{lat}} = \LU{0}{{\mathbf{v}}_{PN}} \times \LU{0}{{\mathbf{w}}}_2`\ 
     - | vector in lateral direction, parallel to ground plane
   * - | \ :math:`D1`\  transformation matrix
     - | \ :math:`\LU{0,D1}{{\mathbf{A}}} = [\LU{0}{{\mathbf{w}}_1},\, \LU{0}{{\mathbf{w}}_2},\, \LU{0}{{\mathbf{w}}_3}]`\ 
     - | transformation of special disc coordinates \ :math:`D1`\  to global coordinates
   * - | connector forces
     - | \ :math:`\LU{J1}{{\mathbf{f}}}=[f_{t,x},\,f_{t,y},\,f_n]\tp`\ 
     - | joint force vector at contact point in joint 1 coordinates: x=lateral direction, y=longitudinal direction, z=plane normal (contact normal)



Geometric relations
-------------------

The main geometrical setup is shown in the following figure:


.. image:: ../../theDoc/figures/ObjectJointRollingDiscSketch.png
   :width: 600


First, the contact point \ :math:`\LU{0}{{\mathbf{p}}}_{C}`\  must be computed.
With the helper vector,

.. math::

   \LU{0}{{\mathbf{x}}} = \LU{0}{{\mathbf{w}}}_1 \times \LU{0}{{\mathbf{v}}_{PN}}


we create a disc coordinate system \ :math:`D1`\  (\ :math:`\LU{0}{{\mathbf{w}}}_1, \; \LU{0}{{\mathbf{w}}}_2, \; \LU{0}{{\mathbf{w}}}_3`\ ), with the longitudinal direction,

.. math::

   \LU{0}{{\mathbf{w}}}_2 = \frac{1}{|\LU{0}{{\mathbf{x}}}|} \LU{0}{{\mathbf{x}}}


and the vector to the contact point,

.. math::

   \LU{0}{{\mathbf{w}}}_3 = \LU{0}{{\mathbf{w}}}_1 \times \LU{0}{{\mathbf{w}}}_2


The vector from marker \ :math:`m0`\  position to the contact point can be computed from

.. math::

   \LU{0}{{\mathbf{p}}}_{C} = \LU{0}{{\mathbf{p}}}_{m1} + r \cdot \LU{0}{{\mathbf{w}}}_3 - \LU{0}{{\mathbf{p}}}_{m0}


The velocity of the contact point at the disc is computed from,

.. math::

   \LU{0}{{\mathbf{v}}}_{C} = \LU{0}{{\mathbf{v}}}_{m1} + \LU{0}{\tomega}_{m1} \times (r\cdot \LU{0}{{\mathbf{w}}}_3) - \left(\LU{0}{{\mathbf{v}}}_{m0} + \LU{0}{\tomega}_{m0} \times \LU{0}{{\mathbf{p}}}_{C} \right)


A second coordinate system, denoted as \ :math:`J1`\ , is defined by vectors (\ :math:`\LU{0}{{\mathbf{w}}}_{lat}, \; \LU{0}{{\mathbf{w}}}_2, \;  \LU{0}{{\mathbf{v}}}_{PN}`\ ), using

.. math::

   \LU{0}{{\mathbf{w}}}_{lat} = \LU{0}{{\mathbf{v}}_{PN}} \times \LU{0}{{\mathbf{w}}}_2


Note that \ **in the case that**\  the rolling axis \ :math:`\LU{0}{{\mathbf{w}}}_1`\  lies in the rolling plane, we obtain the special case
\ :math:`\LU{0}{{\mathbf{w}}}_{lat} = \LU{0}{{\mathbf{w}}}_1`\  and \ :math:`\LU{0}{{\mathbf{w}}}_3 = -\LU{0}{{\mathbf{v}}}_{PN}`\ .
                                                                 

Computation of normal and tangential forces
-------------------------------------------

The connector forces at the contact point \ :math:`C`\  are computed as follows. 
The normal contact force reads

.. math::

   f_n = \left(k_c \cdot \LU{0}{{\mathbf{p}}}_{C} + d_c \cdot \LU{0}{{\mathbf{v}}}_{C} \right)\tp \LU{0}{{\mathbf{v}}_{PN}} .


Note that due to the projection onto \ :math:`\LU{0}{{\mathbf{v}}_{PN}}`\ , this equation also works for inclined planes
and reference points, that are not at \ :math:`[0,0,0]\tp`\ .
The inplane velocity in joint coordinates,

.. math::

   \LU{J1}{{\mathbf{v}}_t} = [\LU{0}{{\mathbf{v}}}_{C}\tp \LU{0}{{\mathbf{w}}}_{lat}, \; \LU{0}{{\mathbf{v}}}_{C}\tp \LU{0}{{\mathbf{w}}}_2 ]\tp ,


is used for the computation of tangential forces,

.. math::

   \LU{J1}{{\mathbf{f}}_t} = [f_{t,x} ,\; f_{t,y}]\tp = \LU{J1}{\tmu} \cdot \left( \phi(|{\mathbf{v}}_t|,v_\mu) \cdot f_n \cdot \LU{J1}{{\mathbf{e}}_t} \right) ,


with the regularization function, see Geradin and Cardona  (Sec.\ 7.9.3), if \ ``useLinearProportionalZone=False``\ ,

.. math::

   \phi(v, v_\mu) = \left\{ \begin{array}{ccl} \displaystyle \left( 2-\frac{v}{v_\mu} \right)\frac{v}{v_\mu} & \mathrm{if} & v \le v_\mu \\ 1 & \mathrm{if} & v > v_\mu \\ \end{array} \right.


and the linear regularization function, if \ ``useLinearProportionalZone=True``\ ,

.. math::

   \phi(v, v_\mu) = \left\{ \begin{array}{ccl} \displaystyle \frac{v}{v_\mu} & \mathrm{if} & v \le v_\mu \\ 1 & \mathrm{if} & v > v_\mu \\ \end{array} \right.


The direction of tangential slip is given as

.. math::

   \LU{J1}{{\mathbf{e}}_t} = \left\{ \begin{array}{ccl} \displaystyle \frac{\LU{J1}{{\mathbf{v}}_t}}{|{\mathbf{v}}_t|} &\mathrm{if}& |{\mathbf{v}}_t|>0 \\ \vp{0}{0} &\mathrm{else}& \\ \end{array} \right.


The friction coefficient matrix \ :math:`\LU{J1}{\tmu}`\  is given in joint coordinates and computed from

.. math::

   \LU{J1}{\tmu} = \mp{\mu_x + d_x \cdot |{\mathbf{v}}_t|}{0}{0}{\mu_y + d_y \cdot |{\mathbf{v}}_t|}


where for isotropic behaviour of surface and wheel, it will give a diagonal matrix with the friction coefficient in the diagonal.
In case that the dry friction angle \ :math:`\alpha_t`\  is not zero, the \ :math:`\tmu`\  changes to

.. math::

   \LU{J1}{\tmu} = \mp{\cos(\alpha_t)}{\sin(\alpha_t)}{-\sin(\alpha_t)}{\cos(\alpha_t)} \mp{\mu_x + d_x \cdot |{\mathbf{v}}_t|}{0}{0}{\mu_y + d_y \cdot |{\mathbf{v}}_t|} \mp{\cos(\alpha_t)}{-\sin(\alpha_t)}{\sin(\alpha_t)}{\cos(\alpha_t)}



Connector forces
----------------

Finally, the connector forces read in joint coordinates

.. math::
   :label: eq-connectorrollingdiscpenalty-forces

   \LU{J1}{{\mathbf{f}}} = \vr{f_{t,x}}{f_{t,y}}{f_n}


and in global coordinates, they are computed from

.. math::

   \LU{0}{{\mathbf{f}}} = f_{t,x}\LU{0}{{\mathbf{w}}}_{lat} + f_{t,y} \LU{0}{{\mathbf{w}}}_2 + f_n \LU{0}{{\mathbf{v}}}_{PN}


Due to the fact that the marker positions are not collocated with the contact point, 
there are additional torques that need to be considered in the action on the body.
The torque onto the disc (marker \ :math:`m1`\ ) is computed as

.. math::

   \LU{0}{\ttau_{m1}} = (r\cdot \LU{0}{{\mathbf{w}}}_3) \times \LU{0}{{\mathbf{f}}}


The torque onto the ground (marker \ :math:`m0`\ ) is computed as

.. math::

   \LU{0}{\ttau_{m0}} = \LU{0}{{\mathbf{p}}}_{C} \times \LU{0}{{\mathbf{f}}}


Note that if \ ``activeConnector = False``\ , we replace Eq. :eq:`eq-connectorrollingdiscpenalty-forces`\  with

.. math::

   \LU{J1}{{\mathbf{f}}} = \Null




Relevant Examples and TestModels with weblink:

    \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Examples/), \ `leggedRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/leggedRobot.py>`_\  (Examples/), \ `mobileMecanumWheelRobotWithLidar.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mobileMecanumWheelRobotWithLidar.py>`_\  (Examples/), \ `reinforcementLearningRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/reinforcementLearningRobot.py>`_\  (Examples/), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TestModels/), \ `laserScannerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/laserScannerTest.py>`_\  (TestModels/), \ `mecanumWheelRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mecanumWheelRollingDiscTest.py>`_\  (TestModels/), \ `rollingCoinPenaltyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rollingCoinPenaltyTest.py>`_\  (TestModels/), \ `rollingDiscTangentialForces.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rollingDiscTangentialForces.py>`_\  (TestModels/), \ `rotatingTableTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rotatingTableTest.py>`_\  (TestModels/), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TestModels/), \ `createRollingDiscPenaltyTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createRollingDiscPenaltyTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


