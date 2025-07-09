

.. _sec-item-objectrigidbody:

ObjectRigidBody
===============

A 3D rigid body which is attached to a 3D rigid body node. The rotation parametrization of the rigid body follows the rotation parametrization of the node. Use Euler parameters in the general case (no singularities) in combination with implicit solvers (GeneralizedAlpha or TrapezoidalIndex2), Tait-Bryan angles for special cases, e.g., rotors where no singularities occur if you rotate about \ :math:`x`\  or \ :math:`z`\  axis, or use Lie-group formulation with rotation vector together with explicit solvers. REMARK: Use the class \ ``RigidBodyInertia``\ , see Section :ref:`sec-rigidbodyutilities-rigidbodyinertia---init--`\  and \ ``CreateRigidBody(...)``\ , see Section :ref:`sec-mainsystemextensions-createrigidbody`\ , of \ ``exudyn.rigidBodyUtilities``\  to handle inertia, \ :ref:`COM <COM>`\  and mass. \addExampleImage{ObjectRigidBody}

\ **Additional information for ObjectRigidBody**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Body``\ , \ ``SingleNoded``\ 
* | Requested \ ``Node``\  type = \ ``Position``\  + \ ``Orientation``\  + \ ``RigidBody``\ 
* | \ **Short name**\  for Python = \ ``RigidBody``\ 
* | \ **Short name**\  for Python visualization object = \ ``VRigidBody``\ 


The item \ **ObjectRigidBody**\  with type = 'RigidBody' has the following parameters:

* | **name** [type = String, default = '']:
  | objects's unique name
* | **physicsMass** [\ :math:`m`\ , type = UReal, default = 0.]:
  | mass [SI:kg] of rigid body
* | **physicsInertia** [\ :math:`\LU{b}{{\mathbf{j}}_6}`\ , type = Vector6D, default = [0.,0.,0., 0.,0.,0.]]:
  | inertia components [SI:kgm\ :math:`^2`\ ]: \ :math:`[J_{xx}, J_{yy}, J_{zz}, J_{yz}, J_{xz}, J_{xy}]`\  in body-fixed coordinate system and w.r.t. to the reference point of the body, NOT necessarily w.r.t. to \ :ref:`COM <COM>`\ ; use the class RigidBodyInertia of exudynRigidBodyUtilities.py and CreateRigidBody(...) of MainSystem to handle inertia, \ :ref:`COM <COM>`\  and mass
* | **physicsCenterOfMass** [\ :math:`\LU{b}{{\mathbf{b}}_{COM}}`\ , type = Vector3D, size = 3, default = [0.,0.,0.]]:
  | local position of \ :ref:`COM <COM>`\  relative to the body's reference point; if the vector of the \ :ref:`COM <COM>`\  is [0,0,0], the computation will not consider additional terms for the \ :ref:`COM <COM>`\  and it is faster
* | **nodeNumber** [\ :math:`n0`\ , type = NodeIndex, default = invalid (-1)]:
  | node number (type NodeIndex) for rigid body node
* | **visualization** [type = VObjectRigidBody]:
  | parameters for visualization of item



The item VObjectRigidBody has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **graphicsDataUserFunction** [type = PyFunctionGraphicsData, default =  0]:
  | A Python function which returns a bodyGraphicsData object, which is a list of graphics data in a dictionary computed by the user function; the graphics elements need to be defined in the local body coordinates and are transformed by mbs to global coordinates
* | **graphicsData** [type = BodyGraphicsData]:
  | Structure contains data for body visualization; data is defined in special list / dictionary structure


----------

.. _description-objectrigidbody:

DESCRIPTION of ObjectRigidBody
------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}\cConfig(\pLocB) = \LU{0}{\pRef}\cConfig + \LU{0}{\pRef}\cRef + \LU{0b}{\Rot}\pLocB`\ 
  | global position vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``Displacement``\ : \ :math:`\LU{0}{{\mathbf{u}}}\cConfig + \LU{0b}{\Rot}\pLocB`\ 
  | global displacement vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}\cConfig(\pLocB) = \LU{0}{\dot{\mathbf{u}}}\cConfig + \LU{0b}{\Rot}(\LU{b}{\tomega} \times \pLocB\cConfig)`\ 
  | global velocity vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``VelocityLocal``\ : \ :math:`\LU{b}{{\mathbf{v}}}\cConfig(\pLocB) = \LU{b0}{\Rot} \LU{0}{{\mathbf{v}}}\cConfig(\pLocB)`\ 
  | local (body-fixed) velocity vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``RotationMatrix``\ : \ :math:`\mathrm{vec}(\LU{0b}{\Rot})=[A_{00},\,A_{01},\,A_{02},\,A_{10},\,\ldots,\,A_{21},\,A_{22}]\cConfig\tp`\ 
  | vector with 9 components of the rotation matrix (row-major format)
* | ``Rotation``\ : 
  | vector with 3 components of the Euler angles in xyz-sequence (R=Rx*Ry*Rz), recomputed from rotation matrix
* | ``AngularVelocity``\ : \ :math:`\LU{0}{\tomega}\cConfig`\ 
  | angular velocity of body
* | ``AngularVelocityLocal``\ : \ :math:`\LU{b}{\tomega}\cConfig`\ 
  | local (body-fixed) 3D velocity vector of node
* | ``Acceleration``\ : \ :math:`\LU{0}{{\mathbf{a}}}\cConfig(\pLocB) = \LU{0}{\ddot{\mathbf{u}}} + \LU{0}{\talpha} \times (\LU{0b}{\Rot} \pLocB) +  \LU{0}{\tomega} \times ( \LU{0}{\tomega} \times(\LU{0b}{\Rot} \pLocB))`\ 
  | global acceleration vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``AccelerationLocal``\ : \ :math:`\LU{b}{{\mathbf{a}}}\cConfig(\pLocB) = \LU{b0}{\Rot} \LU{0}{{\mathbf{a}}}\cConfig(\pLocB)`\ 
  | local (body-fixed) acceleration vector of body-fixed point given by local position vector \ :math:`\pLocB`\ 
* | ``AngularAcceleration``\ : \ :math:`\LU{0}{\talpha}\cConfig`\ 
  | angular acceleration vector of body
* | ``AngularAccelerationLocal``\ : \ :math:`\LU{b}{\talpha}\cConfig = \LU{b0}{\Rot} \LU{0}{\talpha}\cConfig`\ 
  | local angular acceleration vector of body



Definition of quantities
------------------------


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | intermediate variables
     - | symbol
     - | description
   * - | inertia tensor
     - | \ :math:`\LU{b}{{\mathbf{J}}} = \LU{b}{\mr{J_{xx}}{J_{xy}}{J_{xz}} {J_{xy}}{J_{yy}}{J_{yz}} {J_{xz}}{J_{yz}}{J_{zz}}}`\ 
     - | symmetric inertia tensor, based on components of \ :math:`\LU{b}{{\mathbf{j}}_6}`\ , in body-fixed (local) coordinates and w.r.t.\ body's reference point
   * - | reference coordinates
     - | \ :math:`{\mathbf{q}}\cRef = [\pRef\tp\cRef,\,\tpsi\tp\cRef]\tp`\ 
     - | defines reference configuration, \ **DIFFERENT**\  meaning from body's reference point!
   * - | (relative) current coordinates
     - | \ :math:`{\mathbf{q}}\cCur = [\pRef\tp\cCur,\,\tpsi\tp\cCur]\tp`\ 
     - | unknowns in solver; \ **relative**\  to the reference coordinates; current coordinates at initial configuration = initial coordinates \ :math:`{\mathbf{q}}\cIni`\ 
   * - | current velocity coordinates
     - | \ :math:`\dot {\mathbf{q}}\cCur = [{\mathbf{v}}\tp\cCur,\,\dot \tpsi\tp\cCur]\tp = [\dot {\mathbf{p}}\tp\cCur,\,\dot \ttheta\tp\cCur]\tp`\ 
     - | current velocity coordinates
   * - | body's reference point
     - | \ :math:`\pRefG\cConfig + \pRefG\cRef = \LU{0}{{\mathbf{p}}}(n_0)\cConfig`\ 
     - | position of \ **body's reference point**\  provided by node \ :math:`n_0`\  in any configuration except for reference; if \ :math:`\LU{b}{{\mathbf{b}}_{COM}}==[0,\;0,\;0]\tp`\ , this position becomes equal to the \ :ref:`COM <COM>`\  position
   * - | reference body's reference point
     - | \ :math:`\pRefG\cRef = \LU{0}{{\mathbf{p}}}(n_0)\cRef`\ 
     - | position of \ **body's reference point**\  in reference configuration
   * - | body's reference point displacement
     - | \ :math:`\LU{0}{{\mathbf{u}}}\cConfig = \pRefG\cConfig = [q_0,\;q_1,\;q_2]\cConfig\tp = \LU{0}{{\mathbf{u}}}(n_0)\cConfig`\ 
     - | displacement of \ **body's reference point**\  which is provided by node \ :math:`n_0`\  in any configuration
   * - | body's reference point velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}\cConfig = \dot \pRefG\cConfig = [\dot q_0,\;\dot q_1,\;\dot q_2]\cConfig\tp = \LU{0}{{\mathbf{v}}}(n_0)\cConfig`\ 
     - | velocity of \ **body's reference point**\  which is provided by node \ :math:`n_0`\  in any configuration
   * - | body's reference point acceleration
     - | \ :math:`\LU{0}{{\mathbf{a}}}\cConfig = [\ddot q_0,\;\ddot q_1,\;\ddot q_2]\cConfig\tp`\ 
     - | acceleration of \ **body's reference point**\  which is provided by node \ :math:`n_0`\  in any configuration
   * - | rotation coordinates
     - | \ :math:`\ttheta_{\mathrm{config}} = \tpsi(n_0)\cRef + \tpsi(n_0)\cConfig`\ 
     - | (total) rotation parameters of body as provided by node \ :math:`n_0`\  in any configuration
   * - | rotation parameters
     - | \ :math:`\ttheta_{\mathrm{config}} = \tpsi(n_0)\cRef + \tpsi(n_0)\cConfig`\ 
     - | (total) rotation parameters of body as provided by node \ :math:`n_0`\  in any configuration
   * - | body rotation matrix
     - | \ :math:`\LU{0b}{\Rot}\cConfig = \LU{0b}{\Rot}(n_0)\cConfig`\ 
     - | rotation matrix which transforms local to global coordinates as given by node
   * - | local position
     - | \ :math:`\pLocB = [\LU{b}{b_0},\,\LU{b}{b_1},\,\LU{b}{b_2}]\tp`\ 
     - | local position as used by markers or sensors
   * - | angular velocity
     - | \ :math:`\LU{0}{\tomega}\cConfig = \LU{0}{[\omega_0(n_0),\,\omega_1(n_0),\,\omega_2(n_0)]}\cConfig\tp`\ 
     - | global angular velocity of body as provided by node \ :math:`n_0`\  in any configuration
   * - | local angular velocity
     - | \ :math:`\LU{b}{\tomega}\cConfig`\ 
     - | local angular velocity of body as provided by node \ :math:`n_0`\  in any configuration
   * - | body angular acceleration
     - | \ :math:`\LU{0}{\talpha}\cConfig = \LU{0}{\dot \tomega}\cConfig`\ 
     - | angular acceleratoin of body as provided by node \ :math:`n_0`\  in any configuration
   * - | applied forces
     - | \ :math:`\LU{0}{{\mathbf{f}}}_a = [f_0,\;f_1,\;f_2]\tp`\ 
     - | calculated from loads, connectors, ...
   * - | applied torques
     - | \ :math:`\LU{0}{\ttau}_a = [\tau_0,\;\tau_1,\;\tau_2]\tp`\ 
     - | calculated from loads, connectors, ...
   * - | constraint reaction forces
     - | \ :math:`\LU{0}{{\mathbf{f}}}_\lambda = [f_{\lambda 0},\;f_{\lambda 1},\;f_{\lambda 2}]\tp`\ 
     - | calculated from joints or constraint)
   * - | constraint reaction torques
     - | \ :math:`\LU{0}{\ttau}_\lambda = [\tau_{\lambda 0},\;\tau_{\lambda 1},\;\tau_{\lambda 2}]\tp`\ 
     - | calculated from joints or constraints


Rotation parametrization
------------------------

The equations of motion of the rigid body build upon a specific parameterization of the rigid body coordinates.
Rigid body coordinates are defined by the underlying node given by \ ``nodeNumber``\  \ :math:`n0`\ .
Appropriate nodes are 

+  \ ``NodeRigidBodyEP``\  (Euler parameters)
+  \ ``NodeRigidBodyRxyz``\  (Euler angles / Tait Bryan angles)
+  \ ``NodeRigidBodyRotVecLG``\  (Rotation vector with Lie group integration option)

Note that all operations for rotation parameters, such as the computation of the rotation matrix, must be performed with the 
rotation parameters \ :math:`\ttheta`\ , see table above, which are the sum of reference and current coordinates.

The angular velocity in body-fixed coordinates is related to the rotation parameters by means of a matrix \ :math:`\LU{b}{{\mathbf{G}}_{rp}}`\ ,

.. math::
   :label: eq-objectrigidbody-omegalocal

   \LU{b}{\tomega} = \LU{b}{{\mathbf{G}}_{rp}} \dot \ttheta = \LU{b}{{\mathbf{G}}_{rp}} \dot \tpsi ,


and is specific for any rotation parametrization \ :math:`rp`\ .
The angular velocity in global coordinates is related to the rotation parameters by means of a matrix \ :math:`\LU{0}{{\mathbf{G}}_{rp}}`\ ,

.. math::
   :label: eq-objectrigidbody-omega

   \LU{0}{\tomega} = \LU{0}{{\mathbf{G}}_{rp}} \dot \ttheta.


The local angular accelerations follow as

.. math::
   :label: eq-objectrigidbody-alpha

   \LU{b}{\talpha} = \LU{b}{\dot \tomega}= \LU{b}{{\mathbf{G}}_{rp}} \ddot \ttheta + \LU{b}{\dot {\mathbf{G}}_{rp}} \dot \ttheta ,


remember that derivatives for angular velocities can also be done in the local frame. In case of Euler parameters and the Lie-group rotation vector we find that
\ :math:`\LU{b}{\dot {\mathbf{G}}_{rp}} \dot \ttheta = \Null`\ .


Equations of motion for \ :ref:`COM <COM>`\ 
--------------------------------------------

The equations of motion for a rigid body, the so-called Newton-Euler equations, can be written for the special case of the reference point \ :math:`=`\  \ :ref:`COM <COM>`\  and split for translations and rotations, using a coordinate-free notation,

.. math::
   :label: eq-objectrigidbody-eomcom0

   \mp{m \mathbf{I}_{3 \times 3}}{\Null}{\Null}{{\mathbf{J}}} \vp{{\mathbf{a}}_{COM}}{\talpha} = \vp{\Null}{-\tilde \tomega {\mathbf{J}} \tomega} + \vp{{\mathbf{f}}_a}{\ttau_a} + \vp{{\mathbf{f}}_\lambda}{\ttau_\lambda}


with the \ :math:`3\times 3`\  unit matrix \ :math:`\mathbf{I}_{3 \times 3}`\  and forces \ :math:`{\mathbf{f}}`\  resp.\ torques \ :math:`\ttau`\  as discribed in the table above.
A change of the reference point, using the vector \ :math:`{\mathbf{b}}_{COM}`\  from the body's reference point \ :math:`{\mathbf{p}}`\  to the \ :ref:`COM <COM>`\  position, is simple by replacing \ :ref:`COM <COM>`\  accelerations using the common relation known from Euler

.. math::

   {\mathbf{a}}_{COM} =  {\mathbf{a}} + \tilde \talpha {\mathbf{b}}_{COM} + \tilde \tomega \tilde \tomega {\mathbf{b}}_{COM} ,


which is inserted into the first line of Eq. :eq:`eq-objectrigidbody-eomcom0`\ . Additionally, the second line of Eq. :eq:`eq-objectrigidbody-eomcom0`\ 
(second Euler equation related to rate of angular momentum) is rewritten for an arbitrary reference point, \ :math:`{\mathbf{b}}_{COM}`\  denoting the vector from the body reference point to \ :ref:`COM <COM>`\ , using the well known relation

.. math::

   m \tilde {\mathbf{b}}_{COM} \talpha +  {\mathbf{J}} \talpha + \tilde \tomega {\mathbf{J}} \tomega = \ttau_a + \ttau_\lambda




Equations of motion for arbitrary reference point
-------------------------------------------------

This immediately leads to the equations of motion for the rigid body with respect to an arbitrary reference point (\ :math:`\neq`\  \ :ref:`COM <COM>`\ ), 
see e.g.\ (page 258ff.), which have the general coordinate-free form

.. math::
   :label: eq-objectrigidbody-eomarbitrary

   \mp{m \mathbf{I}_{3 \times 3}}{-m \tilde {\mathbf{b}}_{COM}}{m \tilde {\mathbf{b}}_{COM}}{{\mathbf{J}}} \vp{{\mathbf{a}}}{\talpha} = \vp{-m \tilde \tomega \tilde \tomega {\mathbf{b}}_{COM} }{-\tilde \tomega {\mathbf{J}} \tomega} + \vp{{\mathbf{f}}_a}{\ttau_a} + \vp{{\mathbf{f}}_\lambda}{\ttau_\lambda} ,


in which \ :math:`{\mathbf{J}}`\  is the inertia tensor w.r.t.\ the chosen reference point (which has local coordinates \ :math:`\LU{b}{[0,0,0]\tp}`\ ).
Eq. :eq:`eq-objectrigidbody-eomarbitrary`\  can be written in the global frame (0),

.. math::
   :label: eq-objectrigidbody-eomglobal

   \mp{m \mathbf{I}_{3 \times 3}}{-m \LU{0}{\tilde {\mathbf{b}}_{COM}}} {m \LU{0}{\tilde {\mathbf{b}}_{COM}}}{\LU{0}{{\mathbf{J}}}} \vp{\LU{0}{{\mathbf{a}}}}{\LU{0}{\talpha}} = \vp{-m \LU{0}{\tilde \tomega} \LU{0}{\tilde \tomega} \LU{0}{{\mathbf{b}}_{COM}} } {-\LU{0}{\tilde \tomega} \LU{0}{{\mathbf{J}}} \LU{0}{\tomega}} + \vp{\LU{0}{{\mathbf{f}}_a}}{\LU{0}{\ttau_a}} + \vp{\LU{0}{{\mathbf{f}}_\lambda}}{\LU{0}{\ttau_\lambda}} .


Expressing the translational part (first line) of Eq. :eq:`eq-objectrigidbody-eomglobal`\  in the global frame (0), using local coordinates (b) for 
quantities that are constant in the body-fixed frame, \ :math:`\LU{b}{{\mathbf{J}}}`\  and \ :math:`\LU{b}{{\mathbf{b}}_{COM}}`\ , thus expressing also the 
angular velocity \ :math:`\LU{b}{\tomega}`\  in the body-fixed frame,
applying Eq. :eq:`eq-objectrigidbody-omegalocal`\  and Eq. :eq:`eq-objectrigidbody-alpha`\ , and using the relations

.. math::

   \LU{0}{\tilde \tomega}  \LU{0}{\tilde \tomega} \LU{0}{{\mathbf{b}}_{COM}} &=& \LU{0b}{\Rot} \LU{b}{\tilde \tomega} \LU{b}{\tilde \tomega} \LU{b}{{\mathbf{b}}_{COM}} = - \LU{0b}{\Rot} \LU{b}{\tilde \tomega} \LU{b}{\tilde {\mathbf{b}}_{COM}} \LU{b}{\tomega} = -\LU{0b}{\Rot} \LU{b}{\tilde \tomega} \LU{b}{\tilde {\mathbf{b}}_{COM}} \LU{b}{{\mathbf{G}}_{rp}} \dot \ttheta , \\
   -m \LU{0}{\tilde {\mathbf{b}}_{COM}} \LU{0}{\tilde \talpha} &=& -m \LU{0b}{\Rot} \LU{b}{\tilde {\mathbf{b}}_{COM}} \LU{b}{\tilde \talpha} = -m \LU{0b}{\Rot} \LU{b}{\tilde {\mathbf{b}}_{COM}} \left( \LU{b}{{\mathbf{G}}_{rp}} \ddot \ttheta + \LU{b}{\dot {\mathbf{G}}_{rp}} \dot \ttheta \right) ,


we obtain

.. math::
   :label: eq-objectrigidbody-eom

   &&\mp{m \mathbf{I}_{3 \times 3}}  {-m \LU{0b}{\Rot} \LU{b}{\tilde {\mathbf{b}}_{COM}}\LU{b}{{\mathbf{G}}_{rp}}}  {m \LU{b}{{\mathbf{G}}_{rp}\tp} \LU{b}{\tilde {\mathbf{b}}_{COM}}\LU{0b}{\Rot\tp}}  {\LU{b}{{\mathbf{G}}_{rp}\tp}\LU{b}{{\mathbf{J}}}\LU{b}{{\mathbf{G}}_{rp}}} \vp{\LU{0}{{\mathbf{a}}}}{\ddot \ttheta} \nonumber \\
   &&= \vp{m \LU{0b}{\Rot} \LU{b}{\tilde \tomega} \LU{b}{\tilde {\mathbf{b}}_{COM}} \LU{b}{\tomega}  + m \LU{0b}{\Rot} \LU{b}{\tilde {\mathbf{b}}_{COM}}\LU{b}{\dot {\mathbf{G}}_{rp}} \dot \ttheta} {-\LU{b}{{\mathbf{G}}_{rp}\tp}\LU{b}{\tilde \tomega} \LU{b}{{\mathbf{J}}} \LU{b}{\tomega} - \LU{b}{{\mathbf{G}}_{rp}\tp} \LU{b}{{\mathbf{J}}} \LU{b}{\dot {\mathbf{G}}_{rp}} \dot \ttheta} + \vp{\LU{0}{{\mathbf{f}}}_a}{\LU{0}{{\mathbf{G}}_{rp}\tp}\LU{0}{\ttau}_a} + \vp{\LU{0}{{\mathbf{f}}}_\lambda}{{\mathbf{f}}_{\theta,\lambda}}


with constraint reaction forces \ :math:`{\mathbf{f}}_{\theta,\lambda}`\  for the rotation parameters. 
Note that 
the last line has been pre-multiplied with \ :math:`\LU{b}{{\mathbf{G}}_{rp}\tp}`\  (in order to make the mass matrix symmetric) and that
\ :math:`\LU{b}{\dot {\mathbf{G}}_{rp}} \dot \ttheta = \Null`\  in case of Euler parameters and the Lie-group rotation vector .


Euler parameters
----------------

In case of Euler parameters, a constraint equation is automatically added, reading for the index 3 case

.. math::
   :label: eq-objectrigidbody-eulerparameters

   g_\theta(\ttheta) = \theta_0^2 + \theta_1^2 + \theta_2^2 + \theta_3^2 - 1 = 0


and for the index 2 case

.. math::
   :label: eq-objectrigidbody-eulerparametersvel

   \dot g_\theta(\ttheta) = 2 \theta_0 \dot \theta_0 + 2 \theta_1 \dot \theta_1 + 2 \theta_2 \dot \theta_2 + 2 \theta_3 \dot \theta_3 = 0


Given a Lagrange parameter (algebraic variable) \ :math:`\lambda_\theta`\  related to the Euler parameter constraint \ :eq:`eq-objectrigidbody-eulerparameters`\ , the constraint reaction forces in Eq. :eq:`eq-objectrigidbody-eom`\  then read

.. math::

   {\mathbf{f}}_{\theta,\lambda} = \frac{\partial g_\theta}{\ttheta\tp} \lambda_\theta = [2\theta_0,\; 2\theta_1,\; 2\theta_2,\; 2\theta_3]\tp



--------

\ **Userfunction**\ : ``graphicsDataUserFunction(mbs, itemNumber)`` 


A user function, which is called by the visualization thread in order to draw user-defined objects.
The function can be used to generate any \ ``BodyGraphicsData``\ , see Section  :ref:`sec-graphicsdata`\ .
Use \ ``exudyn.graphics``\  functions, see Section  :ref:`sec-module-graphics`\ , to create more complicated objects. 
Note that \ ``graphicsDataUserFunction``\  needs to copy lots of data and is therefore
inefficient and only designed to enable simpler tests, but not large scale problems.

For an example for \ ``graphicsDataUserFunction``\  see ObjectGround, Section :ref:`sec-item-objectground`\ .

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments /  return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides reference to mbs, which can be used in user function to access all data of the object
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number of the object in mbs, allowing easy access
   * - | \returnValue
     - | BodyGraphicsData
     - | list of \ ``GraphicsData``\  dictionaries, see Section  :ref:`sec-graphicsdata`\ 


For creating a \ ``ObjectRigidBody``\ , there is a \ ``rigidBodyUtilities``\  function \ ``CreateRigidBody``\ , 
see Section :ref:`sec-mainsystemextensions-createrigidbody`\ , which simplifies the setup of a rigid body significantely!


Relevant Examples and TestModels with weblink:

    \ `rigid3Dexample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigid3Dexample.py>`_\  (Examples/), \ `rigidBodyIMUtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidBodyIMUtest.py>`_\  (Examples/), \ `addPrismaticJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addPrismaticJoint.py>`_\  (Examples/), \ `addRevoluteJoint.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/addRevoluteJoint.py>`_\  (Examples/), \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Examples/), \ `ballBearningModel.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ballBearningModel.py>`_\  (Examples/), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Examples/), \ `bungeeJump.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bungeeJump.py>`_\  (Examples/), \ `camFollowerExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/camFollowerExample.py>`_\  (Examples/), \ `chainDriveExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chainDriveExample.py>`_\  (Examples/), \ `chatGPTupdate.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate.py>`_\  (Examples/), \ `chatGPTupdate2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chatGPTupdate2.py>`_\  (Examples/), \ `explicitLieGroupIntegratorPythonTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorPythonTest.py>`_\  (TestModels/), \ `explicitLieGroupIntegratorTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorTest.py>`_\  (TestModels/), \ `explicitLieGroupMBSTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupMBSTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


