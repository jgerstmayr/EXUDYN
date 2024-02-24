

.. _sec-item-objectjointgeneric:

ObjectJointGeneric
==================

A generic joint in 3D; constrains components of the absolute position and rotations of two points given by PointMarkers or RigidMarkers. An additional local rotation (rotationMarker) can be used to adjust the three rotation axes and/or sliding axes.

\ **Additional information for ObjectJointGeneric**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\  + \ ``Orientation``\ 
* | \ **Short name**\  for Python = \ ``GenericJoint``\ 
* | \ **Short name**\  for Python visualization object = \ ``VGenericJoint``\ 


The item \ **ObjectJointGeneric**\  with type = 'JointGeneric' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, size =  2, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector
* | **constrainedAxes** [\ :math:`{\mathbf{j}}=[j_0,\,\ldots,\,j_5]`\ , type = ArrayIndex, size = 6, default = [1,1,1,1,1,1]]:
  | flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; for \ :math:`j_i`\ , two values are possible: 0=free axis, 1=constrained axis
* | **rotationMarker0** [\ :math:`\LU{m0,J0}{\Rot}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker \ :math:`m0`\ ; translation and rotation axes for marker \ :math:`m0`\  are defined in the local body coordinate system and additionally transformed by rotationMarker0
* | **rotationMarker1** [\ :math:`\LU{m1,J1}{\Rot}`\ , type = Matrix3D, default = [[1,0,0], [0,1,0], [0,0,1]]]:
  | local rotation matrix for marker \ :math:`m1`\ ; translation and rotation axes for marker \ :math:`m1`\  are defined in the local body coordinate system and additionally transformed by rotationMarker1
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **offsetUserFunctionParameters** [\ :math:`{\mathbf{p}}_{par}`\ , type = Vector6D, default = [0.,0.,0.,0.,0.,0.]]:
  | vector of 6 parameters for joint's offsetUserFunction
* | **offsetUserFunction** [\ :math:`\mathrm{UF} \in \Rcal^6`\ , type = PyFunctionVector6DmbsScalarIndexVector6D, default =  0]:
  | A Python function which defines the time-dependent (fixed) offset of translation (indices 0,1,2) and rotation (indices 3,4,5) joint coordinates with parameters (mbs, t, offsetUserFunctionParameters)
* | **offsetUserFunction_t** [\ :math:`\mathrm{UF} \in \Rcal^6`\ , type = PyFunctionVector6DmbsScalarIndexVector6D, default =  0]:
  | (NOT IMPLEMENTED YET)time derivative of offsetUserFunction using the same parameters
* | **alternativeConstraints** [type = Bool, default = False]:
  | this is an experimental flag, may change in future: if uses alternative contraint equations for rotations, currently in case of 3 locked rotations: \ :math:`\LU{0}{{\mathbf{t}}}_{x0}\tp (\LU{0}{{\mathbf{t}}}_{y1} \times \LU{0}{{\mathbf{t}}}_{z0})`\ , \ :math:`\LU{0}{{\mathbf{t}}}_{y0}\tp (\LU{0}{{\mathbf{t}}}_{z1} \times \LU{0}{{\mathbf{t}}}_{x0})`\ , \ :math:`\LU{0}{{\mathbf{t}}}_{z0}\tp (\LU{0}{{\mathbf{t}}}_{x1} \times \LU{0}{{\mathbf{t}}}_{y0})`\ ; this avoids 180° flips of the standard configuration in static computations, but leads to different values in Lagrange multipliers
* | **visualization** [type = VObjectJointGeneric]:
  | parameters for visualization of item



The item VObjectJointGeneric has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **axesRadius** [type = float, default = 0.1]:
  | radius of joint axes to draw
* | **axesLength** [type = float, default = 0.4]:
  | length of joint axes to draw
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectjointgeneric:

DESCRIPTION of ObjectJointGeneric
---------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
  | current global position of position marker \ :math:`m0`\ 
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
  | current global velocity of position marker \ :math:`m0`\ 
* | ``DisplacementLocal``\ : \ :math:`\LU{J0}{\Delta{\mathbf{p}}}`\ 
  | relative displacement in local joint0 coordinates; uses local J0 coordinates even for spherical joint configuration
* | ``VelocityLocal``\ : \ :math:`\LU{J0}{\Delta{\mathbf{v}}}`\ 
  | relative translational velocity in local joint0 coordinates
* | ``Rotation``\ : \ :math:`\LU{J0}{\ttheta}= [\theta_0,\theta_1,\theta_2]\tp`\ 
  | relative rotation parameters (Tait Bryan Rxyz); if all axes are fixed, this output represents the rotational drift; for a revolute joint with free Z-axis, it contains the rotation in the Z-component
* | ``AngularVelocityLocal``\ : \ :math:`\LU{J0}{\Delta\tomega}`\ 
  | relative angular velocity in local joint0 coordinates; if all axes are fixed, this output represents the angular velocity constraint error; for a revolute joint, it contains the angular velocity of this axis
* | ``ForceLocal``\ : \ :math:`\LU{J0}{{\mathbf{f}}}`\ 
  | joint force in local \ :math:`J0`\  coordinates
* | ``TorqueLocal``\ : \ :math:`\LU{J0}{{\mathbf{m}}}`\ 
  | joint torque in local \ :math:`J0`\  coordinates; depending on joint configuration, the result may not be the according torque vector



.. _sec-objectjointgeneric-definitionofquantities:


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
     - | current global position which is provided by marker m0
   * - | marker m0 orientation
     - | \ :math:`\LU{0,m0}{\Rot}`\ 
     - | current rotation matrix provided by marker m0
   * - | joint J0 orientation
     - | \ :math:`\LU{0,J0}{\Rot} = \LU{0,m0}{\Rot} \LU{m0,J0}{\Rot}`\ 
     - | joint \ :math:`J0`\  rotation matrix
   * - | joint J0 orientation vectors
     - | \ :math:`\LU{0,J0}{\Rot} = [\LU{0}{{\mathbf{t}}_{x0}},\,\LU{0}{{\mathbf{t}}_{y0}},\,\LU{0}{{\mathbf{t}}_{z0}}]\tp`\ 
     - | orientation vectors (represent local \ :math:`x`\ , \ :math:`y`\ , and \ :math:`z`\  axes) in global coordinates, used for definition of constraint equations
   * - | marker m1 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1}`\ 
     - | accordingly
   * - | marker m1 orientation
     - | \ :math:`\LU{0,m1}{\Rot}`\ 
     - | current rotation matrix provided by marker m1
   * - | joint J1 orientation
     - | \ :math:`\LU{0,J1}{\Rot} = \LU{0,m1}{\Rot} \LU{m1,J1}{\Rot}`\ 
     - | joint \ :math:`J1`\  rotation matrix
   * - | joint J1 orientation vectors
     - | \ :math:`\LU{0,J1}{\Rot} = [\LU{0}{{\mathbf{t}}_{x1}},\,\LU{0}{{\mathbf{t}}_{y1}},\,v{\mathbf{t}}_{z1}]\tp`\ 
     - | orientation vectors (represent local \ :math:`x`\ , \ :math:`y`\ , and \ :math:`z`\  axes) in global coordinates, used for definition of constraint equations
   * - | marker m0 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity which is provided by marker m0
   * - | marker m1 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1}`\ 
     - | accordingly
   * - | marker m0 velocity
     - | \ :math:`\LU{b}{\tomega}_{m0}`\ 
     - | current local angular velocity vector provided by marker m0
   * - | marker m1 velocity
     - | \ :math:`\LU{b}{\tomega}_{m1}`\ 
     - | current local angular velocity vector provided by marker m1
   * - | Displacement
     - | \ :math:`\LU{0}{\Delta{\mathbf{p}}}=\LU{0}{{\mathbf{p}}}_{m1} - \LU{0}{{\mathbf{p}}}_{m0}`\ 
     - | used, if all translational axes are constrained
   * - | Velocity
     - | \ :math:`\LU{0}{\Delta{\mathbf{v}}} = \LU{0}{{\mathbf{v}}}_{m1} - \LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | used, if all translational axes are constrained (velocity level)
   * - | DisplacementLocal
     - | \ :math:`\LU{J0}{\Delta{\mathbf{p}}}`\ 
     - | \ :math:`\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \LU{0}{\Delta{\mathbf{p}}}`\ 
   * - | VelocityLocal
     - | \ :math:`\LU{J0}{\Delta{\mathbf{v}}}`\ 
     - | \ :math:`\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \LU{0}{\Delta{\mathbf{v}}}`\  \ :math:`\ldots`\  note that this is the global relative velocity projected into the local \ :math:`J0`\  coordinate system
   * - | AngularVelocityLocal
     - | \ :math:`\LU{J0}{\Delta\omega}`\ 
     - | \ :math:`\left(\LU{0,m0}{\Rot}\LU{m0,J0}{\Rot}\right)\tp \left( \LU{0,m1}{\Rot} \LU{m1}{\omega} - \LU{0,m0}{\Rot} \LU{m0}{\omega} \right)`\ 
   * - | algebraic variables
     - | \ :math:`{\mathbf{z}}=[\lambda_0,\,\ldots,\,\lambda_5]\tp`\ 
     - | vector of algebraic variables (Lagrange multipliers) according to the algebraic equations


Connector constraint equations
------------------------------


\ **Equations for translational part (\ ``activeConnector = True``\ )** :

If \ :math:`[j_0,\,\ldots,\,j_2] = [1,1,1]\tp`\ , meaning that all translational coordinates are fixed,
the translational index 3 constraints read (\ :math:`UF_{0,1,2}(mbs, t, {\mathbf{p}}_{par})`\  is the translational part of the user function \ :math:`UF`\ ),

.. math::

   \LU{0}{{\mathbf{p}}}_{m1} - \LU{0}{{\mathbf{p}}}_{m0} - UF_{0,1,2}(mbs, t, i_N, {\mathbf{p}}_{par}) = \Null


and the translational index 2 constraints read

.. math::

   \LU{0}{{\mathbf{v}}}_{m1} - \LU{0}{{\mathbf{v}}}_{m0} - UF_{t;0,1,2}(mbs, t, i_N, {\mathbf{p}}_{par})= \Null


and \ ``iN``\  represents the itemNumber (=objectNumber).
If \ :math:`[j_0,\,\ldots,\,j_2] \neq [1,1,1]\tp`\ , meaning that at least one translational coordinate is free,
the translational index 3 constraints read for every component \ :math:`k \in [0,1,2]`\  of the vector \ :math:`\LU{J0}{\Delta{\mathbf{p}}}`\ 

.. math::

   \LU{J0}{\Delta p_k} - UF_{k}(mbs, t, i_N, {\mathbf{p}}_{par}) &=& 0 \quad \mathrm{if} \quad j_k = 1 \quad \mathrm{and}\\
   \lambda_k &=& 0 \quad \mathrm{if} \quad j_k = 0 \\



and the translational index 2 constraints read for every component \ :math:`k \in [0,1,2]`\  of the vector \ :math:`\LU{J0}{\Delta{\mathbf{v}}}`\ 

.. math::

   \LU{J0}{\Delta v_k} - UF\_t_{k}(mbs, t, i_N, {\mathbf{p}}_{par})  &=& 0 \quad \mathrm{if} \quad j_k = 1 \quad \mathrm{and}\\
   \lambda_k &=& 0 \quad \mathrm{if} \quad j_k = 0 \\




\ **Equations for rotational part (\ ``activeConnector = True``\ )** :

The following equations are exemplarily for certain constrained rotation axes configurations, which shall represent all other possibilities.
Note that the axes are always given in global coordinates, compare the table in Section :ref:`sec-objectjointgeneric-definitionofquantities`\ .

Equations are only given for the index 3 case; the index 2 case can be derived from these equations easily (see C++ code...).
In case of user functions, the additional rotation matrix \ :math:`\LU{J0,J0U}{\Rot}(UF_{3,4,5}(mbs, t, {\mathbf{p}}_{par}))`\ , in which the three components of 
\ :math:`UF_{3,4,5}`\  are interpreted as Tait-Bryan angles that are added to the joint frame.

If \ **3 rotation axes are constrained**\  (e.g., translational or planar joint),  \ :math:`[j_3,\,\ldots,\,j_5] = [1,1,1]\tp`\ , the index 3 constraint equations read

.. math::

   \LU{0}{{\mathbf{t}}}_{z0}\tp \LU{0}{{\mathbf{t}}}_{y1} &=& 0 \\
   \LU{0}{{\mathbf{t}}}_{z0}\tp \LU{0}{{\mathbf{t}}}_{x1} &=& 0 \\
   \LU{0}{{\mathbf{t}}}_{x0}\tp \LU{0}{{\mathbf{t}}}_{y1} &=& 0


If \ **2 rotation axes are constrained**\  (revolute joint), e.g., \ :math:`[j_3,\,\ldots,\,j_5] = [0,1,1]\tp`\ , the index 3 constraint equations read

.. math::

   \lambda_3 &=& 0 \\
   \LU{0}{{\mathbf{t}}}_{x0}\tp \LU{0}{{\mathbf{t}}}_{y1} &=& 0 \\
   \LU{0}{{\mathbf{t}}}_{x0}\tp \LU{0}{{\mathbf{t}}}_{z1} &=& 0


If \ **1 rotation axis is constrained**\  (universal joint), e.g.,  \ :math:`[j_3,\,\ldots,\,j_5] = [1,0,0]\tp`\ , the index 3 constraint equations read

.. math::

   \LU{0}{{\mathbf{t}}}_{y0}\tp \LU{0}{{\mathbf{t}}}_{z1} &=& 0 \\
   \lambda_4 &=& 0 \\
   \lambda_5 &=& 0


if \ ``activeConnector = False``\ , 

.. math::

   {\mathbf{z}} = \Null



--------

\ **Userfunction**\ : ``offsetUserFunction(mbs, t, itemNumber, offsetUserFunctionParameters)`` 


A user function, which computes scalar offset for relative joint translation and joint rotation for the GenericJoint, 
e.g., in order to move or rotate a body on a prescribed trajectory.
It is NECESSARY to use sufficiently smooth functions, having \ **initial offsets**\  consistent with \ **initial configuration**\  of bodies, 
either zero or compatible initial offset-velocity, and no initial accelerations.
The \ ``offsetUserFunction``\  is \ **ONLY used**\  in case of static computation or index3 (generalizedAlpha) time integration.
In order to be on the safe side, provide both  \ ``offsetUserFunction``\  and  \ ``offsetUserFunction_t``\ .

Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
\ ``mbs.GetObjectParameter(itemNumber, ...)``\ , see the according description of \ ``GetObjectParameter``\ .

The user function gets time and the offsetUserFunctionParameters as an input and returns the computed offset vector 
for all relative translational and rotational joint coordinates:

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments / return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs in which underlying item is defined
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs 
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``offsetUserFunctionParameters``\ 
     - | Real
     - | \ :math:`{\mathbf{p}}_{par}`\ , set of parameters which can be freely used in user function
   * - | \returnValue
     - | Real
     - | computed offset vector for given time


--------

\ **Userfunction**\ : ``offsetUserFunction_t(mbs, t, itemNumber, offsetUserFunctionParameters)`` 


A user function, which computes an offset \ **velocity**\  vector for the GenericJoint.
It is NECESSARY to use sufficiently smooth functions, having \ **initial offset velocities**\  consistent with \ **initial velocities**\  of bodies.
The \ ``offsetUserFunction_t``\  is used instead of \ ``offsetUserFunction``\  in case of \ ``velocityLevel = True``\ , 
or for index2 time integration and needed for computation of initial accelerations in second order implicit time integrators.

Note that itemNumber represents the index of the object in mbs, which can be used to retrieve additional data from the object through
\ ``mbs.GetObjectParameter(itemNumber, ...)``\ , see the according description of \ ``GetObjectParameter``\ .

The user function gets time and the offsetUserFunctionParameters as an input and returns the computed offset velocity vector 
for all relative translational and rotational joint coordinates:

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | arguments / return
     - | type or size
     - | description
   * - | \ ``mbs``\ 
     - | MainSystem
     - | provides MainSystem mbs in which underlying item is defined
   * - | \ ``t``\ 
     - | Real
     - | current time in mbs 
   * - | \ ``itemNumber``\ 
     - | Index
     - | integer number of the object in mbs, allowing easy access to all object data via mbs.GetObjectParameter(itemNumber, ...)
   * - | \ ``offsetUserFunctionParameters``\ 
     - | Real
     - | \ :math:`{\mathbf{p}}_{par}`\ , set of parameters which can be freely used in user function
   * - | \returnValue
     - | Real
     - | computed offset velocity vector for given time



--------

\ **User function example**\ :



.. code-block:: python

    #simple example, computing only the translational offset for x-coordinate
    from math import sin, cos, pi
    def UFoffset(mbs, t, itemNumber, offsetUserFunctionParameters): 
        return [offsetUserFunctionParameters[0]*(1 - cos(t*10*2*pi)), 0,0,0,0,0]





Relevant Examples and TestModels with weblink:

    \ `ANCFrotatingCable2D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ANCFrotatingCable2D.py>`_\  (Examples/), \ `beamTutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beamTutorial.py>`_\  (Examples/), \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Examples/), \ `CMSexampleCourse.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/CMSexampleCourse.py>`_\  (Examples/), \ `craneReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/craneReevingSystem.py>`_\  (Examples/), \ `fourBarMechanism3D.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/fourBarMechanism3D.py>`_\  (Examples/), \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Examples/), \ `leggedRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/leggedRobot.py>`_\  (Examples/), \ `mouseInteractionExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mouseInteractionExample.py>`_\  (Examples/), \ `multiMbsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/multiMbsTest.py>`_\  (Examples/), \ `netgenSTLtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/netgenSTLtest.py>`_\  (Examples/), \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Examples/), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TestModels/), \ `revoluteJointPrismaticJointTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/revoluteJointPrismaticJointTest.py>`_\  (TestModels/), \ `abaqusImportTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/abaqusImportTest.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


