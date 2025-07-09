

.. _sec-item-objectjointspherical:

ObjectJointSpherical
====================

A spherical joint, which constrains the relative translation between two position based markers.

\ **Additional information for ObjectJointSpherical**\ :

* | This \ ``Object``\  has/provides the following types = \ ``Connector``\ , \ ``Constraint``\ 
* | Requested \ ``Marker``\  type = \ ``Position``\ 
* | \ **Short name**\  for Python = \ ``SphericalJoint``\ 
* | \ **Short name**\  for Python visualization object = \ ``VSphericalJoint``\ 


The item \ **ObjectJointSpherical**\  with type = 'JointSpherical' has the following parameters:

* | **name** [type = String, default = '']:
  | constraints's unique name
* | **markerNumbers** [\ :math:`[m0,m1]\tp`\ , type = ArrayMarkerIndex, size =  2, default = [ invalid [-1], invalid [-1] ]]:
  | list of markers used in connector; \ :math:`m1`\  is the moving coin rigid body and \ :math:`m0`\  is the marker for the ground body, which use the localPosition=[0,0,0] for this marker!
* | **constrainedAxes** [\ :math:`{\mathbf{j}}=[j_0,\,\ldots,\,j_2]`\ , type = ArrayIndex, size = 3, default = [1,1,1]]:
  | flag, which determines which translation (0,1,2) and rotation (3,4,5) axes are constrained; for \ :math:`j_i`\ , two values are possible: 0=free axis, 1=constrained axis
* | **activeConnector** [type = Bool, default = True]:
  | flag, which determines, if the connector is active; used to deactivate (temporarily) a connector or constraint
* | **visualization** [type = VObjectJointSpherical]:
  | parameters for visualization of item



The item VObjectJointSpherical has the following parameters:

* | **show** [type = Bool, default = True]:
  | set true, if item is shown in visualization and false if it is not shown
* | **jointRadius** [type = float, default = 0.1]:
  | radius of joint to draw
* | **color** [type = Float4, default = [-1.,-1.,-1.,-1.]]:
  | RGBA connector color; if R==-1, use default color


----------

.. _description-objectjointspherical:

DESCRIPTION of ObjectJointSpherical
-----------------------------------

\ **The following output variables are available as OutputVariableType in sensors, Get...Output() and other functions**\ :

* | ``Position``\ : \ :math:`\LU{0}{{\mathbf{p}}}_{m0}`\ 
  | current global position of position marker \ :math:`m0`\ 
* | ``Velocity``\ : \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
  | current global velocity of position marker \ :math:`m0`\ 
* | ``Displacement``\ : \ :math:`\LU{0}{\Delta{\mathbf{p}}}=\LU{0}{{\mathbf{p}}}_{m1} - \LU{0}{{\mathbf{p}}}_{m0}`\ 
  | constraint drift or relative motion, if not all axes fixed
* | ``Force``\ : \ :math:`\LU{0}{{\mathbf{f}}}`\ 
  | joint force in global coordinates



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
     - | current global position which is provided by marker \ :math:`m0`\ 
   * - | marker m1 position
     - | \ :math:`\LU{0}{{\mathbf{p}}}_{m1}`\ 
     - | current global position which is provided by marker \ :math:`m1`\ 
   * - | marker m0 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | current global velocity which is provided by marker \ :math:`m0`\ 
   * - | marker m1 velocity
     - | \ :math:`\LU{0}{{\mathbf{v}}}_{m1}`\ 
     - | current global velocity which is provided by marker \ :math:`m1`\ 
   * - | relative velocity
     - | \ :math:`\LU{0}{\Delta{\mathbf{v}}} = \LU{0}{{\mathbf{v}}}_{m1} - \LU{0}{{\mathbf{v}}}_{m0}`\ 
     - | constraint velocity error, or relative velocity if not all axes fixed
   * - | algebraic variables
     - | \ :math:`{\mathbf{z}}=[\lambda_0,\,\ldots,\,\lambda_2]\tp`\ 
     - | vector of algebraic variables (Lagrange multipliers) according to the algebraic equations


Connector constraint equations
------------------------------


\ **\ ``activeConnector = True``\ :** 
If \ :math:`[j_0,\,\ldots,\,j_2] = [1,1,1]\tp`\ , meaning that all translational coordinates are fixed,
the translational index 3 constraints read

.. math::

   \LU{0}{{\mathbf{p}}}_{m1} - \LU{0}{{\mathbf{p}}}_{m0} = \Null


and the translational index 2 constraints read

.. math::

   \LU{0}{{\mathbf{v}}}_{m1} - \LU{0}{{\mathbf{v}}}_{m0} = \Null


If \ :math:`[j_0,\,\ldots,\,j_2] \neq [1,1,1]\tp`\ , meaning that at least one translational coordinate is free,
the translational index 3 constraints read for every component \ :math:`k \in [0,1,2]`\  of the vector \ :math:`\LU{0}{\Delta{\mathbf{p}}}`\ 

.. math::

   \LU{0}{\Delta p_k} &=& 0 \quad \mathrm{if} \quad j_k = 1 \quad \mathrm{and}\\
   \lambda_k &=& 0 \quad \mathrm{if} \quad j_k = 0 \\



and the translational index 2 constraints read for every component \ :math:`k \in [0,1,2]`\  of the vector \ :math:`\LU{0}{\Delta{\mathbf{v}}}`\ 

.. math::

   \LU{0}{\Delta v_k} &=& 0 \quad \mathrm{if} \quad j_k = 1 \quad \mathrm{and}\\
   \lambda_k &=& 0 \quad \mathrm{if} \quad j_k = 0 \\




\ **\ ``activeConnector = False``\ :** 

.. math::

   {\mathbf{z}} = \Null



Example for body position marker
--------------------------------

In this example, we study the constraint equations for two body position marker, see Section :ref:`sec-item-markerbodyposition`\ ,
based on rigid bodies, see Section :ref:`sec-item-objectrigidbody`\ . 
The markers \ :math:`m_0`\  and \ :math:`m_1`\  have the positions

.. math::

   \LU{0}{{\mathbf{p}}_0}(\pLocB_0) = \LU{0}{{\mathbf{r}}_{\mathrm{ref},0}} + \LU{0}{{\mathbf{u}}_{0}} + \LU{0b}{\Rot_0}\pLocB_0, \quad \LU{0}{{\mathbf{p}}_1}(\pLocB_1) = \LU{0}{{\mathbf{r}}_{\mathrm{ref},1}} + \LU{0}{{\mathbf{u}}_{1}} + \LU{0b}{\Rot_1}\pLocB_1 .


From there, we can derive the 3 constraint equation

.. math::

   \LU{0}{{\mathbf{r}}_{\mathrm{ref},1}} + \LU{0}{{\mathbf{u}}_{1}} + \LU{0b}{\Rot_1}\pLocB_1 - \left(\LU{0}{{\mathbf{r}}_{\mathrm{ref},0}} + \LU{0}{{\mathbf{u}}_{0}} + \LU{0b}{\Rot_0}\pLocB_0 \right) = \Null .


The constraint jacobians simply follow from the position jacobians of the respective markers \ :math:`\LU{0}{{\mathbf{J}}_\mathrm{pos,0}}`\ 
and  \ :math:`\LU{0}{{\mathbf{J}}_\mathrm{pos,1}}`\ . 
The position jacobians are added to the system jacobian at rows according to the global indices of the constraint equations
and the columns are determined by the coordinate indices of the bodies' coordinates.


Relevant Examples and TestModels with weblink:

    \ `NGsolveLinearFEM.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveLinearFEM.py>`_\  (Examples/), \ `newtonsCradle.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/newtonsCradle.py>`_\  (Examples/), \ `bungeeJump.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bungeeJump.py>`_\  (Examples/), \ `chainDriveExample.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/chainDriveExample.py>`_\  (Examples/), \ `humanRobotInteraction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/humanRobotInteraction.py>`_\  (Examples/), \ `NGsolvePostProcessingStresses.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePostProcessingStresses.py>`_\  (Examples/), \ `objectFFRFreducedOrderNetgen.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/objectFFRFreducedOrderNetgen.py>`_\  (Examples/), \ `sliderCrank3DwithANCFbeltDrive.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/sliderCrank3DwithANCFbeltDrive.py>`_\  (Examples/), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TestModels/), \ `createFunctionsTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/createFunctionsTest.py>`_\  (TestModels/), \ `driveTrainTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/driveTrainTest.py>`_\  (TestModels/), \ `mainSystemExtensionsTests.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mainSystemExtensionsTests.py>`_\  (TestModels/), \ `genericJointUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/genericJointUserFunctionTest.py>`_\  (TestModels/), \ `kinematicTreeConstraintTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/kinematicTreeConstraintTest.py>`_\  (TestModels/), \ `objectFFRFreducedOrderAccelerations.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFreducedOrderAccelerations.py>`_\  (TestModels/)



\ **The web version may not be complete. For details, consider also the Exudyn PDF documentation** : `theDoc.pdf <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/theDoc/theDoc.pdf>`_ 


