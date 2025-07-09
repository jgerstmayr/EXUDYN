
.. _sec-module-basicutilities:

Module: basicUtilities
======================

Basic utility functions and constants, not depending on numpy or other python modules.

- Author:    Johannes Gerstmayr 
- Date:      2020-03-10 (created) 
- | Notes:
  | Additional constants are defined:
  | pi = 3.1415926535897932
  | sqrt2 = 2\*\*0.5
  | g=9.81
  | eye2D (2x2 diagonal matrix)
  | eye3D (3x3 diagonal matrix)
  | Two variables 'gaussIntegrationPoints' and 'gaussIntegrationWeights' define integration points and weights for function GaussIntegrate(...)


.. _sec-basicutilities-clearworkspace:

Function: ClearWorkspace
^^^^^^^^^^^^^^^^^^^^^^^^
`ClearWorkspace <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L44>`__\ ()

- | \ *function description*\ :
  | clear all workspace variables except for system variables with '_' at beginning,
  | 'func' or 'module' in name; it also deletes all items in exudyn.sys and exudyn.variables,
  | EXCEPT from exudyn.sys['renderState'] for pertaining the previous view of the renderer
- | \ *notes*\ :
  | Use this function with CARE! In Spyder, it is certainly safer to add the preference Run\ :math:`\ra`\ 'remove all variables before execution'. It is recommended to call ClearWorkspace() at the very beginning of your models, to avoid that variables still exist from previous computations which may destroy repeatability of results
- | \ *example*\ :

.. code-block:: python

  import exudyn as exu
  import exudyn.utilities
  #clear workspace at the very beginning, before loading other modules and potentially destroying unwanted things ...
  ClearWorkspace()       #cleanup
  #now continue with other code
  from exudyn.itemInterface import *
  SC = exu.SystemContainer()
  mbs = SC.AddSystem()
  ...


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `springDamperUserFunctionNumbaJIT.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/springDamperUserFunctionNumbaJIT.py>`_\  (Ex), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM), \ `runTestExamples.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/runTestExamples.py>`_\  (TM)



----


.. _sec-basicutilities-smartround2string:

Function: SmartRound2String
^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SmartRound2String <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L83>`__\ (\ ``x``\ , \ ``prec = 3``\ )

- | \ *function description*\ :
  | round to max number of digits; may give more digits if this is shorter; using in general the format() with '.g' option, but keeping decimal point and using exponent where necessary



----


.. _sec-basicutilities-diagonalmatrix:

Function: DiagonalMatrix
^^^^^^^^^^^^^^^^^^^^^^^^
`DiagonalMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L98>`__\ (\ ``rowsColumns``\ , \ ``value = 1``\ )

- | \ *function description*\ :
  | create a diagonal or identity matrix; used for interface.py, avoiding the need for numpy
- | \ *input*\ :
  | \ ``rowsColumns``\ : provides the number of rows and columns
  | \ ``value``\ : initialization value for diagonal terms
- | \ *output*\ :
  | list of lists representing a matrix



----


.. _sec-basicutilities-norml2:

Function: NormL2
^^^^^^^^^^^^^^^^
`NormL2 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L112>`__\ (\ ``vector``\ )

- | \ *function description*\ :
  | compute L2 norm for vectors without switching to numpy or math module
- | \ *input*\ :
  | vector as list or in numpy format
- | \ *output*\ :
  | L2-norm of vector

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `HydraulicActuatorStaticInitialization.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/HydraulicActuatorStaticInitialization.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `reinforcementLearningRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/reinforcementLearningRobot.py>`_\  (Ex), \ `springsDeactivateConnectors.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/springsDeactivateConnectors.py>`_\  (Ex), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `explicitLieGroupIntegratorTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorTest.py>`_\  (TM), \ `fourBarMechanismIftomm.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/fourBarMechanismIftomm.py>`_\  (TM)



----


.. _sec-basicutilities-vsum:

Function: VSum
^^^^^^^^^^^^^^
`VSum <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L121>`__\ (\ ``vector``\ )

- | \ *function description*\ :
  | compute sum of all values of vector
- | \ *input*\ :
  | vector as list or in numpy format
- | \ *output*\ :
  | sum of all components of vector

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `serialRobotFlexible.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotFlexible.py>`_\  (Ex), \ `serialRobotInteractiveLimits.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInteractiveLimits.py>`_\  (Ex), \ `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_\  (Ex), \ `serialRobotKinematicTree.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTree.py>`_\  (Ex), \ `serialRobotTSD.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTSD.py>`_\  (Ex), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM), \ `serialRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/serialRobotTest.py>`_\  (TM)



----


.. _sec-basicutilities-vadd:

Function: VAdd
^^^^^^^^^^^^^^
`VAdd <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L130>`__\ (\ ``v0``\ , \ ``v1``\ )

- | \ *function description*\ :
  | add two vectors instead using numpy
- | \ *input*\ :
  | vectors v0 and v1 as list or in numpy format
- | \ *output*\ :
  | component-wise sum of v0 and v1

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `mobileMecanumWheelRobotWithLidar.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/mobileMecanumWheelRobotWithLidar.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM), \ `laserScannerTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/laserScannerTest.py>`_\  (TM), \ `mecanumWheelRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mecanumWheelRollingDiscTest.py>`_\  (TM), \ `NGsolveCrankShaftTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/NGsolveCrankShaftTest.py>`_\  (TM), \ `rigidBodyCOMtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyCOMtest.py>`_\  (TM), \ `simulatorCouplingTwoMbs.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/simulatorCouplingTwoMbs.py>`_\  (TM)



----


.. _sec-basicutilities-vsub:

Function: VSub
^^^^^^^^^^^^^^
`VSub <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L141>`__\ (\ ``v0``\ , \ ``v1``\ )

- | \ *function description*\ :
  | subtract two vectors instead using numpy: result = v0-v1
- | \ *input*\ :
  | vectors v0 and v1 as list or in numpy format
- | \ *output*\ :
  | component-wise difference of v0 and v1

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Ex), \ `NGsolveGeometry.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveGeometry.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `ObjectFFRFconvergenceTestHinge.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ObjectFFRFconvergenceTestHinge.py>`_\  (Ex), \ `NGsolveCrankShaftTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/NGsolveCrankShaftTest.py>`_\  (TM), \ `rigidBodyCOMtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyCOMtest.py>`_\  (TM)



----


.. _sec-basicutilities-vmult:

Function: VMult
^^^^^^^^^^^^^^^
`VMult <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L152>`__\ (\ ``v0``\ , \ ``v1``\ )

- | \ *function description*\ :
  | scalar multiplication of two vectors instead using numpy: result = v0' \* v1
- | \ *input*\ :
  | vectors v0 and v1 as list or in numpy format
- | \ *output*\ :
  | sum of all component wise products: c0[0]*v1[0] + v0[1]*v1[0] + ...



----


.. _sec-basicutilities-scalarmult:

Function: ScalarMult
^^^^^^^^^^^^^^^^^^^^
`ScalarMult <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L162>`__\ (\ ``scalar``\ , \ ``v``\ )

- | \ *function description*\ :
  | multiplication vectors with scalar: result = scalar \* v
- | \ *input*\ :
  | value \ \*scalar\*\  and vector \ \*v\*\  as list or in numpy format
- | \ *output*\ :
  | scalar multiplication of all components of v: [scalar*v[0], scalar*v[1], ...]

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `pendulumFriction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/pendulumFriction.py>`_\  (TM), \ `sliderCrank3Dbenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dbenchmark.py>`_\  (TM), \ `sliderCrank3Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dtest.py>`_\  (TM)



----


.. _sec-basicutilities-normalize:

Function: Normalize
^^^^^^^^^^^^^^^^^^^
`Normalize <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L171>`__\ (\ ``v``\ )

- | \ *function description*\ :
  | take a 3D vector and return a normalized 3D vector (L2Norm=1)
- | \ *input*\ :
  | vector v as list or in numpy format
- | \ *output*\ :
  | vector v multiplied with scalar such that L2-norm of vector is 1

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `contactCurveWithLongCurve.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/contactCurveWithLongCurve.py>`_\  (Ex), \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Ex), \ `NGsolveGeometry.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveGeometry.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `ObjectFFRFconvergenceTestHinge.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ObjectFFRFconvergenceTestHinge.py>`_\  (Ex), \ `NGsolveCrankShaftTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/NGsolveCrankShaftTest.py>`_\  (TM)



----


.. _sec-basicutilities-vec2tilde:

Function: Vec2Tilde
^^^^^^^^^^^^^^^^^^^
`Vec2Tilde <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L187>`__\ (\ ``v``\ )

- | \ *function description*\ :
  | apply tilde operator (skew) to 3D-vector and return skew matrix
- | \ *input*\ :
  | 3D vector v as list or in numpy format
- | \ *output*\ :
  | matrix as list of lists with the skew-symmetric matrix from v:
  | \ :math:`\left[\!\! \begin{array}{ccc} 0 & -v[2] & v[1] \\ v[2] & 0 & -v[0] \\ -v[1] & v[0] & 0  \end{array} \!\!\right]`\ 

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `explicitLieGroupMBSTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupMBSTest.py>`_\  (TM)



----


.. _sec-basicutilities-tilde2vec:

Function: Tilde2Vec
^^^^^^^^^^^^^^^^^^^
`Tilde2Vec <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L194>`__\ (\ ``m``\ )

- | \ *function description*\ :
  | take skew symmetric matrix and return vector (inverse of Skew(...))
- | \ *input*\ :
  | list of lists containing a skew-symmetric matrix (3x3)
- | \ *output*\ :
  | list containing the vector v (inverse function of Vec2Tilde(...))



----


.. _sec-basicutilities-gaussintegrate:

Function: GaussIntegrate
^^^^^^^^^^^^^^^^^^^^^^^^
`GaussIntegrate <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L221>`__\ (\ ``functionOfX``\ , \ ``integrationOrder``\ , \ ``a``\ , \ ``b``\ )

- | \ *function description*\ :
  | compute numerical integration of functionOfX in interval [a,b] using Gaussian integration
- | \ *input*\ :
  | \ ``functionOfX``\ : scalar, vector or matrix-valued function with scalar argument (X or other variable)
  | \ ``integrationOrder``\ : odd number in \{1,3,5,7,9\}; currently maximum order is 9
  | \ ``a``\ : integration range start
  | \ ``b``\ : integration range end
- | \ *output*\ :
  | (scalar or vectorized) integral value



----


.. _sec-basicutilities-lobattointegrate:

Function: LobattoIntegrate
^^^^^^^^^^^^^^^^^^^^^^^^^^
`LobattoIntegrate <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L257>`__\ (\ ``functionOfX``\ , \ ``integrationOrder``\ , \ ``a``\ , \ ``b``\ )

- | \ *function description*\ :
  | compute numerical integration of functionOfX in interval [a,b] using Lobatto integration
- | \ *input*\ :
  | \ ``functionOfX``\ : scalar, vector or matrix-valued function with scalar argument (X or other variable)
  | \ ``integrationOrder``\ : odd number in \{1,3,5\}; currently maximum order is 5
  | \ ``a``\ : integration range start
  | \ ``b``\ : integration range end
- | \ *output*\ :
  | (scalar or vectorized) integral value

