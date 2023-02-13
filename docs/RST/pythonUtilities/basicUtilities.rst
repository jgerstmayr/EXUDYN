
.. _sec-module-basicUtilities:

Module: basicUtilities
======================

Basic utility functions and constants, not depending on numpy or other python modules.

- Author:    Johannes Gerstmayr 
- Date:      2020-03-10 (created) 
- | Notes:
  | Additional constants are defined:
  | pi = 3.1415926535897932
  | sqrt2 = 2**0.5
  | g=9.81
  | eye2D (2x2 diagonal matrix)
  | eye3D (3x3 diagonal matrix)
  | Two variables 'gaussIntegrationPoints' and 'gaussIntegrationWeights' define integration points and weights for function GaussIntegrate(...)


.. _sec-basicUtilities-ClearWorkspace:

`ClearWorkspace <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L42>`__\ ()

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


Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `springDamperUserFunctionNumbaJIT.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/springDamperUserFunctionNumbaJIT.py>`_\  (Ex), \ `ACFtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/ACFtest.py>`_\  (TM)


----

.. _sec-basicUtilities-DiagonalMatrix:

`DiagonalMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L90>`__\ (\ ``rowsColumns``\ , \ ``value = 1``\ )

- | \ *function description*\ :
  | create a diagonal or identity matrix; used for interface.py, avoiding the need for numpy
- | \ *input*\ :

  | \ ``rowsColumns``\ : provides the number of rows and columns
  | \ ``value``\ : initialization value for diagonal terms
- | \ *output*\ :
  | list of lists representing a matrix


----

.. _sec-basicUtilities-NormL2:

`NormL2 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L104>`__\ (\ ``vector``\ )

- | \ *function description*\ :
  | compute L2 norm for vectors without switching to numpy or math module
- | \ *input*\ :
  | vector as list or in numpy format
- | \ *output*\ :
  | L2-norm of vector

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `bicycleIftommBenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/bicycleIftommBenchmark.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `serialRobotTestDH2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTestDH2.py>`_\  (Ex), \ `springsDeactivateConnectors.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/springsDeactivateConnectors.py>`_\  (Ex), \ `distanceSensor.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/distanceSensor.py>`_\  (TM), \ `explicitLieGroupIntegratorTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupIntegratorTest.py>`_\  (TM), \ `fourBarMechanismRedundant.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/fourBarMechanismRedundant.py>`_\  (TM), \ `genericODE2test.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/genericODE2test.py>`_\  (TM)


----

.. _sec-basicUtilities-VSum:

`VSum <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L113>`__\ (\ ``vector``\ )

- | \ *function description*\ :
  | compute sum of all values of vector
- | \ *input*\ :
  | vector as list or in numpy format
- | \ *output*\ :
  | sum of all components of vector

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `serialRobotFlexible.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotFlexible.py>`_\  (Ex), \ `serialRobotKinematicTree.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotKinematicTree.py>`_\  (Ex), \ `serialRobotTestDH2.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTestDH2.py>`_\  (Ex), \ `serialRobotTSD.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTSD.py>`_\  (Ex), \ `movingGroundRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/movingGroundRobotTest.py>`_\  (TM), \ `serialRobotTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/serialRobotTest.py>`_\  (TM)


----

.. _sec-basicUtilities-VAdd:

`VAdd <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L122>`__\ (\ ``v0``\ , \ ``v1``\ )

- | \ *function description*\ :
  | add two vectors instead using numpy
- | \ *input*\ :
  | vectors v0 and v1 as list or in numpy format
- | \ *output*\ :
  | component-wise sum of v0 and v1

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `carRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/carRollingDiscTest.py>`_\  (TM), \ `mecanumWheelRollingDiscTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/mecanumWheelRollingDiscTest.py>`_\  (TM), \ `NGsolveCrankShaftTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/NGsolveCrankShaftTest.py>`_\  (TM), \ `rigidBodyCOMtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyCOMtest.py>`_\  (TM), \ `sliderCrank3Dbenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dbenchmark.py>`_\  (TM), \ `sliderCrank3Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dtest.py>`_\  (TM)


----

.. _sec-basicUtilities-VSub:

`VSub <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L133>`__\ (\ ``v0``\ , \ ``v1``\ )

- | \ *function description*\ :
  | subtract two vectors instead using numpy: result = v0-v1
- | \ *input*\ :
  | vectors v0 and v1 as list or in numpy format
- | \ *output*\ :
  | component-wise difference of v0 and v1

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `ObjectFFRFconvergenceTestHinge.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ObjectFFRFconvergenceTestHinge.py>`_\  (Ex), \ `NGsolveCrankShaftTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/NGsolveCrankShaftTest.py>`_\  (TM), \ `rigidBodyCOMtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/rigidBodyCOMtest.py>`_\  (TM)


----

.. _sec-basicUtilities-VMult:

`VMult <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L144>`__\ (\ ``v0``\ , \ ``v1``\ )

- | \ *function description*\ :
  | scalar multiplication of two vectors instead using numpy: result = v0' \* v1
- | \ *input*\ :
  | vectors v0 and v1 as list or in numpy format
- | \ *output*\ :
  | sum of all component wise products: c0[0]\*v1[0] + v0[1]\*v1[0] + ...


----

.. _sec-basicUtilities-ScalarMult:

`ScalarMult <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L154>`__\ (\ ``scalar``\ , \ ``v``\ )

- | \ *function description*\ :
  | multiplication vectors with scalar: result = scalar \* v
- | \ *input*\ :
  | value {\it scalar} and vector {\it v} as list or in numpy format
- | \ *output*\ :
  | scalar multiplication of all components of v: [scalar\*v[0], scalar\*v[1], ...]

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `pendulumFriction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/pendulumFriction.py>`_\  (TM), \ `sliderCrank3Dbenchmark.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dbenchmark.py>`_\  (TM), \ `sliderCrank3Dtest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/sliderCrank3Dtest.py>`_\  (TM)


----

.. _sec-basicUtilities-Normalize:

`Normalize <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L163>`__\ (\ ``v``\ )

- | \ *function description*\ :
  | take a 3D vector and return a normalized 3D vector (L2Norm=1)
- | \ *input*\ :
  | vector v as list or in numpy format
- | \ *output*\ :
  | vector v multiplied with scalar such that L2-norm of vector is 1

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `ObjectFFRFconvergenceTestHinge.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ObjectFFRFconvergenceTestHinge.py>`_\  (Ex), \ `NGsolveCrankShaftTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/NGsolveCrankShaftTest.py>`_\  (TM)


----

.. _sec-basicUtilities-Vec2Tilde:

`Vec2Tilde <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L178>`__\ (\ ``v``\ )

- | \ *function description*\ :
  | apply tilde operator (skew) to 3D-vector and return skew matrix
- | \ *input*\ :
  | 3D vector v as list or in numpy format
- | \ *output*\ :
  | matrix as list of lists containing the skew-symmetric matrix computed from v: \ :math:`\mr{0}{-v[2]}{v[1]} {v[2]}{0}{-v[0]} {-v[1]}{v[0]}{0}`\

Relevant Examples (Ex) and TestModels (TM) with weblink:

    \ `explicitLieGroupMBSTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/explicitLieGroupMBSTest.py>`_\  (TM)


----

.. _sec-basicUtilities-Tilde2Vec:

`Tilde2Vec <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L185>`__\ (\ ``m``\ )

- | \ *function description*\ :
  | take skew symmetric matrix and return vector (inverse of Skew(...))
- | \ *input*\ :
  | list of lists containing a skew-symmetric matrix (3x3)
- | \ *output*\ :
  | list containing the vector v (inverse function of Vec2Tilde(...))


----

.. _sec-basicUtilities-GaussIntegrate:

`GaussIntegrate <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L212>`__\ (\ ``functionOfX``\ , \ ``integrationOrder``\ , \ ``a``\ , \ ``b``\ )

- | \ *function description*\ :
  | compute numerical integration of functionOfX in interval [a,b] using Gaussian integration
- | \ *input*\ :

  | \ ``functionOfX``\ : scalar, vector or matrix-valued function with scalar argument (X or other variable)
  | \ ``integrationOrder``\ : odd number in {1,3,5,7,9}; currently maximum order is 9
  | \ ``a``\ : integration range start
  | \ ``b``\ : integration range end
- | \ *output*\ :
  | (scalar or vectorized) integral value


----

.. _sec-basicUtilities-LobattoIntegrate:

`LobattoIntegrate <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L248>`__\ (\ ``functionOfX``\ , \ ``integrationOrder``\ , \ ``a``\ , \ ``b``\ )

- | \ *function description*\ :
  | compute numerical integration of functionOfX in interval [a,b] using Lobatto integration
- | \ *input*\ :

  | \ ``functionOfX``\ : scalar, vector or matrix-valued function with scalar argument (X or other variable)
  | \ ``integrationOrder``\ : odd number in {1,3,5}; currently maximum order is 5
  | \ ``a``\ : integration range start
  | \ ``b``\ : integration range end
- | \ *output*\ :
  | (scalar or vectorized) integral value
