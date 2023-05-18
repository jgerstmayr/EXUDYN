
.. _sec-module-liegroupbasics:

Module: lieGroupBasics
======================

Lie group methods and formulas for Lie group integration.

- Author:    Stefan Holzinger, Johannes Gerstmayr 
- Date:      2020-09-11 
- | References:
  | 
  | For details on Lie group methods used here, see the references .                Lie group methods for rotation vector are described in Holzinger and Gerstmayr .


.. _sec-liegroupbasics-sinc:

Function: Sinc
^^^^^^^^^^^^^^
`Sinc <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L35>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the cardinal sine function in radians
- | \ *input*\ :
  | scalar float or int value
- | \ *output*\ :
  | float value in radians
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-cot:

Function: Cot
^^^^^^^^^^^^^
`Cot <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L48>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the cotangent function cot(x)=1/tan(x) in radians
- | \ *input*\ :
  | scalar float or int value
- | \ *output*\ :
  | float value in radians
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-r3xso3matrix2rotationmatrix:

Function: R3xSO3Matrix2RotationMatrix
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`R3xSO3Matrix2RotationMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L57>`__\ (\ ``G``\ )

- | \ *function description*\ :
  | computes 3x3 rotation matrix from 7x7 R3xSO(3) matrix, see
- | \ *input*\ :
  | G: 7x7 matrix as np.array
- | \ *output*\ :
  | 3x3 rotation matrix as np.array
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-r3xso3matrix2translation:

Function: R3xSO3Matrix2Translation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`R3xSO3Matrix2Translation <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L66>`__\ (\ ``G``\ )

- | \ *function description*\ :
  | computes translation part of R3xSO(3) matrix, see
- | \ *input*\ :
  | G: 7x7 matrix as np.array
- | \ *output*\ :
  | 3D vector as np.array containg translational part of R3xSO(3)
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-r3xso3matrix:

Function: R3xSO3Matrix
^^^^^^^^^^^^^^^^^^^^^^
`R3xSO3Matrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L76>`__\ (\ ``x``\ , \ ``R``\ )

- | \ *function description*\ :
  | builds 7x7 matrix as element of the Lie group R3xSO(3), see
- | \ *input*\ :
  | \ ``x``\ : 3D vector as np.array representing the translation part corresponding to R3
  | \ ``R``\ : 3x3 rotation matrix as np.array
- | \ *output*\ :
  | 7x7 matrix as np.array
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-expso3:

Function: ExpSO3
^^^^^^^^^^^^^^^^
`ExpSO3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L99>`__\ (\ ``Omega``\ )

- | \ *function description*\ :
  | compute the matrix exponential map on the Lie group SO(3), see
- | \ *input*\ :
  | 3D rotation vector as np.array
- | \ *output*\ :
  | 3x3 matrix as np.array
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-exps3:

Function: ExpS3
^^^^^^^^^^^^^^^
`ExpS3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L112>`__\ (\ ``Omega``\ )

- | \ *function description*\ :
  | compute the quaternion exponential map on the Lie group S(3), see
- | \ *input*\ :
  | 3D rotation vector as np.array
- | \ *output*\ :
  | 4D vector as np.array containing four Euler parameters
  | entry zero of output represent the scalar part of Euler parameters
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-logso3:

Function: LogSO3
^^^^^^^^^^^^^^^^
`LogSO3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L124>`__\ (\ ``R``\ )

- | \ *function description*\ :
  | compute the matrix logarithmic map on the Lie group SO(3)
- | \ *input*\ :
  | 3x3 rotation matrix as np.array
- | \ *output*\ :
  | 3x3 skew symmetric matrix as np.array
- | \ *author*\ :
  | Johannes Gerstmayr
- | \ *notes*\ :
  | improved accuracy for very small angles as well as angles phi close to pi AS WELL AS at phi=pi



----


.. _sec-liegroupbasics-texpso3:

Function: TExpSO3
^^^^^^^^^^^^^^^^^
`TExpSO3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L160>`__\ (\ ``Omega``\ )

- | \ *function description*\ :
  | compute the tangent operator corresponding to ExpSO3, see
- | \ *input*\ :
  | 3D rotation vector as np.array
- | \ *output*\ :
  | 3x3 matrix as np.array
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-texpso3inv:

Function: TExpSO3Inv
^^^^^^^^^^^^^^^^^^^^
`TExpSO3Inv <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L184>`__\ (\ ``Omega``\ )

- | \ *function description*\ :
  | compute the inverse of the tangent operator TExpSO3, see 
  | this function was improved, see coordinateMaps.pdf by Stefan Holzinger
- | \ *input*\ :
  | 3D rotation vector as np.array
- | \ *output*\ :
  | 3x3 matrix as np.array
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-expse3:

Function: ExpSE3
^^^^^^^^^^^^^^^^
`ExpSE3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L207>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the matrix exponential map on the Lie group SE(3), see
- | \ *input*\ :
  | 6D incremental motion vector as np.array
- | \ *output*\ :
  | 4x4 homogeneous transformation matrix as np.array
- | \ *author*\ :
  | Stefan Holzinger

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_\  (Ex)



----


.. _sec-liegroupbasics-logse3:

Function: LogSE3
^^^^^^^^^^^^^^^^
`LogSE3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L219>`__\ (\ ``H``\ )

- | \ *function description*\ :
  | compute the matrix logarithm on the Lie group SE(3), see
- | \ *input*\ :
  | 4x4 homogeneous transformation matrix as np.array
- | \ *output*\ :
  | 4x4 skew symmetric matrix as np.array
- | \ *author*\ :
  | Stefan Holzinger

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `serialRobotInverseKinematics.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInverseKinematics.py>`_\  (Ex)



----


.. _sec-liegroupbasics-texpse3:

Function: TExpSE3
^^^^^^^^^^^^^^^^^
`TExpSE3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L236>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the tangent operator corresponding to ExpSE3, see
- | \ *input*\ :
  | 6D incremental motion vector as np.array
- | \ *output*\ :
  | 6x6 matrix as np.array
- | \ *author*\ :
  | Stefan Holzinger
- | \ *notes*\ :
  | improved accuracy for very small angles as well as angles phi



----


.. _sec-liegroupbasics-texpse3inv:

Function: TExpSE3Inv
^^^^^^^^^^^^^^^^^^^^
`TExpSE3Inv <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L282>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the inverse of tangent operator TExpSE3, see
- | \ *input*\ :
  | 6D incremental motion vector as np.array
- | \ *output*\ :
  | 6x6 matrix as np.array
- | \ *author*\ :
  | Stefan Holzinger
- | \ *notes*\ :
  | improved accuracy for very small angles as well as angles phi



----


.. _sec-liegroupbasics-expr3xso3:

Function: ExpR3xSO3
^^^^^^^^^^^^^^^^^^^
`ExpR3xSO3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L314>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the matrix exponential map on the Lie group R3xSO(3), see
- | \ *input*\ :
  | 6D incremental motion vector as np.array
- | \ *output*\ :
  | 7x7 matrix as np.array
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-texpr3xso3:

Function: TExpR3xSO3
^^^^^^^^^^^^^^^^^^^^
`TExpR3xSO3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L325>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the tangent operator corresponding to ExpR3xSO3, see
- | \ *input*\ :
  | 6D incremental motion vector as np.array
- | \ *output*\ :
  | 6x6 matrix as np.array
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-texpr3xso3inv:

Function: TExpR3xSO3Inv
^^^^^^^^^^^^^^^^^^^^^^^
`TExpR3xSO3Inv <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L335>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the inverse of tangent operator TExpR3xSO3
- | \ *input*\ :
  | 6D incremental motion vector as np.array
- | \ *output*\ :
  | 6x6 matrix as np.array
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-compositionruledirectproductr3ands3:

Function: CompositionRuleDirectProductR3AndS3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CompositionRuleDirectProductR3AndS3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L359>`__\ (\ ``q0``\ , \ ``incrementalMotionVector``\ )

- | \ *function description*\ :
  | compute composition operation for pairs in the Lie group R3xS3
- | \ *input*\ :
  | \ ``q0``\ : 7D vector as np.array containing position coordinates and Euler parameters
  | \ ``incrementalMotionVector``\ : 6D incremental motion vector as np.array
- | \ *output*\ :
  | 7D vector as np.array containing composed position coordinates and composed Euler parameters
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-compositionrulesemidirectproductr3ands3:

Function: CompositionRuleSemiDirectProductR3AndS3
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CompositionRuleSemiDirectProductR3AndS3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L382>`__\ (\ ``q0``\ , \ ``incrementalMotionVector``\ )

- | \ *function description*\ :
  | compute composition operation for pairs in the Lie group R3 semiTimes S3 (corresponds to SE(3))
- | \ *input*\ :
  | \ ``q0``\ : 7D vector as np.array containing position coordinates and Euler parameters
  | \ ``incrementalMotionVector``\ : 6D incremental motion vector as np.array
- | \ *output*\ :
  | 7D vector as np.array containing composed position coordinates and composed Euler parameters
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-compositionruledirectproductr3andr3rotvec:

Function: CompositionRuleDirectProductR3AndR3RotVec
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CompositionRuleDirectProductR3AndR3RotVec <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L408>`__\ (\ ``q0``\ , \ ``incrementalMotionVector``\ )

- | \ *function description*\ :
  | compute composition operation for pairs in the group obtained from the direct product of R3 and R3, see 
  | the rotation vector is used as rotation parametrizations
  | this composition operation can be used in formulations which represent the translational velocities in the global (inertial) frame
- | \ *input*\ :
  | \ ``q0``\ : 6D vector as np.array containing position coordinates and rotation vector
  | \ ``incrementalMotionVector``\ : 6D incremental motion vector as np.array
- | \ *output*\ :
  | 7D vector as np.array containing composed position coordinates and composed rotation vector
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-compositionrulesemidirectproductr3andr3rotvec:

Function: CompositionRuleSemiDirectProductR3AndR3RotVec
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CompositionRuleSemiDirectProductR3AndR3RotVec <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L433>`__\ (\ ``q0``\ , \ ``incrementalMotionVector``\ )

- | \ *function description*\ :
  | compute composition operation for pairs in the group obtained from the direct product of R3 and R3.
  | the rotation vector is used as rotation parametrizations
  | this composition operation can be used in formulations which represent the translational velocities in the local (body-attached) frame
- | \ *input*\ :
  | \ ``q0``\ : 6D vector as np.array containing position coordinates and rotation vector
  | \ ``incrementalMotionVector``\ : 6D incremental motion vector as np.array
- | \ *output*\ :
  | 6D vector as np.array containing composed position coordinates and composed rotation vector
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-compositionruledirectproductr3andr3rotxyzangles:

Function: CompositionRuleDirectProductR3AndR3RotXYZAngles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CompositionRuleDirectProductR3AndR3RotXYZAngles <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L459>`__\ (\ ``q0``\ , \ ``incrementalMotionVector``\ )

- | \ *function description*\ :
  | compute composition operation for pairs in the group obtained from the direct product of R3 and R3.
  | Cardan-Tait/Bryan (CTB) angles are used as rotation parametrizations
  | this composition operation can be used in formulations which represent the translational velocities in the global (inertial) frame
- | \ *input*\ :
  | \ ``q0``\ : 6D vector as np.array containing position coordinates and Cardan-Tait/Bryan angles
  | \ ``incrementalMotionVector``\ : 6D incremental motion vector as np.array
- | \ *output*\ :
  | 6D vector as np.array containing composed position coordinates and composed Cardan-Tait/Bryan angles
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-compositionrulesemidirectproductr3andr3rotxyzangles:

Function: CompositionRuleSemiDirectProductR3AndR3RotXYZAngles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CompositionRuleSemiDirectProductR3AndR3RotXYZAngles <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L484>`__\ (\ ``q0``\ , \ ``incrementalMotionVector``\ )

- | \ *function description*\ :
  | compute composition operation for pairs in the group obtained from the direct product of R3 and R3.
  | Cardan-Tait/Bryan (CTB) angles are used as rotation parametrizations
  | this composition operation can be used in formulations which represent the translational velocities in the local (body-attached) frame
- | \ *input*\ :
  | \ ``q0``\ : 6D vector as np.array containing position coordinates and Cardan-Tait/Bryan angles
  | \ ``incrementalMotionVector``\ : 6D incremental motion vector as np.array
- | \ *output*\ :
  | 6D vector as np.array containing composed position coordinates and composed Cardan-Tait/Bryan angles
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-compositionruleforeulerparameters:

Function: CompositionRuleForEulerParameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CompositionRuleForEulerParameters <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L509>`__\ (\ ``q``\ , \ ``p``\ )

- | \ *function description*\ :
  | compute composition operation for Euler parameters (unit quaternions)
  | this composition operation is quaternion multiplication, see 
- | \ *input*\ :
  | \ ``q``\ : 4D vector as np.array containing Euler parameters
  | \ ``p``\ : 4D vector as np.array containing Euler parameters
- | \ *output*\ :
  | 4D vector as np.array containing composed (multiplied) Euler parameters
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-compositionruleforrotationvectors:

Function: CompositionRuleForRotationVectors
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CompositionRuleForRotationVectors <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L525>`__\ (\ ``v0``\ , \ ``Omega``\ )

- | \ *function description*\ :
  | compute composition operation for rotation vectors v0 and Omega, see
- | \ *input*\ :
  | \ ``v0``\ : 3D rotation vector as np.array
  | \ ``Omega``\ : 3D (incremental) rotation vector as np.array
- | \ *output*\ :
  | 3D vector as np.array containing composed rotation vector v
- | \ *author*\ :
  | Stefan Holzinger



----


.. _sec-liegroupbasics-compositionrulerotxyzanglesrotationvector:

Function: CompositionRuleRotXYZAnglesRotationVector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CompositionRuleRotXYZAnglesRotationVector <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L548>`__\ (\ ``alpha0``\ , \ ``Omega``\ )

- | \ *function description*\ :
  | compute composition operation for RotXYZ angles, see
- | \ *input*\ :
  | \ ``alpha0``\ : 3D vector as np.array containing RotXYZ angles
  | \ ``Omega``\ :  3D vector as np.array containing the (incremental) rotation vector
- | \ *output*\ :
  | 3D vector as np.array containing composed RotXYZ angles
- | \ *author*\ :
  | Stefan Holzinger

