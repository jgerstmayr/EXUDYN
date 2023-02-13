
.. _sec-module-lieGroupBasics:

Module: lieGroupBasics
======================

Lie group methods and formulas for Lie group integration.

- Author:    Stefan Holzinger 
- Date:      2020-09-11 
- | References:
  | 
  | For details on Lie group methods used here, see the references \cite{Henderson1977, Simo1988, Bruels2011, Sonneville2014, Sonneville2017, Terze2016, Mueller2017}.                Lie group methods for rotation vector are described in Holzinger and Gerstmayr \cite{HolzingerGerstmayr2020, Holzinger2021}.


.. _sec-lieGroupBasics-Sinc:

`Sinc <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L34>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the cardinal sine function in radians
- | \ *input*\ :
  | scalar float or int value
- | \ *output*\ :
  | float value in radians


----

.. _sec-lieGroupBasics-Cot:

`Cot <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L46>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the cotangent function cot(x)=1/tan(x) in radians
- | \ *input*\ :
  | scalar float or int value
- | \ *output*\ :
  | float value in radians


----

.. _sec-lieGroupBasics-R3xSO3Matrix2RotationMatrix:

`R3xSO3Matrix2RotationMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L54>`__\ (\ ``G``\ )

- | \ *function description*\ :
  | computes 3x3 rotation matrix from 7x7 R3xSO(3) matrix, see \cite{Bruels2011}
- | \ *input*\ :
  | G: 7x7 matrix as np.array
- | \ *output*\ :
  | 3x3 rotation matrix as np.array


----

.. _sec-lieGroupBasics-R3xSO3Matrix2Translation:

`R3xSO3Matrix2Translation <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L62>`__\ (\ ``G``\ )

- | \ *function description*\ :
  | computes translation part of R3xSO(3) matrix, see \cite{Bruels2011}
- | \ *input*\ :
  | G: 7x7 matrix as np.array
- | \ *output*\ :
  | 3D vector as np.array containg translational part of R3xSO(3)


----

.. _sec-lieGroupBasics-R3xSO3Matrix:

`R3xSO3Matrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L71>`__\ (\ ``x``\ , \ ``R``\ )

- | \ *function description*\ :
  | builds 7x7 matrix as element of the Lie group R3xSO(3), see \cite{Bruels2011}
- | \ *input*\ :

  | \ ``x``\ : 3D vector as np.array representing the translation part corresponding to R3
  | \ ``R``\ : 3x3 rotation matrix as np.array
- | \ *output*\ :
  | 7x7 matrix as np.array


----

.. _sec-lieGroupBasics-ExpSO3:

`ExpSO3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L93>`__\ (\ ``Omega``\ )

- | \ *function description*\ :
  | compute the matrix exponential map on the Lie group SO(3), see \cite{Mueller2017}
- | \ *input*\ :
  | 3D rotation vector as np.array
- | \ *output*\ :
  | 3x3 matrix as np.array


----

.. _sec-lieGroupBasics-ExpS3:

`ExpS3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L105>`__\ (\ ``Omega``\ )

- | \ *function description*\ :
  | compute the quaternion exponential map on the Lie group S(3), see \cite{Terze2016, Mueller2017}
- | \ *input*\ :
  | 3D rotation vector as np.array
- | \ *output*\ :

  | 4D vector as np.array containing four Euler parameters
  | entry zero of output represent the scalar part of Euler parameters


----

.. _sec-lieGroupBasics-LogSO3:

`LogSO3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L115>`__\ (\ ``R``\ )

- | \ *function description*\ :
  | compute the matrix logarithmic map on the Lie group SO(3), see \cite{Sonneville2014, Sonneville2017}
- | \ *input*\ :
  | 3x3 rotation matrix as np.array
- | \ *output*\ :
  | 3x3 skew symmetric matrix as np.array


----

.. _sec-lieGroupBasics-TExpSO3:

`TExpSO3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L134>`__\ (\ ``Omega``\ )

- | \ *function description*\ :
  | compute the tangent operator corresponding to ExpSO3, see \cite{Bruels2011}
- | \ *input*\ :
  | 3D rotation vector as np.array
- | \ *output*\ :
  | 3x3 matrix as np.array


----

.. _sec-lieGroupBasics-TExpSO3Inv:

`TExpSO3Inv <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L154>`__\ (\ ``Omega``\ )

- | \ *function description*\ :

  | compute the inverse of the tangent operator TExpSO3, see \cite{Sonneville2014}
  | this function was improved, see coordinateMaps.pdf by Stefan Holzinger
- | \ *input*\ :
  | 3D rotation vector as np.array
- | \ *output*\ :
  | 3x3 matrix as np.array


----

.. _sec-lieGroupBasics-ExpSE3:

`ExpSE3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L176>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the matrix exponential map on the Lie group SE(3), see \cite{Bruels2011}
- | \ *input*\ :
  | 6D incremental motion vector as np.array
- | \ *output*\ :
  | 4x4 homogeneous transformation matrix as np.array


----

.. _sec-lieGroupBasics-LogSE3:

`LogSE3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L187>`__\ (\ ``H``\ )

- | \ *function description*\ :
  | compute the matrix logarithm on the Lie group SE(3), see \cite{Sonneville2014}
- | \ *input*\ :
  | 4x4 homogeneous transformation matrix as np.array
- | \ *output*\ :
  | 4x4 skew symmetric matrix as np.array


----

.. _sec-lieGroupBasics-TExpSE3:

`TExpSE3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L202>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the tangent operator corresponding to ExpSE3, see \cite{Bruels2011}
- | \ *input*\ :
  | 6D incremental motion vector as np.array
- | \ *output*\ :
  | 6x6 matrix as np.array


----

.. _sec-lieGroupBasics-TExpSE3Inv:

`TExpSE3Inv <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L228>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the inverse of tangent operator TExpSE3, see \cite{Sonneville2014}
- | \ *input*\ :
  | 6D incremental motion vector as np.array
- | \ *output*\ :
  | 6x6 matrix as np.array


----

.. _sec-lieGroupBasics-ExpR3xSO3:

`ExpR3xSO3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L252>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the matrix exponential map on the Lie group R3xSO(3), see \cite{Bruels2011}
- | \ *input*\ :
  | 6D incremental motion vector as np.array
- | \ *output*\ :
  | 7x7 matrix as np.array


----

.. _sec-lieGroupBasics-TExpR3xSO3:

`TExpR3xSO3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L262>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the tangent operator corresponding to ExpR3xSO3, see \cite{Bruels2011}
- | \ *input*\ :
  | 6D incremental motion vector as np.array
- | \ *output*\ :
  | 6x6 matrix as np.array


----

.. _sec-lieGroupBasics-TExpR3xSO3Inv:

`TExpR3xSO3Inv <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L271>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | compute the inverse of tangent operator TExpR3xSO3
- | \ *input*\ :
  | 6D incremental motion vector as np.array
- | \ *output*\ :
  | 6x6 matrix as np.array


----

.. _sec-lieGroupBasics-CompositionRuleDirectProductR3AndS3:

`CompositionRuleDirectProductR3AndS3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L294>`__\ (\ ``q0``\ , \ ``incrementalMotionVector``\ )

- | \ *function description*\ :
  | compute composition operation for pairs in the Lie group R3xS3
- | \ *input*\ :

  | \ ``q0``\ : 7D vector as np.array containing position coordinates and Euler parameters
  | \ ``incrementalMotionVector``\ : 6D incremental motion vector as np.array
- | \ *output*\ :
  | 7D vector as np.array containing composed position coordinates and composed Euler parameters


----

.. _sec-lieGroupBasics-CompositionRuleSemiDirectProductR3AndS3:

`CompositionRuleSemiDirectProductR3AndS3 <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L316>`__\ (\ ``q0``\ , \ ``incrementalMotionVector``\ )

- | \ *function description*\ :
  | compute composition operation for pairs in the Lie group R3 semiTimes S3 (corresponds to SE(3))
- | \ *input*\ :

  | \ ``q0``\ : 7D vector as np.array containing position coordinates and Euler parameters
  | \ ``incrementalMotionVector``\ : 6D incremental motion vector as np.array
- | \ *output*\ :
  | 7D vector as np.array containing composed position coordinates and composed Euler parameters


----

.. _sec-lieGroupBasics-CompositionRuleDirectProductR3AndR3RotVec:

`CompositionRuleDirectProductR3AndR3RotVec <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L341>`__\ (\ ``q0``\ , \ ``incrementalMotionVector``\ )

- | \ *function description*\ :

  | compute composition operation for pairs in the group obtained from the direct product of R3 and R3, see \cite{HolzingerGerstmayr2020}
  | the rotation vector is used as rotation parametrizations
  | this composition operation can be used in formulations which represent the translational velocities in the global (inertial) frame
- | \ *input*\ :

  | \ ``q0``\ : 6D vector as np.array containing position coordinates and rotation vector
  | \ ``incrementalMotionVector``\ : 6D incremental motion vector as np.array
- | \ *output*\ :
  | 7D vector as np.array containing composed position coordinates and composed rotation vector


----

.. _sec-lieGroupBasics-CompositionRuleSemiDirectProductR3AndR3RotVec:

`CompositionRuleSemiDirectProductR3AndR3RotVec <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L365>`__\ (\ ``q0``\ , \ ``incrementalMotionVector``\ )

- | \ *function description*\ :

  | compute composition operation for pairs in the group obtained from the direct product of R3 and R3.
  | the rotation vector is used as rotation parametrizations
  | this composition operation can be used in formulations which represent the translational velocities in the local (body-attached) frame
- | \ *input*\ :

  | \ ``q0``\ : 6D vector as np.array containing position coordinates and rotation vector
  | \ ``incrementalMotionVector``\ : 6D incremental motion vector as np.array
- | \ *output*\ :
  | 6D vector as np.array containing composed position coordinates and composed rotation vector


----

.. _sec-lieGroupBasics-CompositionRuleDirectProductR3AndR3RotXYZAngles:

`CompositionRuleDirectProductR3AndR3RotXYZAngles <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L390>`__\ (\ ``q0``\ , \ ``incrementalMotionVector``\ )

- | \ *function description*\ :

  | compute composition operation for pairs in the group obtained from the direct product of R3 and R3.
  | Cardan-Tait/Bryan (CTB) angles are used as rotation parametrizations
  | this composition operation can be used in formulations which represent the translational velocities in the global (inertial) frame
- | \ *input*\ :

  | \ ``q0``\ : 6D vector as np.array containing position coordinates and Cardan-Tait/Bryan angles
  | \ ``incrementalMotionVector``\ : 6D incremental motion vector as np.array
- | \ *output*\ :
  | 6D vector as np.array containing composed position coordinates and composed Cardan-Tait/Bryan angles


----

.. _sec-lieGroupBasics-CompositionRuleSemiDirectProductR3AndR3RotXYZAngles:

`CompositionRuleSemiDirectProductR3AndR3RotXYZAngles <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L414>`__\ (\ ``q0``\ , \ ``incrementalMotionVector``\ )

- | \ *function description*\ :

  | compute composition operation for pairs in the group obtained from the direct product of R3 and R3.
  | Cardan-Tait/Bryan (CTB) angles are used as rotation parametrizations
  | this composition operation can be used in formulations which represent the translational velocities in the local (body-attached) frame
- | \ *input*\ :

  | \ ``q0``\ : 6D vector as np.array containing position coordinates and Cardan-Tait/Bryan angles
  | \ ``incrementalMotionVector``\ : 6D incremental motion vector as np.array
- | \ *output*\ :
  | 6D vector as np.array containing composed position coordinates and composed Cardan-Tait/Bryan angles


----

.. _sec-lieGroupBasics-CompositionRuleForEulerParameters:

`CompositionRuleForEulerParameters <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L438>`__\ (\ ``q``\ , \ ``p``\ )

- | \ *function description*\ :

  | compute composition operation for Euler parameters (unit quaternions)
  | this composition operation is quaternion multiplication, see \cite{Terze2016}
- | \ *input*\ :

  | \ ``q``\ : 4D vector as np.array containing Euler parameters
  | \ ``p``\ : 4D vector as np.array containing Euler parameters
- | \ *output*\ :
  | 4D vector as np.array containing composed (multiplied) Euler parameters


----

.. _sec-lieGroupBasics-CompositionRuleForRotationVectors:

`CompositionRuleForRotationVectors <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L453>`__\ (\ ``v0``\ , \ ``Omega``\ )

- | \ *function description*\ :
  | compute composition operation for rotation vectors v0 and Omega, see \cite{Holzinger2021}
- | \ *input*\ :

  | \ ``v0``\ : 3D rotation vector as np.array
  | \ ``Omega``\ : 3D (incremental) rotation vector as np.array
- | \ *output*\ :
  | 3D vector as np.array containing composed rotation vector v


----

.. _sec-lieGroupBasics-CompositionRuleRotXYZAnglesRotationVector:

`CompositionRuleRotXYZAnglesRotationVector <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/lieGroupBasics.py\#L475>`__\ (\ ``alpha0``\ , \ ``Omega``\ )

- | \ *function description*\ :
  | compute composition operation for RotXYZ angles, see \cite{Holzinger2021}
- | \ *input*\ :

  | \ ``alpha0``\ : 3D vector as np.array containing RotXYZ angles
  | \ ``Omega``\ :  3D vector as np.array containing the (incremental) rotation vector
- | \ *output*\ :
  | 3D vector as np.array containing composed RotXYZ angles

