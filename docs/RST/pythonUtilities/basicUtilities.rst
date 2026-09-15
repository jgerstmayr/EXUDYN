
.. _sec-module-basicutilities:

Module: basicUtilities
======================

Basic utility functions and constants; they depend on numpy only, not on exudyn.

- Author:    Johannes Gerstmayr 
- Date:      2020-03-10 (created) 
- | Notes:
  | Additional constants are defined:
  | pi = 3.1415926535897932
  | sqrt2 = 2\*\*0.5
  | g=9.81
  | Two variables 'gaussIntegrationPoints' and 'gaussIntegrationWeights' define integration points and weights for function GaussIntegrate(...)


.. _sec-basicutilities-clearworkspace:

Function: ClearWorkspace
^^^^^^^^^^^^^^^^^^^^^^^^
`ClearWorkspace <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L28>`__\ ()

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
`SmartRound2String <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L84>`__\ (\ ``x``\ , \ ``prec = 3``\ )

- | \ *function description*\ :
  | round to max number of digits; may give more digits if this is shorter; using in general the format() with '.g' option, but keeping decimal point and using exponent where necessary



----


.. _sec-basicutilities-normalize:

Function: Normalize
^^^^^^^^^^^^^^^^^^^
`Normalize <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L96>`__\ (\ ``v``\ )

- | \ *function description*\ :
  | take a vector and return it normalized to L2-norm 1; a zero vector is returned as zero vector
- | \ *input*\ :
  | vector v as list or in numpy format
- | \ *output*\ :
  | \ ``list``\ : v multiplied with a scalar such that its L2-norm is 1, or the zero vector; a list, as
  | callers append the result to lists of normals

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `contactCurveWithLongCurve.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/contactCurveWithLongCurve.py>`_\  (Ex), \ `NGsolveCMStutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveCMStutorial.py>`_\  (Ex), \ `NGsolveGeometry.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolveGeometry.py>`_\  (Ex), \ `NGsolvePistonEngine.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/NGsolvePistonEngine.py>`_\  (Ex), \ `ObjectFFRFconvergenceTestHinge.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/ObjectFFRFconvergenceTestHinge.py>`_\  (Ex)



----


.. _sec-basicutilities-gaussintegrate:

Function: GaussIntegrate
^^^^^^^^^^^^^^^^^^^^^^^^
`GaussIntegrate <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L128>`__\ (\ ``functionOfX``\ , \ ``integrationOrder``\ , \ ``a``\ , \ ``b``\ )

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


.. _sec-basicutilities-lobattointegrate:

Function: LobattoIntegrate
^^^^^^^^^^^^^^^^^^^^^^^^^^
`LobattoIntegrate <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/basicUtilities.py\#L168>`__\ (\ ``functionOfX``\ , \ ``integrationOrder``\ , \ ``a``\ , \ ``b``\ )

- | \ *function description*\ :
  | compute numerical integration of functionOfX in interval [a,b] using Lobatto integration
- | \ *input*\ :
  | \ ``functionOfX``\ : scalar, vector or matrix-valued function with scalar argument (X or other variable)
  | \ ``integrationOrder``\ : odd number in {1,3,5}; currently maximum order is 5
  | \ ``a``\ : integration range start
  | \ ``b``\ : integration range end
- | \ *output*\ :
  | (scalar or vectorized) integral value

