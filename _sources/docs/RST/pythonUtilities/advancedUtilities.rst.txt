
.. _sec-module-advancedutilities:

Module: advancedUtilities
=========================

Advanced utility functions only depending on numpy or specified exudyn modules;
Here, we gather special functions, which are depending on other modules and do not fit into exudyn.utilities as they cannot be imported e.g. in rigidBodyUtilities

- Author:    Johannes Gerstmayr 
- Date:      2023-01-06 (created) 


.. _sec-advancedutilities-plotlinecode:

Function: PlotLineCode
^^^^^^^^^^^^^^^^^^^^^^
`PlotLineCode <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L30>`__\ (\ ``index``\ )

- | \ *function description*\ :
  | helper functions for matplotlib, returns a list of 28 line codes to be used in plot, e.g. 'r-' for red solid line
- | \ *input*\ :
  | index in range(0:28)
- | \ *output*\ :
  | a color and line style code for matplotlib plot

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `serialRobotInteractiveLimits.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotInteractiveLimits.py>`_\  (Ex), \ `serialRobotTSD.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/serialRobotTSD.py>`_\  (Ex)



----


.. _sec-advancedutilities-findobjectindex:

Function: FindObjectIndex
^^^^^^^^^^^^^^^^^^^^^^^^^
`FindObjectIndex <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L47>`__\ (\ ``i``\ , \ ``globalVariables``\ )

- | \ *function description*\ :
  | simple function to find object index i within the local or global scope of variables
- | \ *input*\ :
  | i, the integer object number and  globalVariables=globals()
- | \ *example*\ :

.. code-block:: python

  FindObjectIndex(2, locals() )  #usually sufficient
  FindObjectIndex(2, globals() ) #wider search




----


.. _sec-advancedutilities-findnodeindex:

Function: FindNodeIndex
^^^^^^^^^^^^^^^^^^^^^^^
`FindNodeIndex <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L63>`__\ (\ ``i``\ , \ ``globalVariables``\ )

- | \ *function description*\ :
  | simple function to find node index i within the local or global scope of variables
- | \ *input*\ :
  | i, the integer node number and  globalVariables=globals()
- | \ *example*\ :

.. code-block:: python

  FindObjectIndex(2, locals() )  #usually sufficient
  FindObjectIndex(2, globals() ) #wider search




----


.. _sec-advancedutilities-islistorarray:

Function: IsListOrArray
^^^^^^^^^^^^^^^^^^^^^^^
`IsListOrArray <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L79>`__\ (\ ``data``\ , \ ``checkIfNoneEmpty = False``\ )

- | \ *function description*\ :
  | checks, if data is of type list or np.array; used in functions to check input data
- | \ *input*\ :
  | \ ``data``\ : any type, preferrably list or numpy.array
  | \ ``checkIfNoneEmpty``\ : if True, function only returns True if type is list or array AND if length is non-zero
- | \ *output*\ :
  | returns True/False



----


.. _sec-advancedutilities-raisetypeerror:

Function: RaiseTypeError
^^^^^^^^^^^^^^^^^^^^^^^^
`RaiseTypeError <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L112>`__\ (\ ``where = ''``\ , \ ``argumentName = ''``\ , \ ``received = None``\ , \ ``expectedType = None``\ , \ ``dim = None``\ , \ ``cols = None``\ )

- | \ *function description*\ :
  | internal function which is used to raise common errors in case of wrong types; dim is used for vectors and square matrices, cols is used for non-square matrices



----


.. _sec-advancedutilities-isvalidbool:

Function: IsValidBool
^^^^^^^^^^^^^^^^^^^^^
`IsValidBool <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L145>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | return True, if x is int, float, np.double, np.integer or similar types that can be automatically casted to pybind11



----


.. _sec-advancedutilities-isvalidrealint:

Function: IsValidRealInt
^^^^^^^^^^^^^^^^^^^^^^^^
`IsValidRealInt <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L154>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | return True, if x is int, float, np.double, np.integer or similar types that can be automatically casted to pybind11



----


.. _sec-advancedutilities-isvalidprealint:

Function: IsValidPRealInt
^^^^^^^^^^^^^^^^^^^^^^^^^
`IsValidPRealInt <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L164>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | return True, if x is valid Real/Int and positive



----


.. _sec-advancedutilities-isvalidurealint:

Function: IsValidURealInt
^^^^^^^^^^^^^^^^^^^^^^^^^
`IsValidURealInt <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L170>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | return True, if x is valid Real/Int and unsigned (non-negative)



----


.. _sec-advancedutilities-isreal:

Function: IsReal
^^^^^^^^^^^^^^^^
`IsReal <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L176>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | return True, if x is any python or numpy float type; could also be called IsFloat(), but Real has special meaning in Exudyn



----


.. _sec-advancedutilities-isinteger:

Function: IsInteger
^^^^^^^^^^^^^^^^^^^
`IsInteger <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L183>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | return True, if x is any python or numpy float type



----


.. _sec-advancedutilities-isvector:

Function: IsVector
^^^^^^^^^^^^^^^^^^
`IsVector <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L190>`__\ (\ ``v``\ , \ ``expectedSize = None``\ )

- | \ *function description*\ :
  | check if v is a valid vector with floats or ints; if expectedSize!=None, the length is also checked



----


.. _sec-advancedutilities-isintvector:

Function: IsIntVector
^^^^^^^^^^^^^^^^^^^^^
`IsIntVector <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L204>`__\ (\ ``v``\ , \ ``expectedSize = None``\ )

- | \ *function description*\ :
  | check if v is a valid vector with floats or ints; if expectedSize!=None, the length is also checked



----


.. _sec-advancedutilities-issquarematrix:

Function: IsSquareMatrix
^^^^^^^^^^^^^^^^^^^^^^^^
`IsSquareMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L219>`__\ (\ ``m``\ , \ ``expectedSize = None``\ )

- | \ *function description*\ :
  | check if v is a valid vector with floats or ints; if expectedSize!=None, the length is also checked



----


.. _sec-advancedutilities-isvalidobjectindex:

Function: IsValidObjectIndex
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`IsValidObjectIndex <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L236>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | return True, if x is valid exudyn object index



----


.. _sec-advancedutilities-isvalidnodeindex:

Function: IsValidNodeIndex
^^^^^^^^^^^^^^^^^^^^^^^^^^
`IsValidNodeIndex <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L242>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | return True, if x is valid exudyn node index



----


.. _sec-advancedutilities-isvalidmarkerindex:

Function: IsValidMarkerIndex
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`IsValidMarkerIndex <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L248>`__\ (\ ``x``\ )

- | \ *function description*\ :
  | return True, if x is valid exudyn marker index



----


.. _sec-advancedutilities-fillinsubmatrix:

Function: FillInSubMatrix
^^^^^^^^^^^^^^^^^^^^^^^^^
`FillInSubMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L263>`__\ (\ ``subMatrix``\ , \ ``destinationMatrix``\ , \ ``destRow``\ , \ ``destColumn``\ )

- | \ *function description*\ :
  | fill submatrix into given destinationMatrix; all matrices must be numpy arrays
- | \ *input*\ :
  | \ ``subMatrix``\ : input matrix, which is filled into destinationMatrix
  | \ ``destinationMatrix``\ : the subMatrix is entered here
  | \ ``destRow``\ : row destination of subMatrix
  | \ ``destColumn``\ : column destination of subMatrix
- | \ *output*\ :
  | destinationMatrix is changed after function call
- | \ *notes*\ :
  | may be erased in future!

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `objectFFRFTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectFFRFTest.py>`_\  (TM)



----


.. _sec-advancedutilities-sweepsin:

Function: SweepSin
^^^^^^^^^^^^^^^^^^
`SweepSin <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L282>`__\ (\ ``t``\ , \ ``t1``\ , \ ``f0``\ , \ ``f1``\ )

- | \ *function description*\ :
  | compute sin sweep at given time t
- | \ *input*\ :
  | \ ``t``\ : evaluate of sweep at time t
  | \ ``t1``\ : end time of sweep frequency range
  | \ ``f0``\ : start of frequency interval [f0,f1] in Hz
  | \ ``f1``\ : end of frequency interval [f0,f1] in Hz
- | \ *output*\ :
  | evaluation of sin sweep (in range -1..+1)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `objectGenericODE2Test.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectGenericODE2Test.py>`_\  (TM)



----


.. _sec-advancedutilities-sweepcos:

Function: SweepCos
^^^^^^^^^^^^^^^^^^
`SweepCos <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L293>`__\ (\ ``t``\ , \ ``t1``\ , \ ``f0``\ , \ ``f1``\ )

- | \ *function description*\ :
  | compute cos sweep at given time t
- | \ *input*\ :
  | \ ``t``\ : evaluate of sweep at time t
  | \ ``t1``\ : end time of sweep frequency range
  | \ ``f0``\ : start of frequency interval [f0,f1] in Hz
  | \ ``f1``\ : end of frequency interval [f0,f1] in Hz
- | \ *output*\ :
  | evaluation of cos sweep (in range -1..+1)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `rigidRotor3DbasicBehaviour.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidRotor3DbasicBehaviour.py>`_\  (Ex), \ `rigidRotor3Drunup.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/rigidRotor3Drunup.py>`_\  (Ex), \ `objectGenericODE2Test.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectGenericODE2Test.py>`_\  (TM)



----


.. _sec-advancedutilities-frequencysweep:

Function: FrequencySweep
^^^^^^^^^^^^^^^^^^^^^^^^
`FrequencySweep <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L304>`__\ (\ ``t``\ , \ ``t1``\ , \ ``f0``\ , \ ``f1``\ )

- | \ *function description*\ :
  | frequency according to given sweep functions SweepSin, SweepCos
- | \ *input*\ :
  | \ ``t``\ : evaluate of frequency at time t
  | \ ``t1``\ : end time of sweep frequency range
  | \ ``f0``\ : start of frequency interval [f0,f1] in Hz
  | \ ``f1``\ : end of frequency interval [f0,f1] in Hz
- | \ *output*\ :
  | frequency in Hz

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `objectGenericODE2Test.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/objectGenericODE2Test.py>`_\  (TM)



----


.. _sec-advancedutilities-smoothstep:

Function: SmoothStep
^^^^^^^^^^^^^^^^^^^^
`SmoothStep <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L315>`__\ (\ ``x``\ , \ ``x0``\ , \ ``x1``\ , \ ``value0``\ , \ ``value1``\ )

- | \ *function description*\ :
  | step function with smooth transition from value0 to value1; transition is computed with cos function
- | \ *input*\ :
  | \ ``x``\ : argument at which function is evaluated
  | \ ``x0``\ : start of step (f(x) = value0)
  | \ ``x1``\ : end of step (f(x) = value1)
  | \ ``value0``\ : value before smooth step
  | \ ``value1``\ : value at end of smooth step
- | \ *output*\ :
  | returns f(x)

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `beamTutorial.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beamTutorial.py>`_\  (Ex), \ `beltDriveALE.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveALE.py>`_\  (Ex), \ `beltDriveReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/beltDriveReevingSystem.py>`_\  (Ex), \ `craneReevingSystem.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/craneReevingSystem.py>`_\  (Ex), \ `leggedRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/leggedRobot.py>`_\  (Ex)



----


.. _sec-advancedutilities-smoothstepderivative:

Function: SmoothStepDerivative
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`SmoothStepDerivative <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L334>`__\ (\ ``x``\ , \ ``x0``\ , \ ``x1``\ , \ ``value0``\ , \ ``value1``\ )

- | \ *function description*\ :
  | derivative of SmoothStep using same arguments
- | \ *input*\ :
  | \ ``x``\ : argument at which function is evaluated
  | \ ``x0``\ : start of step (f(x) = value0)
  | \ ``x1``\ : end of step (f(x) = value1)
  | \ ``value0``\ : value before smooth step
  | \ ``value1``\ : value at end of smooth step
- | \ *output*\ :
  | returns d/dx(f(x))

Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `leggedRobot.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/leggedRobot.py>`_\  (Ex)



----


.. _sec-advancedutilities-indexfromvalue:

Function: IndexFromValue
^^^^^^^^^^^^^^^^^^^^^^^^
`IndexFromValue <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L350>`__\ (\ ``data``\ , \ ``value``\ , \ ``tolerance = 1e-7``\ , \ ``assumeConstantSampleRate = False``\ , \ ``rangeWarning = True``\ )

- | \ *function description*\ :
  | get index from value in given data vector (numpy array); usually used to get specific index of time vector; this function is slow (linear search), if sampling rate is non-constant; otherwise set assumeConstantSampleRate=True!
- | \ *input*\ :
  | \ ``data``\ : containing (almost) equidistant values of time
  | \ ``value``\ : e.g., time to be found in data
  | \ ``tolerance``\ : tolerance, which is accepted (default: tolerance=1e-7)
  | \ ``rangeWarning``\ : warn, if index returns out of range; if warning is deactivated, function uses the closest value
- | \ *output*\ :
  | index
- | \ *notes*\ :
  | to obtain the interpolated value of a time-signal array, use GetInterpolatedSignalValue() in exudyn.signalProcessing



----


.. _sec-advancedutilities-roundmatrix:

Function: RoundMatrix
^^^^^^^^^^^^^^^^^^^^^
`RoundMatrix <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L382>`__\ (\ ``matrix``\ , \ ``treshold = 1e-14``\ )

- | \ *function description*\ :
  | set all entries in matrix to zero which are smaller than given treshold; operates directly on matrix
- | \ *input*\ :
  | matrix as np.array, treshold as positive value
- | \ *output*\ :
  | changes matrix



----


.. _sec-advancedutilities-convertfunctiontosymbolic:

Function: ConvertFunctionToSymbolic
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`ConvertFunctionToSymbolic <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L402>`__\ (\ ``mbs``\ , \ ``function``\ , \ ``userFunctionName``\ , \ ``itemIndex = None``\ , \ ``itemTypeName = None``\ , \ ``verbose = 0``\ )

- | \ *function description*\ :
  | Internal function to convert a Python user function into a dictionary containing the symbolic representation;
  | this function is under development and should be used with care
- | \ *input*\ :
  | \ ``mbs``\ : MainSystem, needed currently for interface
  | \ ``function``\ : Python function with interface according to desired user function
  | \ ``itemIndex``\ : item index, such as ObjectIndex or LoadIndex; -1 indicates MainSystem; if None, itemTypeName must be provided instead
  | \ ``itemTypeName``\ : use of type name, such as ObjectConnectorSpringDamper; in this case, itemIndex must be None
  | \ ``itemIndex``\ : item index, such as ObjectIndex or LoadIndex; -1 indicates MainSystem
  | \ ``userFunctionName``\ : name of user function item, see documentation; this is required, because some items have several user functions, which need to be distinguished
  | \ ``verbose``\ : if > 0, according output is printed
- | \ *output*\ :
  | return dictionary with 'functionName', 'argList', and 'returnList'



----


.. _sec-advancedutilities-createsymbolicuserfunction:

Function: CreateSymbolicUserFunction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`CreateSymbolicUserFunction <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/advancedUtilities.py\#L542>`__\ (\ ``mbs``\ , \ ``function``\ , \ ``userFunctionName``\ , \ ``itemIndex = None``\ , \ ``itemTypeName = None``\ , \ ``verbose = 0``\ )

- | \ *function description*\ :
  | Helper function to convert a Python user function into a symbolic user function;
  | this function is under development and should be used with care
- | \ *input*\ :
  | \ ``mbs``\ : MainSystem, needed currently for interface
  | \ ``function``\ : Python function with interface according to desired user function
  | \ ``itemIndex``\ : item index, such as ObjectIndex or LoadIndex; -1 indicates MainSystem; if None, itemTypeName must be provided instead
  | \ ``itemTypeName``\ : use of type name, such as ObjectConnectorSpringDamper; in this case, itemIndex must be None
  | \ ``userFunctionName``\ : name of user function item, see documentation; this is required, because some items have several user functions, which need to be distinguished
  | \ ``verbose``\ : if > 0, according output may be printed
- | \ *output*\ :
  | returns symbolic user function; this can be transfered into an item using TransferUserFunction2Item
- | \ *notes*\ :
  | keep the return value alive in a variable (or list), as it contains the expression tree which must exist for the lifetime of the user function
- | \ *example*\ :

.. code-block:: python

  oGround = mbs.AddObject(ObjectGround())
  node = mbs.AddNode(NodePoint(referenceCoordinates = [1.05,0,0]))
  oMassPoint = mbs.AddObject(MassPoint(nodeNumber = node, physicsMass=1))
  symbolicFunc = CreateSymbolicUserFunction(mbs, function=springForceUserFunction,
                                            userFunctionName='springForceUserFunction',
                                            itemTypeName='ObjectConnectorSpringDamper')
  m0 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oGround, localPosition=[0,0,0]))
  m1 = mbs.AddMarker(MarkerBodyPosition(bodyNumber=oMassPoint, localPosition=[0,0,0]))
  co = mbs.AddObject(ObjectConnectorSpringDamper(markerNumbers=[m0,m1],
                     referenceLength = 1, stiffness = 100, damping = 1,
                     springForceUserFunction=symbolicFunc))
  print(symbolicFunc.Evaluate(mbs, 0., 0, 1.1, 0.,  100., 0., 13.) )


Relevant Examples (Ex) and TestModels (TM) with weblink to github:

    \ `cartesianSpringDamperUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/cartesianSpringDamperUserFunction.py>`_\  (Ex), \ `SpringDamperMassUserFunction.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/SpringDamperMassUserFunction.py>`_\  (Ex), \ `symbolicUserFunctionMasses.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/Examples/symbolicUserFunctionMasses.py>`_\  (Ex), \ `symbolicUserFunctionTest.py <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/TestModels/symbolicUserFunctionTest.py>`_\  (TM)


.. _sec-module-advancedutilities-class-expectedtype(enum):

CLASS ExpectedType(Enum) (in module advancedUtilities)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
**class description**: 

    internal type which is used for type checking in exudyn Python user functions; used to create unique error messages


