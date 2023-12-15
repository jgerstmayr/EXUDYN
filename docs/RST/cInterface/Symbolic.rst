

.. _sec-cinterface-symbolic:

********
Symbolic
********

The Symbolic sub-module in \ ``exudyn.symbolic``\  allows limited symbolic manipulations in Exudyn and is currently under development In particular, symbolic user functions can be created, which allow significant speedup of Python user functions. However, \ **always veryfy your symbolic expressions or user functions**\ , as behavior may be unexpected in some cases. 

symbolic.Real
=============




The symbolic Real type allows to replace Python's float by a symbolic quantity. The \ ``symbolic.Real``\  may be directly set to a float and be evaluated as float. However, turing on recording by using 	extttexudyn.symbolic.SetRecording(True) (on by default), results are stored as expression trees, which may be evaluated in C++ or Python, in particular in user functions, see the following example:

.. code-block:: python
   :linenos:
   
   import exudyn as exu
   esym = exu.symbolic     #abbreviation
   SymReal = esym.Real     #abbreviation
   
   #create some variables
   a = SymReal(42.,"a")    #use named expression
   b = SymReal(13)         #b is 13
   c = a+b*7.+1.-3         #c stores expression tree
   d = c                   #d and c are containing same tree!
   print('a: ',a,' = ',a.Evaluate())
   print('c: ',c,' = ',c.Evaluate())
   
   #use special functions:
   d = a+b*esym.sin(a)+esym.cos(SymReal(7))
   print('d: ',d,' = ',d.Evaluate())
   
   #compute derivatives (automatic differentiation):
   x = SymReal(0.5,"x")
   f = a+b*esym.sin(x)+esym.cos(SymReal(7))+x**4
   print('f=',f.Evaluate(), ', diff=',f.Diff(x))
   
   #turn off recording of trees (globally for all symbolic.Real!):
   esym.SetRecording(False)
   x = SymReal(42) #now, only represents a value
   y = x/3.       #directly evaluates to 14

To create a symbolic Real, use \ ``aa=symbolic.Real(1.23)``\  to build a Python object aa with value 1.23. In order to use a named value, use \ ``pi=symbolic.Real(3.14,'pi')``\ . Note that in the following, we use the abbreviation \ ``SymReal=exudyn.symbolic.Real``\ . Member functions of \ ``SymReal``\ , which are \ **not recorded**\ , are:

\ The class **symbolic.Real** has the following **functions and structures**:

* | **SetValue**\ (\ *valueInit*\ ): 
  | Set either internal float value or value of named expression; cannot change symbolic expressions.
  | *Example*:

  .. code-block:: python

     b = SymReal(13)
     b.SetValue(14) #now b is 14\#b.SetValue(a+3.) #not possible!

* | **Evaluate**\ (): 
  | return evaluated expression (prioritized) or stored Real value.
* | **Diff**\ (\ *var*\ ): 
  | return derivative of stored expression with respect to given variable.
* | **value**:
  | access to internal float value, which is used in case that symbolic.Real has been built from a float (but without a name and without symbolic expression)
* | **\_\_float\_\_**:
  | evaluation of expression and conversion of symbolic.Real to Python float
* | **\_\_str\_\_**:
  | conversion of symbolic.Real to string
* | **\_\_repr\_\_**:
  | representation of symbolic.Real in Python



The remaining operators and mathematical functions are recorded within expressions. Main mathematical operators for \ ``SymReal``\  exist, similar to Python, such as:

.. code-block:: python
   :linenos:
   
   a = SymReal(1)
   b = SymReal(2)
   
   r1 = a+b
   r1 = a-b
   r1 = a*b
   r1 = a/b
   r1 = -a
   r1 = a**b
   
   c = SymReal(3.3)
   c += b
   c -= b
   c *= b
   c /= b
   
   c = (a == b)
   c = (a != b)
   c = (a < b)
   c = (a > b)
   c = (a <= b)
   c = (a >= b)
   
   #in most cases, we can also mix with float:
   c = a*7 + SymReal.sin(8)

Mathematical functions may be called with an \ ``SymReal``\  or with a \ ``float``\ . Most standard mathematical functions exist for \ ``SymReal``\ , e.g., as \ ``SymReal.abs``\ . \ **HINT**\ : function names are lower-case for compatibility with Python's math library. Thus, you can easily exchange math.sin with SymReal.sin, and you may want to use a generic name, such as myMath=SymReal in order to switch between Python and symbolic user functions. The following functions exist:

\ The class **symbolic.Real** has the following **functions and structures**:

* | **isfinite**\ (\ *x*\ ): 
  | according to specification of C++ std::isfinite
* | **abs**\ (\ *x*\ ): 
  | according to specification of C++ std::fabs
* | **round**\ (\ *x*\ ): 
  | according to specification of C++ std::round
* | **ceil**\ (\ *x*\ ): 
  | according to specification of C++ std::ceil
* | **floor**\ (\ *x*\ ): 
  | according to specification of C++ std::floor
* | **sqrt**\ (\ *x*\ ): 
  | according to specification of C++ std::sqrt
* | **exp**\ (\ *x*\ ): 
  | according to specification of C++ std::exp
* | **log**\ (\ *x*\ ): 
  | according to specification of C++ std::log
* | **sin**\ (\ *x*\ ): 
  | according to specification of C++ std::sin
* | **cos**\ (\ *x*\ ): 
  | according to specification of C++ std::cos
* | **tan**\ (\ *x*\ ): 
  | according to specification of C++ std::tan
* | **asin**\ (\ *x*\ ): 
  | according to specification of C++ std::asin
* | **acos**\ (\ *x*\ ): 
  | according to specification of C++ std::acos
* | **atan**\ (\ *x*\ ): 
  | according to specification of C++ std::atan
* | **sinh**\ (\ *x*\ ): 
  | according to specification of C++ std::sinh
* | **cosh**\ (\ *x*\ ): 
  | according to specification of C++ std::cosh
* | **tanh**\ (\ *x*\ ): 
  | according to specification of C++ std::tanh
* | **asinh**\ (\ *x*\ ): 
  | according to specification of C++ std::asinh
* | **acosh**\ (\ *x*\ ): 
  | according to specification of C++ std::acosh
* | **atanh**\ (\ *x*\ ): 
  | according to specification of C++ std::atanh



The following table lists special functions for \ ``SymReal``\ : 

\ The class **symbolic.Real** has the following **functions and structures**:

* | **sign**\ (\ *x*\ ): 
  | returns 0 for x=0, -1 for x<0 and 1 for x>1.
* | **Not**\ (\ *x*\ ): 
  | returns logical not of expression, equal to Python's 'not'. Not(True)=False, Not(0.)=True, Not(-0.1)=False
* | **min**\ (\ *x*\ , \ *y*\ ): 
  | return minimum of x and y. 
* | **max**\ (\ *x*\ , \ *y*\ ): 
  | return maximum of x and y. 
* | **mod**\ (\ *x*\ , \ *y*\ ): 
  | return floating-point remainder of the division operation x / y. For example, mod(5.1, 3) gives 2.1 as a remainder.
* | **pow**\ (\ *x*\ , \ *y*\ ): 
  | return \ :math:`x^y`\ . 
* | **max**\ (\ *x*\ , \ *y*\ ): 
  | return maximum of x and y. 
* | **IfThenElse**\ (\ *condition*\ , \ *ifTrue*\ , \ *ifFalse*\ ): 
  | Symbolic function for conditional evaluation. If the condition evaluates to True, the expression ifTrue is evaluated, while otherwise expression ifFalse is evaluated
  | *Example*:

  .. code-block:: python

     x=SymReal(-1)
     y=SymReal(2,'y')
     a=SymReal.IfThenElse(x<0, y+1, y-1))

* | **SetRecording**\ (\ *flag*\ ): 
  | Set current (global / module-wide) status of expression recording. By default, recording is on.
  | *Example*:

  .. code-block:: python

     SymReal.SetRecording(True)

* | **GetRecording**\ (): 
  | Get current (global / module-wide) status of expression recording.
  | *Example*:

  .. code-block:: python

     SymReal.GetRecording()




symbolic.Vector
===============




A symbolic Vector type to replace Python's (1D) numpy array in symbolic expressions. The \ ``symbolic.Vector``\  may be directly set to a list of floats or (1D) numpy array and be evaluated as array. However, turing on recording by using 	extttexudyn.symbolic.SetRecording(True) (on by default), results are stored as expression trees, which may be evaluated in C++ or Python, in particular in user functions, see the following example:

.. code-block:: python
   :linenos:
   
   import exudyn as exu
   esym = exu.symbolic
   
   SymVector = esym.Vector
   SymReal = esym.Real 
   
   a = SymReal(42.,"a")
   b = SymReal(13)
   c = a-3*b
   
   #create from list:
   v1 = SymVector([1,3,2])
   print('v1: ',v1)
   
   #create from numpy array:
   v2 = SymVector(np.array([1,3,2]))
   print('v2 initial: ',v2)
   
   #create from list, mixing symbolic expressions and numbers:
   v2 = SymVector([a,42,c])
   
   print('v2 now: ',v2,"=",v2.Evaluate())
   print('v1+v2: ',v1+v2,"=",(v1+v2).Evaluate()) #evaluate as vector
   
   print('v1*v2: ',v1*v2,"=",(v1*v2).Evaluate()) #evaluate as Real
   
   #access of vector component:
   print('v1[2]: ',v1[2],"=",v1[2].Evaluate())   #evaluate as Real

To create a symbolic Vector, use \ ``aa=symbolic.Vector([3,4.2,5]``\  to build a Python object aa with values [3,4.2,5]. In order to use a named vector, use \ ``v=symbolic.Vector([3,4.2,5],'myVec')``\ . Vectors can be also created from mixed symbolic expressions and numbers, such as \ ``v=symbolic.Vector([x,x**2,3.14])``\ , however, this cannot become a named vector as it contains expressions. There is a significance difference to numpy, such that '*' represents the scalar vector multplication which gives a scalar. Furthermore, the comparison operator '==' gives only True, if all components are equal, and the operator '!=' gives True, if any component is unequal. Note that in the following, we use the abbreviation \ ``SymVector=exudyn.symbolic.Vector``\ . Note that only functions are able to be recorded. Member functions of \ ``SymVector``\  are:

\ The class **symbolic.Vector** has the following **functions and structures**:

* | **Evaluate**\ (): 
  | Return evaluated expression (prioritized) or stored vector value. (not recorded)
* | **SetVector**\ (\ *vector*\ ): 
  | Set stored vector or named vector expression to new given (non-symbolic) vector. Only works, if SymVector contains no expression. (may lead to inconsistencies in recording)
* | **NumberOfItems**\ (): 
  | Get size of Vector (may require to evaluate expression; not recording)
* | **data[index]= ...**\ \ *i*\ : 
  | bracket [] operator for setting a component of the vector. Only works, if SymVector contains no expression. (may lead to inconsistencies in recording)
  | *Example*:

  .. code-block:: python

     v1 = SymVector([1,3,2])
     v1[2]=13.

* | **NormL2**\ (): 
  | return (symbolic) L2-norm of vector.
  | *Example*:

  .. code-block:: python

     v1 = SymVector([1,4,8])
     length = v1.NormL2() #gives 9.

* | **MultComponents**\ (\ *other*\ ): 
  | Perform component-wise multiplication of vector times other vector and return result. This corresponds to the numpy multiplication using '*'.
  | *Example*:

  .. code-block:: python

     v1 = SymVector([1,2,4])
     v2 = SymVector([1,0.5,0.25])
     v3 = v1.MultComponents(v2)

* | **... = data[index]**\ \ *i*\ : 
  | return (symbolic) component of vector, allowing only read-access. Index may also evaluate from an expression.
  | *Example*:

  .. code-block:: python

     v1 = SymVector([1,3,2])
     print('v1.z=',v1[2])

* | **\_\_str\_\_**:
  | conversion of SymVector to string
* | **\_\_repr\_\_**:
  | representation of SymVector in Python



Standard vector operators are available for \ ``SymVector``\ , see the following examples:

.. code-block:: python
   :linenos:
   
   v = SymVector([1,3,2])
   w = SymVector([3.3,2.2,1.1])
   
   u = v+w
   u = v-w
   u = -v
   #scalar multiplication; evaluates to SymReal:
   x = v*w 
   #NOTE: component-wise multiplication, returns SymVector:
   u = v.MultComponents(w)
   
   #inplace operators:
   v += w
   v -= w
   v *= SymReal(0.5)

symbolic.VariableSet
====================




A container for symbolic variables, in particular for exchange between user functions and the model. For details, see the following example:

.. code-block:: python
   :linenos:
   
   import exudyn as exu
   import math
   SymReal = exu.symbolic.Real
   
   #use global variable set:
   variables = exu.symbolic.variables
   
   #create a named Real
   a = SymReal("a",42.)
   
   #regular way to add variable:
   variables.Add('pi', math.pi)
   
   #add named variable (doesn't need a name):
   variables.Add(a)
   
   #print current variable set
   print(variables)
   
   print('pi=',variables.Get('pi').Evaluate()) #3.14
   print('a=',variables.Get('a')) #prints 'a'
   
   x=variables.Get('a')
   print('x=',x.Evaluate()) #x=42
   
   #override a
   variables.Set('a',3.33)
   
   #x is depending on a:
   print('x:',x,"=",x.Evaluate()) #3.33
   
   #create your own variable set
   mySet = esym.VariableSet()

\ The class **symbolic.VariableSet** has the following **functions and structures**:

* | **Add**\ (\ *name*\ , \ *value*\ ): 
  | Add a variable with name and value (name may not exist)
* | **Add**\ (\ *namedReal*\ ): 
  | Add a variable with named real (name may not exist)
* | **Set**\ (\ *name*\ , \ *value*\ ): 
  | Set a variable with name and value (adds new or overrides existing)
* | **Get**\ (\ *name*\ ): 
  | Get a variable by name
* | **Exists**\ (\ *name*\ ): 
  | Return True, if variable name exists
* | **Reset**\ (): 
  | Erase all variables and reset VariableSet
* | **NumberOfItems**\ (\ *name*\ ): 
  | Return True, if variable name exists
* | **GetNames**\ (): 
  | Get list of stored variable names
* | **data[index]= ...**\ \ *name*\ , \ *value*\ : 
  | bracket [] operator for setting a variable to a specific value
* | **... = data[index]**\ \ *name*\ : 
  | bracket [] operator for getting a specific variable by name
* | **\_\_str\_\_**:
  | create string of set of variables
* | **\_\_repr\_\_**:
  | representation of SymVector in Python



symbolic.UserFunction
=====================




A class for creating and handling symbolic user functions in C++. Use these functions for high performance extensions, e.g., of existing objects or loadsFor details, see the following example:

.. code-block:: python
   :linenos:
   
   import exudyn as exu
   esym = exu.symbolic
   from exudyn.utilities import * #advancedUtilities with user function utilities included
   SymReal = exu.symbolic.Real
   
   #regular Python user function with esym math functions
   def UFload(mbs, t, load):
       return load*esym.sin(10*(2*pi)*t)
   
   #add ground and mass point:
   oGround = mbs.CreateGround()
   oMassPoint = mbs.CreateMassPoint(referencePosition=[1.+0.05,0,0], physicsMass=1)
   
   #add marker and load:
   mc = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=mbs.GetObject[oMassPoint]['nodeNumber'], coordinate=0))
   load = mbs.AddLoad(LoadCoordinate(markerNumber=mc, load=10))
   
   #create symbolic user function from Python user function:
   symFuncLoad = CreateSymbolicUserFunction(mbs, UFload, load, 'loadUserFunction',1)
   #set this user function to C++ object:
   c.TransferUserFunction2Item(mbs, load, 'loadUserFunction')    
   
   #print string of symbolic expression of user function (to check if it looks ok):
   print('load user function: ',symFuncLoad)
   
   #test evaluate user function; requires args of user function:
   print('load user function: ',symFuncLoad.Evaluate(mbs, 0.025, 10.))
       
   #now you could add further items or simulate ...

\ The class **symbolic.UserFunction** has the following **functions and structures**:

* | **Evaluate**\ (): 
  | Evaluate symbolic function with test values; requires exactly same args as Python user functions; this is slow and only intended for testing
* | **SetUserFunctionFromDict**\ (\ *mainSystem*\ , \ *fcnDict*\ , \ *itemIndex*\ , \ *userFunctionName*\ ): 
  | Create C++ std::function (as requested in C++ item) with symbolic user function as recorded in given dictionary, as created with ConvertFunctionToSymbolic(...).
* | **TransferUserFunction2Item**\ (\ *mainSystem*\ , \ *itemIndex*\ , \ *userFunctionName*\ ): 
  | Transfer the std::function to a given object, load or other; this needs to be done purely in C++ to avoid Pybind overheads.
* | **\_\_repr\_\_**:
  | Representation of Symbolic function
* | **\_\_str\_\_**:
  | Convert stored symbolic function to string


