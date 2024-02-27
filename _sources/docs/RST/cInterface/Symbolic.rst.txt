

.. _sec-cinterface-symbolic:

********
Symbolic
********

The Symbolic sub-module in \ ``exudyn.symbolic``\  allows limited symbolic manipulations in Exudyn and is currently under development In particular, symbolic user functions can be created, which allow significant speedup of Python user functions. However, \ **always veryfy your symbolic expressions or user functions**\ , as behavior may be unexpected in some cases. 

symbolic.Real
=============




The symbolic Real type allows to replace Python's float by a symbolic quantity. The \ ``symbolic.Real``\  may be directly set to a float and be evaluated as float. However, turing on recording by using 	exttt{exudyn.symbolic.SetRecording(True)} (on by default), results are stored as expression trees, which may be evaluated in C++ or Python, in particular in user functions, see the following example:

.. code-block:: python
   :linenos:
   
   import exudyn as exu
   esym = exu.symbolic     #abbreviation
   SymReal = esym.Real     #abbreviation
   
   #create some variables
   a = SymReal('a',42.)    #use named expression
   b = SymReal(13)         #b is 13
   c = a+b*7.+1.-3         #c stores expression tree
   d = c                   #d and c are containing same tree!
   print('a: ',a,' = ',a.Evaluate())
   print('c: ',c,' = ',c.Evaluate())
   
   #use special functions:
   d = a+b*esym.sin(a)+esym.cos(SymReal(7))
   print('d: ',d,' = ',d.Evaluate())
   
   a.SetValue(14)          #variable a set to new value; influences d
   print('d: ',d,' = ',d.Evaluate())
   
   a = SymReal(1000)       #a is now a new variable; not updated in d!
   print('d: ',d,' = ',d.Evaluate())
   
   #compute derivatives (automatic differentiation):
   x = SymReal("x",0.5)
   f = a+b*esym.sin(x)+esym.cos(SymReal(7))+x**4
   print('f=',f.Evaluate(), ', diff=',f.Diff(x))
   
   #turn off recording of trees (globally for all symbolic.Real!):
   esym.SetRecording(False)
   x = SymReal(42) #now, only represents a value
   y = x/3.       #directly evaluates to 14

To create a symbolic Real, use \ ``aa=symbolic.Real(1.23)``\  to build a Python object aa with value 1.23. In order to use a named value, use \ ``pi=symbolic.Real('pi',3.14)``\ . Note that in the following, we use the abbreviation \ ``SymReal=exudyn.symbolic.Real``\ . Member functions of \ ``SymReal``\ , which are \ **not recorded**\ , are:

\ The class **symbolic.Real** has the following **functions and structures**:

* | **\_\_init\_\_**\ (\ *value*\ ): 
  | Construct symbolic.Real from float.
* | **\_\_init\_\_**\ (\ *name*\ , \ *value*\ ): 
  | Construct named symbolic.Real from name and float.
* | **SetValue**\ (\ *valueInit*\ ): 
  | Set either internal float value or value of named expression; cannot change symbolic expressions.
  | *Example*:

  .. code-block:: python

     b = SymReal(13)
     b.SetValue(14) #now b is 14
     #b.SetValue(a+3.) #not possible!

* | **Evaluate**\ (): 
  | return evaluated expression (prioritized) or stored Real value.
* | **Diff**\ (\ *var*\ ): 
  | (UNTESTED!) return derivative of stored expression with respect to given symbolic named variable; NOTE: when defining the expression of the variable which shall be differentiated, the variable may only be changed with the SetValue(...) method hereafter!
  | *Example*:

  .. code-block:: python

     x=SymReal('x',2)
     f=3*x+x**2*sin(x)
     f.Diff(x) #evaluate derivative w.r.t. x

* | **value**:
  | access to internal float value, which is used in case that symbolic.Real has been built from a float (but without a name and without symbolic expression)
* | operator **\_\_float\_\_**\ ():
  | evaluation of expression and conversion of symbolic.Real to Python float
* | operator **\_\_str\_\_**\ ():
  | conversion of symbolic.Real to string
* | operator **\_\_repr\_\_**\ ():
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

Mathematical functions may be called with an \ ``SymReal``\  or with a \ ``float``\ . Most standard mathematical functions exist for \ ``symbolic``\ , e.g., as \ ``symbolic.abs``\ . \ **HINT**\ : function names are lower-case for compatibility with Python's math library. Thus, you can easily exchange math.sin with esym.sin, and you may want to use a generic name, such as myMath=symbolic in order to switch between Python and symbolic user functions. The following functions exist:

\ The class **symbolic** has the following **functions and structures**:

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

\ The class **symbolic** has the following **functions and structures**:

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

     symbolic.Real.GetRecording()




symbolic.Vector
===============




A symbolic Vector type to replace Python's (1D) numpy array in symbolic expressions. The \ ``symbolic.Vector``\  may be directly set to a list of floats or (1D) numpy array and be evaluated as array. However, turing on recording by using 	exttt{exudyn.symbolic.SetRecording(True)} (on by default), results are stored as expression trees, which may be evaluated in C++ or Python, in particular in user functions, see the following example:

.. code-block:: python
   :linenos:
   
   import exudyn as exu
   esym = exu.symbolic
   
   SymVector = esym.Vector
   SymReal = esym.Real 
   
   a = SymReal('a',42.)
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

To create a symbolic Vector, use \ ``aa=symbolic.Vector([3,4.2,5]``\  to build a Python object aa with values [3,4.2,5]. In order to use a named vector, use \ ``v=symbolic.Vector('myVec',[3,4.2,5])``\ . Vectors can be also created from mixed symbolic expressions and numbers, such as \ ``v=symbolic.Vector([x,x**2,3.14])``\ , however, this cannot become a named vector as it contains expressions. There is a significance difference to numpy, such that '*' represents the scalar vector multplication which gives a scalar. Furthermore, the comparison operator '==' gives only True, if all components are equal, and the operator '!=' gives True, if any component is unequal. Note that in the following, we use the abbreviation \ ``SymVector=exudyn.symbolic.Vector``\ . Note that only functions are able to be recorded. Member functions of \ ``SymVector``\  are:

\ The class **symbolic.Vector** has the following **functions and structures**:

* | **\_\_init\_\_**\ (\ *vector*\ ): 
  | Construct symbolic.Vector from vector represented as numpy array or list (which may contain symbolic expressions).
* | **\_\_init\_\_**\ (\ *name*\ , \ *vector*\ ): 
  | Construct named symbolic.Vector from name and vector represented as numpy array or list (which may contain symbolic expressions).
* | **Evaluate**\ (): 
  | Return evaluated expression (prioritized) or stored vector value. (not recorded)
* | **SetVector**\ (\ *vector*\ ): 
  | Set stored vector or named vector expression to new given (non-symbolic) vector. Only works, if SymVector contains no expression. (may lead to inconsistencies in recording)
* | **NumberOfItems**\ (): 
  | Get size of Vector (may require to evaluate expression; not recording)
* | operator **\_\_setitem\_\_**\ (index):
  | bracket [] operator for setting a component of the vector. Only works, if SymVector contains no expression. (may lead to inconsistencies in recording)
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

* | operator **\_\_getitem\_\_**\ (index):
  | bracket [] operator to return (symbolic) component of vector, allowing read-access. Index may also evaluate from an expression.
* | operator **\_\_str\_\_**\ ():
  | conversion of SymVector to string
* | operator **\_\_repr\_\_**\ ():
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

symbolic.Matrix
===============




A symbolic Matrix type to replace Python's (2D) numpy array in symbolic expressions. The \ ``symbolic.Matrix``\  may be directly set to a list of list of floats or (2D) numpy array and be evaluated as array. However, turing on recording by using 	exttt{exudyn.symbolic.SetRecording(True)} (on by default), results are stored as expression trees, which may be evaluated in C++ or Python, in particular in user functions, see the following example:

.. code-block:: python
   :linenos:
   
   import exudyn as exu
   import numpy as np
   esym = exu.symbolic
   
   SymMatrix = esym.Matrix
   SymReal = esym.Real 
   
   a = SymReal('a',42.)
   b = SymReal(13)
   
   #create matrix from list of lists
   m1 = SymMatrix([[1,3,2],[4,5,6]])
   
   #create symbolic matrix from list of lists
   m3 = SymMatrix([[a,3*b,2],[4,5,6]])
   
   #create from numpy array
   m2 = SymMatrix(np.ones((3,3))-np.eye(3))
   
   m1 += m3
   m1 *= 3
   m1 -= 3*m3
   print('m1: ',m1)
   print('m2: ',m2)
   

To create a symbolic Matrix, use \ ``aa=symbolic.Matrix([[3,4.2],[3.3,1.2]]``\  to build a Python object aa. In order to use a named matrix, use \ ``v=symbolic.Matrix('myMat',[3,4.2,5])``\ . Matrixs can be also created from mixed symbolic expressions and numbers, such as \ ``v=symbolic.Matrix([x,x**2,3.14])``\ , however, this cannot become a named matrix as it contains expressions. There is a significance difference to numpy, such that '*' represents the matrix multplication (compute components from row times column operations). Note that in the following, we use the abbreviation \ ``SymMatrix=exudyn.symbolic.Matrix``\ . Member functions of \ ``SymMatrix``\  are:

\ The class **symbolic.Matrix** has the following **functions and structures**:

* | **\_\_init\_\_**\ (\ *matrix*\ ): 
  | Construct symbolic.Matrix from vector represented as numpy array or list of lists (which may contain symbolic expressions).
* | **\_\_init\_\_**\ (\ *name*\ , \ *matrix*\ ): 
  | Construct named symbolic.Matrix from name and vector represented as numpy array or list of lists (which may contain symbolic expressions).
* | **Evaluate**\ (): 
  | Return evaluated expression (prioritized) or stored Matrix value. (not recorded)
* | **SetMatrix**\ (\ *matrix*\ ): 
  | Set stored Matrix or named Matrix expression to new given (non-symbolic) Matrix. Only works, if SymMatrix contains no expression. (may lead to inconsistencies in recording)
* | **NumberOfRows**\ (): 
  | Get number of rows (may require to evaluate expression; not recording)
* | **NumberOfColumns**\ (): 
  | Get number of columns (may require to evaluate expression; not recording)
* | operator **\_\_setitem\_\_**\ (row, column):
  | bracket [] operator for (symbolic) component of Matrix (write-access). Only works, if SymMatrix contains no expression. (may lead to inconsistencies in recording)
* | operator **\_\_getitem\_\_**\ (row, column):
  | bracket [] operator for (symbolic) component of Matrix (read-access). Row and column may also evaluate from an expression.
* | operator **\_\_str\_\_**\ ():
  | conversion of SymMatrix to string
* | operator **\_\_repr\_\_**\ ():
  | representation of SymMatrix in Python



Standard Matrix operators are available for \ ``SymMatrix``\ , see the following examples:

.. code-block:: python
   :linenos:
   
   m1 = SymMatrix([[1,7],[4,5]])
   m2 = SymMatrix([[1,2.2],[4,4.3]])
   v = SymVector([1.5,3])
   
   m3 = m1+m2
   m3 = m1-m2
   m3 = m1*m2
   
   #multiply with scalar
   m3 = 13*m2
   m3 = m2*3.14
   
   #multiply with vector
   m3 = m2*v
   
   #transposed:
   m3 = v*m2 #equals numpy operation m2.T @ v
   
   #inplace operators:
   m1 += m1
   m1 -= m1
   m1 *= 3.14

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
   a = SymReal('a',42.)
   
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
* | operator **\_\_str\_\_**\ ():
  | create string of set of variables
* | operator **\_\_repr\_\_**\ ():
  | representation of SymMatrix in Python



symbolic.UserFunction
=====================




A class for creating and handling symbolic user functions in C++. Use these functions for high performance extensions, e.g., of existing objects or loadsFor details, see the following example:

.. code-block:: python
   :linenos:
   
   import exudyn as exu
   esym = exu.symbolic
   from exudyn.utilities import * #advancedUtilities with user function utilities included
   SymReal = exu.symbolic.Real
   
   SC = exu.SystemContainer()
   mbs = SC.AddSystem()
   
   #regular Python user function with esym math functions
   def UFload(mbs, t, load):
       return load*esym.sin(10*(2*pi)*t)
   
   #create symbolic user function from Python user function:
   symFuncLoad = CreateSymbolicUserFunction(mbs, UFload, load, 'loadUserFunction',verbose=1)
   
   #add ground and mass point:
   oGround = mbs.CreateGround()
   oMassPoint = mbs.CreateMassPoint(referencePosition=[1.+0.05,0,0], physicsMass=1)
   
   #add marker and load:
   mc = mbs.AddMarker(MarkerNodeCoordinate(nodeNumber=mbs.GetObject(oMassPoint)['nodeNumber'], coordinate=0))
   load = mbs.AddLoad(LoadCoordinate(markerNumber=mc, load=10,
                                     loadUserFunction=symFuncLoad))
   
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
* | operator **\_\_repr\_\_**\ ():
  | Representation of Symbolic function
* | operator **\_\_str\_\_**\ ():
  | Convert stored symbolic function to string


