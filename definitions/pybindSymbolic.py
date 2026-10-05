#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  the symbolic submodule; pb holds the chapter start, symbolicModule the classes and
#           functions, whose stubs go into stubSymbolic.pyi.
#           The calls are recorded by PybindInterface (pybindTypes.py) and replayed by
#           tools/generators/pybindEmitter.py into pybind_manual_classes.h, the stub fragments and
#           the Python-C++ interface documentation.
#
#           DESCRIPTIONS: read definitions/README.md, section "Writing a
#           description", before writing or changing one - what the text may
#           contain, and how it is checked.
#
# Author:   Johannes Gerstmayr
# Date:     2018-05-18 (created in autoGeneratePyBindings.py), 2026-09-14 (moved to definitions/)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from pybindTypes import *

pb = PybindInterface()
symbolicModule = PybindInterface()

#add symbolic as a submodule rather than a class (similar to exudyn, but in other file...)

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
pb.CreateNewRSTfile('Symbolic')

pb.AddDocu(r"""The Symbolic sub-module in `exudyn.symbolic` allows limited symbolic manipulations in Exudyn and is currently under development In particular, symbolic user functions can be created, which allow significant speedup of Python user functions. However, **always verify your symbolic expressions or user functions**, as behavior may be unexpected in some cases. """,
            section='Symbolic', sectionLevel=1,sectionLabel='sec:cinterface:symbolic')

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#currently, only latex + RST binding:
pyClassStr = 'symbolic.Real'
classStr = 'Symbolic::SReal'



symbolicModule.DefPyStartClass(classStr, pyClassStr, '', subSection=True)

symbolicModule.AddDocu(r"""The symbolic Real type allows to replace Python's float by a symbolic quantity. The `symbolic.Real` may be directly set to a float and be evaluated as float. However, turning on recording by using `exudyn.symbolic.SetRecording(True)` (on by default), results are stored as expression trees, which may be evaluated in C++ or Python, in particular in user functions, see the following example:"""
            )

symbolicModule.AddDocuNotebook('python/Notebooks/reference/symbolic.ipynb', part='real')

symbolicModule.AddDocu(r"""To create a symbolic Real, use `aa=symbolic.Real(1.23)` to build a Python object aa with value 1.23. In order to use a named value, use `pi=symbolic.Real('pi',3.14)`. Note that in the following, we use the abbreviation `SymReal=exudyn.symbolic.Real`, and the examples `esym=exudyn.symbolic`. Member functions of `SymReal`, which are **not recorded**, are:""")

symbolicModule.DefStartTable(pyClassStr)

# def __init__(self, arg1: int, arg2: float) -> None: ...
symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='__init__', cName='', 
                        description="Construct symbolic.Real from float.",
                        argList=['value'],
                        argTypes=['float'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='__init__', cName='', 
                        description="Construct named symbolic.Real from name and float.",
                        argList=['name', 'value'],
                        argTypes=['str', 'float'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='SetValue', cName='', 
                        description="Set either internal float value or value of named expression; cannot change symbolic expressions.",
                        example = r"""b = esym.Real(13)\\b.SetValue(14) #now b is 14\\#b.SetValue(a+3.) #not possible!""",
                        argList=['valueInit'],
                        argTypes=['float'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Evaluate', cName='', 
                        description="return evaluated expression (prioritized) or stored Real value.",
                        returnType='float',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Diff', cName='', 
                        description="(UNTESTED!) return derivative of stored expression with respect to given symbolic named variable; NOTE: when defining the expression of the variable which shall be differentiated, the variable may only be changed with the SetValue(...) method hereafter!",
                        example = r"""x=esym.Real('x',2)\\f=3*x+x**2*esym.sin(x)\\f.Diff(x) #evaluate derivative w.r.t. x""",
                        returnType='float',
                        argList=['var'],
                        argTypes=['symbolic.Real'],
                        )

symbolicModule.DefDataAccess('value','access to internal float value, which is used in case that symbolic.Real has been built from a float (but without a name and without symbolic expression)',
                       dataType = 'float')

symbolicModule.DefOperator('__float__','evaluation of expression and conversion of symbolic.Real to Python float',
                       returnType = 'float')

symbolicModule.DefOperator('__str__','conversion of symbolic.Real to string',
                       returnType = 'str')

symbolicModule.DefOperator('__repr__','representation of symbolic.Real in Python',
                       returnType = 'str')

symbolicModule.DefFinishTable()#only finalize latex table


symbolicModule.AddDocu(r"""The remaining operators and mathematical functions are recorded within expressions. Main mathematical operators for `SymReal` exist, similar to Python, such as:""")

symbolicModule.AddDocuNotebook('python/Notebooks/reference/symbolic.ipynb', part='realOperators')

symbolicModule.AddDocu(r"""Mathematical functions may be called with an `SymReal` or with a `float`. Most standard mathematical functions exist for `symbolic`, e.g., as `symbolic.abs`. **HINT**: function names are lower-case for compatibility with Python's math library. Thus, you can easily exchange math.sin with esym.sin, and you may want to use a generic name, such as myMath=symbolic in order to switch between Python and symbolic user functions. The following functions exist:""")

symbolicModule.StubCode('\n#functions directly in symbolic module:\n')

pyClassStr = 'symbolic'
classStr = ''
#these functions are in symbolic.Real:
symbolicModule.DefStartTable(pyClassStr)
fnList=['isfinite','abs',#'sign',
        'round','ceil','floor',
        'sqrt','exp','log',
        'sin','cos','tan','asin','acos','atan',
        'sinh','cosh','tanh','asinh','acosh','atanh',
                ]

for fnName in fnList:
    cfnName = fnName
    if fnName=='abs' or fnName=='mod':
        cfnName = 'f'+cfnName
    #empty class indicates that these are functions in symbolic:
    symbolicModule.DefPyFunctionAccess(cClass='', pyName=fnName, cName='', 
                            description="according to specification of C++ std::"+cfnName,
                            argList=['x'],
                            argTypes=['symbolic.Real'],
                            returnType='symbolic.Real',
                            )

symbolicModule.DefFinishTable()#only finalize latex table

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++

symbolicModule.AddDocu(r"""The following table lists special functions for `SymReal`: """)

symbolicModule.DefStartTable(pyClassStr)

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='sign', cName='', 
                        description="returns 0 for x=0, -1 for x<0 and 1 for x>1.",
                        argList=['x'],
                        argTypes=['symbolic.Real'],
                        returnType='symbolic.Real',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Not', cName='', 
                        description="returns logical not of expression, equal to Python's 'not'. Not(True)=False, Not(0.)=True, Not(-0.1)=False",
                        argList=['x'],
                        argTypes=['symbolic.Real'],
                        returnType='symbolic.Real',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='min', cName='', 
                        description="return minimum of x and y. ",
                        argList=['x','y'],
                        argTypes=['symbolic.Real','symbolic.Real'],
                        returnType='symbolic.Real',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='max', cName='', 
                        description="return maximum of x and y. ",
                        argList=['x','y'],
                        argTypes=['symbolic.Real','symbolic.Real'],
                        returnType='symbolic.Real',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='mod', cName='', 
                        description="return floating-point remainder of the division operation x / y. For example, mod(5.1, 3) gives 2.1 as a remainder.",
                        argList=['x','y'],
                        argTypes=['symbolic.Real','symbolic.Real'],
                        returnType='symbolic.Real',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='pow', cName='', 
                        description=r"""return $x^y$. """,
                        argList=['x','y'],
                        argTypes=['symbolic.Real','symbolic.Real'],
                        returnType='symbolic.Real',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='IfThenElse', cName='', 
                        description="Symbolic function for conditional evaluation. If the condition evaluates to True, the expression ifTrue is evaluated, while otherwise expression ifFalse is evaluated",
                        example = r"""x=esym.Real(-1)\\y=esym.Real('y',2)\\a=esym.IfThenElse(x<0, y+1, y-1)""",
                        argList=['condition','ifTrue','ifFalse'],
                        argTypes=['symbolic.Real','symbolic.Real','symbolic.Real'],
                        returnType='symbolic.Real',
                        )

#+++++++++++++++++++++++
symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='SetRecording', cName='', 
                        description="Set current (global / module-wide) status of expression recording. By default, recording is on.",
                        example = "esym.SetRecording(True)",
                        argList=['flag'],
                        argTypes=['bool'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='GetRecording', cName='', 
                        description="Get current (global / module-wide) status of expression recording.",
                        example = "esym.GetRecording()",
                        returnType='bool',
                        )

symbolicModule.DefFinishTable()#only finalize latex table
symbolicModule.StubCode('\n')

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#currently, only latex + RST binding:
pyClassStr = 'symbolic.Vector'
classStr = 'Symbolic::SymbolicRealVector'

symbolicModule.DefPyStartClass(classStr, pyClassStr, '', subSection=True)

symbolicModule.AddDocu(r"""A symbolic Vector type to replace Python's (1D) numpy array in symbolic expressions. The `symbolic.Vector` may be directly set to a list of floats or (1D) numpy array and be evaluated as array. However, turning on recording by using `exudyn.symbolic.SetRecording(True)` (on by default), results are stored as expression trees, which may be evaluated in C++ or Python, in particular in user functions, see the following example:"""
            )

symbolicModule.AddDocuNotebook('python/Notebooks/reference/symbolic.ipynb', part='vector')

symbolicModule.AddDocu(r"""To create a symbolic Vector, use `aa=symbolic.Vector([3,4.2,5]` to build a Python object aa with values [3,4.2,5]. In order to use a named vector, use `v=symbolic.Vector('myVec',[3,4.2,5])`. Vectors can be also created from mixed symbolic expressions and numbers, such as `v=symbolic.Vector([x,x**2,3.14])`, however, this cannot become a named vector as it contains expressions. There is a significance difference to numpy, such that '*' represents the scalar vector multplication which gives a scalar. Furthermore, the comparison operator '==' gives only True, if all components are equal, and the operator '!=' gives True, if any component is unequal. Note that in the following, we use the abbreviation `SymVector=exudyn.symbolic.Vector`, and the examples `esym=exudyn.symbolic`. Note that only functions are able to be recorded. Member functions of `SymVector` are:""")

symbolicModule.DefStartTable(pyClassStr)

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='__init__', cName='', 
                        description="Construct symbolic.Vector from vector represented as numpy array or list (which may contain symbolic expressions).",
                        argList=['vector'],
                        argTypes=['List[float]'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='__init__', cName='', 
                        description="Construct named symbolic.Vector from name and vector represented as numpy array or list (which may contain symbolic expressions).",
                        argList=['name', 'vector'],
                        argTypes=['str', 'List[float]'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Evaluate', cName='', 
                        description="Return evaluated expression (prioritized) or stored vector value. (not recorded)",
                        returnType='List[float]',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='SetVector', cName='', 
                        description="Set stored vector or named vector expression to new given (non-symbolic) vector. Only works, if SymVector contains no expression. (may lead to inconsistencies in recording)",
                        argList=['vector'],
                        argTypes=['symbolic.Vector'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfItems', cName='', 
                        description="Get size of Vector (may require to evaluate expression; not recording)",
                        returnType='int',
                        )

# symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', cName='', 
#                         description="bracket [] operator for setting a component of the vector. Only works, if SymVector contains no expression. (may lead to inconsistencies in recording)",
#                         example = "v1 = esym.Vector([1,3,2])\\\\v1[2]=13.",
#                         argList=['i'],
#                         argTypes=['symbolic.Real'],
#                         returnType='None',
#                         )
symbolicModule.DefOperator(name='__setitem__', 
                     description="bracket [] operator for setting a component of the vector. Only works, if SymVector contains no expression. (may lead to inconsistencies in recording)",
                     argList=['index'],
                     argTypes=['symbolic.Real'],
                     returnType='symbolic.Real',
                     )

#recorded:
symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='NormL2', cName='', 
                        description="return (symbolic) L2-norm of vector.",
                        example = r"""v1 = esym.Vector([1,4,8])\\length = v1.NormL2() #gives 9.""",
                        returnType='symbolic.Real',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='MultComponents', cName='', 
                        description="Perform component-wise multiplication of vector times other vector and return result. This corresponds to the numpy multiplication using '*'.",
                        example = r"""v1 = esym.Vector([1,2,4])\\v2 = esym.Vector([1,0.5,0.25])\\v3 = v1.MultComponents(v2)""",
                        argList=['other'],
                        argTypes=['symbolic.Vector'],
                        returnType='symbolic.Real',
                        )

symbolicModule.DefOperator(name='__getitem__', 
                     description="bracket [] operator to return (symbolic) component of vector, allowing read-access. Index may also evaluate from an expression.",
                     argList=['index'],
                     argTypes=['symbolic.Real'],
                     returnType='symbolic.Real',
                     )


symbolicModule.DefOperator('__str__','conversion of SymVector to string',
                       returnType = 'str')

symbolicModule.DefOperator('__repr__','representation of SymVector in Python',
                       returnType = 'str')


#+++++++++
symbolicModule.DefFinishTable()#only finalize latex table
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++

symbolicModule.AddDocu(r"""Standard vector operators are available for `SymVector`, see the following examples:""")

symbolicModule.AddDocuNotebook('python/Notebooks/reference/symbolic.ipynb', part='vectorOperators')

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#currently, only latex + RST binding:
pyClassStr = 'symbolic.Matrix'
classStr = 'Symbolic::SymbolicRealMatrix'

symbolicModule.DefPyStartClass(classStr, pyClassStr, '', subSection=True)

symbolicModule.AddDocu(r"""A symbolic Matrix type to replace Python's (2D) numpy array in symbolic expressions. The `symbolic.Matrix` may be directly set to a list of list of floats or (2D) numpy array and be evaluated as array. However, turning on recording by using `exudyn.symbolic.SetRecording(True)` (on by default), results are stored as expression trees, which may be evaluated in C++ or Python, in particular in user functions, see the following example:"""
            )

symbolicModule.AddDocuNotebook('python/Notebooks/reference/symbolic.ipynb', part='matrix')

symbolicModule.AddDocu(r"""To create a symbolic Matrix, use `aa=symbolic.Matrix([[3,4.2],[3.3,1.2]]` to build a Python object aa. In order to use a named matrix, use `v=symbolic.Matrix('myMat',[3,4.2,5])`. Matrixs can be also created from mixed symbolic expressions and numbers, such as `v=symbolic.Matrix([x,x**2,3.14])`, however, this cannot become a named matrix as it contains expressions. There is a significance difference to numpy, such that '*' represents the matrix multplication (compute components from row times column operations). Note that in the following, we use the abbreviation `SymMatrix=exudyn.symbolic.Matrix`, and the examples `esym=exudyn.symbolic`. Member functions of `SymMatrix` are:""")

symbolicModule.DefStartTable(pyClassStr)

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='__init__', cName='', 
                        description="Construct symbolic.Matrix from vector represented as numpy array or list of lists (which may contain symbolic expressions).",
                        argList=['matrix'],
                        argTypes=['List[List[float]]'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='__init__', cName='', 
                        description="Construct named symbolic.Matrix from name and vector represented as numpy array or list of lists (which may contain symbolic expressions).",
                        argList=['name', 'matrix'],
                        argTypes=['str', 'List[List[float]]'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Evaluate', cName='', 
                        description="Return evaluated expression (prioritized) or stored Matrix value. (not recorded)",
                        returnType='List[float]',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='SetMatrix', cName='', 
                        description="Set stored Matrix or named Matrix expression to new given (non-symbolic) Matrix. Only works, if SymMatrix contains no expression. (may lead to inconsistencies in recording)",
                        argList=['matrix'],
                        argTypes=['NDArray[Any, float]'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfRows', cName='', 
                        description="Get number of rows (may require to evaluate expression; not recording)",
                        returnType='int',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfColumns', cName='', 
                        description="Get number of columns (may require to evaluate expression; not recording)",
                        returnType='int',
                        )

symbolicModule.DefOperator(name='__setitem__', 
                     description="bracket [] operator for (symbolic) component of Matrix (write-access). Only works, if SymMatrix contains no expression. (may lead to inconsistencies in recording)",
                     argList=['row','column'],
                     argTypes=['symbolic.Real','symbolic.Real'],
                     returnType='symbolic.Real',
                     )

#recorded:
# symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='NormL2', cName='', 
#                         description="return (symbolic) L2-norm of Matrix.",
#                         example = "v1 = esym.Matrix([1,4,8])\\\\length = v1.NormL2() #gives 9.",
#                         returnType='symbolic.Real',
#                         )

# symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='MultComponents', cName='', 
#                         description="Perform component-wise multiplication of Matrix times other Matrix and return result. This corresponds to the numpy multiplication using '*'.",
#                         example = "v1 = esym.Matrix([1,2,4])\\\\v2 = esym.Matrix([1,0.5,0.25])\\\\v3 = v1.MultComponents(v2)",
#                         argList=['other'],
#                         argTypes=['sym.Matrix'],
#                         returnType='symbolic.Real',
#                         )

symbolicModule.DefOperator(name='__getitem__', 
                     description="bracket [] operator for (symbolic) component of Matrix (read-access). Row and column may also evaluate from an expression.",
                     argList=['row','column'],
                     argTypes=['symbolic.Real','symbolic.Real'],
                     returnType='symbolic.Real',
                     )


symbolicModule.DefOperator('__str__','conversion of SymMatrix to string',
                     returnType = 'str')

symbolicModule.DefOperator('__repr__','representation of SymMatrix in Python',
                     returnType = 'str')


#+++++++++
symbolicModule.DefFinishTable()#only finalize latex table
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++

symbolicModule.AddDocu(r"""Standard Matrix operators are available for `SymMatrix`, see the following examples:""")

symbolicModule.AddDocuNotebook('python/Notebooks/reference/symbolic.ipynb', part='matrixOperators')


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#currently, only latex + RST binding:
pyClassStr = 'symbolic.VariableSet'
classStr = 'Symbolic::VariableSet'

symbolicModule.DefPyStartClass(classStr, pyClassStr, '', subSection=True)

symbolicModule.AddDocu("A container for symbolic variables, in particular for exchange between "+
            "user functions and the model. "+
            "For details, see the following example:"
            )

symbolicModule.AddDocuNotebook('python/Notebooks/reference/symbolic.ipynb', part='variableSet')

symbolicModule.DefStartTable(pyClassStr)

# 		//+++++++++++++++++++++++++++++++++++++++++++

# 		.def("__getitem__", [](Symbolic::SymbolicVariableSet& self, std::string name)
# 			{ return self.GetVariable(name); })
# 		.def("__setitem__", [](Symbolic::SymbolicVariableSet& self, std::string name, Real value)
# 			{ return self.SetVariable(name, value); })

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Add', cName='', 
                        description="Add a variable with name and value (name may not exist)",
                        argList=['name','value'],
                        argTypes=['str','float'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Add', cName='', 
                        description="Add a variable with named real (name may not exist)",
                        argList=['namedReal'],
                        argTypes=['symbolic.Real'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Set', cName='', 
                        description="Set a variable with name and value (adds new or overrides existing)",
                        argList=['name','value'],
                        argTypes=['str','float'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Get', cName='', 
                        description="Get a variable by name",
                        argList=['name'],
                        argTypes=['str'],
                        returnType='symbolic.Real',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Exists', cName='', 
                        description="Return True, if variable name exists",
                        argList=['name'],
                        argTypes=['str'],
                        returnType='bool',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='', 
                        description="Erase all variables and reset VariableSet",
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfItems', cName='', 
                        description="Return True, if variable name exists",
                        argList=['name'],
                        argTypes=['str'],
                        returnType='bool',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='GetNames', cName='', 
                        description="Get list of stored variable names",
                        returnType='List[str]',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='__setitem__', cName='', 
                        description="bracket [] operator for setting a variable to a specific value",
                        argList=['name','value'],
                        argTypes=['str','float'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='__getitem__', cName='', 
                        description="bracket [] operator for getting a specific variable by name",
                        argList=['name'],
                        argTypes=['str'],
                        returnType='symbolic.Real',
                        )


symbolicModule.DefOperator('__str__','create string of set of variables',
                       returnType = 'str')

symbolicModule.DefOperator('__repr__','representation of SymMatrix in Python',
                       returnType = 'str')


#+++++++++
symbolicModule.DefFinishTable()#only finalize latex table
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#currently, only latex + RST binding:
pyClassStr = 'symbolic.UserFunction'
classStr = 'Symbolic::PySymbolicUserFunction'

symbolicModule.DefPyStartClass(classStr, pyClassStr, '', subSection=True)

symbolicModule.AddDocu("A class for creating and handling symbolic user functions in C++. "+
            "Use these functions for high performance extensions, e.g., of existing objects or loads"+
            "For details, see the following example:"
            )

symbolicModule.AddDocuNotebook('python/Notebooks/reference/symbolic.ipynb', part='userFunction')

symbolicModule.DefStartTable(pyClassStr)

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Evaluate', cName='', 
                        description="Evaluate symbolic function with test values; requires exactly same args as Python user functions; this is slow and only intended for testing",
                        # returnType='None', #Real or Vector
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='SetUserFunctionFromDict', cName='', 
                        description="Create C++ std::function (as requested in C++ item) with symbolic user function as recorded in given dictionary, as created with ConvertFunctionToSymbolic(...).",
                        argList=['mainSystem','fcnDict','itemIndex','userFunctionName'],
                        argTypes=['MainSystem','dict','ItemIndex','str'],
                        returnType='None',
                        )

# not needed any more (use userFunction directly, same as Python function)
# symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='TransferUserFunction2Item', cName='', 
#                         description="Transfer the std::function to a given object, load or other; this needs to be done purely in C++ to avoid Pybind overheads.",
#                         argList=['mainSystem','itemIndex','userFunctionName'],
#                         argTypes=['MainSystem','ItemIndex','str'],
#                         returnType='None',
#                         )

symbolicModule.DefOperator('__repr__','Representation of Symbolic function',
                       returnType = 'str')

symbolicModule.DefOperator('__str__','Convert stored symbolic function to string',
                       returnType = 'str')


#+++++++++
symbolicModule.DefFinishTable()#only finalize latex table
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++


