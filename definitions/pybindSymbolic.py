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

pb.AddDocu(r"""The sub-module `exudyn.symbolic` (abbreviated `esym`) records mathematical expressions that are written in Python as expression graphs, which C++ evaluates. Its main use are **symbolic user functions**: a Python user function is recorded once, and from then on each call is computed in C++ without calling Python - for the representative user functions of the items 5 to 44 times faster than the Python function, at about the cost of the built-in force laws (#2982). A symbolic user function needs no Python interpreter at the time it is called, so it is drawn by the render thread as well, and it can be written out as code.

**What is recorded.** The types are `esym.Real`, `esym.Vector` and `esym.Matrix`. Each is a number (or a vector or matrix of numbers), a named variable - `esym.Real('x', 1.)`, whose value may be changed later with `SetValue` - or an expression of them. While recording is on (the default, `esym.SetRecording`), every operator and function of a symbolic value creates a node that points to its operands, so that a value used several times is one node; an operation of numbers only is computed at once. The operators and numpy's functions mean what they mean for numpy arrays, `@` for the products of vectors and matrices, `np.multiply` for the product of the components, so that one Python function computes with numbers and records with symbolic values; what numpy defines differently, and `math.sin` or `float()` of a symbolic value, raise a `TypeError` instead of recording a wrong expression (#3015, #3017). The mathematical functions are those of Python's math module, computed as in `exudyn.cMath` with the same bits on every platform (#3074). `exudyn.symbolicUtilities` has the rotation matrices and their conversions for symbolic values, [](#sec-module-symbolicutilities).

**Symbolic user functions.** `CreateSymbolicUserFunction(mbs, function, userFunctionName, itemIndex)`, [](#sec-advancedutilities-createsymbolicuserfunction), calls the Python function once with symbolic arguments named after its parameters and keeps the result as an `esym.UserFunction`, which is given to the item instead of the Python function; the object must be kept alive in a variable as long as the item uses it. Vectors and matrices as arguments and as the result, a sparse matrix result and the parameters of the user items are supported; most user functions of the items have a symbolic version, and the user bodies and nodes, `ObjectUserBody` and `NodeUser`, are written with them, see [](#sec-useritems).

**How a recording is evaluated.** After the recording, the graph is sorted into a list of operations, the flat program, which computes every node once per call, in this order; nothing is kept from one call to the next. Before that, `CreateSymbolicUserFunction` applies `esym.Optimize`: an expression written twice - `esym.cos(q[2])` in two places - becomes one node, and `1*x`, `x+0` and operations of numbers are removed, with the same results bit for bit (#3085). The list is evaluated by the table of operations: a scalar operation of numbers - `+`, `*`, `sin`, `pow`, a comparison, `IfThenElse` - is computed by its row of the table, without a call of its node, and a vector or matrix operation by its node; `esym.SetTableEvaluation(False)` evaluates every operation by its node instead, with the same results bit for bit (#3157). `UserFunction.GetFlatProgram()` shows the list, `flatEvaluation = False` evaluates the graph as a tree instead, and `UserFunction.Evaluate(...)` calls the function from Python, to compare it with the Python function. `Diff` gives the derivative of an expression with respect to a named variable at the present values, a number, not an expression.

**A recording as code.** `GetFlatProgram(format=...)` of a user function or of expressions and `ToString(format=...)` of an expression write it as Python of numpy, of `exudyn.symbolic` or of sympy, which computes what Exudyn computes; sympy writes C, C++, Fortran, Matlab or LaTeX from it and computes derivatives as expressions (#3087, #3088), see [](#sec-module-misc.symbolicexport).

**What to know.**

- A Python `if`, `and`, `or` or `not` on a symbolic value raises a `TypeError` while recording, as it would take one branch for every call (#3096): a decision of each call is written with `esym.IfThenElse(condition, a, b)`, which computes both branches, a decision at recording with `x.Evaluate()`. An `if` on a Python value - a setting of the script - is decided at recording, as any Python code of the function. A loop is recorded as often as it runs, so its count cannot depend on a symbolic value.
- Only `exudyn.symbolic`, its types and numpy's functions of them are recorded: a call into another library is a constant of the recording. The outputs of items - `mbs.GetNodeOutput`, `GetObjectOutput`, `GetObjectOutputBody`, `GetMarkerOutput`, `GetSensorValues` - are read by C++ at every call; every other attribute of the `mbs` raises while recording (#2984), [](#sec-symbolicuserfunctions).
- With `simulationSettings.parallel.numberOfThreads` > 1, an item whose user functions are all symbolic is computed in the threads of the parallel loops, as the items without user functions: each thread evaluates its own copy of the recording, with the same results. An item with a Python user function, and an item that reads it - a connector on its marker, a body on its node -, is computed in the main thread (#2978).
- A recording is linear in its size at any depth - a sum of many terms in a loop, a chain of 200000 operations -: the value of a new expression is computed from the values its operands had when they were recorded, and recording, deleting, `str()`, the flat program and Optimize do not recurse (#3090). `Evaluate()` and `Diff()` of an expression evaluate the tree, which recurses: deeper than 10000 operations (`esym.GetDepth`) they raise a `ValueError`, and so does `flatEvaluation = False`; a symbolic user function of any depth is evaluated by its flat program. The recordings of the test models are at most 44 deep.
- **Always verify a symbolic user function** against its Python function, with `UserFunction.Evaluate` or a short simulation of both.
""",
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
                        description="Set either internal float value or value of named expression; cannot change symbolic expressions. Raises while CreateSymbolicUserFunction records a function, as the recording would not follow the new value (#3090); SetVector, SetMatrix and a component of a named vector or matrix do the same.",
                        example = r"""b = esym.Real(13)\\b.SetValue(14) #now b is 14\\#b.SetValue(a+3.) #not possible!""",
                        argList=['valueInit'],
                        argTypes=['float'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='ToString', cName='',
                        description="the expression as text: format='Python' (default) the text of str(), with the functions by their bare names; 'symbolic', 'numpy' or 'sympy' an expression of that module, which Python can compute, written by exudyn.misc.symbolicExport (#3088)",
                        example = r"""x=esym.Real('x',2)\\f=esym.cos(x)*x\\f.ToString(format='numpy') #'(np.cos(x) * x)'""",
                        argList=['format'],
                        defaultArgs=["'Python'"],
                        argTypes=['str'],
                        returnType='str',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Evaluate', cName='', 
                        description="return evaluated expression (prioritized) or stored Real value.",
                        returnType='float',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Diff', cName='', 
                        description="return the derivative of the stored expression with respect to the given named variable, at the current values; the variable may be changed hereafter with SetValue(...) only; Diff(vectorVariable, component) differentiates by a component of a named symbolic.Vector (#3115). round, ceil, floor, mod, min, max, isfinite, Not and the comparisons have no derivative and raise an error",
                        example = r"""x=esym.Real('x',2)\\f=3*x+x**2*esym.sin(x)\\f.Diff(x) #evaluate derivative w.r.t. x""",
                        returnType='float',
                        argList=['var'],
                        argTypes=['symbolic.Real'],
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='GetStatistics', cName='',
                        description="the counts of the nodes of the expression, as esym.GetStatistics (#3142)",
                        argList=['detailed'],
                        defaultArgs=['False'],
                        argTypes=['bool'],
                        returnType='dict',
                        )

symbolicModule.DefDataAccess('value','access to internal float value, which is used in case that symbolic.Real has been built from a float (but without a name and without symbolic expression)',
                       dataType = 'float')

symbolicModule.DefOperator('__float__','conversion of symbolic.Real to Python float; while recording, an expression or a named variable raises a TypeError, as math.sin(x) or float(x) would record a constant - use Evaluate() for the value (#3017)',
                       returnType = 'float')

symbolicModule.DefOperator('__bool__','symbolic.Real as a condition (if, and, or, not), its value not zero; while recording, an expression or a named variable raises a TypeError, as one branch would be taken for every call - use esym.IfThenElse(condition, a, b), or Evaluate() for a decision with the value at recording (#3096)',
                       returnType = 'bool')

symbolicModule.DefOperator('__str__','conversion of symbolic.Real to string',
                       returnType = 'str')

symbolicModule.DefOperator('__repr__','representation of symbolic.Real in Python',
                       returnType = 'str')

symbolicModule.DefFinishTable()#only finalize latex table


symbolicModule.AddDocu(r"""The remaining operators and mathematical functions are recorded within expressions. Main mathematical operators for `SymReal` exist, similar to Python, such as:""")

symbolicModule.AddDocuNotebook('python/Notebooks/reference/symbolic.ipynb', part='realOperators')

symbolicModule.AddDocu(r"""Mathematical functions may be called with a `SymReal` or with a `float` and return a `SymReal`. Their names, arguments and results are those of Python's math module - so that `math.sin` and `esym.sin` can be exchanged, e.g. as `functions=esym` - and the functions with one argument `x` are `isfinite`, `abs`, `round`, `ceil`, `floor`, `sqrt`, `exp`, `log`, `sin`, `cos`, `tan`, `asin`, `acos`, `atan`, `sinh`, `cosh`, `tanh`, `asinh`, `acosh` and `atanh`; the others are in the table below. They are computed

- `exp`, `log`, `pow`, `sin`, `cos`, `tan`, `asin`, `acos`, `atan`, `atan2`, `sinh`, `cosh`, `tanh`: by the portable implementation compiled into Exudyn (from musl libc), the functions of `exudyn.cMath`, which give the same bits on every platform, see [](#sec-exudyn-cmath) - not by the C++ library of the platform;
- `sqrt`, `abs`, `round`, `ceil`, `floor`, `mod`, `min`, `max`, `isfinite`: by the C++ library, exactly, so the same on every platform;
- `asinh`, `acosh`, `atanh`: by the C++ library of the platform, whose results may differ in the last bit between platforms.""")

symbolicModule.StubCode('\n#functions directly in symbolic module:\n')

pyClassStr = 'symbolic'
classStr = ''
#these functions are in symbolic.Real; documented by the text above, the stubs only (#3074)
fnList=['isfinite','abs',#'sign',
        'round','ceil','floor',
        'sqrt','exp','log',
        'sin','cos','tan','asin','acos','atan',
        'sinh','cosh','tanh','asinh','acosh','atanh',
                ]

portableFunctions = ['exp', 'log', 'sin', 'cos', 'tan', 'asin', 'acos', 'atan', 'sinh', 'cosh', 'tanh']
platformFunctions = ['asinh', 'acosh', 'atanh']
for fnName in fnList:
    if fnName in portableFunctions:
        computed = "computed as exudyn.cMath." + fnName + ", the same bits on every platform"
    elif fnName in platformFunctions:
        computed = "computed by the C++ library of the platform"
    else:
        computed = "computed exactly by the C++ library"
    #empty class indicates that these are functions in symbolic:
    symbolicModule.DefPyFunctionAccess(cClass='', pyName=fnName, cName='', 
                            description=fnName + "(x) as in Python's math module, recorded; " + computed,
                            argList=['x'],
                            argTypes=['symbolic.Real'],
                            returnType='symbolic.Real',
                            addDocu=False,
                            )

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++

symbolicModule.AddDocu(r"""The following table lists the functions with two or more arguments, the special functions and the module data: """)

symbolicModule.DefStartTable(pyClassStr)

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='sign', cName='', 
                        description="returns 0 for x=0, -1 for x<0 and 1 for x>0.",
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

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='atan2', cName='', 
                        description=r"""return the angle of the point (x, y) in $[-\pi, \pi]$, as math.atan2; y is the first argument.""",
                        argList=['y','x'],
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
                        description="Symbolic function for conditional evaluation. If the condition evaluates to True, the expression ifTrue is evaluated, while otherwise expression ifFalse is evaluated; the condition and the branches may be symbolic or numbers (#3072)",
                        example = r"""x=esym.Real(-1)\\y=esym.Real('y',2)\\a=esym.IfThenElse(x<0, y+1, y-1)""",
                        argList=['condition','ifTrue','ifFalse'],
                        argTypes=['symbolic.Real','symbolic.Real','symbolic.Real'],
                        returnType='symbolic.Real',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='cross', cName='', 
                        description="the cross product of two vectors of 3 components, symbolic or one of them a list or numpy array, as numpy.cross (#3070)",
                        example = r"""a = esym.Vector([1,0,0])\\b = esym.Vector([0,1,0])\\c = esym.cross(a, b) #gives [0,0,1]""",
                        argList=['a','b'],
                        argTypes=['symbolic.Vector','symbolic.Vector'],
                        returnType='symbolic.Vector',
                        )

symbolicModule.DefDataAccess('pi', 'the float closest to pi, the same as math.pi, so that a function written with functions.pi works with exudyn.symbolic (#3071)',
                       dataType = 'float', isTopLevel=True)

#+++++++++++++++++++++++
symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='SetRecording', cName='', 
                        description="Set current (global / module-wide) status of expression recording. By default, recording is on.",
                        example = "esym.SetRecording(True)",
                        argList=['flag'],
                        argTypes=['bool'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Optimize', cName='',
                        description="new expressions, built from one esym.Real, Vector or Matrix or a list of them, which share with them what does not change: mergeDuplicates=True makes equal subexpressions and leaves one node - the same class and the same operands, a number with the same bits, the same named variable - so that a value written twice is computed once; level=1 applies the rules that keep the result bit for bit (1*x, x/1, x+0, x**1, x**0 -> 1, -(-x), operations of numbers, IfThenElse of a number, a component of a vector or matrix of components), level=2 also 0*x, 0/x, x-x, x/x (which lose a nan or inf of x) and the combination of numbers, 2*(3*x) -> 6*x (which changes the rounding); level=0 merges only. CreateSymbolicUserFunction optimizes with level=1 by default (#3085)",
                        example = r"""x=esym.Real('x',2)\\e=esym.Optimize(1*x + 0 + esym.sin(x)*esym.sin(x)) #(x + (sin(x) * sin(x))), sin(x) one node""",
                        argList=['expressions', 'level', 'mergeDuplicates'],
                        defaultArgs=['', '1', 'True'],
                        argTypes=['Any', 'int', 'bool'],
                        returnType='Any',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='GetFlatProgramData', cName='',
                        description="the flat program of expressions as a Python structure, a dict of arguments, operations and results: entry k of operations is node k of the recording, the tK of GetFlatProgram(), its operands given by their numbers, which are smaller than k; arguments are the named variables, in the order given or of their first appearance, results named by resultNames, else result or result[i] (#3087)",
                        example = r"""x=esym.Real('x',2)\\y=esym.Real('y',3)\\data=esym.GetFlatProgramData([x**2+y, esym.cos(x)*y])""",
                        argList=['expressions', 'arguments', 'resultNames'],
                        defaultArgs=['', 'None', 'None'],
                        argTypes=['Any', 'Any', 'Any'],
                        returnType='dict',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='GetFlatProgram', cName='',
                        description="the flat program of one expression or of a list of them as text; format 'Python': one line tK = ... per operation, each node computed once, then the results, as UserFunction.GetFlatProgram(); 'symbolic', 'numpy', 'sympy': a Python function name(arguments) of that module, written by exudyn.misc.symbolicExport, which computes what Exudyn computes (#3088); flatten=False: each result as one expression; arguments and resultNames as in GetFlatProgramData (#3087)",
                        example = r"""x=esym.Real('x',2)\\y=esym.Real('y',3)\\print(esym.GetFlatProgram([x**2+y, esym.cos(x)*y], format='numpy', name='F'))""",
                        argList=['expressions', 'arguments', 'resultNames', 'format', 'flatten', 'name'],
                        defaultArgs=['', 'None', 'None', "'Python'", 'True', "'F'"],
                        argTypes=['Any', 'Any', 'Any', 'str', 'bool', 'str'],
                        returnType='str',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='GetDepth', cName='',
                        description="the depth of one esym.Real, Vector or Matrix, or of each of a list of them: the number of operations on the longest path to a leaf, 0 for a number or a named variable; Evaluate() and Diff() of an expression deeper than 10000 raise, as they recurse (#3090)",
                        example = r"""x=esym.Real('x',2)\\esym.GetDepth([x, esym.sin(x*x + 1)]) #[0, 3]""",
                        argList=['expressions'],
                        argTypes=['Any'],
                        returnType='Any',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Function', cName='',
                        description="a recording of one esym.Real, Vector or Matrix in declared inputs, without an item (#3149): inputs is a list of named variables or a VariableSet, whose variables are the inputs in their order; the result is an esym.UserFunction whose Evaluate(*inputs), EvaluateJacobian(inputName, *inputs), GetFlatProgram and GetStatistics work as for a recorded user function, without MainSystem; the expressions may read variables of esym.variables, every other named variable must be an input",
                        example = r"""scope = esym.VariableSet()\\q = scope.Add('q', [1., 2.])\\t = scope.Add('t', 0.)\\F = esym.Function(esym.sin(t)*q, scope, 'F')\\F.Evaluate([1., 1.], 0.5)""",
                        argList=['result', 'inputs', 'name'],
                        defaultArgs=['', '', "'Function'"],
                        argTypes=['Any', 'Any', 'str'],
                        returnType='Any',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='SetTableEvaluation', cName='',
                        description="True (default): every symbolic user function is evaluated and differentiated by the table of operations, SymbolicProgram, which computes the scalar operations of the flat program without calling their nodes; False: by the flat program of the nodes, the fallback. Both give the same results to the bit (#3157)",
                        example = r"""esym.SetTableEvaluation(False)""",
                        argList=['flag'],
                        argTypes=['bool'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='GetTableEvaluation', cName='',
                        description="True, if the symbolic user functions are evaluated by the table of operations, see SetTableEvaluation (#3157)",
                        returnType='bool',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='GetStatistics', cName='',
                        description="the counts of the nodes of one esym.Real, Vector or Matrix, or of a list of them together, by one traversal of the graph, a dict: nodes (the distinct nodes of the graph), operations, leaves (numbers, variables, item outputs), shared (the nodes used by more than one operation), depth (as GetDepth), treeSize (the nodes counted once per path, as str() of a tree and the tree Diff walk them; a float), flatOperations (the operations the flat program writes, as GetFlatProgram() counts them), tableOperations (those of them computed by the table of operations, the scalar operations of scalar operands, #3160); detailed=True also byOperation (the count per operation of the flat program, '+', 'sin', 'number', 'variable', 'itemOutput', ...) and byKind (per Real, Vector and Matrix) (#3142)",
                        example = r"""x=esym.Real('x',2)\\y=esym.sin(x)*x\\esym.GetStatistics([y, y+x], detailed=True)""",
                        argList=['expressions', 'detailed'],
                        defaultArgs=['', 'False'],
                        argTypes=['Any', 'bool'],
                        returnType='dict',
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

symbolicModule.AddDocu(r"""To create a symbolic Vector, use `aa=symbolic.Vector([3,4.2,5]` to build a Python object aa with values [3,4.2,5]. In order to use a named vector, use `v=symbolic.Vector('myVec',[3,4.2,5])`. Vectors can be also created from mixed symbolic expressions and numbers, such as `v=symbolic.Vector([x,x**2,3.14])`, however, this cannot become a named vector as it contains expressions. The operators and functions mean what they mean for numpy arrays, so that one user function computes with numpy arrays and records with symbolic vectors: `a @ b` and `a.dot(b)` give the scalar product, `a.multiply(b)` or `np.multiply(a, b)` the product of the components, `a / s` the division by a scalar; numpy's functions (`np.multiply`, `np.dot`, `np.sin`, ...) and a numpy array plus, times or @ a symbolic vector record the expression (#3015). What numpy defines differently is refused with a TypeError: `a * b` of two vectors (numpy multiplies the components; `exudyn.special.deprecations.symbolicStarProduct` gives the scalar product for the transition, #3016) and `a == b` (numpy compares the components). A symbolic vector is a sequence, with `len(a)`, `a[-1]` and `sum(a)`; a vector argument of run-time size that CreateSymbolicUserFunction cannot size has no size while it is recorded, so that `len()` and loops over it raise a TypeError. Note thatin the following, we use the abbreviation `SymVector=exudyn.symbolic.Vector`, and the examples `esym=exudyn.symbolic`. Note that only functions are able to be recorded. Member functions of `SymVector` are:""")

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

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='ToString', cName='',
                        description="the expression as text: format='Python' (default) the text of str(), with the functions by their bare names; 'symbolic', 'numpy' or 'sympy' an expression of that module, which Python can compute, written by exudyn.misc.symbolicExport (#3088)",
                        example = r"""x=esym.Real('x',2)\\f=esym.cos(x)*x\\f.ToString(format='numpy') #'(np.cos(x) * x)'""",
                        argList=['format'],
                        defaultArgs=["'Python'"],
                        argTypes=['str'],
                        returnType='str',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Evaluate', cName='', 
                        description="Return evaluated expression (prioritized) or stored vector value. (not recorded)",
                        returnType='List[float]',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Diff', cName='', 
                        description="the derivative of the vector expression by a named symbolic.Real, or by a component of a named symbolic.Vector with Diff(vectorVariable, component), at the current values, as numpy array (#3115); a comparison and an output of an item have no derivative and raise",
                        example = r"""q=esym.Vector('q',[1,2,3])\\v=esym.sin(q[0])*q\\v.Diff(q, 0) #derivative by q[0]""",
                        returnType='NDArray',
                        argList=['var'],
                        argTypes=['symbolic.Real'],
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='GetStatistics', cName='',
                        description="the counts of the nodes of the expression, as esym.GetStatistics (#3142)",
                        argList=['detailed'],
                        defaultArgs=['False'],
                        argTypes=['bool'],
                        returnType='dict',
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
                        description="Perform component-wise multiplication of vector times other vector and return result; the same as multiply.",
                        example = r"""v1 = esym.Vector([1,2,4])\\v2 = esym.Vector([1,0.5,0.25])\\v3 = v1.MultComponents(v2)""",
                        argList=['other'],
                        argTypes=['symbolic.Vector'],
                        returnType='symbolic.Real',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='multiply', cName='',
                        description="the product of the components with another vector, symbolic or a list or numpy array, as numpy's multiply and the operator * of numpy arrays (#3015)",
                        example = r"""v1 = esym.Vector([1,2,4])\v2 = esym.Vector([1,0.5,0.25])\v3 = v1.multiply(v2) #gives [1,1,1]""",
                        argList=['other'],
                        argTypes=['symbolic.Vector'],
                        returnType='symbolic.Vector',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='dot', cName='',
                        description="the scalar product with another vector, symbolic or a list or numpy array, as numpy's dot (#3011)",
                        example = r"""v1 = esym.Vector([1,2,4])\v2 = esym.Vector([1,0.5,0.25])\s = v1.dot(v2) #gives 3.""",
                        argList=['other'],
                        argTypes=['symbolic.Vector'],
                        returnType='symbolic.Real',
                        )

symbolicModule.DefOperator(name='__matmul__', 
                     description="the operator @: with a vector the scalar product, with a matrix the product vector times matrix, as for numpy arrays (#3011)",
                     argList=['other'],
                     argTypes=['symbolic.Vector'],
                     returnType='symbolic.Real',
                     )

symbolicModule.DefOperator(name='__rmatmul__', 
                     description="the operator @ with a numpy array on the left: a constant matrix times this vector, or the scalar product with a constant vector (#3011)",
                     argList=['other'],
                     argTypes=['numpy.ndarray'],
                     returnType='symbolic.Vector',
                     )

symbolicModule.DefOperator(name='__mul__',
                     description="the operator *: with a scalar the vector times the scalar; of two vectors a TypeError, as numpy multiplies their components - use @, dot() or multiply() (#3015)",
                     argList=['other'],
                     argTypes=['symbolic.Real'],
                     returnType='symbolic.Vector',
                     )

symbolicModule.DefOperator(name='__add__',
                     description="the operator +: with a vector, a list or a numpy array the sum, with a scalar the scalar added to each component, as numpy (#3018); - alike",
                     argList=['other'],
                     argTypes=['symbolic.Vector'],
                     returnType='symbolic.Vector',
                     )

symbolicModule.DefOperator(name='__truediv__',
                     description="the operator /: the vector divided by a scalar, a float or a symbolic.Real (#3015)",
                     argList=['other'],
                     argTypes=['symbolic.Real'],
                     returnType='symbolic.Vector',
                     )

symbolicModule.DefOperator(name='__getitem__',
                     description="bracket [] operator to return (symbolic) component of vector, allowing read-access; a negative index counts from the end and an index outside raises IndexError, as for lists (#3015); a slice v[a:b] gives a vector of the components (#3018). Index may also evaluate from an expression.",
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

symbolicModule.AddDocu(r"""To create a symbolic Matrix, use `aa=symbolic.Matrix([[3,4.2],[3.3,1.2]]` to build a Python object aa. In order to use a named matrix, use `v=symbolic.Matrix('myMat',[3,4.2,5])`. Matrixs can be also created from mixed symbolic expressions and numbers, such as `v=symbolic.Matrix([x,x**2,3.14])`, however, this cannot become a named matrix as it contains expressions. The products are those of numpy: `A @ B`, `A @ v`, `v @ A` and `A.dot(B)` are the matrix products, and `A * B`, `A * v` and `v * A` raise a TypeError, as numpy multiplies the components (#3015). As the result of a symbolic user function, a matrix built from components keeps only the components that are not the constant 0, which go into the sparse system matrix (#3022). Note thatin the following, we use the abbreviation `SymMatrix=exudyn.symbolic.Matrix`, and the examples `esym=exudyn.symbolic`. Member functions of `SymMatrix` are:""")

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

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='ToString', cName='',
                        description="the expression as text: format='Python' (default) the text of str(), with the functions by their bare names; 'symbolic', 'numpy' or 'sympy' an expression of that module, which Python can compute, written by exudyn.misc.symbolicExport (#3088)",
                        example = r"""x=esym.Real('x',2)\\f=esym.cos(x)*x\\f.ToString(format='numpy') #'(np.cos(x) * x)'""",
                        argList=['format'],
                        defaultArgs=["'Python'"],
                        argTypes=['str'],
                        returnType='str',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Evaluate', cName='', 
                        description="Return evaluated expression (prioritized) or stored Matrix value. (not recorded)",
                        returnType='List[float]',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Diff', cName='', 
                        description="the derivative of the matrix expression by a named symbolic.Real, or by a component of a named symbolic.Vector with Diff(vectorVariable, component), at the current values, as numpy array (#3115)",
                        returnType='NDArray',
                        argList=['var'],
                        argTypes=['symbolic.Real'],
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='GetStatistics', cName='',
                        description="the counts of the nodes of the expression, as esym.GetStatistics (#3142)",
                        argList=['detailed'],
                        defaultArgs=['False'],
                        argTypes=['bool'],
                        returnType='dict',
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

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='dot', cName='', 
                        description="the product with a vector or a matrix, as numpy's dot (#3011)",
                        example = r"""m = esym.Matrix([[1,2],[3,4]])\v = esym.Vector([1,0])\w = m.dot(v) #gives [1,3]""",
                        argList=['other'],
                        argTypes=['symbolic.Vector'],
                        returnType='symbolic.Vector',
                        )

symbolicModule.DefOperator(name='__matmul__', 
                     description="the operator @: the product with a vector or a matrix, as for numpy arrays (#3011)",
                     argList=['other'],
                     argTypes=['symbolic.Vector'],
                     returnType='symbolic.Vector',
                     )

symbolicModule.DefOperator(name='__rmatmul__', 
                     description="the operator @ with a numpy array on the left: a constant matrix times this matrix, or a constant vector times it (#3011)",
                     argList=['other'],
                     argTypes=['numpy.ndarray'],
                     returnType='symbolic.Matrix',
                     )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='transpose', cName='',
                        description="the transposed matrix, recorded; the property T gives the same, as for numpy arrays (#3018)",
                        example = r"""m = esym.Matrix([[1,2],[3,4]])\mT = m.T #gives [[1,3],[2,4]]""",
                        returnType='symbolic.Matrix',
                        )

symbolicModule.DefOperator(name='__getitem__',
                     description="bracket [] operator for (symbolic) component of Matrix (read-access), A[i, j]; A[i] gives row i as symbolic.Vector, as numpy (#3018). Row and column may also evaluate from an expression.",
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
            "user functions and the model, and the scope of the names of its variables: named Reals, Vectors and Matrices, "+
            "a name unique in its set, in the order they were added - the inputs of esym.Function (#3149). "+
            "exudyn.symbolic.variables is the set that the recordings read at every call. "+
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
                        description="Add a variable with name and value, which may not exist, and return it: a number gives a named symbolic.Real, a list or 1D array a named symbolic.Vector, a 2D array a named symbolic.Matrix (#3149)",
                        example = r"""scope = esym.VariableSet()\\q = scope.Add('q', [0., 1.])\\t = scope.Add('t', 0.)""",
                        argList=['name','value'],
                        argTypes=['str','Any'],
                        returnType='Any',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Add', cName='',
                        description="Add a named symbolic.Real, Vector or Matrix under its name, which may not exist, and return it",
                        argList=['namedVariable'],
                        argTypes=['Any'],
                        returnType='Any',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Set', cName='',
                        description="Set a variable with name and value - a number for a Real, a list or array for a Vector or Matrix -; adds new or overrides existing",
                        argList=['name','value'],
                        argTypes=['str','Any'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='Get', cName='',
                        description="Get a variable by name: a named symbolic.Real, Vector or Matrix",
                        argList=['name'],
                        argTypes=['str'],
                        returnType='Any',
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
                        description="Return the number of variables",
                        returnType='int',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='GetNames', cName='',
                        description="Get list of stored variable names, in the order they were added",
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

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='EvaluateJacobian', cName='', 
                        description="the Jacobian of the result by the argument named argumentName - e.g. 'qTotal' -, at the arguments given after it as for Evaluate, by Diff of the recording, one column per component of the argument: a vector for a scalar result, a matrix (result x argument) for a vector result, an array (rows x columns x argument) for a matrix result; slow, as Evaluate (#3115)",
                        argList=['argumentName'],
                        argTypes=['str'],
                        returnType='NDArray',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='SetUserFunctionFromDict', cName='', 
                        description="Create C++ std::function (as requested in C++ item) with symbolic user function as recorded in given dictionary, as created with ConvertFunctionToSymbolic(...).",
                        argList=['mainSystem','fcnDict','itemIndex','userFunctionName'],
                        argTypes=['MainSystem','dict','ItemIndex','str'],
                        returnType='None',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='GetFlatProgram', cName='',
                        description="the list of operations by which the result is evaluated, as text: one operation tK = ... per line, then the result; each number, vector or matrix that the recorded function uses several times is computed once per call (#3053, #3054); format 'symbolic', 'numpy' or 'sympy': a Python function with the arguments of the user function, written by exudyn.misc.symbolicExport, the variables of exudyn.symbolic.variables read from there (symbolic) or as keyword arguments (#3088); flatten=False: the result as one expression (#3087)",
                        argList=['format', 'flatten'],
                        defaultArgs=["'Python'", 'True'],
                        argTypes=['str', 'bool'],
                        returnType='str',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='GetFlatProgramData', cName='',
                        description="the flat program as a Python structure, a dict: name, arguments (of the Python signature, with type and size), variables (of exudyn.symbolic.variables that the recording reads), operations (entry k is node k, its operands by number) and results; exudyn.misc.symbolicExport writes it as code (#3087)",
                        returnType='dict',
                        )

symbolicModule.DefPyFunctionAccess(cClass=classStr, pyName='GetStatistics', cName='',
                        description="the counts of the nodes of the recorded result, as esym.GetStatistics (#3142)",
                        argList=['detailed'],
                        defaultArgs=['False'],
                        argTypes=['bool'],
                        returnType='dict',
                        )

symbolicModule.DefDataAccess('flatEvaluation','True (default): the result is evaluated by the list of operations of GetFlatProgram(); False: as a tree, with the same results, for comparison (#3053, #3054)',
                       dataType = 'bool')

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


