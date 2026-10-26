
#stub information for class Real functions
class Real:
    @overload
    def __init__(self, value: float) -> None: 
        """Construct Real from float."""
        ...
    @overload
    def __init__(self, name: str, value: float) -> None: 
        """Construct named Real from name and float."""
        ...
    @overload
    def SetValue(self, valueInit: float) -> None: 
        """Set either internal float value or value of named expression; cannot change symbolic expressions.
        
        Raises while CreateSymbolicUserFunction records a function, as the recording would not follow the new value (#3090); SetVector, SetMatrix and a component of a named vector or matrix do the same.
        
        Examples:
            b = esym.Real(13)
            b.SetValue(14) #now b is 14
            #b.SetValue(a+3.) #not possible!
        """
        ...
    @overload
    def ToString(self, format: str='Python') -> str: 
        """The expression as text: format='Python' (default) the text of str(), with the functions by their bare names; 'symbolic', 'numpy' or 'sympy' an expression of that module, which Python can compute, written by exudyn.misc.symbolicExport (#3088).
        
        Examples:
            x=esym.Real('x',2)
            f=esym.cos(x)*x
            f.ToString(format='numpy') #'(np.cos(x) * x)'
        """
        ...
    @overload
    def Evaluate(self) -> float: 
        """Return evaluated expression (prioritized) or stored Real value."""
        ...
    @overload
    def Diff(self, var: Real) -> float: 
        """Return the derivative of the stored expression with respect to the given named variable, at the current values; the variable may be changed hereafter with SetValue(...) only; Diff(vectorVariable, component) differentiates by a component of a named Vector (#3115).
        
        round, ceil, floor, mod, min, max, isfinite, Not and the comparisons have no derivative and raise an error
        
        Examples:
            x=esym.Real('x',2)
            f=3*x+x**2*esym.sin(x)
            f.Diff(x) #evaluate derivative w.r.t. x
        """
        ...
    value:float
    """access to internal float value, which is used in case that Real has been built from a float (but without a name and without symbolic expression)."""
    @overload
    def __float__(self) -> float: ...
    @overload
    def __bool__(self) -> bool: ...
    @overload
    def __str__(self) -> str: ...
    @overload
    def __repr__(self) -> str: ...

#functions directly in symbolic module:
@overload
def isfinite(x: Real) -> Real: 
    """Isfinite(x) as in Python's math module, recorded; computed exactly by the C++ library."""
    ...
@overload
def abs(x: Real) -> Real: 
    """Abs(x) as in Python's math module, recorded; computed exactly by the C++ library."""
    ...
@overload
def round(x: Real) -> Real: 
    """Round(x) as in Python's math module, recorded; computed exactly by the C++ library."""
    ...
@overload
def ceil(x: Real) -> Real: 
    """Ceil(x) as in Python's math module, recorded; computed exactly by the C++ library."""
    ...
@overload
def floor(x: Real) -> Real: 
    """Floor(x) as in Python's math module, recorded; computed exactly by the C++ library."""
    ...
@overload
def sqrt(x: Real) -> Real: 
    """Sqrt(x) as in Python's math module, recorded; computed exactly by the C++ library."""
    ...
@overload
def exp(x: Real) -> Real: 
    """Exp(x) as in Python's math module, recorded; computed as exudyn.cMath.exp, the same bits on every platform."""
    ...
@overload
def log(x: Real) -> Real: 
    """Log(x) as in Python's math module, recorded; computed as exudyn.cMath.log, the same bits on every platform."""
    ...
@overload
def sin(x: Real) -> Real: 
    """Sin(x) as in Python's math module, recorded; computed as exudyn.cMath.sin, the same bits on every platform."""
    ...
@overload
def cos(x: Real) -> Real: 
    """Cos(x) as in Python's math module, recorded; computed as exudyn.cMath.cos, the same bits on every platform."""
    ...
@overload
def tan(x: Real) -> Real: 
    """Tan(x) as in Python's math module, recorded; computed as exudyn.cMath.tan, the same bits on every platform."""
    ...
@overload
def asin(x: Real) -> Real: 
    """Asin(x) as in Python's math module, recorded; computed as exudyn.cMath.asin, the same bits on every platform."""
    ...
@overload
def acos(x: Real) -> Real: 
    """Acos(x) as in Python's math module, recorded; computed as exudyn.cMath.acos, the same bits on every platform."""
    ...
@overload
def atan(x: Real) -> Real: 
    """Atan(x) as in Python's math module, recorded; computed as exudyn.cMath.atan, the same bits on every platform."""
    ...
@overload
def sinh(x: Real) -> Real: 
    """Sinh(x) as in Python's math module, recorded; computed as exudyn.cMath.sinh, the same bits on every platform."""
    ...
@overload
def cosh(x: Real) -> Real: 
    """Cosh(x) as in Python's math module, recorded; computed as exudyn.cMath.cosh, the same bits on every platform."""
    ...
@overload
def tanh(x: Real) -> Real: 
    """Tanh(x) as in Python's math module, recorded; computed as exudyn.cMath.tanh, the same bits on every platform."""
    ...
@overload
def asinh(x: Real) -> Real: 
    """Asinh(x) as in Python's math module, recorded; computed by the C++ library of the platform."""
    ...
@overload
def acosh(x: Real) -> Real: 
    """Acosh(x) as in Python's math module, recorded; computed by the C++ library of the platform."""
    ...
@overload
def atanh(x: Real) -> Real: 
    """Atanh(x) as in Python's math module, recorded; computed by the C++ library of the platform."""
    ...
@overload
def sign(x: Real) -> Real: 
    """Returns 0 for x=0, -1 for x<0 and 1 for x>0."""
    ...
@overload
def Not(x: Real) -> Real: 
    """Returns logical not of expression, equal to Python's 'not'.
    
    Not(True)=False, Not(0.)=True, Not(-0.1)=False
    """
    ...
@overload
def min(x: Real, y: Real) -> Real: 
    """Return minimum of x and y."""
    ...
@overload
def max(x: Real, y: Real) -> Real: 
    """Return maximum of x and y."""
    ...
@overload
def mod(x: Real, y: Real) -> Real: 
    """Return floating-point remainder of the division operation x / y.
    
    For example, mod(5.1, 3) gives 2.1 as a remainder.
    """
    ...
@overload
def atan2(y: Real, x: Real) -> Real: 
    r"""Return the angle of the point (x, y) in $[-\pi, \pi]$, as math.atan2; y is the first argument."""
    ...
@overload
def pow(x: Real, y: Real) -> Real: 
    """Return $x^y$."""
    ...
@overload
def IfThenElse(condition: Real, ifTrue: Real, ifFalse: Real) -> Real: 
    """Symbolic function for conditional evaluation.
    
    If the condition evaluates to True, the expression ifTrue is evaluated, while otherwise expression ifFalse is evaluated; the condition and the branches may be symbolic or numbers (#3072)
    
    Examples:
        x=esym.Real(-1)
        y=esym.Real('y',2)
        a=esym.IfThenElse(x<0, y+1, y-1)
    """
    ...
@overload
def cross(a: Vector, b: Vector) -> Vector: 
    """The cross product of two vectors of 3 components, symbolic or one of them a list or numpy array, as numpy.cross (#3070).
    
    Examples:
        a = esym.Vector([1,0,0])
        b = esym.Vector([0,1,0])
        c = esym.cross(a, b) #gives [0,0,1]
    """
    ...
pi:float
"""the float closest to pi, the same as math.pi, so that a function written with functions.pi works with exudyn.symbolic (#3071)."""
@overload
def SetRecording(flag: bool) -> None: 
    """Set current (global / module-wide) status of expression recording.
    
    By default, recording is on.
    
    Examples:
        esym.SetRecording(True)
    """
    ...
@overload
def Optimize(expressions: Any, level: int=1, mergeDuplicates: bool=True) -> Any: 
    """New expressions, built from one esym.Real, Vector or Matrix or a list of them, which share with them what does not change: mergeDuplicates=True makes equal subexpressions and leaves one node - the same class and the same operands, a number with the same bits, the same named variable - so that a value written twice is computed once; level=1 applies the rules that keep the result bit for bit (1*x, x/1, x+0, x**1, x**0 -> 1, -(-x), operations of numbers, IfThenElse of a number, a component of a vector or matrix of components), level=2 also 0*x, 0/x, x-x, x/x (which lose a nan or inf of x) and the combination of numbers, 2*(3*x) -> 6*x (which changes the rounding); level=0 merges only.
    
    CreateSymbolicUserFunction optimizes with level=1 by default (#3085)
    
    Examples:
        x=esym.Real('x',2)
        e=esym.Optimize(1*x + 0 + esym.sin(x)*esym.sin(x)) #(x + (sin(x) * sin(x))), sin(x) one node
    """
    ...
@overload
def GetFlatProgramData(expressions: Any, arguments: Any=None, resultNames: Any=None) -> dict: 
    """The flat program of expressions as a Python structure, a dict of arguments, operations and results: entry k of operations is node k of the recording, the tK of GetFlatProgram(), its operands given by their numbers, which are smaller than k; arguments are the named variables, in the order given or of their first appearance, results named by resultNames, else result or result[i] (#3087).
    
    Examples:
        x=esym.Real('x',2)
        y=esym.Real('y',3)
        data=esym.GetFlatProgramData([x**2+y, esym.cos(x)*y])
    """
    ...
@overload
def GetFlatProgram(expressions: Any, arguments: Any=None, resultNames: Any=None, format: str='Python', flatten: bool=True, name: str='F') -> str: 
    """The flat program of one expression or of a list of them as text; format 'Python': one line tK = ...
    
    per operation, each node computed once, then the results, as UserFunction.GetFlatProgram(); 'symbolic', 'numpy', 'sympy': a Python function name(arguments) of that module, written by exudyn.misc.symbolicExport, which computes what Exudyn computes (#3088); flatten=False: each result as one expression; arguments and resultNames as in GetFlatProgramData (#3087)
    
    Examples:
        x=esym.Real('x',2)
        y=esym.Real('y',3)
        print(esym.GetFlatProgram([x**2+y, esym.cos(x)*y], format='numpy', name='F'))
    """
    ...
@overload
def GetDepth(expressions: Any) -> Any: 
    """The depth of one esym.Real, Vector or Matrix, or of each of a list of them: the number of operations on the longest path to a leaf, 0 for a number or a named variable; Evaluate() and Diff() of an expression deeper than 10000 raise, as they recurse (#3090).
    
    Examples:
        x=esym.Real('x',2)
        esym.GetDepth([x, esym.sin(x*x + 1)]) #[0, 3]
    """
    ...
@overload
def GetRecording() -> bool: 
    """Get current (global / module-wide) status of expression recording.
    
    Examples:
        esym.GetRecording()
    """
    ...


#stub information for class Vector functions
class Vector:
    @overload
    def __init__(self, vector: List[float]) -> None: 
        """Construct Vector from vector represented as numpy array or list (which may contain symbolic expressions)."""
        ...
    @overload
    def __init__(self, name: str, vector: List[float]) -> None: 
        """Construct named Vector from name and vector represented as numpy array or list (which may contain symbolic expressions)."""
        ...
    @overload
    def ToString(self, format: str='Python') -> str: 
        """The expression as text: format='Python' (default) the text of str(), with the functions by their bare names; 'symbolic', 'numpy' or 'sympy' an expression of that module, which Python can compute, written by exudyn.misc.symbolicExport (#3088).
        
        Examples:
            x=esym.Real('x',2)
            f=esym.cos(x)*x
            f.ToString(format='numpy') #'(np.cos(x) * x)'
        """
        ...
    @overload
    def Evaluate(self) -> List[float]: 
        """Return evaluated expression (prioritized) or stored vector value.
        
        (not recorded)
        """
        ...
    @overload
    def Diff(self, var: Real) -> NDArray: 
        """The derivative of the vector expression by a named Real, or by a component of a named Vector with Diff(vectorVariable, component), at the current values, as numpy array (#3115); a comparison and an output of an item have no derivative and raise.
        
        Examples:
            q=esym.Vector('q',[1,2,3])
            v=esym.sin(q[0])*q
            v.Diff(q, 0) #derivative by q[0]
        """
        ...
    @overload
    def SetVector(self, vector: Vector) -> None: 
        """Set stored vector or named vector expression to new given (non-symbolic) vector.
        
        Only works, if SymVector contains no expression. (may lead to inconsistencies in recording)
        """
        ...
    @overload
    def NumberOfItems(self) -> int: 
        """Get size of Vector (may require to evaluate expression; not recording)."""
        ...
    @overload
    def __setitem__(self, index: Real) -> Real: ...
    @overload
    def NormL2(self) -> Real: 
        """Return (symbolic) L2-norm of vector.
        
        Examples:
            v1 = esym.Vector([1,4,8])
            length = v1.NormL2() #gives 9.
        """
        ...
    @overload
    def MultComponents(self, other: Vector) -> Real: 
        """Perform component-wise multiplication of vector times other vector and return result; the same as multiply.
        
        Examples:
            v1 = esym.Vector([1,2,4])
            v2 = esym.Vector([1,0.5,0.25])
            v3 = v1.MultComponents(v2)
        """
        ...
    @overload
    def multiply(self, other: Vector) -> Vector: 
        r"""The product of the components with another vector, symbolic or a list or numpy array, as numpy's multiply and the operator * of numpy arrays (#3015).
        
        Examples:
            v1 = esym.Vector([1,2,4])\v2 = esym.Vector([1,0.5,0.25])\v3 = v1.multiply(v2) #gives [1,1,1]
        """
        ...
    @overload
    def dot(self, other: Vector) -> Real: 
        r"""The scalar product with another vector, symbolic or a list or numpy array, as numpy's dot (#3011).
        
        Examples:
            v1 = esym.Vector([1,2,4])\v2 = esym.Vector([1,0.5,0.25])\s = v1.dot(v2) #gives 3.
        """
        ...
    @overload
    def __matmul__(self, other: Vector) -> Real: ...
    @overload
    def __rmatmul__(self, other: numpy.ndarray) -> Vector: ...
    @overload
    def __mul__(self, other: Real) -> Vector: ...
    @overload
    def __add__(self, other: Vector) -> Vector: ...
    @overload
    def __truediv__(self, other: Real) -> Vector: ...
    @overload
    def __getitem__(self, index: Real) -> Real: ...
    @overload
    def __str__(self) -> str: ...
    @overload
    def __repr__(self) -> str: ...

#stub information for class Matrix functions
class Matrix:
    @overload
    def __init__(self, matrix: List[List[float]]) -> None: 
        """Construct Matrix from vector represented as numpy array or list of lists (which may contain symbolic expressions)."""
        ...
    @overload
    def __init__(self, name: str, matrix: List[List[float]]) -> None: 
        """Construct named Matrix from name and vector represented as numpy array or list of lists (which may contain symbolic expressions)."""
        ...
    @overload
    def ToString(self, format: str='Python') -> str: 
        """The expression as text: format='Python' (default) the text of str(), with the functions by their bare names; 'symbolic', 'numpy' or 'sympy' an expression of that module, which Python can compute, written by exudyn.misc.symbolicExport (#3088).
        
        Examples:
            x=esym.Real('x',2)
            f=esym.cos(x)*x
            f.ToString(format='numpy') #'(np.cos(x) * x)'
        """
        ...
    @overload
    def Evaluate(self) -> List[float]: 
        """Return evaluated expression (prioritized) or stored Matrix value.
        
        (not recorded)
        """
        ...
    @overload
    def Diff(self, var: Real) -> NDArray: 
        """The derivative of the matrix expression by a named Real, or by a component of a named Vector with Diff(vectorVariable, component), at the current values, as numpy array (#3115)."""
        ...
    @overload
    def SetMatrix(self, matrix: NDArray[Any, float]) -> None: 
        """Set stored Matrix or named Matrix expression to new given (non-symbolic) Matrix.
        
        Only works, if SymMatrix contains no expression. (may lead to inconsistencies in recording)
        """
        ...
    @overload
    def NumberOfRows(self) -> int: 
        """Get number of rows (may require to evaluate expression; not recording)."""
        ...
    @overload
    def NumberOfColumns(self) -> int: 
        """Get number of columns (may require to evaluate expression; not recording)."""
        ...
    @overload
    def __setitem__(self, row: Real, column: Real) -> Real: ...
    @overload
    def dot(self, other: Vector) -> Vector: 
        r"""The product with a vector or a matrix, as numpy's dot (#3011).
        
        Examples:
            m = esym.Matrix([[1,2],[3,4]])\v = esym.Vector([1,0])\w = m.dot(v) #gives [1,3]
        """
        ...
    @overload
    def __matmul__(self, other: Vector) -> Vector: ...
    @overload
    def __rmatmul__(self, other: numpy.ndarray) -> Matrix: ...
    @overload
    def transpose(self) -> Matrix: 
        r"""The transposed matrix, recorded; the property T gives the same, as for numpy arrays (#3018).
        
        Examples:
            m = esym.Matrix([[1,2],[3,4]])\mT = m.T #gives [[1,3],[2,4]]
        """
        ...
    @overload
    def __getitem__(self, row: Real, column: Real) -> Real: ...
    @overload
    def __str__(self) -> str: ...
    @overload
    def __repr__(self) -> str: ...

#stub information for class VariableSet functions
class VariableSet:
    @overload
    def Add(self, name: str, value: float) -> None: 
        """Add a variable with name and value (name may not exist)."""
        ...
    @overload
    def Add(self, namedReal: Real) -> None: 
        """Add a variable with named real (name may not exist)."""
        ...
    @overload
    def Set(self, name: str, value: float) -> None: 
        """Set a variable with name and value (adds new or overrides existing)."""
        ...
    @overload
    def Get(self, name: str) -> Real: 
        """Get a variable by name."""
        ...
    @overload
    def Exists(self, name: str) -> bool: 
        """Return True, if variable name exists."""
        ...
    @overload
    def Reset(self) -> None: 
        """Erase all variables and reset VariableSet."""
        ...
    @overload
    def NumberOfItems(self, name: str) -> bool: 
        """Return True, if variable name exists."""
        ...
    @overload
    def GetNames(self) -> List[str]: 
        """Get list of stored variable names."""
        ...
    @overload
    def __setitem__(self, name: str, value: float) -> None: 
        """Bracket [] operator for setting a variable to a specific value."""
        ...
    @overload
    def __getitem__(self, name: str) -> Real: 
        """Bracket [] operator for getting a specific variable by name."""
        ...
    @overload
    def __str__(self) -> str: ...
    @overload
    def __repr__(self) -> str: ...

#stub information for class UserFunction functions
class UserFunction:
    @overload
    def EvaluateJacobian(self, argumentName: str) -> NDArray: 
        """The Jacobian of the result by the argument named argumentName - e.g. 'qTotal' -, at the arguments given after it as for Evaluate, by Diff of the recording, one column per component of the argument: a vector for a scalar result, a matrix (result x argument) for a vector result, an array (rows x columns x argument) for a matrix result; slow, as Evaluate (#3115)."""
        ...
    @overload
    def SetUserFunctionFromDict(self, mainSystem: MainSystem, fcnDict: dict, itemIndex: ItemIndex, userFunctionName: str) -> None: 
        """Create C++ std::function (as requested in C++ item) with symbolic user function as recorded in given dictionary, as created with ConvertFunctionToSymbolic(...)."""
        ...
    @overload
    def GetFlatProgram(self, format: str='Python', flatten: bool=True) -> str: 
        """The list of operations by which the result is evaluated, as text: one operation tK = ...
        
        per line, then the result; each number, vector or matrix that the recorded function uses several times is computed once per call (#3053, #3054); format 'symbolic', 'numpy' or 'sympy': a Python function with the arguments of the user function, written by exudyn.misc.symbolicExport, the variables of exudyn.variables read from there (symbolic) or as keyword arguments (#3088); flatten=False: the result as one expression (#3087)
        """
        ...
    @overload
    def GetFlatProgramData(self) -> dict: 
        """The flat program as a Python structure, a dict: name, arguments (of the Python signature, with type and size), variables (of exudyn.variables that the recording reads), operations (entry k is node k, its operands by number) and results; exudyn.misc.symbolicExport writes it as code (#3087)."""
        ...
    flatEvaluation:bool
    """True (default): the result is evaluated by the list of operations of GetFlatProgram(); False: as a tree, with the same results, for comparison (#3053, #3054)."""
    @overload
    def __repr__(self) -> str: ...
    @overload
    def __str__(self) -> str: ...
