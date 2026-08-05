#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is the EXUDYN stub file initialization
#
# Author:   Johannes Gerstmayr
# Date:     2023-05-09
#
# Notes:    Under development; see https://peps.python.org/pep-0484/#stub-files
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#from typing import Dict, List, Optional

from typing import (
    Annotated, 
    Any,
    # ByteString,
    # Callable,
    # Container,
    Callable,
    Dict,
    # Generic,
    # IO,
    # Iterable,
    # Iterator,
    List,
    Literal,
    # Mapping,
    # NoReturn,
    # Optional,
    overload,
    # Sequence,
    # Sized,
    # SupportsComplex,
    # SupportsFloat,
    # SupportsInt,
    # Text,
    Tuple, #for Tuple[int, int]
    # Type,
    TypeVar,
    Union,
)
from numpy.typing import ArrayLike, NDArray
from enum import Enum
import numpy as np


#DType = TypeVar("DType", bound=np.generic)
# NPreal3D = Annotated[NDArray[float], Literal[3]]
# NPint3D = Annotated[NDArray[int], Literal[3]]

#type variables for size in numpy arrays
T1 = TypeVar("T1", bound=int)
T2 = TypeVar("T2", bound=int)

# Dimension types represented as typles
Shape = Tuple
Shape1D = Shape[T1]
Shape2D = Shape[T1, T2]
# Shape3D = Shape[T1, T2, T3]
# ShapeND = Shape[T1, ...]
# ShapeNDType = TypeVar("ShapeNDType", bound=ShapeND)

#LENGTH = Literal[2]
#NDArray[Shape2D[3,3], np.float64]


import exudyn
from exudyn import (ObjectIndex, NodeIndex, MarkerIndex, LoadIndex, SensorIndex,
                    VSettingsMaterial)





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
        
        Examples:
            b = SymReal(13)
            b.SetValue(14) #now b is 14
            #b.SetValue(a+3.) #not possible!
        """
        ...
    @overload
    def Evaluate(self) -> float: 
        """Return evaluated expression (prioritized) or stored Real value."""
        ...
    @overload
    def Diff(self, var: Real) -> float: 
        """(UNTESTED!) return derivative of stored expression with respect to given symbolic named variable; NOTE: when defining the expression of the variable which shall be differentiated, the variable may only be changed with the SetValue(...) method hereafter!.
        
        Examples:
            x=SymReal('x',2)
            f=3*x+x**2*sin(x)
            f.Diff(x) #evaluate derivative w.r.t. x
        """
        ...
    value:float
    """access to internal float value, which is used in case that Real has been built from a float (but without a name and without symbolic expression)."""
    @overload
    def __float__(self) -> float: ...
    @overload
    def __str__(self) -> str: ...
    @overload
    def __repr__(self) -> str: ...

#functions directly in symbolic module:
@overload
def isfinite(x: Real) -> Real: 
    """According to specification of C++ std::isfinite."""
    ...
@overload
def abs(x: Real) -> Real: 
    """According to specification of C++ std::fabs."""
    ...
@overload
def round(x: Real) -> Real: 
    """According to specification of C++ std::round."""
    ...
@overload
def ceil(x: Real) -> Real: 
    """According to specification of C++ std::ceil."""
    ...
@overload
def floor(x: Real) -> Real: 
    """According to specification of C++ std::floor."""
    ...
@overload
def sqrt(x: Real) -> Real: 
    """According to specification of C++ std::sqrt."""
    ...
@overload
def exp(x: Real) -> Real: 
    """According to specification of C++ std::exp."""
    ...
@overload
def log(x: Real) -> Real: 
    """According to specification of C++ std::log."""
    ...
@overload
def sin(x: Real) -> Real: 
    """According to specification of C++ std::sin."""
    ...
@overload
def cos(x: Real) -> Real: 
    """According to specification of C++ std::cos."""
    ...
@overload
def tan(x: Real) -> Real: 
    """According to specification of C++ std::tan."""
    ...
@overload
def asin(x: Real) -> Real: 
    """According to specification of C++ std::asin."""
    ...
@overload
def acos(x: Real) -> Real: 
    """According to specification of C++ std::acos."""
    ...
@overload
def atan(x: Real) -> Real: 
    """According to specification of C++ std::atan."""
    ...
@overload
def sinh(x: Real) -> Real: 
    """According to specification of C++ std::sinh."""
    ...
@overload
def cosh(x: Real) -> Real: 
    """According to specification of C++ std::cosh."""
    ...
@overload
def tanh(x: Real) -> Real: 
    """According to specification of C++ std::tanh."""
    ...
@overload
def asinh(x: Real) -> Real: 
    """According to specification of C++ std::asinh."""
    ...
@overload
def acosh(x: Real) -> Real: 
    """According to specification of C++ std::acosh."""
    ...
@overload
def atanh(x: Real) -> Real: 
    """According to specification of C++ std::atanh."""
    ...
@overload
def sign(x: Real) -> Real: 
    """Returns 0 for x=0, -1 for x<0 and 1 for x>1."""
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
def pow(x: Real, y: Real) -> Real: 
    """Return $x^y$."""
    ...
@overload
def max(x: Real, y: Real) -> Real: 
    """Return maximum of x and y."""
    ...
@overload
def IfThenElse(condition: Real, ifTrue: Real, ifFalse: Real) -> Real: 
    """Symbolic function for conditional evaluation.
    
    If the condition evaluates to True, the expression ifTrue is evaluated, while otherwise expression ifFalse is evaluated
    
    Examples:
        x=SymReal(-1)
        y=SymReal(2,'y')
        a=SymReal.IfThenElse(x<0, y+1, y-1))
    """
    ...
@overload
def SetRecording(flag: bool) -> None: 
    """Set current (global / module-wide) status of expression recording.
    
    By default, recording is on.
    
    Examples:
        SymReal.SetRecording(True)
    """
    ...
@overload
def GetRecording() -> bool: 
    """Get current (global / module-wide) status of expression recording.
    
    Examples:
        Real.GetRecording()
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
    def Evaluate(self) -> List[float]: 
        """Return evaluated expression (prioritized) or stored vector value.
        
        (not recorded)
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
            v1 = SymVector([1,4,8])
            length = v1.NormL2() #gives 9.
        """
        ...
    @overload
    def MultComponents(self, other: Vector) -> Real: 
        """Perform component-wise multiplication of vector times other vector and return result.
        
        This corresponds to the numpy multiplication using '*'.
        
        Examples:
            v1 = SymVector([1,2,4])
            v2 = SymVector([1,0.5,0.25])
            v3 = v1.MultComponents(v2)
        """
        ...
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
    def Evaluate(self) -> List[float]: 
        """Return evaluated expression (prioritized) or stored Matrix value.
        
        (not recorded)
        """
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
    def SetUserFunctionFromDict(self, mainSystem: MainSystem, fcnDict: dict, itemIndex: ItemIndex, userFunctionName: str) -> None: 
        """Create C++ std::function (as requested in C++ item) with symbolic user function as recorded in given dictionary, as created with ConvertFunctionToSymbolic(...)."""
        ...
    @overload
    def __repr__(self) -> str: ...
    @overload
    def __str__(self) -> str: ...
