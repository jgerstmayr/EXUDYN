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
from exudyn import (ObjectIndex, NodeIndex, MarkerIndex, LoadIndex, SensorIndex)

from exudyn.graphicsDataUtilities import color4red, color4default


