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
NPreal3D = Annotated[NDArray[float], Literal[3]]
NPint3D = Annotated[NDArray[int], Literal[3]]

import exudyn
from exudyn import (ObjectIndex, NodeIndex, MarkerIndex, LoadIndex, SensorIndex,
                    # MainSystem, SystemContainer,
                    # Vector2DList, Vector3DList, Vector6DList,
                    # MatrixContainer, 
                    # VisualizationSettings, #could be erased in future
                    )

# class ItemType(Enum):
#     _None = 0
#     Node = 1
#     Object = 2
#     Marker = 3
#     Load = 4
#     Sensor = 5

# class LinearSolverSettings:
#     ignoreRedundantConstraints: bool
#     ignoreSingularJacobian: bool
#     pivotTreshold: bool
#     reuseAnalyzedPattern: bool
#     showCausingItems: bool

# class Parallel:
#     multithreadedLLimitJacobians: int
#     multithreadedLLimitLoads: int
#     multithreadedLLimitMassMatrices: int
#     multithreadedLLimitResiduals: int
#     numberOfThreads: int
#     taskSplitMinItems: int
#     taskSplitTasksPerThread: int
  
# class SimulationSettings:
#     linearSolverSettings: LinearSolverSettings
#     parallel: Parallel
    
  
# class SystemContainer:
#     visualizationSettings: VisualizationSettings
#     def __init__(self): ...
#     @overload
#     def AddSystem(self) -> exudyn.MainSystem: ...
#     @overload
#     def GetVersionString(self) -> str: ...

# class MainSystem:
#     @overload
#     def AddNode(self, pyObject: dict) -> exudyn.NodeIndex: ...
#     @overload
#     def GetNode(self, nodeNumber: int) -> dict: ...
#     @overload
#     def GetLoad(self, loadNumber=-1) -> dict: ...
#     def GetMarker(self, markerNumber=[0.,0.]) -> dict: ...

#     @overload
#     def AddObject(self, pyObject: dict) -> exudyn.ObjectIndex: ...
#     # @overload
#     # def AddMassPoint(self, referenceCoordinates: list = [0.,0.,0.],
#     #                            initialCoordinates: list = [0.,0.,0.],
#     #                            initialVelocities: list = [0.,0.,0.],
#     #                            physicsMass: float=0,
#     #                            gravity: list = [0.,0.,0.],
#     #                            graphicsDataList: list = []) -> exudyn.ObjectIndex: ...
