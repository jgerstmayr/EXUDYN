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
                    # MainSystem, SystemContainer,
                    # Vector2DList, Vector3DList, Vector6DList,
                    # MatrixContainer, 
                    # VisualizationSettings, #could be erased in future
                    )
from exudyn.graphicsDataUtilities import color4red, color4default


# class MainSystem:
#     @overload
#     def CreateMassPoint(self,#node quantities
#                                 referenceCoordinates = [0.,0.,0.],
#                                 initialCoordinates = [0.,0.,0.],
#                                 initialVelocities = [0.,0.,0.],
#                                 #object quantities:
#                                 physicsMass=0,
#                                 gravity = [0.,0.,0.],
#                                 graphicsDataList = [],
#                                 #graphics, mixed
#                                 drawSize = -1,
#                                 color =  [-1.,-1.,-1.,-1.],
#                                 show = True, #if graphicsDataList is empty, node is shown, otherwise body is shown
#                                 name = '',   #both for node and object
#                                 create2D = False, #NodePoint2D, MassPoint2D
#                                 returnDict = False, #if True, returns dictionary of all data
#                                 ) -> Union[ObjectIndex, dict]: ...
    
#     @overload
#     def CreateSpringDamper(self,
#                                   bodyOrNode0:int, bodyOrNode1:int,
#                                   localPosition0: list = [0.,0.,0.],
#                                   localPosition1: list = [0.,0.,0.],
#                                   #
#                                   referenceLength = None, 
#                                   stiffness: float = 0., damping: float = 0., force: float = 0.,
#                                   velocityOffset: float = 0., 
#                                   show=True, drawSize: float=-1, color=[-1.,-1.,-1.,-1.],
#                                   ) -> ObjectIndex: ...

#     @overload
#     def CreateRevoluteJoint(mbs, bodyNumber0:ObjectIndex, bodyNumber1:ObjectIndex, 
#                                   position:[float,float,float], 
#                                   axis:[float,float,float], useGlobalFrame=True, 
#                                   show=True, axisRadius=0.1, axisLength=0.4,
#                                   color=[-1.,-1.,-1.,-1.]) -> [ObjectIndex, MarkerIndex, MarkerIndex]: ...

#     @overload
#     def CreatePrismaticJoint(mbs, bodyNumber0:ObjectIndex, bodyNumber1:ObjectIndex, 
#                                   position:[float,float,float], 
#                                   axis:[float,float,float], useGlobalFrame=True, 
#                                   show=True, axisRadius=0.1, axisLength=0.4,
#                                   color=[-1.,-1.,-1.,-1.]) -> [ObjectIndex, MarkerIndex, MarkerIndex]: ...

#     @overload
#     def CreateGenericJoint(mbs, bodyNumber0:ObjectIndex, bodyNumber1:ObjectIndex, 
#                                   position:[float,float,float], rotation0=np.eye(3),
#                                   constrainedAxes=[1,1,1, 1,1,1], useGlobalFrame=True, 
#                                   show=True, axesRadius=0.1, axesLength=0.4,
#                                   color=[-1.,-1.,-1.,-1.]) -> [ObjectIndex, MarkerIndex, MarkerIndex]: ...
    
    # @overload
    # def AddMassPoint(self, referenceCoordinates: list = [0.,0.,0.],
    #                            initialCoordinates: list = [0.,0.,0.],
    #                            initialVelocities: list = [0.,0.,0.],
    #                            physicsMass: float=0,
    #                            gravity: list = [0.,0.,0.],
    #                            graphicsDataList: list = []) -> exudyn.ObjectIndex: ...
