#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  One type model for all generators: how a type name of
#           definitions/ is spelled in each destination. It replaces eight hand-written tables
#           (typeConversion and typeCasts for items and for structures, convertToDict,
#           typeConversionStub, type2PyTyping, cppTypeNames) that overlapped and disagreed.
#
#           Render(typeName, destination, context):
#             destination 'cppStorage'  - the C++ member type (Index, Vector3D, std::string, ...)
#                         'cppExchange' - the type a pybind value is cast to or from in the generated
#                                         dict access (std::vector<Real>, py::array_t<Real>, ...);
#                                         temporary, it goes when the generated code calls FromPython
#                         'dictType'    - the 'type' entry of a structure's GetDictionaryWithTypeInfo
#                         'stub'        - the type in the .pyi stubs (Tuple[float,float,float], ...)
#                         'pyTyping'    - the type shown in docstrings and itemInterface type hints
#             context     'items' or 'structures'
#
#           Rules first, then 'names' (spellings the same for items and structures), then
#           'exceptions': the remaining differences between items and structures, each with its
#           reason. A name without rule or exception passes through
#           unchanged, as the old TypeConversion did - most C++ function signatures rely on it.
#
#           The facts come from definitions/definitionTypes.py: the range and item-kind forms of
#           Real/float/Index/ArrayIndex, and the user-function signatures.
#
# Author:   Johannes Gerstmayr, Claude-JG
# Date:     2026-09-14 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import re
import sys

repositoryRoot = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
definitionsDirectory = os.path.join(repositoryRoot, 'definitions')
if definitionsDirectory not in sys.path:
    sys.path.insert(0, definitionsDirectory)

import definitionTypes as dt    # noqa: E402

destinations = ['cppStorage', 'cppExchange', 'dictType', 'stub', 'pyTyping']
contexts = ['items', 'structures']

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#facts derived from definitions/definitionTypes.py

#the base type of each range and item-kind form: UReal -> Real, NodeIndex -> Index, ...
baseType = {}
for _spec in [dt.TReal, dt.Tfloat, dt.TIndex, dt.TArrayIndex]:
    for _form in _spec.constrainedForms.values():
        baseType[_form] = str(_spec)
itemIndexKinds = [dt.TIndex.constrainedForms[kind] for kind in [dt.ItemNode, dt.ItemObject, dt.ItemMarker, dt.ItemLoad, dt.ItemSensor]]

userFunctionSignatures = dt.userFunctionSignatures
userFunctionStorageTemplate = 'PythonUserFunctionBase< {UFT} >'   #how items store a user function

#fixed-size names: Vector3D, Matrix6D, Float4, Index2, UInt3, NodeIndex4
_sizedName = re.compile(r'^(Vector|Matrix|Float|Index|UInt|NodeIndex)(\d+)D?$')


def SizedName(typeName):
    """(family, size) for a fixed-size name such as Vector3D or Float4, else (None, None)"""
    match = _sizedName.match(typeName)
    if match is None:
        return None, None
    return match.group(1), int(match.group(2))


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#named spellings, the same for items and structures: destination -> {typeName: spelling}
names = {
    'cppStorage': {
        'Int': 'Index',                             #C++ uses Index for every integer
        'NumpyMatrixI': 'MatrixI',
    },
    'cppExchange': {
        'Int': 'Index',
        'JointTypeList': 'std::vector<Joint::Type>',
        'NumpyVector': 'py::array_t<Real>', 'NumpyMatrix': 'py::array_t<Real>', 'NumpyMatrixI': 'py::array_t<Index>',
        #lists of fixed-size vectors and matrices are exchanged through their Py... wrapper classes
        'Vector2DList': 'PyVector2DList', 'Vector3DList': 'PyVector3DList', 'Vector6DList': 'PyVector6DList',
        'Matrix3DList': 'PyMatrix3DList',
    },
    'dictType': {                                   #the 'type' entry of GetDictionaryWithTypeInfo
        'ResizableVector': 'Vector', 'NumpyVector': 'Vector', 'NumpyMatrix': 'Matrix',
        'StdArray33F': 'MatrixFloat',
        'Index2': 'IndexArray', 'Index4': 'IndexArray', 'ArrayIndex': 'IndexArray',
        'ArrayFloat': 'VectorFloat', 'Float3': 'VectorFloat', 'Float4': 'VectorFloat',
    },
    'stub': {
        'ArrayIndex': 'List[int]', 'ArrayFloat': 'List[float]', 'Index2': 'Tuple[int,int]',
        'void': 'None', 'std::string': 'str',
    },
    'pyTyping': {
        'Vector': 'array_like', 'ArrayIndex': 'array_like',
    },
}

#the differences between items and structures that remain, each with its reason:
#(destination, context) -> {typeName: spelling}
exceptions = {
    ('cppStorage', 'items'): {
        #items store numpy parameters as Vector/Matrix members; structures use these types only as
        #return values of solver functions, which hand out py::array_t directly
        'NumpyVector': 'Vector', 'NumpyMatrix': 'Matrix',
    },
    ('cppStorage', 'structures'): {
        'NumpyVector': 'py::array_t<Real>', 'NumpyMatrix': 'py::array_t<Real>',
    },
}

_scalarCpp = {'Bool': 'bool', 'Real': 'Real', 'float': 'float', 'Index': 'Index',
              'String': 'std::string', 'FileName': 'std::string'}
_scalarPython = {'Bool': 'bool', 'Int': 'int', 'Index': 'int', 'Real': 'float', 'float': 'float',
                 'String': 'str', 'FileName': 'str'}
_arrayLike = ['Matrix', 'SymmetricMatrix', 'NumpyVector', 'NumpyMatrix', 'NumpyMatrixI', 'StdArray33F']


def _Scalar(typeName, table):
    """spelling of a scalar or string, through its base type; None if typeName is none"""
    name = baseType.get(typeName, typeName)
    if typeName in itemIndexKinds or typeName in ['ArrayNodeIndex', 'ArrayObjectIndex', 'ArrayMarkerIndex', 'ArraySensorIndex']:
        return None  #item indices have their own rules
    return table.get(name, None)


def _CppStorage(typeName, context):
    scalar = _Scalar(typeName, _scalarCpp)
    if scalar is not None:
        return scalar
    if context == 'items':
        if typeName in itemIndexKinds:
            return 'Index'  #in C++ all indices are the same
        if baseType.get(typeName) == 'ArrayIndex':
            return 'ArrayIndex'
        family, size = SizedName(typeName)
        if family == 'NodeIndex':
            return 'Index' + str(size)
        if typeName in userFunctionSignatures:
            return userFunctionSignatures[typeName]
    if typeName in ['Vector', 'Matrix']:
        return typeName
    return None


def _CppExchange(typeName, context):
    """the same for items and structures: variable sizes are std::vector, fixed sizes std::array"""
    scalar = _Scalar(typeName, _scalarCpp)
    if scalar is not None and typeName not in itemIndexKinds:
        return scalar
    family, size = SizedName(typeName)
    if typeName == 'Vector' or family == 'Vector':
        return 'std::vector<Real>'
    if typeName == 'ArrayIndex':
        return 'std::vector<Index>'
    if typeName == 'ArrayFloat':
        return 'std::vector<float>'
    if family == 'Float':
        return 'std::array<float,' + str(size) + '>'
    if family in ['Index', 'UInt']:
        return 'std::array<Index,' + str(size) + '>'
    if family == 'Matrix':
        return 'std::array<std::array<Real,' + str(size) + '>,' + str(size) + '>'
    return None


def _Python(typeName, destination):
    """stub and pyTyping share scalars; they differ in how fixed sizes and arrays are written"""
    scalar = _Scalar(typeName, _scalarPython)
    if scalar is not None:
        return scalar
    family, size = SizedName(typeName)
    arrayLike = 'ArrayLike' if destination == 'stub' else 'array_like'
    if destination == 'stub':
        if family == 'Float' and size in [3, 4]:
            return 'Tuple[' + ','.join(['float'] * size) + ']'
        if typeName in _arrayLike or family == 'Matrix':
            return arrayLike
        return None
    #pyTyping: short fixed sizes as element lists, item indices by their kind
    if typeName in itemIndexKinds:
        return typeName
    if family in ['Vector', 'Float']:
        return '[' + ','.join(['float'] * size) + ']' if size <= 4 else arrayLike
    if family in ['Index', 'Matrix'] or typeName in _arrayLike:
        return arrayLike
    return None


def Render(typeName, destination, context):
    """the spelling of typeName in destination ('cppStorage', 'cppExchange', 'dictType', 'stub',
    'pyTyping') for context ('items' or 'structures'); unknown names pass through unchanged"""
    typeName = str(typeName)
    special = exceptions.get((destination, context), {})
    if typeName in special:
        return special[typeName]
    if typeName in names.get(destination, {}):
        return names[destination][typeName]
    if typeName in userFunctionSignatures and destination in ['cppStorage', 'cppExchange']:
        return userFunctionSignatures[typeName]    #items wrap it for storage, see CppMemberType
    if typeName in userFunctionSignatures and destination == 'stub':
        return 'Any'
    if destination == 'cppStorage':
        result = _CppStorage(typeName, context)
    elif destination == 'cppExchange':
        result = _CppExchange(typeName, context)
    elif destination in ['stub', 'pyTyping']:
        result = _Python(typeName, destination)
    elif destination == 'dictType':
        result = None
    else:
        raise ValueError('typeModel.Render: unknown destination ' + repr(destination))
    return typeName if result is None else result


def ConstraintNote(typeName):
    """the range of a constrained type as the start of a C++ comment ('must be > 0; '), '' otherwise;
    C++ stores PReal, UReal, ... as their base type"""
    for spec in [dt.TReal, dt.Tfloat, dt.TIndex]:
        if str(typeName) == spec.constrainedForms['greaterThan']:
            return 'must be > 0; '
        if str(typeName) == spec.constrainedForms['minimum']:
            return 'must be >= 0; '
    return ''


def CppMemberType(typeName, context):
    """the C++ member type; items store a user function wrapped in PythonUserFunctionBase"""
    rendered = Render(typeName, 'cppStorage', context)
    if context == 'items' and str(typeName) in userFunctionSignatures:
        return userFunctionStorageTemplate.replace('{UFT}', rendered)
    return rendered
