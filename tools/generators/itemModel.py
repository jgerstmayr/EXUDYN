#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Facts about items that more than one emitter needs, in one place (revision2026 step R4.3,
#           part 2b): the type tables and type predicates the item generator used to define inline,
#           the user-function types, how a member of definitions/ renders to the strings of the old
#           representation, and per-item accessors the emitters use instead of re-deriving them.
#
#           Everything here was moved, not rewritten: the type tables and predicates verbatim from
#           src/pythonGenerator/pythonAutoGenerateObjects.py, the rendering from
#           tools/generators/definitionLoader.py - the byte-identity gate is what keeps it so.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-14 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys

repositoryRoot = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                               '..', '..'))
definitionsDirectory = os.path.join(repositoryRoot, 'definitions')
if definitionsDirectory not in sys.path:
    sys.path.insert(0, definitionsDirectory)

import definitionTypes          # noqa: E402

itemModules = ['itemDefsNodes', 'itemDefsObjects', 'itemDefsMarkers', 'itemDefsLoads',
               'itemDefsSensors']
itemTypeOrder = ['Node', 'Object', 'Marker', 'Load', 'Sensor']

#possible types for certain items (object, marker, node, ...)
possibleTypes = {'Object':['_None','Ground','Connector','Constraint','Body','SingleNoded','MultiNoded','FiniteElement','SuperElement'],
                 'Node':['_None','Ground','Position2D','Orientation2D','Point2DSlope1','Position','Orientation','RigidBody',
                         'RotationEulerParameters','RotationRxyz','RotationRotationVector','RotationLieGroup',
                         'GenericODE2','GenericData'],
                 'Marker':['_None','Node','Object','Body','Position','Orientation','Coordinate','BodyLine','BodySurface',
                           'BodyVolume','BodyMass','BodySurfaceNormal'],
                 'Load':[], 'Sensor':[]}

#consider templated function in future:
#     template<typename ReturnType, typename... Args>
# ReturnType invokeFunction(std::function<ReturnType(Args...)> func, Args... args) {
#     return func(args...); //
# }
#example use:
# std::function<double(double, int)> multiply = [](double x, int y) -> double {
#         return x * y;
#     };
# std::cout << invokeFunction(multiply, 3.5, 2) << std::endl; // Outputs: 7.0

useNewUserFunctions = True

#the C++ signature of each user-function type; the table lives in definitions/definitionTypes.py
#
pyFunctionTypeConversion = definitionTypes.userFunctionSignatures
pyFunctionTypeConversionUFtemplate = '{UFT}'
if useNewUserFunctions:
    pyFunctionTypeConversionUFtemplate = 'PythonUserFunctionBase< {UFT} >'


#this for mutable args
def IsASafelyVector(parameterType):
    if ((parameterType == 'Vector2D') or 
        (parameterType == 'Vector3D') or
        (parameterType == 'Vector4D') or 
        (parameterType == 'Vector6D') or
        (parameterType == 'Vector7D') or
        (parameterType == 'Vector9D') or
        (parameterType == 'NumpyVector')
        ):
        return True
    else:
        return False

def IsAVector(parameterType):
    if ((parameterType == 'Vector') 
        or (parameterType == 'Float4')
        or IsASafelyVector(parameterType)
        ):
        return True
    else:
        return False

#this for mutable args
def IsASimpleMatrix(parameterType):
    if ((parameterType == 'Matrix3D') or
        (parameterType == 'Matrix6D') or
        (parameterType == 'NumpyMatrix') or 
        (parameterType == 'NumpyMatrixI')
        ):
        return True
    else:
        return False

def IsAMatrixVectorSpecial(parameterType):
    if ((parameterType == 'Vector3DList') or
        (parameterType == 'Matrix3DList') or
        (parameterType == 'PyMatrixContainer')
        ):
        return True
    else:
        return False

def IsAArrayIndex(parameterType):
    if ((parameterType == 'ArrayNodeIndex') or
        (parameterType == 'NodeIndex2') or
        (parameterType == 'NodeIndex3') or
        (parameterType == 'NodeIndex4') or
        (parameterType == 'ArrayObjectIndex') or
        (parameterType == 'ArrayMarkerIndex') or
        (parameterType == 'ArrayLoadIndex') or   #unused
        (parameterType == 'ArraySensorIndex')
        ):
        return True
    else:
        return False

#this function finds out, if a parameter is set with a special Set...Safely function in C++
def IsASetSafelyParameter(parameterType):
    if (IsASafelyVector(parameterType) or 
        IsASimpleMatrix(parameterType) or 
        IsAMatrixVectorSpecial(parameterType) or 
        (parameterType == 'String') or
        (parameterType == 'ItemName')
        # (parameterType == 'Vector2D') or 
        # (parameterType == 'Vector3D') or
        # (parameterType == 'Vector4D') or 
        # (parameterType == 'Vector6D') or
        # (parameterType == 'Vector7D') or
        # (parameterType == 'Vector9D') or
        # (parameterType == 'NumpyVector') or
        # (parameterType == 'Matrix3D') or
        # (parameterType == 'Matrix6D') or
        # (parameterType == 'NumpyMatrix') or 
        # (parameterType == 'NumpyMatrixI') or #for index arrays, mesh, ...
        ):
        return True
    else:
        return False

def GetSetSafelyFunctionName(parType):
    if parType[0:6] == 'Vector' and parType[-1] == 'D': #any Vector[]D
        val = parType[6:-1]   #gives number
        safelyFunctionName =  'SetSlimVectorTemplateSafely<Real, '+val+'>'
    elif parType[0:6] == 'Matrix' and parType[-1] == 'D': #any Vector[]D
        val = parType[6:-1]   #gives number
        safelyFunctionName =  'SetConstMatrixTemplateSafely<'+val+','+val+'>'
    else:
        safelyFunctionName = 'Set'+parType+'Safely'
    return safelyFunctionName 


#SetConstMatrixTemplateSafely<3, 3>(d, item, destination);

#return true, if the the parameter triggers an internal get/set function for conversion, e.g., BeamSection
def IsInternalSetGetParameter(parameterType):
    #needs to automatically generate Internal function
    if ((parameterType == 'BeamSection')
        #or (parameterType == 'BeamSectionGeometry') #this is directly stored in visualization
        ):
        return True
    else:
        return False


#return True for types, which get a range check and does a .def_property access in pybind and a set/get function
def IsTypeWithRangeCheck(origType):
    if origType.find('PInt') != -1 or origType.find('UInt') != -1 or origType.find('PReal') != -1 or origType.find('UReal') != -1:
        return True
    return False

#check if type is a item index (NodeIndex, ...)
def IsItemIndex(parameterType):
    if (
        (parameterType == 'NodeIndex') or
        (parameterType == 'ObjectIndex') or
        (parameterType == 'MarkerIndex') or
        (parameterType == 'LoadIndex') or
        (parameterType == 'SensorIndex') or
        (parameterType == 'NodeIndex2') or
        (parameterType == 'NodeIndex3') or
        (parameterType == 'NodeIndex4') or
        (parameterType == 'ArrayNodeIndex') or
        (parameterType == 'ArrayObjectIndex') or
        (parameterType == 'ArrayMarkerIndex') or
        (parameterType == 'ArraySensorIndex')
        ):
        return True
    else:
        return False

#split the symbol of a parameter - a $...$ span at the very START of its description - off the
#description; return [descriptionWithoutSymbol, mathSymbolString], the symbol '' if there is none
def ExtractMathSymbol(s):
    mathSymbolString=""
    stringWithoutSymbol=""
    if s[0]=='$':
        splitString = s.split('$')
        n = len(splitString)
        
        if n == 3: #one symbol + text
            mathSymbolString = "$" + splitString[1] + "$"
            stringWithoutSymbol=splitString[2]
        elif n%2 != 1:
            print("ERROR: did not find closing $ for description/variable; str =", s)
        else: #several symbols, but one leading
            mathSymbolString = "$" + splitString[1] + "$"
            mathSign=''
            for i in range(2,n):
                if i%2 == 1:
                    sAdd = splitString[i]
                else:
                    sAdd = splitString[i]
                stringWithoutSymbol+=mathSign+sAdd
                mathSign = '$'

#        print("splitString=",splitString)
#        print("mathSymbolString=",mathSymbolString)
#        print("stringWithoutSymbol=",stringWithoutSymbol)
    else:
        stringWithoutSymbol=s

    return [stringWithoutSymbol, mathSymbolString]


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#rendering of definitions/ members to the strings of the old representation
#header keys holding a real bool in definitions/, a 'True'/'False' string for the generators
booleanHeaderKeys = set(['writePybindIncludes', 'appendToFile', 'addDictionaryAccess',
                         'excludeFromTheDoc'])

#the old item parser turned every literal backslash-n into a newline - except inside the
#multi-line blocks; the structure parser did so only for three header keys. The definitions store
#the readable form, so the generators get the parser's form back.
mangleRules = {'items':      {'all': True, 'keys': set(), 'verbatim': set(['equations',
                                                                           'miniExample'])},
                'structures': {'all': False, 'verbatim': set(),
                               'keys': set(['classDescription', 'sectionText', 'cppText'])}}

BACKSLASH, NEWLINE = chr(92), chr(10)


def Mangle(text, key, source):
    rule = mangleRules[source]
    if key in rule['verbatim'] or (not rule['all'] and key not in rule['keys']):
        return text
    return text.replace(BACKSLASH + 'n', NEWLINE)


def LineType(member):
    text = 'F' if 'Function' in member['kind'] else 'V'
    for field, letter in (('isVirtual', 'v'), ('isStatic', 's'), ('fromParent', 'p'),
                          ('isLinked', 'L')):
        #isVirtual defaults to True in the constructors but only means something for functions
        if member.get(field, False) and (field != 'isVirtual' or text == 'F'):
            text += letter
    return text


def Size(member):
    """the old 'size' column, derived from the shape the type carries"""
    if 'Function' in member['kind']:
        return ''
    typeSpec = member.get('type', '')
    size = getattr(typeSpec, 'size', None)
    if size is not None:
        if isinstance(size, tuple):          #the old format spelled a matrix as its flat count
            product = 1
            for value in size:
                product *= value
            return str(product)
        return str(size)
    fixed = {'Float3': '3', 'Float4': '4', 'StdArray33F': '3x3', 'Matrix6D': '36'}
    fixed.update({name: str(n) for n, name in definitionTypes.vectorSizes.items()})
    fixed.update({name: str(r * c) for (r, c), name in definitionTypes.matrixSizes.items()})
    fixed.update({name: str(n) for n, name in definitionTypes.indexTupleSizes.items()})
    fixed.update({name: str(n) for n, name in definitionTypes.nodeIndexTupleSizes.items()})
    if str(typeSpec) in fixed:
        return fixed[str(typeSpec)]
    return member.get('size', '') or ''


def DefaultValueString(member):
    if 'Function' in member['kind']:
        implementation = member.get('implementation', None)
        return '' if implementation is None else implementation
    deprecated = member.get('deprecated', None)
    if deprecated is not None:
        return deprecated.ToCpp()
    value = member.get('defaultValue', '')
    if value is definitionTypes.NoDefaultValue or value is None or value == '':
        return ''
    return definitionTypes.CppLiteral(value, str(member.get('type', '')))


def _DefaultValueRendering(member, Literal):
    """the Python or the document rendering of a member's default value (#2682, RG3.24.3): the
    same three cases as DefaultValueString, so that the three renderings cannot drift apart"""
    if 'Function' in member['kind']:
        return ''                    #a function's implementation is C++ and is never a default
    if member.get('deprecated', None) is not None:
        return ''                    #a deprecated member has no default value; the field carries
                                     #the version it went out in, which the C++ emitter reads
    value = member.get('defaultValue', '')
    if value is definitionTypes.NoDefaultValue or value is None or value == '':
        return ''
    return Literal(value, str(member.get('type', '')))


def DefaultValuePython(member):
    """the default value as the Python that itemInterface.py and the stubs write"""
    return _DefaultValueRendering(member, definitionTypes.PythonLiteral)


def DefaultValueDocument(member):
    """the default value as a documentation table shows it"""
    return _DefaultValueRendering(member, definitionTypes.DocumentLiteral)


def Flags(member, source, structureClassNames):
    flags = member.get('cFlags', '') or ''
    isFunction = 'Function' in member['kind']
    if isFunction and member.get('implementation', None) is None:
        flags += 'D'
    if source == 'items':
        if isFunction:
            flags += 'I'        #inert on functions; restored for byte identity of the old input
        elif 'n' in flags:
            flags = flags.replace('n', '')
        else:
            flags += 'I'
    else:
        if 'N' not in flags:
            flags = 'P' + flags  #SFNoPybind 'N' inverts P; 'N' stays, structureModel.HasFlag reads it
        if str(member.get('type', '')) in structureClassNames:
            flags = flags.replace('P', 'PS', 1) if 'P' in flags else 'S' + flags
    return flags


def OutputVariablesString(entries):
    """the dict literal both generators eval() after doubling the backslashes"""
    if not entries:
        return ''
    parts = []
    for entry in entries:
        description = entry['description']
        quote = chr(34) if "'" in description else "'"
        parts.append("'" + entry['outputVariable'].name + "':" + quote + description + quote)
    return '{' + ', '.join(parts) + '}'


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#user functions of MainSystem; they are not items, but share the symbolic user-function machinery
mainSystemUserFunctions = [
    {'itemType': 'MainSystem', 'classType': '', 'userFunctionName': 'preStepUserFunction',
     'pyUserFunctionType': 'PyFunctionBoolMbsScalar'},
    {'itemType': 'MainSystem', 'classType': '', 'userFunctionName': 'postStepUserFunction',
     'pyUserFunctionType': 'PyFunctionBoolMbsScalar'},
    {'itemType': 'MainSystem', 'classType': '', 'userFunctionName': 'postNewtonFunction',
     'pyUserFunctionType': 'PyFunctionVector2DMbsScalar'},
    ]


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#per-item accessors over definitions/
def ItemDefinitions():
    """all item definitions in generation order"""
    definitions = []
    for moduleName in itemModules:
        definitions += __import__(moduleName).definitions
    return definitions


def IsParameter(member):
    return member['kind'] == 'ItemParameter'


def IsInterfaceParameter(member):
    """a parameter that is part of the Python dictionary interface (the old 'I' flag)"""
    return IsParameter(member) and 'n' not in (member.get('cFlags', '') or '')


def IsReadOnly(member):
    return 'R' in (member.get('cFlags', '') or '')


def TypeName(member):
    return str(member.get('type', ''))


def Destination(member):
    return str(member.get('destination', ''))


def Description(member):
    """the description as the old item parser delivered it"""
    return Mangle(member.get('description', '') or '', 'parameterDescription', 'items')


def ClassDescription(definition):
    return Mangle(str(definition.get('classDescription', '') or ''), 'classDescription', 'items')


def FunctionImplementation(definition, pythonName):
    """the C++ body of a function member, '' if absent or declaration only"""
    for member in definition['members']:
        if 'Function' in member['kind'] and member['pythonName'] == pythonName:
            return member.get('implementation', None) or ''
    return None


def HasPythonInterface(definition):
    """True if the item has at least one own interface parameter (the old condition lineType == 'V'
    and flag I - inherited 'Vp' members do not count)"""
    return any(IsInterfaceParameter(m) and not m.get('fromParent', False)
               for m in definition['members'])


def SymbolicUserFunctions(definition):
    """the PyFunction parameters of one item, as the symbolic user-function set records them"""
    classType = definition.get('classType', '')
    result = []
    for member in definition['members']:
        if IsInterfaceParameter(member) and 'PyFunction' in TypeName(member):
            result.append({'itemType': definition['className'].replace(classType, ''),
                           'classType': classType,
                           'userFunctionName': member.get('cplusplusName', '') or member['pythonName'],
                           'pyUserFunctionType': TypeName(member),
                           'stdFunctionType': pyFunctionTypeConversion[TypeName(member)],
                           #the Python def of the definition file, where there is one: it is what
                           #the arguments are really called (revision2026b step RG12.4, #2664)
                           'userFunction': member.get('userFunction'),
                           })
    return result


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#direct member access for the emitters: each predicate states what a letter
#of the old string representation (lineType, cFlags) used to encode, read from the member itself
def IsFunction(member):
    return 'Function' in member['kind']


def IsVariable(member):
    """a member variable, including one declared by the parent class (old lineType containing V)"""
    return not IsFunction(member)


def FromParent(member):
    return bool(member.get('fromParent', False))


def IsOwnVariable(member):
    """a member variable of this class itself (old lineType == 'V')"""
    return IsVariable(member) and not FromParent(member) and not member.get('isLinked', False)


def IsVirtualFunction(member):
    return IsFunction(member) and bool(member.get('isVirtual', False))


def IsStaticFunction(member):
    return IsFunction(member) and bool(member.get('isStatic', False))


def IsDeclarationOnly(member):
    """a function whose body is in the hand-written .cpp (old flag D)"""
    return IsFunction(member) and member.get('implementation', None) is None


def HasFlag(member, letter):
    """a flag stored in cFlags: CFConst 'C', CFMutable 'U', CFReadOnly 'R'"""
    return letter in (member.get('cFlags', '') or '')


def CppName(member):
    return Mangle(member.get('cplusplusName', '') or member['pythonName'], 'cplusplusName', 'items')


def DefaultValue(member):
    """the C++ spelling of the default value, or the body of a function"""
    return Mangle(DefaultValueString(member), 'defaultValue', 'items')


def Args(member):
    return Mangle(member.get('args', '') or '', 'args', 'items')


def Header(definition, key):
    """a class header value as a string ('' if absent)"""
    value = definition.get(key, '')
    if value is None:
        return ''
    if key in booleanHeaderKeys:
        return 'True' if value else 'False'
    return Mangle(str(value), key, 'items')


def OutputVariableNames(definition):
    """the names of the item's output variables, in definition order, each once"""
    names = []
    for entry in definition.get('outputVariables', []) or []:
        if entry['outputVariable'].name not in names:
            names.append(entry['outputVariable'].name)
    return names
