#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Shared vocabulary and constructors for the Exudyn definition files.
#
# Details:  HAND-WRITTEN and reviewed. This is the SINGLE definition of the flag letters, the
#           destinations, the closed-set header values and the member constructors.
#
#           Why hand-written: the flag letters used to have no central definition at all - the
#           old generator tested them as bare literals (parameter['cFlags'].find('I') and
#           friends, 16 such sites in pythonAutoGenerateObjects.py), and the step-31 emitter carried
#           its own copy of the table. Two independent transcriptions of the same vocabulary can
#           drift with nothing to notice. So the table lives here, once;
#           tools/generators/definitionLoader.py imports it and owns no table of its own, and a
#           flag, type or closed-set value that is missing here is a NameError when the
#           definitions are imported. Drift becomes an error instead of a silent difference.
#
#           Why constants and not strings: a flag spelled as a bare letter is a value that
#           nothing checks, and a mistyped letter changed the build silently. As a name it is a
#           NameError at import, and an editor can complete it.
#
#           Layout: three blocks, by where the vocabulary is USED -
#             SHARED      types, default values and sentinels used by items AND structures
#             ITEMS       destinations, item flags, closed header sets, item constructors and
#                         the function library resolution
#             STRUCTURES  structure flags, Deprecated, structure constructors
#           A name used by only one kind of definition lives in that block; one-off types are
#           listed most-used first.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-13 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


#==================================================================================================
# SHARED - used by item AND structure definitions
#==================================================================================================

#--------------------------------------------------------------------- default values
#A default value is a real Python value: True, False, 0., 0, 1.5 - not the string '0.'. Three
#renderings are needed and they differ, so they are computed rather than stored:
#
#   C++        the literal written into the generated header      true   0.   -1.f
#   Python     the value written into itemInterface.py            True   0.   -1.
#   document   the readable form for the reference tables         True   0.   -1.
#
#CppFloatLiteral() is the C++ side for numbers. It has to reproduce the spelling the definitions
#used, because the generated set is compared byte for byte: an integral float is written with a
#trailing dot and no zero (0., 1., -1.), exponent form is used exactly where Python's repr() uses
#it (with the + and any leading exponent zeros stripped, so 1e+38 and 1e-08 become 1e38 and 1e-8),
#and the f suffix comes from the DECLARED TYPE, never from the value - which is possible because
#every default carrying a suffix sits on a float type.
#
#Values that are neither a number nor a bool - a C++ constructor call or a named C++ constant -
#are a CppValue, which carries all three renderings explicitly. They cannot be derived: the
#Python form of Float4({-1.f,-1.f,-1.f,-1.f}) is [-1.,-1.,-1.,-1.], which no rule produces from
#the C++ text.


class CppValue:
    """A default value that is not a plain Python number or bool: it knows its three renderings."""

    def __init__(self, cpp, python, document=None):
        self.cpp = cpp
        self.python = python
        self.document = document if document is not None else python

    def ToCpp(self):
        return self.cpp

    def ToPython(self):
        return self.python

    def ToDocument(self):
        return self.document

    def __repr__(self):
        return 'CppValue(' + repr(self.cpp) + ')'

    def __eq__(self, other):
        return isinstance(other, CppValue) and self.cpp == other.cpp

    def __hash__(self):
        return hash(self.cpp)


#the C++ types whose literals carry an f suffix; the suffix is a property of the type, not of the
#value, which is why it is never stored with the number
floatTypeNames = set(['float', 'UFloat', 'PFloat', 'Float3', 'Float4', 'StdArray33F'])


def CppFloatLiteral(value, typeName=''):
    """The C++ spelling of a Python float: 0. / 1. / -1., exponent form only where repr() uses it,
    and the f suffix taken from the declared type."""
    suffix = 'f' if str(typeName) in floatTypeNames else ''
    if value == int(value) and abs(value) < 1e16:
        return str(int(value)) + '.' + suffix

    text = repr(float(value))
    if 'e' in text:
        mantissa, exponent = text.split('e')
        sign = '-' if exponent.startswith('-') else ''
        text = mantissa + 'e' + sign + str(int(exponent.lstrip('+-')))

    return text + suffix


def CppLiteral(value, typeName=''):
    """The C++ literal for any default value."""
    if isinstance(value, CppValue):
        return value.ToCpp()
    if isinstance(value, bool):
        return 'true' if value else 'false'
    if isinstance(value, float):
        return CppFloatLiteral(value, typeName)
    if isinstance(value, int):
        return str(value)

    return str(value)


#------------------------------------------------ the Python and the document rendering (#2682)
#A default value has THREE renderings: the C++ literal above, the Python value that the generated
#itemInterface and the stubs need, and what a documentation table shows. A CppValue carries all
#three; a plain number, flag or string IS its own Python value; and a default written as C++
#SOURCE TEXT is translated by the tables below, dispatching on the SPELLING and on the declared
#TYPE.
#
#Nothing is guessed from a substring of the text. That was DefaultValue2Python and the
#isDefaultValue half of Str2Latex - two converters that disagreed - whose unconditional strip of
#the letter "f" turned Transformation66List() into Transormation66List() and whose blind
#"(" -> "[" put `Matrix[]`, `PyMatrixContainer[]` and `[ invalid [-1], ... ]` on the reference
#pages. A spelling that no rule covers raises UnknownDefaultValue instead of reaching a generated
#file wrong, and the message says which table to extend.

#the types whose default value is a literal string and not a C++ expression: the generator used to
#skip them by name, "don't do this for file names, because 'f' is erased!"
stringTypeNames = set(['String', 'FileName'])

#Name() - an empty container - as (Python, document). "None" means that the generated interface
#passes nothing and the C++ side builds the empty container itself
emptyContainerValues = {
    'Vector':               ('[]', '[]'),
    'ArrayIndex':           ('[]', '[]'),
    'ArrayFloat':           ('[]', '[]'),
    'Matrix':               ('[]', '[]'),
    'MatrixI':              ('[]', '[]'),
    'ResizableVector':      ('[]', '[]'),
    'ResizableMatrix':      ('[]', '[]'),
    'JointTypeList':        ('[]', '[]'),
    'PyMatrixContainer':    ('None', '[]'),
    'Vector2DList':         ('None', '[]'),
    'Vector3DList':         ('None', '[]'),
    'Vector6DList':         ('None', '[]'),
    'Matrix3DList':         ('None', '[]'),
    'Transformation66List': ('None', '[]'),
    'InertiaList':          ('None', '[]'),
    'BeamSection':          ('exudyn.BeamSection()', 'BeamSection()'),
    'BeamSectionGeometry':  ('exudyn.BeamSectionGeometry()', 'BeamSectionGeometry()'),
}

#an expression that is neither an empty container nor a braced initializer, as (Python, document)
namedDefaultValues = {
    'EXUmath::unitMatrix3D': ('IIDiagMatrix(rowsColumns=3,value=1)', '[[1,0,0], [0,1,0], [0,0,1]]'),
    'EXUmath::zeroMatrix3D': ('IIDiagMatrix(rowsColumns=3,value=0)', '[[0,0,0], [0,0,0], [0,0,0]]'),
    'Matrix6D(6,6,0.)': ('IIDiagMatrix(rowsColumns=6,value=0.)', 'np.zeros((6,6))'),
    #the one rotation matrix, whose own description says "in python use e.g.: [[1,0,0],[0,1,0],[0,0,1]]"
    'EXUmath::Matrix3DFToStdArray33(Matrix3DF(3,3,{1.f,0.f,0.f, 0.f,1.f,0.f, 0.f,0.f,1.f}))':
        ('[[1.,0.,0.], [0.,1.,0.], [0.,0.,1.]]', '[[1.,0.,0.], [0.,1.,0.], [0.,0.,1.]]'),
    #0 rather than the enum, so that an unset type gives a readable error and not an unreadable one
    'OutputVariableType::_None': ('0', 'OutputVariableType::_None'),
    #a C++ pointer member; it is not in the Python interface, and the page says what the C++ says
    'nullptr': ('nullptr', 'nullptr'),
}

#a name that appears INSIDE a braced initializer, as (Python, document)
innerDefaultValues = {
    'EXUstd::InvalidIndex': ('exudyn.InvalidIndex()', 'invalid (-1)'),
}

#the digits a float literal can end with, before its C++ "f"
_numberEnd = '0123456789.'


def _IsIdentifier(text):
    return text.isidentifier()


def _WithoutFloatSuffixes(text):
    """0.05f -> 0.05, {1.f,0.f} -> {1.,0.}: the f of a float literal belongs to the C++ TYPE and
    never to the value. A letter that belongs to a word is left alone, which is the whole
    difference to the str.replace('f', '') this replaces - it turned Transformation66List() into
    Transormation66List() and images/frame into images/rame."""
    characters = list(text)
    kept = []
    for (index, character) in enumerate(characters):
        following = characters[index + 1] if index + 1 < len(characters) else ''
        if (character in 'fF'
                and index > 0 and characters[index - 1] in _numberEnd
                and not (following.isalnum() or following == '_')):
            continue                                     #a float suffix: it goes
        kept.append(character)
    return ''.join(kept)


def _EmptyContainerName(text):
    """the name of Name(), or None"""
    if not text.endswith('()'):
        return None
    name = text[:-2]
    return name if _IsIdentifier(name) else None


def _BracedInitializer(text):
    """(name, inner) of Name({...}), or None"""
    if not text.endswith('})') or '({' not in text:
        return None
    name = text[:text.index('({')]
    return (name, text[len(name) + 2:-2]) if _IsIdentifier(name) else None


def _IsEnumValue(text):
    """Type::Value, which is legible as it is written"""
    parts = text.split('::')
    return len(parts) == 2 and all(_IsIdentifier(part) for part in parts)


def _IsNumber(text):
    """a number that a definition file wrote as a string, '1.' or '0.1f'"""
    try:
        float(text.rstrip('fF'))
    except ValueError:
        return False
    return True


class UnknownDefaultValue(Exception):
    """A default value written as C++ source text that no rule in definitionTypes covers."""


def _BracedRenderings(name, inner):
    """(Python, document) of Name({...}): the braces become a list, the f suffixes go, and a name
    inside is translated by innerDefaultValues. The inner spacing is the author's and is kept."""
    renderings = []
    for index in (0, 1):
        text = inner
        for (innerName, pair) in innerDefaultValues.items():
            text = text.replace(innerName, pair[index])
        renderings.append('[' + _WithoutFloatSuffixes(text) + ']')
    return (renderings[0], renderings[1])


def _SourceTextRenderings(text, typeName):
    """(Python, document) of a default value written as C++ source text."""
    if text in namedDefaultValues:
        return namedDefaultValues[text]

    name = _EmptyContainerName(text)
    if name is not None:
        if name not in emptyContainerValues:
            raise UnknownDefaultValue(
                'the empty container "' + name + '()" has no Python and no document rendering; add '
                'it to emptyContainerValues in definitions/definitionTypes.py')
        return emptyContainerValues[name]

    braced = _BracedInitializer(text)
    if braced is not None:
        return _BracedRenderings(braced[0], braced[1])

    if _IsEnumValue(text):
        return (text, text)                      #an enum value is legible as it is written

    raise UnknownDefaultValue(
        'the default value "' + text + '" (type ' + str(typeName) + ') is C++ source text that no '
        'rule covers; give the parameter a real Python value, or a CppValue that carries all three '
        'renderings, or add the expression to namedDefaultValues in definitions/definitionTypes.py')


def _Renderings(value, typeName):
    """(Python, document) for any default value."""
    if isinstance(value, CppValue):
        return (value.ToPython(), value.ToDocument())
    if isinstance(value, bool):
        return ('True', 'True') if value else ('False', 'False')
    if isinstance(value, float):
        text = CppFloatLiteral(value)            #the f suffix is the C++ type's, never Python's
        return (text, text)
    if isinstance(value, int):
        return (str(value), str(value))
    if value is Required or value is NoDefaultValue or value is None:
        return (str(value), str(value))

    if str(typeName) in stringTypeNames:
        return (str(value), str(value))          #a file name is a string, and the f stays in it

    text = str(value).strip()                    #a default written with a stray space is the same
    if text == '':                               #default; the space used to reach the signature
        return (text, text)
    if _IsNumber(text):                          #a number that a definition file wrote as a string
        return (_WithoutFloatSuffixes(text), _WithoutFloatSuffixes(text))

    return _SourceTextRenderings(text, typeName)


def PythonLiteral(value, typeName=''):
    """The Python value of any default value: what the generated interface and the stubs write."""
    return _Renderings(value, typeName)[0]


def DocumentLiteral(value, typeName=''):
    """What a documentation table shows as the default value."""
    return _Renderings(value, typeName)[1]


class _Required:
    """The value of a field that has to be given. It is the DEFAULT of every required argument, so
    a forgotten field is an error naming the class and the member, instead of an empty string that
    quietly reaches the generated code."""

    def __repr__(self):
        return 'REQUIRED'


Required = _Required()


class _NoDefaultValue:
    """Stated explicitly where a parameter genuinely has no default value - 319 of them, mostly
    substructures and strings. Being a value rather than an empty string, it cannot be confused
    with a default somebody forgot to write."""

    def __repr__(self):
        return 'NoDefaultValue'


NoDefaultValue = _NoDefaultValue()

#--------------------------------------------------------------------- types
#A type that NAMES A STRUCTURE defined in these files gets no constant: it refers to
#the definition itself, which definitionValidator.py checks exists. That is a stronger check than
#a constant (which only verifies spelling) and it removed 45 single-use names.
#
#A type is a str SUBCLASS carrying its constraints, so it compares and hashes exactly like
#the plain name the generators look up (tools/generators/typeModel.py)
#- nothing downstream has to change - while minimum, greaterThan and size travel with it.
#Calling a base type applies a constraint and yields the constrained type:
#
#   TReal                  -> Real          TIndex                -> Index
#   TReal(minimum=0)       -> UReal         TIndex(minimum=0)     -> UInt
#   TReal(greaterThan=0)   -> PReal         TIndex(greaterThan=0) -> PInt
#   TVectorND(3)           -> Vector3D      TIndex(ItemNode)      -> NodeIndex
#   TMatrixND(3, 3)        -> Matrix3D      TArrayIndex(ItemMarker, size=2)
#
#Both range predicates are needed and stay distinct: >= 0 and > 0 are both in use, and the
#generators select CheckForValidUReal vs CheckForValidPReal on exactly that difference.
#Only sizes for which a C++ type exists are accepted, so TVectorND(5) fails at import time
#instead of reaching the compiler; the same call also states the shape, which is why members
#no longer carry a separate size= - it was never checked anyway, see the generator's own note
#"future: also add size check ..." at pythonAutoGenerateObjects.py:904.


class TypeSpec(str):
    """A type name plus the constraints that belong to it."""

    def __new__(cls, name, minimum=None, greaterThan=None, size=None, itemKind=None,
                constrained=None):
        self = str.__new__(cls, name)
        self.minimum = minimum
        self.greaterThan = greaterThan
        self.size = size
        self.itemKind = itemKind
        self.constrainedForms = constrained or {}

        return self

    def __call__(self, itemKind=None, minimum=None, greaterThan=None, size=None):
        if minimum is not None and greaterThan is not None:
            raise ValueError(str(self) + ": give minimum or greaterThan, not both")

        key = itemKind
        if minimum is not None:
            key = "minimum"
        elif greaterThan is not None:
            key = "greaterThan"
        if key is not None and key not in self.constrainedForms:
            raise ValueError(str(self) + " has no " + repr(key) + " form; available: "
                             + ", ".join(sorted(self.constrainedForms)))
        name = self.constrainedForms[key] if key is not None else str(self)

        return TypeSpec(name, minimum=minimum, greaterThan=greaterThan, size=size,
                        itemKind=itemKind, constrained=self.constrainedForms)


#--------------------------------------------------------------------- item kinds
#which index family a type belongs to; an ObjectIndex converts to Index in Python but not
#to a NodeIndex, which is what stops the most common class of user mistake
ItemNode             = 'Node'
ItemObject           = 'Object'
ItemMarker           = 'Marker'
ItemLoad             = 'Load'
ItemSensor           = 'Sensor'

#--------------------------------------------------------------------- scalars with ranges
TReal                = TypeSpec('Real', constrained={'greaterThan': 'PReal', 'minimum': 'UReal'})
Tfloat               = TypeSpec('float', constrained={'greaterThan': 'PFloat', 'minimum': 'UFloat'})
TIndex               = TypeSpec('Index', constrained={'Load': 'LoadIndex', 'Marker': 'MarkerIndex', 'Node': 'NodeIndex', 'Object': 'ObjectIndex', 'Sensor': 'SensorIndex', 'greaterThan': 'PInt', 'minimum': 'UInt'})
TArrayIndex          = TypeSpec('ArrayIndex', constrained={'Marker': 'ArrayMarkerIndex', 'Node': 'ArrayNodeIndex', 'Object': 'ArrayObjectIndex', 'Sensor': 'ArraySensorIndex'})

#--------------------------------------------------------------------- shapes
#the sizes for which a C++ type actually exists; anything else is a typo
vectorSizes          = {2: 'Vector2D', 3: 'Vector3D', 4: 'Vector4D',
                        6: 'Vector6D', 7: 'Vector7D', 9: 'Vector9D'}
matrixSizes          = {(2, 2): 'Matrix2D', (3, 3): 'Matrix3D', (6, 6): 'Matrix6D'}
indexTupleSizes      = {2: 'Index2', 4: 'Index4'}
nodeIndexTupleSizes  = {2: 'NodeIndex2', 3: 'NodeIndex3', 4: 'NodeIndex4'}


def _sized(table, key, what):
    if key not in table:
        raise ValueError(what + " " + repr(key) + " has no C++ type; available: "
                         + ", ".join([repr(k) for k in sorted(table)]))

    return table[key]


def TVectorND(n):
    """A fixed-size vector; only sizes with a C++ type are allowed - see vectorSizes above.
    Any other size raises, so a Vector5D fails here rather than in the compiler."""

    return TypeSpec(_sized(vectorSizes, n, "vector size"), size=n)


def TMatrixND(rows, columns):
    """A fixed-size matrix; only shapes with a C++ type are allowed - see matrixSizes above.
    Any other shape raises."""

    return TypeSpec(_sized(matrixSizes, (rows, columns), "matrix shape"),
                    size=(rows, columns))


def TIndexND(n, itemKind=None):
    """A fixed-size tuple of indices: Index2/Index4, or NodeIndex2/3/4 for node numbers."""
    table = nodeIndexTupleSizes if itemKind == ItemNode else indexTupleSizes

    return TypeSpec(_sized(table, n, "index tuple size"), size=n, itemKind=itemKind)


#--------------------------------------------------------------------- one-off types used by both
#types with no family - user-function signatures, containers, EXUDYN structs; most-used first
Tbool                              = TypeSpec('bool')
Tvoid                              = TypeSpec('void')
TString                            = TypeSpec('String')
TFloat4                            = TypeSpec('Float4')
TNumpyVector                       = TypeSpec('NumpyVector')
TNumpyMatrix                       = TypeSpec('NumpyMatrix')
TOutputVariableType                = TypeSpec('OutputVariableType')
TResizableVector                   = TypeSpec('ResizableVector')



#--------------------------------------------------------------------- the types of a user function
#A user function is written in a definition file as an ordinary Python function - a real def, with
#real annotations and a real docstring - and passed to the ItemParameter it belongs to
#(#2664). These names exist so that such a file stays ordinary Python:
#importable, readable in an editor, and not a string that has to be escaped.
#
#They are NOT re-implementations. The real MainSystem is in the compiled module; this is a name for
#an annotation, and what the documentation prints is the annotation as it is WRITTEN, read from the
#source with ast. The SIZE of an argument is not part of its type: it belongs in the argument's line
#of the docstring, as a formula, where it renders (maintainer, 2026-09-25).
import numpy as np                                                    # noqa: E402

Real = float
Index = int
Bool = bool


class MainSystem:
    """the MainSystem a user function is called with; the class itself is in the compiled module"""


class BodyGraphicsData:
    """the list of graphics dictionaries a graphics user function returns"""


class MatrixContainer:
    """a dense or sparse matrix, as the C++ interface exchanges it"""


#the array types. A user function receives and returns numpy arrays; these names say the SIZE where
#the size is fixed, because that is what the argument table of a user function has always said and
#what the C++ signature needs (StdVector3D and StdVector are not the same argument). A size that is
#not fixed is a formula in the argument's description instead.
Vector = np.ndarray
Vector2D = np.ndarray
Vector3D = np.ndarray
Vector6D = np.ndarray
Matrix3D = np.ndarray
Matrix6D = np.ndarray
NumpyMatrix = np.ndarray
Array = np.ndarray


class ConfigurationType:
    """exudyn.ConfigurationType"""


#numpy is imported for np.ndarray in an annotation; it is the package's only mandatory dependency
_ndarray = np.ndarray


#%%************************************************************************************************
def _member(kind, fields):
    """Common body: check the required fields, record which constructor was used, and default
    cplusplusName to pythonName - which is what the old format meant by leaving that column
    empty."""
    fields = dict(fields)
    fields['kind'] = kind
    for name, value in sorted(fields.items()):
        if isinstance(value, _Required):
            raise ValueError(kind + ' ' + repr(fields.get('pythonName', '?')) + ': '
                             + name + ' is required and was not given')
    if not fields.get('cplusplusName', ''):
        fields['cplusplusName'] = fields.get('pythonName', '')

    return fields


#==================================================================================================
# ITEMS - nodes, objects, markers, loads, sensors
#==================================================================================================

#--------------------------------------------------------------------- destinations (items)
#combine with +, e.g. DestComp+DestParam
DestMain             = 'M'   #Main object
DestComp             = 'C'   #computational object
DestVisu             = 'V'   #visualization object
DestParam            = 'P'   #parameter structure

#--------------------------------------------------------------------- flags (items)
CFReadOnly           = 'R'   #read only; functions are always read only
CFConst              = 'C'   #const member function
CFMutable            = 'U'   #mutable: may be modified in const functions (temporary vectors)
CFNoInterface        = 'n'   #EXCLUDED from the Python dictionary interface. Inverted on
                             #purpose: 947 of 985 parameters are in the interface and 38
                             #are not - all of them temporaries or computed state - so
                             #stating the exception is shorter and says more. The letter
                             #is not one of the old format's; definitionLoader.py translates.
                             #NOTE: this flag is meaningless on a function. The generator
                             #reads the interface flag only inside a block guarded by
                             #lineType 'V' (pythonAutoGenerateObjects.py:1123), so the 'I'
                             #that 1755 of 1850 function rows carried never had an effect.
CFMustBeGiven        = 'Q'   #the default is only a placeholder outside the parameter's range
                             #(InvalidIndex for UInt, 0 for PReal, ...): Add<Kind> raises if it
                             #is not replaced; the validator requires the flag exactly there

#--------------------------------------------------------------------- default values (items)
DVInvalidIndex = CppValue('EXUstd::InvalidIndex', 'exudyn.InvalidIndex()',
                         'invalid (-1)')
DVDefaultColor = CppValue('Float4({-1.f,-1.f,-1.f,-1.f})', '[-1.,-1.,-1.,-1.]')
DVZeroVector3D = CppValue('Vector3D({0.,0.,0.})', '[0.,0.,0.]')

#--------------------------------------------------------------------- parent classes (items)
#CLOSED SETS. A new parent class cannot be introduced by editing a definition file: it needs
#hand-written C++ as well. So a free string here would buy nothing and hide a typo.
ParentClassCObject                  = 'CObject'
ParentClassCObjectBody              = 'CObjectBody'
ParentClassCObjectConnector         = 'CObjectConnector'
ParentClassCObjectConstraint        = 'CObjectConstraint'
ParentClassCObjectSuperElement      = 'CObjectSuperElement'
ParentClassCObjectANCFCable2DBase   = 'CObjectANCFCable2DBase'
ParentClassCNodeODE1                = 'CNodeODE1'
ParentClassCNodeODE2                = 'CNodeODE2'
ParentClassCNodeAE                  = 'CNodeAE'
ParentClassCNodeData                = 'CNodeData'
ParentClassCNodeRigidBody           = 'CNodeRigidBody'
ParentClassCMarker                  = 'CMarker'
ParentClassCLoad                    = 'CLoad'
ParentClassCSensor                  = 'CSensor'

MainParentClassMainObject           = 'MainObject'
MainParentClassMainObjectBody       = 'MainObjectBody'
MainParentClassMainObjectConnector  = 'MainObjectConnector'
MainParentClassMainNode             = 'MainNode'
MainParentClassMainMarker           = 'MainMarker'
MainParentClassMainLoad             = 'MainLoad'
MainParentClassMainSensor           = 'MainSensor'

VisuParentClassVisualizationObject             = 'VisualizationObject'
VisuParentClassVisualizationObjectSuperElement = 'VisualizationObjectSuperElement'
VisuParentClassVisualizationNode               = 'VisualizationNode'
VisuParentClassVisualizationMarker             = 'VisualizationMarker'
VisuParentClassVisualizationLoad               = 'VisualizationLoad'
VisuParentClassVisualizationSensor             = 'VisualizationSensor'

#--------------------------------------------------------------------- class and object types
#CLOSED SETS as well; classType additionally selects the itemDefs file a definition belongs in.
ClassTypeNode        = 'Node'
ClassTypeObject      = 'Object'
ClassTypeMarker      = 'Marker'
ClassTypeLoad        = 'Load'
ClassTypeSensor      = 'Sensor'

ObjectTypeObject        = 'Object'
ObjectTypeBody          = 'Body'
ObjectTypeConnector     = 'Connector'
ObjectTypeConstraint    = 'Constraint'
ObjectTypeJoint         = 'Joint'
ObjectTypeFiniteElement = 'FiniteElement'
ObjectTypeSuperElement  = 'SuperElement'

#--------------------------------------------------------------------- one-off types used only by items
TBool                              = TypeSpec('Bool')
TCObjectType                       = TypeSpec('CObjectType')
TVector                            = TypeSpec('Vector')
TPyMatrixContainer                 = TypeSpec('PyMatrixContainer')
TResizableMatrix                   = TypeSpec('ResizableMatrix')
TSensorType                        = TypeSpec('SensorType')
TBodyGraphicsData                  = TypeSpec('BodyGraphicsData')
TVector3DList                      = TypeSpec('Vector3DList')
TVector6DList                      = TypeSpec('Vector6DList')
TLinkedDataVector                  = TypeSpec('LinkedDataVector')
TLoadType                          = TypeSpec('LoadType')
TMatrix3DList                      = TypeSpec('Matrix3DList')
TNumpyMatrixI                      = TypeSpec('NumpyMatrixI')
TPyFunctionGraphicsData            = TypeSpec('PyFunctionGraphicsData')
TPyFunctionMbsScalarIndexScalar5   = TypeSpec('PyFunctionMbsScalarIndexScalar5')
TPyFunctionVectorMbsScalarIndex2Vector = TypeSpec('PyFunctionVectorMbsScalarIndex2Vector')
TPyFunctionVector3DmbsScalarVector3D = TypeSpec('PyFunctionVector3DmbsScalarVector3D')
TTransformation66List              = TypeSpec('Transformation66List')
TBeamSection                       = TypeSpec('BeamSection')
THomogeneousTransformation         = TypeSpec('HomogeneousTransformation')
TPyFunctionMatrixMbsScalarIndex2Vector = TypeSpec('PyFunctionMatrixMbsScalarIndex2Vector')
TPyFunctionMbsScalarIndexScalar    = TypeSpec('PyFunctionMbsScalarIndexScalar')
TPyFunctionMbsScalarIndexScalar9   = TypeSpec('PyFunctionMbsScalarIndexScalar9')
TPyFunctionVector6DmbsScalarIndexVector6D = TypeSpec('PyFunctionVector6DmbsScalarIndexVector6D')
TAccessFunctionType                = TypeSpec('AccessFunctionType')
TBodyGraphicsDataList              = TypeSpec('BodyGraphicsDataList')
TCNodeGroup                        = TypeSpec('CNodeGroup')
TInertiaList                       = TypeSpec('InertiaList')
TJointTypeList                     = TypeSpec('JointTypeList')
TPyFunctionMatrixContainerMbsScalarIndex2Vector = TypeSpec('PyFunctionMatrixContainerMbsScalarIndex2Vector')
TPyFunctionMatrixContainerMbsScalarIndex2Vector2Scalar = TypeSpec('PyFunctionMatrixContainerMbsScalarIndex2Vector2Scalar')
TPyFunctionMatrixContainerMbsScalarIndex2VectorBool = TypeSpec('PyFunctionMatrixContainerMbsScalarIndex2VectorBool')
TPyFunctionMbsScalar2              = TypeSpec('PyFunctionMbsScalar2')
TPyFunctionMbsScalarIndexScalar11  = TypeSpec('PyFunctionMbsScalarIndexScalar11')
TPyFunctionVector3DmbsScalarIndexScalar4Vector3D = TypeSpec('PyFunctionVector3DmbsScalarIndexScalar4Vector3D')
TPyFunctionVector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D = TypeSpec('PyFunctionVector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D')
TPyFunctionVectorMbsScalarArrayIndexVectorConfiguration = TypeSpec('PyFunctionVectorMbsScalarArrayIndexVectorConfiguration')
TPyFunctionVectorMbsScalarIndex2VectorBool = TypeSpec('PyFunctionVectorMbsScalarIndex2VectorBool')
TPyFunctionVectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D = TypeSpec('PyFunctionVectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D')
TPyFunctionVectorMbsScalarIndexVector = TypeSpec('PyFunctionVectorMbsScalarIndexVector')
TSTDstring                         = TypeSpec('STDstring')

#--------------------------------------------------------------------- user-function signatures
#the C++ std::function each PyFunction... type stands for; the generators render the stored
#member (PythonUserFunctionBase< ... >), the Python interface and the docs from it. Names must
#start with 'PyFunction'
userFunctionSignatures = {'KeyPressUserFunction': 'std::function<bool(int, int, int)>', #renderer key press (VisualizationSettings.interactive)
                          #for MainSystem => see other MainSystemUserFunctions
                          'PyFunctionBoolMbsScalar': 'std::function<bool(const MainSystem&,Real)>',#PreStepUserFunction, PostStepUserFunction
                          'PyFunctionVector2DMbsScalar': 'std::function<StdVector2D(const MainSystem&,Real)>',#PreStepUserFunction, PostStepUserFunction
                          #for items:
                          'PyFunctionGraphicsData': 'std::function<py::object(const MainSystem&,Index)>',
                          'PyFunctionMbsScalar2': 'std::function<Real(const MainSystem&,Real,Real)>',#LoadCoordinate
                          'PyFunctionVector3DmbsScalarVector3D': 'std::function<StdVector3D(const MainSystem&,Real,StdVector3D)>', #LoadForceVector, LoadTorqueVector, LoadMassProportional
                          'PyFunctionMbsScalarIndexScalar': 'std::function<Real(const MainSystem&,Real,Index,Real)>', #ConnectorCoordinate
                          'PyFunctionMbsScalarIndexScalar5': 'std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real)>', #ConnectorSpringDamper, CoordinateSpringDamper, several others
                          'PyFunctionMbsScalarIndexScalar9': 'std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real)>', #ANCFCable2D
                          'PyFunctionMbsScalarIndexScalar11': 'std::function<Real(const MainSystem&,Real,Index,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real,Real)>', #CoordinateSpringDamperExt
                          'PyFunctionVector6DmbsScalarIndexVector6D': 'std::function<StdVector6D(const MainSystem&,Real,Index,StdVector6D)>', #GenericJoint
                          'PyFunctionVector3DmbsScalarIndexScalar4Vector3D': 'std::function<StdVector3D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdVector3D)>', #CartesianSpringDamper
                          'PyFunctionVectorMbsScalarIndex2Vector': 'std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector)>', #ObjectGenericODE2, ObjectFFRF...
                          'PyFunctionMatrixMbsScalarIndex2Vector': 'std::function<NumpyMatrix(const MainSystem&,Real,Index,StdVector,StdVector)>', #ObjectGenericODE2, ObjectFFRF...
                          'PyFunctionMatrixContainerMbsScalarIndex2Vector': 'std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector)>', #ObjectGenericODE2 #changed from PyFunctionMatrixMbsScalarIndex2Vector 2021-09-27
                          'PyFunctionMatrixContainerMbsScalarIndex2Vector2Scalar': 'std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector,Real,Real)>', #ObjectGenericODE2 #Jacobian
                          'PyFunctionVectorMbsScalarIndexVector': 'std::function<StdVector(const MainSystem&,Real,Index,StdVector)>', #ObjectGenericODE1
                          'PyFunctionVector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D': 'std::function<StdVector6D(const MainSystem&,Real,Index,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)>', #RigidBodySpringDamper
                          'PyFunctionVectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D': 'std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdMatrix6D,StdMatrix6D,StdMatrix3D,StdMatrix3D,StdVector6D)>', #RigidBodySpringDamper, postNewtonStep
                          'PyFunctionVectorMbsScalarIndex2VectorBool' : 'std::function<StdVector(const MainSystem&,Real,Index,StdVector,StdVector,bool)>', #CoordinateVectorConstraint
                          'PyFunctionMatrixContainerMbsScalarIndex2VectorBool': 'std::function<py::object(const MainSystem&,Real,Index,StdVector,StdVector,bool)>', #CoordinateVectorConstraint
                          'PyFunctionVectorMbsScalarArrayIndexVectorConfiguration': 'std::function<StdVector(const MainSystem&,Real,StdArrayIndex,StdVector,ConfigurationType)>', #SensorUserFunction
#StdVector3D=std::array<Real,3> does not accept numpy::array                            'PyFunctionVector3DScalarVector3D': 'std::function<StdVector3D(Real,StdVector3D)>', #LoadForceVector, LoadTorqueVector, LoadMassProportional
                          }


#%%************************************************************************************************
def ItemParameter(type=Required, destination=Required, pythonName=Required,
                  defaultValue=Required, description=Required,
                  cFlags='', size='', args='', cplusplusName='',
                  fromParent=False, deprecated=None, userFunction=None, userFunctionExample=None):
    #userFunction: for a parameter that IS a user function, the Python def that says what its
    #arguments are called, what they are, and what they mean - see the header of
    #tools/generators/userFunctionModel.py (#2664)
    #userFunctionExample: the Python code shown under the generated block, as text; it is a script
    #and not a function, so it cannot be a def, and it is fenced as python by the emitter
    return _member('ItemParameter', locals())


#%%************************************************************************************************
def ItemFunction(type=Required, destination=Required, pythonName=Required,
                 description=Required,
                 cFlags='', implementation=None, args='', size='', cplusplusName='',
                 isVirtual=True, isStatic=False):
    return _member('ItemFunction', locals())


#%%************************************************************************************************
def ItemOutputVariable(outputVariable, description):
    """One output variable an item provides. outputVariable is a constant from
    outputVariableTypes.py (OVPosition, OVForceLocal, ...), so a typo is a NameError here rather
    than a key that silently never matches; description is the text shown in the reference
    tables, and may be one of the shared texts in outputVariableDescriptions.py."""

    return {'outputVariable': outputVariable, 'description': description}


#%%************************************************************************************************
def ItemFunctionLib(pythonName, type, destination, classType=None, parentClass=None,
                    description=Required, cFlags='', args='', implementation=None,
                    isVirtual=True, isStatic=False):
    """One entry of definitions/itemFunctions.py: the declaration that items sharing this function
    would otherwise each restate, plus the description they share.

    classType is None where the declaration is the same for every item type - UpdateGraphics and
    CheckPreAssembleConsistency - and parentClass is None where it is the same for every parent
    within that item type, which is the usual case. Either is named only where the item type or
    the parent actually changes the declaration."""

    if isinstance(description, _Required):
        raise ValueError('ItemFunctionLib ' + repr(pythonName) + ': description is required'
                         + ' and was not given')

    return {'classType': classType, 'parentClass': parentClass, 'pythonName': pythonName,
            'type': type, 'destination': destination, 'description': description,
            'cFlags': cFlags, 'args': args, 'implementation': implementation,
            'isVirtual': isVirtual, 'isStatic': isStatic}


#%%************************************************************************************************
def ItemFunctionDef(pythonName, implementation=None, description=None,
                    destination=None, cFlags=None, args=None, cplusplusName=''):
    """Use site: this item overrides a function whose declaration is in the library. The entry is
    found from the class this member belongs to - its classType and cParentClass - and the name.

    destination, cFlags and args default to None and are given only where the name alone is
    ambiguous: the const and non-const halves of an accessor pair, two argument lists under one
    name, and the names used for both a computation and a visualization function. A missing one
    is an error listing the alternatives, not a silent pick.

    description is None unless the text is genuinely item-specific; implementation is None for a
    declaration the .cpp defines, '' for an empty body, and a string for a body."""

    return {'kind': 'ItemFunctionRef', 'pythonName': pythonName,
            'implementation': implementation, 'description': description,
            'destination': destination, 'cFlags': cFlags, 'args': args,
            'cplusplusName': cplusplusName}


def _TypeSumImplementation(cppName, types, conditional=()):
    """C++ body returning a bit combination of the enum cppName ('Node::Type', 'Marker::Type' or
    'AccessFunctionType'), validated against definitions/enumTypes.py"""
    import enumTypes
    valueNames = [value.name for enumType in enumTypes.enumTypes if enumType.cppName == cppName
                  for value in enumType.values]
    if not valueNames:
        raise ValueError('no enum ' + repr(cppName) + ' in definitions/enumTypes.py')
    for name in list(types) + [c[0] for c in conditional]:
        if name == '_None' or name not in valueNames:
            raise ValueError(repr(name) + ' is not a value of ' + cppName)
    scope = cppName[:-len('::Type')] if cppName.endswith('::Type') else cppName
    terms = ['(Index)' + scope + '::' + name for name in types]
    terms += ['(parameters.' + parameter + ' != 0)*(Index)' + scope + '::' + name for name, parameter in conditional]
    if not terms:
        return 'return ' + scope + '::_None;'
    if len(terms) == 1 and not conditional:
        return 'return ' + scope + '::' + types[0] + ';'
    return 'return (' + cppName + ')(' + ' + '.join(terms) + ');'


def ItemRequestedTypes(kind, types, conditional=(), description=None):
    """Use site: the node or marker types an object (or load) requires, as a declared list instead
    of C++ written into the definition. kind is 'Node' or 'Marker'; types are
    value names of Node::Type / Marker::Type in definitions/enumTypes.py, combined as bits; an empty
    list is _None (no single type can be required). conditional holds (typeName, parameterName)
    pairs: the type is added when that parameter is not zero - the only condition the tree needs
    (ObjectContactSphereSphere, ObjectContactSphereTriangle: Orientation if dynamicFriction != 0).
    Expands to GetRequested<kind>Type from definitions/itemFunctions.py with a generated body."""
    if kind not in ('Node', 'Marker'):
        raise ValueError('ItemRequestedTypes: kind must be Node or Marker, not ' + repr(kind))
    member = ItemFunctionDef('GetRequested' + kind + 'Type',
                             implementation=_TypeSumImplementation(kind + '::Type', types, conditional),
                             description=description)
    member['requestedTypes'] = list(types)
    member['conditionalTypes'] = [tuple(c) for c in conditional]
    return member


def ItemTypes(kind, types, description):
    """Use site: the type bits of a node or marker (GetType), as a declared list (revision plan step
    83c); kind is 'Node' or 'Marker', types are value names of Node::Type / Marker::Type"""
    if kind not in ('Node', 'Marker'):
        raise ValueError('ItemTypes: kind must be Node or Marker, not ' + repr(kind))
    member = ItemFunction(type=kind + '::Type', destination=DestComp, cFlags=CFConst,
                          pythonName='GetType',
                          implementation=_TypeSumImplementation(kind + '::Type', types),
                          description=description)
    member['itemTypes'] = list(types)
    return member


def ItemAccessFunctionTypes(types, description=None, bodyMarkers=True):
    """Use site: the access functions an object provides for markers and loads (GetAccessFunctionTypes),
    as a declared list of AccessFunctionType value names; a marker with
    Position (Orientation) needs TranslationalVelocity_qt (AngularVelocity_qt).
    bodyMarkers=False: the types serve the object's own markers (super element, kinematic tree) and the
    general body markers do not work on it, although the types would admit them (#2734)"""
    member = ItemFunctionDef('GetAccessFunctionTypes',
                             implementation=_TypeSumImplementation('AccessFunctionType', types),
                             description=description)
    member['accessFunctionTypes'] = list(types)
    member['bodyMarkers'] = bodyMarkers
    return member


_functionLibrary = None


def _Library():
    """Loaded on first use, so definitionTypes.py and itemFunctions.py do not import each other."""
    global _functionLibrary
    if _functionLibrary is None:
        import itemFunctions
        _functionLibrary = itemFunctions.itemFunctionLibrary

    return _functionLibrary


def _ResolveFunctionReference(reference, className, classType, parentClass):
    candidates = [e for e in _Library()
                  if e['pythonName'] == reference['pythonName']
                  and (e['classType'] is None or e['classType'] == classType)
                  and (e['parentClass'] is None or e['parentClass'] == parentClass)]
    for field in ('destination', 'cFlags', 'args'):
        if reference[field] is not None:
            candidates = [e for e in candidates if e[field] == reference[field]]

    where = className + '.' + reference['pythonName']
    if not candidates:
        raise ValueError(where + ': no entry in definitions/itemFunctions.py for classType '
                         + repr(classType) + ' and parent class ' + repr(parentClass)
                         + ' - add one, or write the function out with ItemFunction(...)')
    if len(candidates) > 1:
        raise ValueError(where + ': ' + str(len(candidates)) + ' library entries match; say which'
                         + ' with destination=, cFlags= or args=. Alternatives: '
                         + '; '.join(repr(e['type']) + ' destination='
                                     + repr(e['destination']) + ' cFlags=' + repr(e['cFlags'])
                                     + ' args=' + repr(e['args']) for e in candidates))

    entry = candidates[0]
    member = dict(entry)
    member.pop('classType')
    member.pop('parentClass')
    member['kind'] = 'ItemFunction'
    member['implementation'] = reference['implementation']
    if reference['description'] is not None:
        member['description'] = reference['description']
    member['cplusplusName'] = reference['cplusplusName'] or reference['pythonName']
    for key in ('requestedTypes', 'conditionalTypes', 'accessFunctionTypes', 'bodyMarkers'): #declared lists
        if key in reference:
            member[key] = reference[key]

    return member


#%%************************************************************************************************
def ItemKindDefinition(kind, overallDescription, detailedDescription=''):
    """Use site: what all items of one kind have in common - the page of the kind in the reference
    manual (#2725). kind is the name of that page: 'Nodes', 'Objects (Body)', ..., 'Sensors';
    overallDescription is the paragraph under its heading, detailedDescription the general section
    that every item of the kind refers to"""
    return {'kind': kind, 'overallDescription': overallDescription,
            'detailedDescription': detailedDescription}


#%%************************************************************************************************
def ItemDefinition(className, members, **header):
    #a member written as ItemFunctionDef(...) is expanded here, where the class this member
    #belongs to is known - the library entry is found from its classType and cParentClass
    members = [_ResolveFunctionReference(m, className, header.get('classType', ''),
                                         header.get('cParentClass', ''))
               if m.get('kind') == 'ItemFunctionRef' else m
               for m in members]
    header['className'] = className
    header['members'] = members

    return header


#==================================================================================================
# STRUCTURES - settings, solver and system structures
#==================================================================================================

#--------------------------------------------------------------------- flags (structures)
SFNoDictType         = 'D'   #no dictionary with type info - NOTE: the legend gives D twice, also
                             #as "definition only"; the generator decides by context
#NOTE: there is no SFSubstructure constant. 'substructure' is DERIVED - it means exactly
#      'the type names one of the structures defined in these files', which held for all 71
#      of them, so declaring it as well was a second statement of the same fact.
SFReturnCopy         = 'V'   #return value policy: copy
SFPybindArgs         = 'G'   #add args for pybind
SFConst              = 'C'   #const function
SFNoPybind           = 'N'   #not in the Python interface (pybind11, stubs, dictionaries, docs); members
                             #without it are in the interface - the generators read that as 'P'
SFDeprecated         = 'X'   #deprecated; the description links to the relocated value

class Deprecated:
    """When a member was deprecated and when it is to be removed. This used to be stored in
    defaultValue as the string 'version;EXP=year' - a deprecated member has no default value, so
    the field was free - and the generator that reads it says "workaround" in its own comment
    (pythonAutoGenerateSystemStructures.py:149). Two facts in one string, parsed by splitting on
    a semicolon, are now two fields."""

    def __init__(self, since, expires):
        self.since = since
        self.expires = expires

    def ToCpp(self):
        """the single string the old format stored"""
        return str(self.since) + ';EXP=' + str(self.expires)

    def __repr__(self):
        return 'Deprecated(' + repr(self.since) + ', ' + repr(self.expires) + ')'


#--------------------------------------------------------------------- one-off types used only by structures
TFloat3                            = TypeSpec('Float3')
TResizableVectorParallel           = TypeSpec('ResizableVectorParallel')
TFileName                          = TypeSpec('FileName')
TInt                               = TypeSpec('Int')
TGeneralMatrixEXUdense             = TypeSpec('GeneralMatrixEXUdense')
TGeneralMatrixEigenSparse          = TypeSpec('GeneralMatrixEigenSparse')
TLinearSolverType                  = TypeSpec('LinearSolverType')
TKeyPressUserFunction              = TypeSpec('KeyPressUserFunction')
TStdArray33F                       = TypeSpec('StdArray33F')
TArrayFloat                        = TypeSpec('ArrayFloat')
TCSolverExplicitTimeInt            = TypeSpec('CSolverExplicitTimeInt')
TCSolverImplicitSecondOrderTimeIntUserFunction = TypeSpec('CSolverImplicitSecondOrderTimeIntUserFunction')
TCSolverStatic                     = TypeSpec('CSolverStatic')
TCrossSectionType                  = TypeSpec('CrossSectionType')
TDynamicSolverType                 = TypeSpec('DynamicSolverType')
TItemType                          = TypeSpec('ItemType')
TTemporaryComputationData          = TypeSpec('TemporaryComputationData')
TTemporaryComputationDataArray     = TypeSpec('TemporaryComputationDataArray')
TVector2DList                      = TypeSpec('Vector2DList')


#%%************************************************************************************************
def StructureParameter(type=Required, pythonName=Required, defaultValue=Required,
                       description=Required,
                       cFlags='', size='', args='', cplusplusName='',
                       isLinked=False, fromParent=False, deprecated=None,
                       memberDefaults=None):
    """one member of a structure

    memberDefaults is only for a member whose type is another structure, and only where THIS
    instance starts from other values than the structure's own defaults: {subMemberName: value},
    each value written exactly as that sub-member's defaultValue would be. It is what makes
    raytracer.material1 a green matt material and openGL.light2 a disabled one, instead of a C++
    constructor somewhere setting them afterwards.
    """
    return _member('StructureParameter', locals())


#%%************************************************************************************************
def StructureFunction(type=Required, pythonName=Required, description=Required,
                      cFlags='', implementation=None, args='', size='', cplusplusName='',
                      isVirtual=True, isLinked=False):
    return _member('StructureFunction', locals())


#%%************************************************************************************************
def StructureDefinition(className, members, **header):
    header['className'] = className
    header['members'] = members

    return header
