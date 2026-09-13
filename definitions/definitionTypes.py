#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Shared vocabulary and constructors for the Exudyn definition files.
#
# Details:  HAND-WRITTEN and reviewed. This is the SINGLE definition of the flag letters, the
#           destinations, the closed-set header values and the member constructors.
#
#           Why hand-written: the flag letters used to have no central definition at all - the
#           old generator tested them as bare literals (parameter['cFlags'].find('I') and
#           friends, 16 such sites in pythonAutoGenerateObjects.py), and the emitter carried its
#           own copy of the table. Two independent transcriptions of the same vocabulary can
#           drift with nothing to notice. So the table lives here, once;
#           tools/generators/definitionEmitter.py imports it and owns no table of its own, and
#           it FAILS if the data uses a letter, a type or a closed-set value that is missing
#           here, naming what to add. Drift becomes an error instead of a silent difference.
#
#           Why constants and not strings: a flag spelled as a bare letter is a value that
#           nothing checks, and a mistyped letter changed the build silently. As a name it is a
#           NameError at import, and an editor can complete it.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-13 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

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
CFInterface          = 'I'   #dictionary interface
CFDeclarationOnly    = 'D'   #declaration only; implementation written by hand in the .cpp
CFOptional           = 'O'   #optional parameter in the dictionary; otherwise the default

#--------------------------------------------------------------------- flags (structures)
SFNoDictType         = 'D'   #no dictionary with type info - NOTE: the legend gives D twice, also
                             #as "definition only"; the generator decides by context
#NOTE: there is no SFSubstructure constant. 'substructure' is DERIVED - it means exactly
#      'the type names one of the structures defined in these files', which held for all 71
#      of them, so declaring it as well was a second statement of the same fact.
SFReturnCopy         = 'V'   #return value policy: copy
SFPybindArgs         = 'G'   #add args for pybind
SFConst              = 'C'   #const function
SFPybind             = 'P'   #write the pybind11 interface
SFDeprecated         = 'X'   #deprecated; the description links to the relocated value

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


DVInvalidIndex = CppValue('EXUstd::InvalidIndex', 'exudyn.InvalidIndex()', 'invalid index')
DVDefaultColor = CppValue('Float4({-1.f,-1.f,-1.f,-1.f})', '[-1.,-1.,-1.,-1.]',
                          'default colour (RGBA -1 means: use the default)')
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
#CLOSED SETS as well; classType additionally selects the file a definition is emitted into.
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

#--------------------------------------------------------------------- types
#A type that NAMES A STRUCTURE defined in these files gets no constant: it refers to
#the definition itself, which the emitter checks exists. That is a stronger check than
#a constant (which only verifies spelling) and it removed 45 single-use names.
#
#A type is a str SUBCLASS carrying its constraints, so it compares and hashes exactly like
#the plain name the generators already look up in typeConversion / typeCasts / type2PyTyping
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
#Only sizes for which a C++ type exists are accepted, so TVectorND(5) fails at emit time
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


#--------------------------------------------------------------------- remaining C++ types
#one-off types with no family: user-function signatures, containers and EXUDYN structs
TAccessFunctionType                = TypeSpec('AccessFunctionType')
TArrayFloat                        = TypeSpec('ArrayFloat')
TBeamSection                       = TypeSpec('BeamSection')
TBodyGraphicsData                  = TypeSpec('BodyGraphicsData')
TBodyGraphicsDataList              = TypeSpec('BodyGraphicsDataList')
TBool                              = TypeSpec('Bool')
TCNodeGroup                        = TypeSpec('CNodeGroup')
TCObjectType                       = TypeSpec('CObjectType')
TCSolverExplicitTimeInt            = TypeSpec('CSolverExplicitTimeInt')
TCSolverImplicitSecondOrderTimeIntUserFunction = TypeSpec('CSolverImplicitSecondOrderTimeIntUserFunction')
TCSolverStatic                     = TypeSpec('CSolverStatic')
TCrossSectionType                  = TypeSpec('CrossSectionType')
TDynamicSolverType                 = TypeSpec('DynamicSolverType')
TFileName                          = TypeSpec('FileName')
TFloat3                            = TypeSpec('Float3')
TFloat4                            = TypeSpec('Float4')
TGeneralMatrixEXUdense             = TypeSpec('GeneralMatrixEXUdense')
TGeneralMatrixEigenSparse          = TypeSpec('GeneralMatrixEigenSparse')
THomogeneousTransformation         = TypeSpec('HomogeneousTransformation')
TInertiaList                       = TypeSpec('InertiaList')
TInt                               = TypeSpec('Int')
TItemType                          = TypeSpec('ItemType')
TJointTypeList                     = TypeSpec('JointTypeList')
TKeyPressUserFunction              = TypeSpec('KeyPressUserFunction')
TLinearSolverType                  = TypeSpec('LinearSolverType')
TLinkedDataVector                  = TypeSpec('LinkedDataVector')
TLoadType                          = TypeSpec('LoadType')
TMatrix3DList                      = TypeSpec('Matrix3DList')
TNumpyMatrix                       = TypeSpec('NumpyMatrix')
TNumpyMatrixI                      = TypeSpec('NumpyMatrixI')
TNumpyVector                       = TypeSpec('NumpyVector')
TOutputVariableType                = TypeSpec('OutputVariableType')
TPyFunctionGraphicsData            = TypeSpec('PyFunctionGraphicsData')
TPyFunctionMatrixContainerMbsScalarIndex2Vector = TypeSpec('PyFunctionMatrixContainerMbsScalarIndex2Vector')
TPyFunctionMatrixContainerMbsScalarIndex2Vector2Scalar = TypeSpec('PyFunctionMatrixContainerMbsScalarIndex2Vector2Scalar')
TPyFunctionMatrixContainerMbsScalarIndex2VectorBool = TypeSpec('PyFunctionMatrixContainerMbsScalarIndex2VectorBool')
TPyFunctionMatrixMbsScalarIndex2Vector = TypeSpec('PyFunctionMatrixMbsScalarIndex2Vector')
TPyFunctionMbsScalar2              = TypeSpec('PyFunctionMbsScalar2')
TPyFunctionMbsScalarIndexScalar    = TypeSpec('PyFunctionMbsScalarIndexScalar')
TPyFunctionMbsScalarIndexScalar11  = TypeSpec('PyFunctionMbsScalarIndexScalar11')
TPyFunctionMbsScalarIndexScalar5   = TypeSpec('PyFunctionMbsScalarIndexScalar5')
TPyFunctionMbsScalarIndexScalar9   = TypeSpec('PyFunctionMbsScalarIndexScalar9')
TPyFunctionVector3DmbsScalarIndexScalar4Vector3D = TypeSpec('PyFunctionVector3DmbsScalarIndexScalar4Vector3D')
TPyFunctionVector3DmbsScalarVector3D = TypeSpec('PyFunctionVector3DmbsScalarVector3D')
TPyFunctionVector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D = TypeSpec('PyFunctionVector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D')
TPyFunctionVector6DmbsScalarIndexVector6D = TypeSpec('PyFunctionVector6DmbsScalarIndexVector6D')
TPyFunctionVectorMbsScalarArrayIndexVectorConfiguration = TypeSpec('PyFunctionVectorMbsScalarArrayIndexVectorConfiguration')
TPyFunctionVectorMbsScalarIndex2Vector = TypeSpec('PyFunctionVectorMbsScalarIndex2Vector')
TPyFunctionVectorMbsScalarIndex2VectorBool = TypeSpec('PyFunctionVectorMbsScalarIndex2VectorBool')
TPyFunctionVectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D = TypeSpec('PyFunctionVectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D')
TPyFunctionVectorMbsScalarIndexVector = TypeSpec('PyFunctionVectorMbsScalarIndexVector')
TPyMatrixContainer                 = TypeSpec('PyMatrixContainer')
TResizableMatrix                   = TypeSpec('ResizableMatrix')
TResizableVector                   = TypeSpec('ResizableVector')
TResizableVectorParallel           = TypeSpec('ResizableVectorParallel')
TSTDstring                         = TypeSpec('STDstring')
TSensorType                        = TypeSpec('SensorType')
TStdArray33F                       = TypeSpec('StdArray33F')
TString                            = TypeSpec('String')
TTemporaryComputationData          = TypeSpec('TemporaryComputationData')
TTemporaryComputationDataArray     = TypeSpec('TemporaryComputationDataArray')
TTransformation66List              = TypeSpec('Transformation66List')
TVector                            = TypeSpec('Vector')
TVector2DList                      = TypeSpec('Vector2DList')
TVector3DList                      = TypeSpec('Vector3DList')
TVector6DList                      = TypeSpec('Vector6DList')
Tbool                              = TypeSpec('bool')
Tvoid                              = TypeSpec('void')


#%%************************************************************************************************
def _member(kind, fields):
    """Common body: record which constructor was used, and default cplusplusName to pythonName -
    which is what the old format meant by leaving that column empty."""
    fields = dict(fields)
    fields['kind'] = kind
    if not fields.get('cplusplusName', ''):
        fields['cplusplusName'] = fields.get('pythonName', '')

    return fields


#%%************************************************************************************************
def ItemParameter(type, destination, pythonName, cFlags='', defaultValue='',
                  size='', args='', cplusplusName='',
                  description='', fromParent=False, deprecated=None):
    return _member('ItemParameter', locals())


#%%************************************************************************************************
def ItemFunction(type, destination, pythonName, cFlags='', implementation='',
                 args='', size='', cplusplusName='',
                 description='', isVirtual=False, isStatic=False):
    return _member('ItemFunction', locals())


#%%************************************************************************************************
def StructureParameter(type, pythonName, cFlags='', defaultValue='', size='',
                       args='', cplusplusName='',
                       description='', isLinked=False, fromParent=False, deprecated=None):
    return _member('StructureParameter', locals())


#%%************************************************************************************************
def StructureFunction(type, pythonName, cFlags='', implementation='', args='',
                      size='', cplusplusName='',
                      description='', isVirtual=False, isLinked=False):
    return _member('StructureFunction', locals())


#%%************************************************************************************************
def ItemOutputVariable(outputVariable, description):
    """One output variable an item provides. outputVariable is a constant from
    outputVariableTypes.py (OVPosition, OVForceLocal, ...), so a typo is a NameError here rather
    than a key that silently never matches; description is the text shown in the reference
    tables, and may be one of the shared texts in outputVariableDescriptions.py."""

    return {'outputVariable': outputVariable, 'description': description}


#%%************************************************************************************************
def ItemDefinition(className, members, **header):
    header['className'] = className
    header['members'] = members

    return header


#%%************************************************************************************************
def StructureDefinition(className, members, **header):
    header['className'] = className
    header['members'] = members

    return header
