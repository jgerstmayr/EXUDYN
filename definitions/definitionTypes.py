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
CFVisualization      = 'V'   #UNDOCUMENTED in the legend; one single use, in NodeRigidBodyEP

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

#--------------------------------------------------------------------- repeated default values
DVInvalidIndex       = 'EXUstd::InvalidIndex'          #an unset index
DVDefaultColor       = 'Float4({-1.f,-1.f,-1.f,-1.f})' #RGBA -1 means "use the default"
DVZeroVector3D       = 'Vector3D({0.,0.,0.})'
DVTrue               = 'true'                          #the C++ literal, not Python True
DVFalse              = 'false'                         #the C++ literal, not Python False
DVZeroReal           = '0.'
DVZeroIndex          = '0'

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
#Only identifier-shaped types get a constant; a type that is a C++ expression stays a plain
#string, because naming "template<class TReal> void" would give a single-use constant longer
#than the thing it names. The name keeps the type's own spelling: 'Bool' and 'bool' are BOTH
#used and are different types, so upper-casing the first letter collided and one silently
#overwrote the other.
TAccessFunctionType                = 'AccessFunctionType'
TArrayFloat                        = 'ArrayFloat'
TArrayIndex                        = 'ArrayIndex'
TArrayMarkerIndex                  = 'ArrayMarkerIndex'
TArrayNodeIndex                    = 'ArrayNodeIndex'
TArrayObjectIndex                  = 'ArrayObjectIndex'
TArraySensorIndex                  = 'ArraySensorIndex'
TBeamSection                       = 'BeamSection'
TBodyGraphicsData                  = 'BodyGraphicsData'
TBodyGraphicsDataList              = 'BodyGraphicsDataList'
TBool                              = 'Bool'
TCNodeGroup                        = 'CNodeGroup'
TCObjectType                       = 'CObjectType'
TCSolverExplicitTimeInt            = 'CSolverExplicitTimeInt'
TCSolverImplicitSecondOrderTimeIntUserFunction = 'CSolverImplicitSecondOrderTimeIntUserFunction'
TCSolverStatic                     = 'CSolverStatic'
TCrossSectionType                  = 'CrossSectionType'
TDynamicSolverType                 = 'DynamicSolverType'
TFileName                          = 'FileName'
TFloat3                            = 'Float3'
TFloat4                            = 'Float4'
TGeneralMatrixEXUdense             = 'GeneralMatrixEXUdense'
TGeneralMatrixEigenSparse          = 'GeneralMatrixEigenSparse'
THomogeneousTransformation         = 'HomogeneousTransformation'
TIndex                             = 'Index'
TIndex2                            = 'Index2'
TIndex4                            = 'Index4'
TInertiaList                       = 'InertiaList'
TInt                               = 'Int'
TItemType                          = 'ItemType'
TJointTypeList                     = 'JointTypeList'
TKeyPressUserFunction              = 'KeyPressUserFunction'
TLinearSolverType                  = 'LinearSolverType'
TLinkedDataVector                  = 'LinkedDataVector'
TLoadIndex                         = 'LoadIndex'
TLoadType                          = 'LoadType'
TMarkerIndex                       = 'MarkerIndex'
TMatrix2D                          = 'Matrix2D'
TMatrix3D                          = 'Matrix3D'
TMatrix3DList                      = 'Matrix3DList'
TMatrix6D                          = 'Matrix6D'
TNodeIndex                         = 'NodeIndex'
TNodeIndex2                        = 'NodeIndex2'
TNodeIndex3                        = 'NodeIndex3'
TNodeIndex4                        = 'NodeIndex4'
TNumpyMatrix                       = 'NumpyMatrix'
TNumpyMatrixI                      = 'NumpyMatrixI'
TNumpyVector                       = 'NumpyVector'
TObjectIndex                       = 'ObjectIndex'
TOutputVariableType                = 'OutputVariableType'
TPFloat                            = 'PFloat'
TPInt                              = 'PInt'
TPReal                             = 'PReal'
TPyFunctionGraphicsData            = 'PyFunctionGraphicsData'
TPyFunctionMatrixContainerMbsScalarIndex2Vector = 'PyFunctionMatrixContainerMbsScalarIndex2Vector'
TPyFunctionMatrixContainerMbsScalarIndex2Vector2Scalar = 'PyFunctionMatrixContainerMbsScalarIndex2Vector2Scalar'
TPyFunctionMatrixContainerMbsScalarIndex2VectorBool = 'PyFunctionMatrixContainerMbsScalarIndex2VectorBool'
TPyFunctionMatrixMbsScalarIndex2Vector = 'PyFunctionMatrixMbsScalarIndex2Vector'
TPyFunctionMbsScalar2              = 'PyFunctionMbsScalar2'
TPyFunctionMbsScalarIndexScalar    = 'PyFunctionMbsScalarIndexScalar'
TPyFunctionMbsScalarIndexScalar11  = 'PyFunctionMbsScalarIndexScalar11'
TPyFunctionMbsScalarIndexScalar5   = 'PyFunctionMbsScalarIndexScalar5'
TPyFunctionMbsScalarIndexScalar9   = 'PyFunctionMbsScalarIndexScalar9'
TPyFunctionVector3DmbsScalarIndexScalar4Vector3D = 'PyFunctionVector3DmbsScalarIndexScalar4Vector3D'
TPyFunctionVector3DmbsScalarVector3D = 'PyFunctionVector3DmbsScalarVector3D'
TPyFunctionVector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D = 'PyFunctionVector6DmbsScalarIndex4Vector3D2Matrix6D2Matrix3DVector6D'
TPyFunctionVector6DmbsScalarIndexVector6D = 'PyFunctionVector6DmbsScalarIndexVector6D'
TPyFunctionVectorMbsScalarArrayIndexVectorConfiguration = 'PyFunctionVectorMbsScalarArrayIndexVectorConfiguration'
TPyFunctionVectorMbsScalarIndex2Vector = 'PyFunctionVectorMbsScalarIndex2Vector'
TPyFunctionVectorMbsScalarIndex2VectorBool = 'PyFunctionVectorMbsScalarIndex2VectorBool'
TPyFunctionVectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D = 'PyFunctionVectorMbsScalarIndex4VectorVector3D2Matrix6D2Matrix3DVector6D'
TPyFunctionVectorMbsScalarIndexVector = 'PyFunctionVectorMbsScalarIndexVector'
TPyMatrixContainer                 = 'PyMatrixContainer'
TReal                              = 'Real'
TResizableMatrix                   = 'ResizableMatrix'
TResizableVector                   = 'ResizableVector'
TResizableVectorParallel           = 'ResizableVectorParallel'
TSTDstring                         = 'STDstring'
TSensorType                        = 'SensorType'
TStdArray33F                       = 'StdArray33F'
TString                            = 'String'
TTemporaryComputationData          = 'TemporaryComputationData'
TTemporaryComputationDataArray     = 'TemporaryComputationDataArray'
TTransformation66List              = 'Transformation66List'
TUFloat                            = 'UFloat'
TUInt                              = 'UInt'
TUReal                             = 'UReal'
TVector                            = 'Vector'
TVector2D                          = 'Vector2D'
TVector2DList                      = 'Vector2DList'
TVector3D                          = 'Vector3D'
TVector3DList                      = 'Vector3DList'
TVector4D                          = 'Vector4D'
TVector6D                          = 'Vector6D'
TVector6DList                      = 'Vector6DList'
TVector7D                          = 'Vector7D'
TVector9D                          = 'Vector9D'
Tbool                              = 'bool'
Tfloat                             = 'float'
Tvoid                              = 'void'


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
                  description='', fromParent=False):
    return _member('ItemParameter', locals())


#%%************************************************************************************************
def ItemFunction(type, destination, pythonName, cFlags='', implementation='',
                 args='', size='', cplusplusName='',
                 description='', isVirtual=False, isStatic=False):
    return _member('ItemFunction', locals())


#%%************************************************************************************************
def StructureParameter(type, pythonName, cFlags='', defaultValue='', size='',
                       args='', cplusplusName='',
                       description='', isLinked=False, fromParent=False):
    return _member('StructureParameter', locals())


#%%************************************************************************************************
def StructureFunction(type, pythonName, cFlags='', implementation='', args='',
                      size='', cplusplusName='',
                      description='', isVirtual=False, isLinked=False):
    return _member('StructureFunction', locals())


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
