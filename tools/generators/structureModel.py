#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Shared facts of the structure emitters (structureHeaderEmitter.py, structureStubEmitter.py,
#           structureDocsEmitter.py): type tables, predicates on classes and parameters, the
#           sorted parameter list, typical paths and the old string records of every structure,
#           rendered from definitions/ by definitionLoader. Moved out of
#           src/pythonGenerator/pythonAutoGenerateSystemStructures.py (revision2026 step R4.3, part 2c).
#           The header and stub emitters read the members directly (revision2026 step R4.4.1); only
#           structureDocsEmitter.py still reads the string records, until revision2026 step R7.1 replaces it.
#
# Usage:    import structureModel as sm
#
# Author:   Johannes Gerstmayr
# Date:     2018-05-18 (created as pythonAutoGenerateSystemStructures.py), 2026-09-14 (structureModel.py)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys

toolsDirectory = os.path.dirname(os.path.abspath(__file__))
if toolsDirectory not in sys.path:
    sys.path.insert(0, toolsDirectory)

import copy

import generatorPaths as paths                                          # noqa: E402,F401
import definitionLoader                                                 # noqa: E402
from autoGenerateHelper import CountLines, Str2Latex, Str2Doxygen, GetDateStr, \
                               PyLatexRST, WriteTextIfDifferent, DocStringGoogleFromPlainText  # noqa: E402,F401

sortStructures = True
ADD_DOCSTRINGS = True
    
#convert special size parameters:
sizeParameterConvert = {'3x3':'9', '2x2':'4'} 

#check if this helps improving type completion:
addDocuClass = True  #add doc string for classes
addDocuMember = True #add doc string for member variables

#return True for types, which get a range check and does a .def_property access in pybind and a set/get function
def IsTypeWithRangeCheck(origType):
    if (origType.find('PInt') != -1 or origType.find('UInt') != -1 or 
        origType.find('PReal') != -1 or origType.find('UReal') != -1 or
        origType.find('PFloat') != -1 or origType.find('UFloat') != -1
        ):
        return True
    return False

#return True for types, which need a .def_property access in pybind and a set/get function
def IsTypeWithSetGetFunction(origType):
    if (origType.find('Matrix3D') != -1 or
        origType.find('Matrix6D') != -1 or
        origType.find('Vector2DList') != -1 or
        origType.find('KeyPressUserFunction') != -1 
        ):
        return True
    return False


settingsClassName2member = {'ContourAdvanced':'advanced','ViewAdvanced':'advanced',
                            'GeneralAdvanced':'advanced','OpenGLAdvanced':'advanced',
                            'RaytracerAdvanced':'advanced','InteractiveAdvanced':'advanced',
                            'WindowDeprecated':'window',
                            'TimeIntegrationSettings':'timeIntegration', 
                            'StaticSolverSettings':'staticSolver', 
                            'ExplicitIntegrationSettings':'explicitIntegration', 
                            'GeneralizedAlphaSettings':'generalizedAlpha',
                            'NewtonSettings':'newton', 
                            'DiscontinuousSettings':'discontinuous', 
                            'NumericalDifferentiationSettings':'numericalDifferentiation',
                            }
#convert settings class name like Contour into contour
def ConvertClassName2member(className):
    className = className.replace('VSettings','') #fixes name prefix for all visualization settings
    if className in settingsClassName2member.keys():
        finalName = settingsClassName2member[className]
    else:
        finalName = className[0:1].lower() + className[1:] #also works for empty strings
    return finalName

#remove special latex commands from string, especially for pybind descriptions
def RemoveLatexCommands(s):
    s = s.replace('\\hac{ODE2}','ODE2')
    s = s.replace('\\hac{ODE1}','ODE1')
    s = s.replace('\\hac{AE}','AE')
    return s

def ClassHasGetSetDictionary(className):
    return (className.find('Solver') == -1 
        or className == 'StaticSolverSettings'
        or className == 'LinearSolverSettings')

def ClassHasBackLink(className):
    return (className == 'VisualizationSettings'
            or className.startswith('VSettings'))

def TopClassName(className):
    if className.startswith('VSettings') or className == 'VisualizationSettings':
        return 'VisualizationSettings'
    else:
        return ''
    
#if it is a substructure, return True; if topclass, return False
def HasTopClass(className):
    return TopClassName(className) != className

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#direct member access (revision2026 step R4.4.1): the header and structure-stub emitters read the
#definitions/ members through these functions. The predicates shared with structureDocsEmitter.py
#also accept the old string records it still reads; that second form goes with revision2026 step R7.1.
import itemModel as _im                                                 # noqa: E402

#generation order: structures appended to one file (appendToFile) depend on it
structureModules = definitionLoader.structureModules
_structureClassNames = None


def StructureDefinitions():
    """all structure definitions in generation order"""
    definitions = []
    for moduleName in structureModules:
        definitions += __import__(moduleName).definitions
    return definitions


def _IsRecord(member):
    """True for an old string record, False for a definitions/ member"""
    return 'kind' not in member


def Header(definition, key):
    """a class header value as the emitters use it: strings, 'True'/'False' for flags, None for
    absent typicalPaths"""
    if key == 'class':
        return definition['className']
    value = definition.get(key, None)
    if key == 'typicalPaths' and value is None:
        return None
    if value is None:
        return ''
    if key in _im.booleanHeaderKeys:
        return 'True' if value else 'False'
    return _im.Mangle(str(value), key, 'structures')


def _H(parseInfo, key):
    """a header value of a definition or of an old parseInfo record"""
    return Header(parseInfo, key) if 'className' in parseInfo else parseInfo[key]


def IsVariable(member):
    if _IsRecord(member):
        return member['lineType'].find('V') != -1
    return member['kind'] == 'StructureParameter'


def IsFunction(member):
    return not IsVariable(member)


def IsLinked(member):
    return bool(member.get('isLinked', False))


def FromParent(member):
    return bool(member.get('fromParent', False))


def IsVirtualFunction(member):
    return IsFunction(member) and bool(member.get('isVirtual', False))


def HasFlag(member, letter):
    """a flag of cFlags: SFConst 'C', SFPybindArgs 'G', SFReturnCopy 'V', SFNoDictType 'D', SFDeprecated
    'X'; 'P' (in the Python interface) is the absence of SFNoPybind 'N' (revision2026 step R4.25)"""
    flags = member.get('cFlags', '') or ''
    if letter == 'P':
        return 'N' not in flags
    return letter in flags


def IsDeclarationOnly(member):
    """a function whose body is written by hand (old flag D)"""
    return IsFunction(member) and (HasFlag(member, 'D') or member.get('implementation', None) is None)


def Description(member):
    if _IsRecord(member):
        return member['parameterDescription']
    return member.get('description', '') or ''


def DefaultCpp(member):
    """the C++ default value of a variable, or the body of a function"""
    if _IsRecord(member):
        return member['defaultValue']
    return _im.DefaultValueString(member)


def MemberDefaults(member):
    """what THIS instance of a sub-structure starts from, {subMemberName: value}, or {}

    The values are written as the sub-member's own defaultValue would be; the emitter looks the
    sub-member up to know whether it is a string (revision2026b step RG6.2.20).
    """
    if _IsRecord(member):
        return {}
    return member.get('memberDefaults', None) or {}


def StructureDefinitionByName(className):
    """the definition of one structure, or None if nothing defines it"""
    for definition in StructureDefinitions():
        if definition['className'] == className:
            return definition
    return None


def Args(member):
    return member.get('args', '') or ''


def Size(member):
    return _im.Size(member)


#just extract and evaluate cflag
def IsDeprecatedParameter(parameter):
    return HasFlag(parameter, 'X')

#a member whose type is a structure defined in definitions/ (a substructure)
def IsStructureParameter(parameter):
    global _structureClassNames
    if _IsRecord(parameter):
        return (parameter['cFlags'].find('S') != -1)
    if _structureClassNames is None:
        _structureClassNames = set(d['className'] for d in StructureDefinitions())
    return str(parameter.get('type', '')) in _structureClassNames

#convert parameter to deprecation version and expiration data
def DParameter2VersionExpiration(parameter):
    changedInfo = DefaultCpp(parameter) #workaround; contains 'version;EXP=....' where in EXP, the expire year is noted where the deprecated parameter will be removed
    if len(changedInfo.split(';')) < 2 or 'EXP=' not in changedInfo:
        raise ValueError('VersionExpiration: parameter '+str(parameter) + 'has illegal version')
    version = changedInfo.split(';')[0]
    expDate = changedInfo.split(';')[1].replace('EXP=','')
    return (version, expDate)
    

#extract parameterdescription depending on deprecated status (then it is the re-link)
def ParameterDescription(parameter):
    IDP = IsDeprecatedParameter(parameter)
    return 'DEPRECATED; Instead use '*IDP + Description(parameter)

def ParameterChanges2LatexRST(parameterChangesList, latexStr, rstStr):
    if len(parameterChangesList):
        text = '\nThe following parameter changes have been made:\n'
        latexStr += text
        rstStr += text+'\n'
        latexStr += '\\bi\n'
        for param in parameterChangesList:
            latexStr += '  \\item '
            latexStr += param[0].replace('visualizationSettings.','')+' $\\ra$ '
            latexStr += param[1].replace('visualizationSettings.','')
            text = ' (changed in version '+param[2]+', expires: '+param[3]+')\n'
            latexStr += text
            rstStr += '  - ' + param[0]+' → '+param[1]+text
        latexStr += '\\ei\n'
        rstStr += '\n'
        # print(rstStr[-200:])
    return (latexStr, rstStr)


def ParameterChanges2Markdown(parameterChangesList):
    """the deprecated parameters of a structure, as a Markdown list (revision2026 step R7.1.6)"""
    if len(parameterChangesList) == 0:
        return ''
    text = '\nThe following parameter changes have been made:\n\n'
    for parameter in parameterChangesList:
        text += ('- `' + parameter[0] + '` ' + chr(8594) + ' `' + parameter[1] + '` (changed in '
                 'version ' + parameter[2] + ', expires: ' + parameter[3] + ')\n')
    return text + '\n'


def ParameterDescription2DocString(text):
    if text.strip().startswith('$'): #formula at beginning
        listStrip = text.split('$')
        if len(listStrip) > 2: 
            text = '$'.join(listStrip[2:])
    return text


parseInfoTemplate = {'class':'',            # C++ class name
             'writeFile':'',        #filename (e.g. SensorData.h)
             'appendToFile':'',     #True, if shall be appended to given file
             'writePybindIncludes':'',#True, if pybind11 includes shall be written for this class
             'addDictionaryAccess':'',#True, if dictionary access function should be added via pybind
             'pythonClass':'',      #name of class in Python or empty
             'parentClass':'',      #name of parent class or empty
             'classDescription':'', #add a (brief, one line) description of class
             'addConstructor':'',   #code added at the end of default constructor
             'linkedClass':'',      #if not empty, this is a class member to which the python interface is linked
             'latexText':'',        #text, which will be added before the class description (e.g., to start a new section)
             'typicalPaths':None,   #comma-separated typical paths
             'cppText':''}          #code which is added before class definition
lineDefinition = ['lineType',       #[V|F[v]]P: V...Value (=member variable), F...Function (access via member function); v ... virtual Function; P ... write Pybind11 interface
                  'pythonName',     #name which is used in python
                  'cplusplusName',     #name which is used in Exudyn (leave empty if it is the same)
                  'size',           #leave empty if size is variable; e.g. 3 (size of vector), 2x3 (2 rows, 3 columns)  %used for vectors and matrices only!
                  'type',           #Bool, Int, Real, UInt, UReal, Vector, Matrix, SymmetricMatrix
                  'defaultValue',   #default value for member variable or function definition
                  'args',           #args for functions
                  'cFlags',         #P(add Pybind11 interface), R(read only), M(modifiableDuringSimulation), C...const function, D...definition only [default is read/write access and that changes are immediately applied and need no reset of the system]
                  'parameterDescription'] #description for parameter used in C++ code


def LegacyStructures():
    """yield (parseInfo, parameterList) for every structure, as the old line parser built them"""
    return definitionLoader.LoadStructureDefinitions(copy.deepcopy(parseInfoTemplate), lineDefinition)


def PythonClassName(parseInfo):
    pythonClass = _H(parseInfo, 'class')
    if _H(parseInfo, 'pythonClass') != '':
        pythonClass = _H(parseInfo, 'pythonClass')
    return pythonClass


def SortedParameters(parameterList):
    #create sorted parameter list; distinguish between structures (cFlags have 'S') and values: adds 0/1 before name for sorting ...
    parameterListSorted=sorted(parameterList, 
                               key=lambda d: str(int(not IsStructureParameter(d)))+d['pythonName'].upper())
    if not sortStructures:
        parameterListSorted = parameterList 
    return parameterListSorted


def HasPybindInterface(parseInfo, parameterList):
    """True if the structure has member variables in the Python interface; only those are documented"""
    hasPybindInterface = False
    for parameter in parameterList:
        if IsVariable(parameter) and HasFlag(parameter, 'P'): #only if it is a member variable
            hasPybindInterface = True

    if (_H(parseInfo, 'class') == 'SolverLocalData'
        #or _H(parseInfo, 'class') == 'SolverFileData'
        ):
        hasPybindInterface = False
    return hasPybindInterface


def TypicalPaths(parseInfo):
    """the typical access paths of a structure in Python, e.g. ['SC.visualizationSettings.nodes']"""
    typicalPaths = []
    if _H(parseInfo, 'typicalPaths') != None:
        typicalPaths = _H(parseInfo, 'typicalPaths')
        class2name = _H(parseInfo, 'class')

        if class2name.endswith('View'):
            typicalPaths += '.view'
            class2name = ''
            
        if typicalPaths.endswith('.view'):
            oldTypicalPath = typicalPaths
            typicalPaths = ''
            sep = ''
            for i in range(4):
                typicalPaths += sep + oldTypicalPath.replace('.view','.view'+str(i))
                sep = ','

        class2name = ConvertClassName2member(class2name)
        
        #remove Settings from structure:
        # conv = ['TimeIntegrationSettings', 'StaticSolverSettings', 'ExplicitIntegrationSettings', 'GeneralizedAlphaSettings',
        # 'NewtonSettings', 'DiscontinuousSettings', 'NumericalDifferentiationSettings']
        # for c in conv:
        #     if c in class2name:
        #         class2name = class2name.replace('Settings','')
        
        typicalPaths = typicalPaths.split(',')
        for i in range(len(typicalPaths)):
            typicalPaths[i] += '.' if (typicalPaths[i]!='' and class2name!='') else ''
            typicalPaths[i] += class2name[0:1].lower() + class2name[1:]
    return typicalPaths


def ParameterChangesList(parseInfo, parameterListSorted, typicalPaths):
    """old and new full path of every deprecated value parameter, with version and expiration"""
    parameterChangesList = [] #old and new parameter (full path)
    for parameter in parameterListSorted:
        if IsDeprecatedParameter(parameter) and not IsStructureParameter(parameter): #for structures, there is no replacement; only for values
            for path in typicalPaths:
                oldParameterStr = path.replace('SC.','') +'.'+ parameter['pythonName']
                baseParameter = oldParameterStr.split('.')[0]
                newParameterStr = baseParameter+'.'+Description(parameter)
                (version, expDate) = DParameter2VersionExpiration(parameter)
                parameterChangesList.append([oldParameterStr, newParameterStr, version, expDate])
    return parameterChangesList
