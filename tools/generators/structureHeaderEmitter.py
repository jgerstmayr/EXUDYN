#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits the C++ structure headers (SimulationSettings.h, VisualizationSettings.h, ...),
#           DictionariesGetSet.h and pybind_modules.h from definitions/ (revision2026 step R4.3,
#           part 2c). Moved out of src/pythonGenerator/pythonAutoGenerateSystemStructures.py. Reads the
#           members directly through the predicates of structureModel.py (revision2026 step R4.4.1).
#
# Usage:    python tools/generators/structureHeaderEmitter.py
#
# Author:   Johannes Gerstmayr
# Date:     2018-05-18 (created as pythonAutoGenerateSystemStructures.py), 2026-09-14 (emitter)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys

toolsDirectory = os.path.dirname(os.path.abspath(__file__))
if toolsDirectory not in sys.path:
    sys.path.insert(0, toolsDirectory)

import copy                                                             # noqa: E402

from structureModel import *                                            # noqa: E402,F403
import typeModel as tm                                                  # noqa: E402


#scalar types written through EPyUtils::FromPython (revision2026 step R4.4.3.5a): None and item
#indices raise as for items; the U.../P... forms carry their range check
scalarRangeForms = {'bool': None, 'float': None, 'Real': None, 'Index': None, 'Int': None,
                    'UReal': 'nonNegative', 'UFloat': 'nonNegative', 'UInt': 'nonNegative',
                    'PReal': 'positive', 'PFloat': 'positive', 'PInt': 'positive'}


#further member types with a FromPython overload and a context (revision2026 step R4.4.3.5b); vectors and index
#arrays are returned as lists through EPyUtils::ToPythonMember
convertedMemberTypes = list(scalarRangeForms) + ['String', 'FileName', 'Float3', 'Float4', 'Index2', 'ArrayIndex',
                                                 'LinearSolverType', 'DynamicSolverType', 'OutputVariableType',
                                                 'ItemType', 'CrossSectionType']
listMemberTypes = ['Float3', 'Float4', 'Index2', 'ArrayIndex']


def RangeArgument(typeName):
    rangeForm = scalarRangeForms.get(typeName, None)
    return '' if rangeForm is None else 'EPyUtils::RangeCheck::' + rangeForm + ', '


def IsDirectScalar(parameter):
    """a converted data member of the structure itself (not linked, not deprecated): its attribute
    access is bound with EPyUtils::MemberGetter/MemberSetter, without Get/Set functions in the header"""
    return (parameter['type'] in convertedMemberTypes and not IsLinked(parameter)
            and parameter['cplusplusName'].find('.') == -1 and not IsDeprecatedParameter(parameter))


#************************************************
#create the C++ header text of one structure
def StructureCppHeader(parseInfo):
    """returns [header text, dictionary get/set text, implementation text]; parseInfo is the
    structure definition of definitions/"""
    parameterList = parseInfo['members']
    dateStr = GetDateStr()
    yearStr = dateStr.split('-')[0]
    cppText = Header(parseInfo, 'cppText') #.replace('\\n','\n') #this is the string for latex documentation
    sGetSetDictionarys = '' #goes into separate file

    #create name for #ifdef macro to include header files only once:
    sHeaderOnce = Header(parseInfo, 'writeFile').split('.')[0]

    #************************************
    #header
    s='' #generate a string for the output file
    #s+='//automatically generated file (pythonAutoGenerateInterfaces.py)\n'
    s+='/** ***********************************************************************************************\n'
    s+='* @class        '+Header(parseInfo, 'class')+'\n'
    s+='* @brief        '+Str2Doxygen(Header(parseInfo, 'classDescription'))+'\n'
    s+='*\n'
    s+='* @author       AUTO: Gerstmayr Johannes\n'
    s+='* @date         AUTO: 2019-07-01 (generated)\n'
    s+='* @date         AUTO: '+ dateStr+' (last modfied)\n'
    s+='*\n'
    s+='* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.\n'
    s+='* @note         Bug reports, support and further information:\n'
    s+='                - email: johannes.gerstmayr@uibk.ac.at\n'
    s+='                - weblink: missing\n'
    s+='                \n'
    s+='************************************************************************************************ **/\n'

    #include header files only once:    
    if Header(parseInfo, 'appendToFile') != 'True':
#        s+='#ifdef _MSC_VER\n'
#        s+='#pragma once\n'
#        s+='#endif\n'
        s+='\n'
        s+='#ifndef '+sHeaderOnce.upper()+'__H\n'
        s+='#define '+sHeaderOnce.upper()+'__H\n'
        s+='\n'
    
        s+='#include <ostream>\n'
        s+='\n'
        s+='#include "Utilities/ReleaseAssert.h"\n'
        s+='#include "Utilities/BasicDefinitions.h"\n'
        s+='#include "Main/OutputVariable.h"\n'
        s+='#include "Linalg/BasicLinalg.h"\n' #for std::array conversion
        s+='\n'

    if cppText != '':
        s += cppText
        s += '\n'

    classInitBackLink = ClassHasBackLink(Header(parseInfo, 'class'))
    classHasBackLink = ClassHasBackLink(Header(parseInfo, 'class')) and HasTopClass(Header(parseInfo, 'class'))

    implementationGetSetStr = '' #implementations that come in the end
    parameterListSorted = SortedParameters(parameterList)

    #************************************
    #class definition:
    strParentClass = ''
    constructorParentClass = ''
    if len(Header(parseInfo, 'parentClass')) != 0:
        strParentClass = ': public ' + Header(parseInfo, 'parentClass')
        constructorParentClass = ': '+Header(parseInfo, 'parentClass')+'()'
    s+='class ' + Header(parseInfo, 'class') + strParentClass + ' // AUTO: \n'
    s+='{\n'


    #************************************
    #member variables:
    sPublic = ''
    sPrivate = ''
    sProtected = ''
    for parameter in parameterListSorted:

        if ((IsVariable(parameter)) and 
            (not IsLinked(parameter)) and 
            (parameter['cplusplusName'].find('.') == -1) and 
            (not IsDeprecatedParameter(parameter) or IsStructureParameter(parameter)) ): #only if it is a member variable, but not linked
            typeStr = tm.Render(parameter['type'], 'cppStorage', 'structures')
            temp = '  ' + typeStr + ' ' + parameter['cplusplusName']+ ';'
            nChar = len(temp)
            alignment = 50
            insertSpaces = ''
            if nChar < alignment:
                insertSpaces = ' '*(alignment-nChar)
            temp += insertSpaces + '//!< AUTO: ' + tm.ConstraintNote(parameter['type']) + Str2Doxygen(ParameterDescription(parameter)) + '\n'

            if (FromParent(parameter)): #make variable private ==> no direct access via C++ or python!
                sPrivate += temp
            else:
                if (HasFlag(parameter, 'P')): #pybind of member variables in this case done via public member variable
                    sPublic += temp
                else:
                    sProtected += temp

    if classHasBackLink:
        #print('class with backlink:',Header(parseInfo, 'class'))
        sPrivate += '  ' + TopClassName(Header(parseInfo, 'class')) + '* backlink; //!< AUTO: backlink for global access of structure\n'

    if (sPublic !='' or sProtected !=''):
        s+='public: // AUTO: \n'
        s+=sPublic #public member variables
        s+=sProtected #protected member variables
        s+='\n'
#    if (sProtected !=''):
#        s+='protected: // AUTO: \n'
#        s+=sProtected #protected member variables
#        s+='\n'
    if (sPrivate !=''):
        s+='private: // AUTO: \n'
        s+=sPrivate #private member variables
        s+='\n'
            
    s+='\npublic: // AUTO: \n' #for member functions ...
    #************************************
    #count number of default parameters
    cntDefaultParameters = 0
    for parameter in parameterList:
        if (IsVariable(parameter)): #only if it is a member variable
            strDefault = DefaultCpp(parameter)
            if len(strDefault): 
                cntDefaultParameters += 1

    #constructor with default initialization:
    if classInitBackLink or cntDefaultParameters or len(Header(parseInfo, 'addConstructor')) != 0:
        s+='  //! AUTO: default constructor with parameter initialization\n'
        s+='  '+Header(parseInfo, 'class')+'()'+constructorParentClass+'\n'
        s+='  {\n'
        if classHasBackLink:
            #print('has backlink:',Header(parseInfo, 'class'))
            s+='    '+'backlink=nullptr;\n'

    
        for parameter in parameterListSorted:
            if (IsVariable(parameter)) and not IsDeprecatedParameter(parameter): #only if it is a member variable
                strDefault = DefaultCpp(parameter)
                if len(strDefault): #only add initialization if default value exists
                    if parameter['type'] == 'String' or parameter['type'] == 'FileName':
                        strDefault = '"' + strDefault + '"'
                    s+='    ' + parameter['cplusplusName'] + ' = ' + strDefault + ';\n'
        s+=Header(parseInfo, 'addConstructor').replace('\\n','\n')
        s+='  };\n'

    #++++++++++++
    #add initializatio nof backlink
    if classInitBackLink:
        s += '  void Init('+TopClassName(Header(parseInfo, 'class')) + '* backlinkInit) //!< AUTO: called from parent structure\n'
        s += '  {\n'
        if classHasBackLink: #not for top class itself
            s += '    backlink = backlinkInit;\n'
        for parameter in parameterListSorted:
            if IsStructureParameter(parameter):# and not IsDeprecatedParameter(parameter):
                s+='    ' + parameter['cplusplusName'] + '.Init(backlinkInit);\n'
        s += '  }\n'
    #++++++++++++

    s+='\n  // AUTO: access functions\n'

    #GetClone() function: #2020-01-03: not used any more
#    s+='  //! AUTO: clone object; specifically for copying instances of derived class, for automatic memory management e.g. in ObjectContainer\n'
#    s+='  virtual ' + Header(parseInfo, 'class') + '* GetClone() const { return new '+Header(parseInfo, 'class')+'(*this); }\n'
#    s+='  \n'
    sDictGet = ''
    sDictGet += '//! AUTO: read access to structure; converting into dictionary\n'
    sDictGet += 'inline py::dict GetDictionaryWithTypeInfo(const ' + Header(parseInfo, 'class') + '& data) {\n'
    sDictGet += '    auto structureDict = py::dict();\n'
    sDictGet += '    auto d = py::dict(); //local dict\n'

    sDictGetPure = ''
    sDictGetPure += '//! AUTO: read access to structure; converting into dictionary without type info\n'
    sDictGetPure += 'inline py::dict GetDictionary(const ' + Header(parseInfo, 'class') + '& data) {\n'
    sDictGetPure += '    auto structureDict = py::dict();\n'

    sDictSet = ''
    sDictSet += '//! AUTO: write access to data structure; converting dictionary d into structure\n'
    sDictSet += 'inline void SetDictionary(' + Header(parseInfo, 'class') + '& data, const py::dict& d) {\n'
    
    #************************************
    #access functions and dictionaries for visualization dialog ...:
    
    if (Header(parseInfo, 'class') == 'VisualizationSettings'):
        parameterListSorted2 = copy.deepcopy(parameterList) #unsorted, sorting as in definition file
    else:
        parameterListSorted2 = copy.deepcopy(parameterListSorted) 
        
        
    for parameter in parameterListSorted2:
        if (IsVariable(parameter)): #only if it is a member variable
            ISP = bool(IsStructureParameter(parameter))
            IDP = bool(IsDeprecatedParameter(parameter))
            IDPNS = bool(IsDeprecatedParameter(parameter)) and not ISP #IDP but not structure

            lineBreakIDP = ''
            deprecationWarning = ''
            if IDPNS:
                lineBreakIDP = '\n    '
                #a real Python DeprecationWarning, not a printed line (#2522, revision2026 step
                #R6.3.4): the user can filter it, promote it with -W error::DeprecationWarning, and
                #sees it once per source location instead of on every read of the setting
                deprecationWarning = 'PyDeprecated("VisualizationSettings parameter '
                deprecationWarning += ConvertClassName2member(Header(parseInfo, 'class'))+'.'+parameter['pythonName']
                deprecationWarning += ' is deprecated! use '+Description(parameter)+' instead!");'+lineBreakIDP
                (version, expDate) = DParameter2VersionExpiration(parameter)
                if expDate <= yearStr:
                    print('parameter outdated '+expDate+':', Header(parseInfo, 'class')+'::'+parameter['cplusplusName'])
                    continue #not included any more with backlinks!
                
                
            origType = parameter['type']
            typeStr = tm.Render(parameter['type'], 'cppStorage', 'structures')
            paramStr = parameter['cplusplusName']
            paramAccessStr = paramStr if not IDPNS else 'backlink->'+Description(parameter)

            paramStrPure = parameter['cplusplusName'] #without 'cSolver.'
            if (paramStrPure.find('.') != -1): #for linked class; mainly solver
                paramStrPure = parameter['pythonName']

            functionStr = paramStrPure #Get/Set function name follows old name, not new parameter
            
            if IDPNS:
                paramStrPure = paramAccessStr.split('.')[-1]

            c = functionStr[0]
            functionStr = c.upper()+functionStr[1:]
            refChar = '&' #use only '&' in read access, if it is no pointer; 
            if typeStr[len(typeStr)-1] == '*':
                refChar = ''

            accessWritten = False

            typeWithRangeCheck = IsTypeWithRangeCheck(origType) and not IsDirectScalar(parameter)
            typeWithGetSetFunction = IsTypeWithSetGetFunction(origType) or IDPNS
            
            getFunction = [] #return type, function decl, impl
            setFunction = [] #return type, function decl, impl
                
            typeCastStr = tm.Render(parameter['type'], 'cppExchange', 'structures')
            if (((typeCastStr.find('std::vector') != -1 or typeCastStr.find('std::array') != -1) and not IsDirectScalar(parameter) and 
                 typeCastStr.find('std::ofstream') == -1 and typeCastStr.find('ExuFile::BinaryFileSettings') == -1) or 
                typeWithRangeCheck or typeWithGetSetFunction or
                (not IsLinked(parameter)  and parameter['cplusplusName'].find('.') != -1)): #then it must get a set/get function!
                accessWritten = True
                
                paramSetStr = paramStrPure + 'Init'
                if typeWithRangeCheck:
                    paramSetStr  = 'EXUstd::GetSafely'+origType+'('+paramSetStr+',"'+paramStrPure+'")'
                
                #print(paramAccessStr + ':' + typeStr + ' gets a setter function')
                s+='  //! AUTO: Set function (needed in pybind) for: ' + Str2Doxygen(ParameterDescription(parameter)) + '\n'
                setFunction.append('void ')
                getReturnStr = typeCastStr
                
                if not typeWithGetSetFunction:
                    setFunction.append('PySet' + functionStr + '(const ' + typeCastStr + refChar + ' ' + paramStrPure + 'Init) ')
                    setFunction.append('{ '+paramAccessStr + ' = ' + paramSetStr + '; }\n')
                else:
                    paramInitStr = paramAccessStr+ '= '+'(const ' + typeStr+ '&)' + paramStrPure + 'Init'
                    #in this case, we need special typecast
                    if (typeStr == 'Matrix3D' or 
                        typeStr == 'Matrix6D'): #in linux casting from std::array<std::array<Real,...>> gives segmentation fault (overrides strangely)
                        
                        typeCastStr = 'py::object'
                        matDim = 3
                        if typeStr == 'Matrix6D':
                            matDim = 6
                        paramInitStr = 'EPyUtils::FromPython<Real, '+str(matDim)+', '+str(matDim)+'>('+paramStrPure+'Init, '+ paramStrPure+')'

                    setFunction.append('PySet' + functionStr + '(const ' + typeCastStr + refChar + ' ' + paramStrPure + 'Init) ')
                    setFunction.append('{ ' + lineBreakIDP + deprecationWarning + paramInitStr+'; '+lineBreakIDP+'}\n')
                        
                    if typeStr == 'Matrix3D' or typeStr == 'Matrix6D': #Matrix type (Matrix3D, ...)
                        getReturnStr = 'py::array_t<Real>' #this makes a numpy array instead of list of lists!
                        typeCastStr = 'EPyUtils::ToPython'

                
                s+= '  '+setFunction[0] + setFunction[1] + setFunction[2]*(1-IDPNS) + ';\n'*IDPNS #spaces/linebreaks included
                s+='  //! AUTO: Read (Copy) access to: ' + Str2Doxygen(ParameterDescription(parameter)) + '\n'

                getFunction.append(getReturnStr + ' ')
                getFunction.append('PyGet' + functionStr + '() const ')
                getFunction.append('{ '+lineBreakIDP+deprecationWarning+'return ' + typeCastStr + '('+ paramAccessStr + '); '+lineBreakIDP+'}\n')
                s+= '  '+getFunction[0] + getFunction[1] + getFunction[2]*(1-IDPNS)+';\n'*IDPNS #spaces/linebreaks included

            if accessWritten:
                s+= '\n'
            
            if IDPNS:
                implementationGetSetStr += '\n'
                cn = Header(parseInfo, 'class')
                implementationGetSetStr += 'inline ' + setFunction[0] + cn + '::'+setFunction[1] + setFunction[2]
                implementationGetSetStr += 'inline ' + getFunction[0] + cn + '::'+getFunction[1] + getFunction[2]
                # print('implementationGetSetStr:',implementationGetSetStr)
            
            #++++++++++++++++++++++++++++++++++++++++++++++++++++++
            #read/write dictionary from hierarchical structure
            if HasFlag(parameter, 'P') and not HasFlag(parameter, 'D'):

                if parameter['pythonName'] == 'itemIdentifier':
                    print("ERROR: pythonName may not be called 'itemIdentifier'") #this term needs to be reserved, as this is the key for a value object
                #check if substructure (folder)
                if IsStructureParameter(parameter):
                    if not IsDeprecatedParameter(parameter):
                        sDictGet += '    structureDict["' + parameter['pythonName'] + '"] = GetDictionaryWithTypeInfo(data.' + parameter['cplusplusName'] + ');\n'
                        sDictGetPure += '    structureDict["' + parameter['pythonName'] + '"] = GetDictionary(data.' + parameter['cplusplusName'] + ');\n'
                        sDictSet+= '    SetDictionary(data.' + parameter['cplusplusName'] + ', py::cast<py::dict>(d["' + parameter['pythonName']  + '"]));\n'
                else: #parameter
                    cValueStr = parameter['cplusplusName'];
                    #cSetStr = parameter['cplusplusName'];
                    if accessWritten: #means that a conversion is necessary
                        cValueStr = 'PyGet' + functionStr + '()'

                    #convert type:
                    pType = tm.Render(parameter['type'], 'dictType', 'structures')
                    #convert size
                    pSize = Size(parameter)
                    if pSize.find('x') != -1: #e.g., 3x3, also 2x2x2 would be possible
                        v = pSize.split('x')
                        if len(v) != 2:
                            print("ERROR: only 2D arrays allowed")
                            
                        pSize = ''
                        sep = ''
                        for item in v:
                            pSize += sep
                            pSize += item
                            sep = ','
                        #print(pSize)

                    if pSize == '':
                        pSize = '{1}' #dicts always have size
                    else: 
                        pSize = '{'+pSize+'}'
                        
                    descrStr = ParameterDescription(parameter).replace("\\_","_").replace("\\","\\\\").replace('$','')
                    typeCastStr = tm.Render(parameter['type'], 'cppExchange', 'structures')
                    
                    if parameter['type'] != 'KeyPressUserFunction' and not IsDeprecatedParameter(parameter): #this would not work for editing dictionary
                        #get functions:
                        sDictGet += '    d = py::dict(); //reset local dict\n'
                        sDictGet += '    d["itemIdentifier"] = std::string(""); //identifier for item\n'
                        valueStr = 'data.' + cValueStr
                        if IsDirectScalar(parameter) and parameter['type'] in listMemberTypes: #lists, as the attribute (revision2026 step R4.4.3.5b)
                            valueStr = 'EPyUtils::ToPythonMember(data.' + parameter['cplusplusName'] + ')'
                        sDictGet += '    d["value"] = ' + valueStr + ';\n'
                        sDictGet += '    d["type"] = "' + pType + '";\n'
                        sDictGet += '    d["size"] = std::vector<int>' + pSize + ';\n' #only used for vectors/matrices (e.g. '3') and matrices (e.g. '3x3')
                        sDictGet += '    d["description"] = "' + descrStr + '";\n'
                        sDictGet += '    structureDict["' + parameter['pythonName'] + '"] = d;\n' #keyName used to identify the object
                        sDictGet += '\n'

                        sDictGetPure += '    structureDict["' + parameter['pythonName'] + '"] = '
                        sDictGetPure += valueStr + ';\n'
                        
                        #set functions:
                        if parameter['type'] in convertedMemberTypes: #the same conversion and checks as the attribute (revision2026 step R4.4.3.5a, b)
                            sDictSet += ('    EPyUtils::FromPython(d["' + parameter['pythonName'] + '"], data.' + parameter['cplusplusName'] + ', '
                                         + RangeArgument(parameter['type']) + '"' + Header(parseInfo, 'class') + '.' + parameter['pythonName'] + '");\n')
                        else:
                            sDictSet += '    data.' + parameter['cplusplusName'] + ' = py::cast<' + typeCastStr + '>(d["' + parameter['pythonName']  + '"]);\n'
                        
            #++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    
        else: # linked variable
            if not IsLinked(parameter):
                strVirtual = ''
                strOverride = ''
                if (IsVirtualFunction(parameter)):
                    strVirtual = 'virtual '
                    strOverride = ' override'
                
                typeStr = tm.Render(parameter['type'], 'cppStorage', 'structures')
                functionStr = parameter['cplusplusName']
                argsStr = Args(parameter)
                strConst = ""
                if HasFlag(parameter, 'C'):
                    strConst = " const"
                
                strDef = ''            
                if IsDeclarationOnly(parameter):
                    strDef = ';'
                else:
                    strDef = ' {\n    ' + DefaultCpp(parameter) + '\n  }\n' #defaultValue is the function body
    
                s+='  //! AUTO: ' + Str2Doxygen(Description(parameter)) + '\n'
                s+='  '+strVirtual + typeStr + ' '
                s+=functionStr + '(' + argsStr + ')' + strConst + strOverride + strDef + '\n'
    
        
    sDictGet += '    return structureDict;\n'
    sDictGet += '}\n\n'
    sDictGetPure += '    return structureDict;\n'
    sDictGetPure += '}\n\n'
    sDictSet += '}\n\n'

        
    if ClassHasGetSetDictionary(Header(parseInfo, 'class')):
        sGetSetDictionarys += sDictGet
        sGetSetDictionarys += sDictGetPure
        sGetSetDictionarys += sDictSet
        #s += '  //! AUTO: read access to structure; converting into dictionary\n'
        #s += '  py::dict GetDictionaryWithTypeInfo() const;\n' #don't do that as we do not want to add pybind to settings files!

    #************************************
    #ostream operator:
#    s+=('  friend std::ostream& operator<<(std::ostream& os, const ' + 
#       Header(parseInfo, 'class') + '& object);\n')

    s+='  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)\n'
    s+='  virtual void Print(std::ostream& os) const\n'
    s+='  {\n'
    s+='    os << "' + Header(parseInfo, 'class') + '" << ":\\n";\n'
    if len(Header(parseInfo, 'parentClass')) != 0:
        s+='    os << ":"; \n'
        s+='    ' + Header(parseInfo, 'parentClass') + '::Print(os);\n'
        
    #output each parameter
    for parameter in parameterListSorted:
        if ((IsVariable(parameter)) and (not IsLinked(parameter)) and 
        (parameter['type']!='TemporaryComputationData') and (parameter['type']!='TemporaryComputationDataArray') and 
        (parameter['type'].find('std::ofstream')==-1) and #(parameter['type'].find('ExuFile::BinaryFileSettings')==-1) and 
        (parameter['type'].find('std::vector<Vector2D>')==-1) and (parameter['type'].find('CrossSectionType')==-1) and
        (parameter['type'].find('userFunction')==-1) and (parameter['type'].find('UserFunction')==-1) and
        not IsDeprecatedParameter(parameter)): #only if it is a member variable; some types not printable
            paramStr = parameter['cplusplusName']
            typeStr = tm.Render(parameter['type'], 'cppStorage', 'structures')
            refChar = ''
            preStr = ''
            postStr = ''
            if typeStr[len(typeStr)-1] == '*':
                refChar = '*' #print content of object, not the pointer address
            if parameter['type'] == 'OutputVariableType': #special case of enum, which is not printable
                preStr = 'GetOutputVariableTypeString('
                postStr= ')'
            if parameter['type'] == 'StdArray33F': #special case of 3x3 Matrix, which is not printable
                preStr = 'Matrix3DF('
                postStr= ')'

            if parameter['type'] == 'StdArray33F': #special case of 3x3 Matrix, which is not printable
                s += '#ifndef __APPLE__\n' #does not compile currently

            s+='    os << "  ' + paramStr + ' = " << ' + preStr + refChar + paramStr + postStr + ' << "\\n";\n'

            if parameter['type'] == 'StdArray33F': #special case of 3x3 Matrix, which is not printable
                s += '#endif\n' #does not compile currently
         
    s+='    os << "\\n";\n'
    s+='  }\n\n' # end ostream operator

    if len(Header(parseInfo, 'parentClass')) == 0:
        s+=('  friend std::ostream& operator<<(std::ostream& os, const ' + Header(parseInfo, 'class') + '& object)\n')
        s+= '  {\n'
        s+= '    object.Print(os);\n'
        s+= '    return os;\n'
        s+= '  }\n\n' # end ostream operator
    
    s+='};\n\n\n' #class

    return [s, sGetSetDictionarys, implementationGetSetStr]

#**************************************************************************************
#create string containing the pybind11 headers/modules for a class
def CreatePybindHeaders(parseInfo):
    parameterList = parseInfo['members']
    #print ('Create Pybind11 includes')

    #remove some \ and other texts from strings written into pybind interface
    def CleanPyDocStrings(s):
        s = s.replace('{ODE2}','ODE2')
        s = s.replace('\\hac','')
        s = s.replace('\\','')
        return s
    
    pickleDictTemplate = """        .def(py::pickle(
            [](const {ClassName}& self) {
                return py::make_tuple(EPyUtils::GetDictionary(self));
            },
            [](const py::tuple& t) {
                CHECKandTHROW(t.size() == 1, "{ClassName}: loading data with pickle received invalid data structure!");
                {ClassName} self;
                EPyUtils::SetDictionary(self,py::cast<py::dict>(t[0]));
                return self;
            }))
"""

    spaces1 = '    '            #first level
    spaces2 = spaces1+'    '    #second level

    s = spaces1 + '//++++++++++++++++++++++++++++++++\n' #create empty string
    #************************************
    #class definition:
    parentClass = ''
    pythonClass = Header(parseInfo, 'class')
    if Header(parseInfo, 'pythonClass') != '':
        pythonClass = Header(parseInfo, 'pythonClass')
        #print('pythonClass=', pythonClass, ', cClass=', Header(parseInfo, 'class'))
        
        
#    if len(Header(parseInfo, 'parentClass')) != 0: #derived class does not work in pybind, if parent class is not defined!
#        parentClass = ', ' + Header(parseInfo, 'parentClass')
    s += spaces1 + 'py::class_<' + Header(parseInfo, 'class') + parentClass + '>(m, "' + pythonClass + '"'
    if addDocuClass:
        s += ', "'+pythonClass+' class"'
    s += ') // AUTO: \n'
    s += spaces2 + '.def(py::init<>())\n'

    #create sorted parameter list; distinguish between structures (cFlags have 'S') and values: adds 0/1 before name for sorting ...
    parameterListSorted=sorted(parameterList, 
                               key=lambda d: str(int(not IsStructureParameter(d)))+d['pythonName'].upper())
    if not sortStructures:
        parameterListSorted = parameterList 

    #************************************
    #member variables access:
    for parameter in parameterListSorted:
        if (IsVariable(parameter)) and (HasFlag(parameter, 'P')): #only if it is a member variable
            ISP = bool(IsStructureParameter(parameter))
            IDP = bool(IsDeprecatedParameter(parameter))

            typeCastStr = tm.Render(parameter['type'], 'cppExchange', 'structures')
            linkedClassStr = ''
            if (len(Header(parseInfo, 'linkedClass')) != 0):
                linkedClassStr = Header(parseInfo, 'linkedClass') + '.'

            if IsDirectScalar(parameter): #revision2026 step R4.4.3.5a
                memberStr = '&' + Header(parseInfo, 'class') + '::' + linkedClassStr + parameter['cplusplusName']
                s += (spaces2 + '.def_property("' + parameter['pythonName'] + '", EPyUtils::MemberGetter(' + memberStr + '), EPyUtils::MemberSetter('
                      + memberStr + ', ' + RangeArgument(parameter['type']) + '"' + Header(parseInfo, 'class') + '.' + parameter['pythonName'] + '")')
                if addDocuMember:
                    s += ', "' + CleanPyDocStrings(Description(parameter)) + '"'
                s += ')\n'
            elif ((typeCastStr.find('std::vector') == -1) and (typeCastStr.find('std::array') == -1) and 
            (not IsLinked(parameter)) and (parameter['cplusplusName'].find('.') == -1)
            and not IsTypeWithRangeCheck(parameter['type']) 
            and not (IsTypeWithSetGetFunction(parameter['type']) or (IDP and not ISP))): #then it has a set/get function! e.g. Int2, Int3, Float2, Float3, .... are array structures ==> must be converted
                s += spaces2 + '.def_readwrite("' + parameter['pythonName'] + '", &' + Header(parseInfo, 'class') + '::' + linkedClassStr + parameter['cplusplusName']
                if addDocuMember:
                    #s += ', "member: ' + parameter['pythonName'] + '"'
                    s += ', "' + CleanPyDocStrings(Description(parameter)) + '"'
                s += ')\n' #extend this to incorporate 'read only' and other flags
            else:
                sReturnValueProperty = '' #for structures that should also have write access
                #not needed: done automatically as reference access for such structures with get/set function
                # if IsLinked(parameter):
                #     sReturnValueProperty += ', py::return_value_policy::reference'
                #access with setter/getter functions and conversions to std::vector
                #functionName = parameter['cplusplusName']
                functionName = parameter['pythonName'] #for linked variables, this is easier to work with linking e.g. to cSolver
                functionName = functionName[0].upper()+functionName[1:]
                s += spaces2 + '.def_property("' + parameter['pythonName'] + '", '
                s += '&' + Header(parseInfo, 'class') + '::PyGet' + functionName + ', '
                if parameter['type'] != 'KeyPressUserFunction' or IsDeprecatedParameter(parameter):
                    s += '&' + Header(parseInfo, 'class') + '::PySet' + functionName + sReturnValueProperty 
                else:
                    #this is quite brute force, and needs to be adjusted in case ...
                    ind12 = ' '*12
                    s += '\n'+ind12+'[](VSettingsInteractive &self, py::object func) {\n'
                    #func.is_none() could be used to detect None as well
                    s += ind12+'  if (py::isinstance<py::int_>(func) && func.cast<int>() == 0) {\n'
                    s += ind12+'    self.PySetKeyPressUserFunction(nullptr); // Resets the function\n' #self.backlink->interactive.keyPressUserFunction
                    s += ind12+'    } else {\n'+ind12+'    self.PySetKeyPressUserFunction(func.cast<std::function<bool(int, int, int)>>());\n'
                    s += ind12+'  }\n'+ind12+'}'
                s += ')\n'
                #    .def_property("name", &Pet::getName, &Pet::setName)
                
    #s += '\n'
    s += spaces2 + '// AUTO: access functions for ' + Header(parseInfo, 'class') + '\n'
            
    for parameter in parameterListSorted:
        if (IsFunction(parameter)) and (HasFlag(parameter, 'P')): #only if it is a member function
            s += spaces2 + '.def("' + parameter['pythonName']
            s += '", &' + Header(parseInfo, 'class') + '::' + parameter['pythonName']
            if parameter['type'] != 'void': #check return_value_policy if not void
                if HasFlag(parameter, 'V'): #pass by value (copy)
                    s += ', py::return_value_policy::copy'
                else:
                    s += ', py::return_value_policy::reference' #extend this to incorporate 'read only' and other flags
            s += ', "' + RemoveLatexCommands(ParameterDescription(parameter)) + '"'
            if (HasFlag(parameter, 'G')): #add py::arg() in order that type completion shows args in python
                argStr = Args(parameter)
                if (argStr != ''):
                    argSplit = argStr.split(',') #split into list of args
                    for item in argSplit:
                        argName = item.split(' ')[-1] #last word in args is the name of the argument, e.g. in const MainSystem& mainSystem ==> mainSystem
                        defaultVal = ''

                        if item.find('=') != -1: #check if there is a default value
                            itemSplit = item.split('=')
                            argName = itemSplit[0].split(' ')[-1] #in left structure, there must be the argument name
                            #print(itemSplit)
                            defaultVal = ' = ' + itemSplit[1]
                        s += ', py::arg("' + argName + '")' + defaultVal
            s+=')\n' 
    
    s += spaces2 + '.def("__repr__", [](const ' + Header(parseInfo, 'class') + ' &item) { return "<' + Header(parseInfo, 'class') + ':\\n" + EXUstd::ToString(item) + " >"; } ) //!< AUTO: add representation for object based on ostream operator\n'
    
    if Header(parseInfo, 'addDictionaryAccess') == 'True':
        s += spaces2 + '.def("GetDictionaryWithTypeInfo", [](const ' + Header(parseInfo, 'class') + ' &item) { return EPyUtils::GetDictionaryWithTypeInfo(item); }) //!< AUTO: add read as dictionary with type information access\n'
    if ClassHasGetSetDictionary(Header(parseInfo, 'class')):
        s += spaces2 + '.def("GetDictionary", [](const ' + Header(parseInfo, 'class') + ' &item) { return EPyUtils::GetDictionary(item); }) //!< AUTO: add read for dictionary access\n'
        s += spaces2 + '.def("SetDictionary", [](' + Header(parseInfo, 'class') + ' &item, const py::dict& d) { return EPyUtils::SetDictionary(item, d); }) //!< AUTO: add write from dictionary access\n'
        s += pickleDictTemplate.replace('{ClassName}', Header(parseInfo, 'class'))
#		.def("GetDictionary", [](const VisualizationSettings &item) { return EPyUtils::GetDictionaryWithTypeInfo(item); }) //!< AUTO: add representation for object based on ostream operator

    s += spaces2 + '; // AUTO: end of class definition!!!\n'
    s += '\n'

    s += spaces1 + '//++++++++++++++++++++++++++++++++\n' #end of pybind11 definition


    return s


def main():
    print('******************************')
    print('Autogenerate system structures')

    #create Python/pybind11 file
    directoryString = paths.autogeneratedDir
    pybindFile = directoryString+'pybind_modules.h'
    getSetFile = directoryString+'DictionariesGetSet.h'
    latexFile = paths.theDocDir+'interfaces.tex'
    stubFile  = paths.generatedDir+'stubSystemStructures.pyi'
    
    writeFilesDict = {} #this will be written finally, ignoring header section!


    writeFilesDict = {} #this will be written finally, ignoring header section!

    WriteFileDict(writeFilesDict, fileName=pybindFile, text=''+
                  '// AUTO:  ++++++++++++++++++++++\n'+
                  '// AUTO:  pybind11 module includes; generated by Johannes Gerstmayr\n'+
                  '// AUTO:  last modified = '+ GetDateStr() + '\n'+
                  '// AUTO:  ++++++++++++++++++++++\n\n', fileMode='w')
    

    


    WriteFileDict(writeFilesDict, fileName=getSetFile, text=''+
                  '// AUTO:  ++++++++++++++++++++++\n'+
                  '// AUTO:  Helper file for dictionaries get/set for system structures; generated by Johannes Gerstmayr\n'+
                  '// AUTO:  Generated by Johannes Gerstmayr\n'+
                  '// AUTO:  Used for SimulationSettings and VisualizationSettings\n'+
                  '// AUTO:  last modified = '+ GetDateStr() + '\n'+
                  '// AUTO:  ++++++++++++++++++++++\n\n'+
                  #++++++++++++++++++++
                  '  #ifndef DICTIONARIESGETSET__H\n'+
                  '  #define DICTIONARIESGETSET__H\n\n'+
                  #++++++++++++++++++++
                  '  #include "Linalg/BasicLinalg.h"\n'+
                  '  #include "Main/CSystem.h"\n'+
                  '  #include "Autogenerated/SimulationSettings.h"\n'+
                  '  #include "Autogenerated/VisualizationSettings.h"\n\n'+
                  '  #include <pybind11/pybind11.h>\n'+
                  '  #include <pybind11/stl.h>\n'+
                  '  #include <pybind11/stl_bind.h>\n'+
                  '  namespace py = pybind11;\n\n'+
                  '  namespace EPyUtils {\n //add namespace for access to dictionaries'
                  , fileMode='w')

    totalNumberOfLines = 0
    fileListHeaderOnce = [] #store all opened files, which get a "#endif " at the end for the #ifdef ... at the beginning
    globalImplementationGetSetStr = '' #implementation part added at end of each structure (visualization, etc.)

    for parseInfo in StructureDefinitions():
        [fileStr, getSetDict, implementationGetSetStr] = StructureCppHeader(parseInfo)
        globalImplementationGetSetStr += implementationGetSetStr
        strFileMode = 'w'

        if Header(parseInfo, 'appendToFile') == 'True':
            strFileMode = 'a'
        else:
            fileListHeaderOnce += [directoryString+Header(parseInfo, 'writeFile')]

        if not HasTopClass(Header(parseInfo, 'class')) and globalImplementationGetSetStr != '':
            fileStr += '\n\n//! implementation:\n'+globalImplementationGetSetStr
            globalImplementationGetSetStr = ''

        WriteFileDict(writeFilesDict, fileName=directoryString+Header(parseInfo, 'writeFile'), 
                      text=fileStr, fileMode=strFileMode)

        #++++++++++++++++++++++++++++++
        #write Python/pybind11 includes
        pybindStr = ''
        if Header(parseInfo, 'writePybindIncludes') == 'True':
            pybindStr = CreatePybindHeaders(parseInfo)
            WriteFileDict(writeFilesDict, fileName=pybindFile, text=pybindStr, fileMode='a')
            WriteFileDict(writeFilesDict, fileName=getSetFile, text=getSetDict, fileMode='a')

        totalNumberOfLines += CountLines(fileStr)+CountLines(pybindStr)+CountLines(getSetDict)

    for fileName in fileListHeaderOnce:
        WriteFileDict(writeFilesDict, fileName=fileName, 
                      text="\n#endif //#ifdef include once...\n", fileMode='a')

    WriteFileDict(writeFilesDict, fileName=getSetFile, text='} //namespace EPyUtils \n\n'+'\n#endif //#ifdef include once...\n',
                       fileMode='a')

    print('total number of lines generated =',totalNumberOfLines)

    totalNumberOfFilesChanged = 0

    #++++++++++++++++++++++++++++++++++++++++
    #now write files and check changes
    for fileName, text in writeFilesDict.items():
        totalNumberOfFilesChanged += int(WriteTextIfDifferent(fileName, text, True) )

    print('total number of files changed =', totalNumberOfFilesChanged)

    return 0


def WriteFileDict(writeFilesDict, fileName, text, fileMode='a'):
    if fileMode=='w' or (fileName not in writeFilesDict):
        writeFilesDict[fileName] = text
    else:
        writeFilesDict[fileName] += text



if __name__ == '__main__':
    sys.exit(main())
