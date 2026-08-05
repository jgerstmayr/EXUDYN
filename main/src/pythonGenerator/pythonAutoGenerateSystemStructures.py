# -*- coding: utf-8 -*-
"""
Created on Fri May 18 08:53:30 2018

@author: Johannes Gerstmayr

goal: automatically generate interfaces for structures
currently: automatic generate structures with ostream and initialization
"""
import datetime # for current date
from autoGenerateHelper import RemoveSpacesTabs, CountLines, TypeConversion, GenerateHeader, SplitString, Str2Latex,\
                               Str2Doxygen, GetDateStr, CutLinesFromString, PyLatexRST, IsEqualIgnoringDateStrings,\
                               WriteTextIfDifferent, DocStringGoogleFromPlainText

import copy
import io
import os
import numpy as np
from exudynVersion import exudynVersionString

SHOW_PARAMETER_CHANGES = False
sortStructures = True
ADD_DOCSTRINGS = True
    
typeCasts = {'Bool':'bool', 'Int':'Index', 'Real':'Real', 'UInt':'Index', 'PInt':'Index', 
             'UReal':'Real',  'PReal':'Real', 'UFloat':'float',  'PFloat':'float', 
             'Vector':'std::vector<Real>', 'Vector3D':'std::vector<Real>', #'Matrix':'Matrix', 'SymmetricMatrix':'Matrix', 
             'ArrayIndex':'std::vector<Index>', 'ArrayFloat':'std::vector<float>', 'String':'std::string', 'FileName':'std::string',
             'Float2': 'std::array<float,2>', 'Float3': 'std::array<float,3>', 'Float4': 'std::array<float,4>',  #e.g. for OpenGL vectors
             'Float9': 'std::array<float,9>', 'Float16': 'std::array<float,16>', #e.g. for OpenGL rotation matrix and homogenous transformation
             'UInt2': 'std::array<Index,2>', 'UInt3': 'std::array<Index,3>', 'UInt4': 'std::array<Index,4>', 
             'Index2': 'std::array<Index,2>', 'Index3': 'std::array<Index,3>', 'Index4': 'std::array<Index,4>', 
             'KeyPressUserFunction': 'std::function<bool(int, int, int)>',
             'Matrix3D': 'std::array<std::array<Real,3>,3>',
             'Matrix6D': 'std::array<std::array<Real,6>,6>',
             'Vector2DList': 'PyVector2DList',
             } #convert parameter types to C++/Exudyn types

#conversion rules for dictionary 'type'; this type conversion adds rules for the user's values in the dictionary
convertToDict = {'ResizableVector':'Vector', 'StdArray33F':'MatrixFloat', 
                 'NumpyVector':'Vector', 'NumpyMatrix':'Matrix', 
                'Index2':'IndexArray', 'Index4':'IndexArray', 
                'ArrayIndex':'IndexArray', 'ArrayFloat':'VectorFloat',
                'Float4':'VectorFloat', 'Float3':'VectorFloat' #,'String':'std::string'
                }

#for LLMs
convertToPython = {'Real': 'float',
                   'std::string': 'str',
                   'Index': 'int',
                   'ResizableVectorParallel': 'numpy.ndarray',
                   'std::vector<float>': 'numpy.ndarray',
                   'std::array<Index,2>': '[int,int]',
                   'std::array<float,3>': '[float,float,float]',
                   'std::array<float,4>': '[float,float,float,float]',
                   }

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

#just extract and evaluate cflag
def IsDeprecatedParameter(parameter):
    return (parameter['cFlags'].find('X') != -1)

#just extract and evaluate cflag
def IsStructureParameter(parameter):
    return (parameter['cFlags'].find('S') != -1)

#convert parameter to deprecation version and expiration data
def DParameter2VersionExpiration(parameter):
    changedInfo = parameter['defaultValue'] #workaround; contains 'version;EXP=....' where in EXP, the expire year is noted where the deprecated parameter will be removed
    if len(changedInfo.split(';')) < 2 or 'EXP=' not in changedInfo:
        raise ValueError('VersionExpiration: parameter '+str(parameter) + 'has illegal version')
    version = changedInfo.split(';')[0]
    expDate = changedInfo.split(';')[1].replace('EXP=','')
    return (version, expDate)
    

#extract parameterdescription depending on deprecated status (then it is the re-link)
def ParameterDescription(parameter):
    IDP = IsDeprecatedParameter(parameter)
    return 'DEPRECATED; Instead use '*IDP + parameter['parameterDescription']

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


def ParameterDescription2DocString(text):
    if text.strip().startswith('$'): #formula at beginning
        listStrip = text.split('$')
        if len(listStrip) > 2: 
            text = '$'.join(listStrip[2:])
    return text

globalList=[]

#************************************************
#create autogenerated .h  file for list of parameters
def WriteFile(parseInfo, parameterList, typeConversion):
    #print (parseInfo)
    #print('file="'+parseInfo['writeFile']+'"')
    plr = PyLatexRST()
    stubStr = '' #string for .pyi file
    spaces4 = '    '
    
    dateStr = GetDateStr()
    yearStr = dateStr.split('-')[0]

    plr.AddDocu(parseInfo['latexText']) #.replace('\\n','\n') #this is the string for latex documentation
    
    cppText = parseInfo['cppText'] #.replace('\\n','\n') #this is the string for latex documentation
    sGetSetDictionarys = '' #goes into separate file

    #create name for #ifdef macro to include header files only once:
    sHeaderOnce = parseInfo['writeFile'].split('.')[0]

    #************************************
    #header
    s='' #generate a string for the output file
    #s+='//automatically generated file (pythonAutoGenerateInterfaces.py)\n'
    s+='/** ***********************************************************************************************\n'
    s+='* @class        '+parseInfo['class']+'\n'
    s+='* @brief        '+Str2Doxygen(parseInfo['classDescription'])+'\n'
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
    if parseInfo['appendToFile'] != 'True':
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

    pythonClass = parseInfo['class']
    if parseInfo['pythonClass'] != '':
        pythonClass = parseInfo['pythonClass']

    classInitBackLink = ClassHasBackLink(parseInfo['class'])
    classHasBackLink = ClassHasBackLink(parseInfo['class']) and HasTopClass(parseInfo['class'])

    implementationGetSetStr = '' #implementations that come in the end
    parameterChangesList = [] #old and new parameter (full path)

    #create sorted parameter list; distinguish between structures (cFlags have 'S') and values: adds 0/1 before name for sorting ...
    parameterListSorted=sorted(parameterList, 
                               key=lambda d: str(int(d['cFlags'].find('S')==-1))+d['pythonName'].upper())
    if not sortStructures:
        parameterListSorted = parameterList 
    # global globalList
    # if parseInfo['class'] == 'NewtonSettings': globalList = parameterListSorted

    #************************************
    
    #Latex doc:
    hasPybindInterface = False
    for parameter in parameterList:
        if (parameter['lineType'].find('V') != -1) and (parameter['cFlags'].find('P') != -1): #only if it is a member variable
            hasPybindInterface = True

    if (parseInfo['class'] == 'SolverLocalData'
        #or parseInfo['class'] == 'SolverFileData'
        ):
        hasPybindInterface = False
        

    if hasPybindInterface: #otherwise do not include the description into latex doc

        stubStr += '\n#information for '+ pythonClass + '\n'
        stubStr += 'class ' + pythonClass + ':\n'
        if ADD_DOCSTRINGS: 
            stubStr += DocStringGoogleFromPlainText(parseInfo['classDescription'],
                                                    addSpaces=' '*4, multiline=True)
        typicalPaths = []
        if parseInfo['typicalPaths'] != None:
            typicalPaths = parseInfo['typicalPaths']
            class2name = parseInfo['class']

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

        #print('typical:',typicalPaths)

        descriptionStr = parseInfo['classDescription']
        if not descriptionStr.endswith('.'): 
            descriptionStr += '. '
        
        plr.sLatex += '\n%+++++++++++++++++++++++++++++++++++\n'
        plr.AddDocu(Str2Latex(descriptionStr, replaceCurlyBracket=False)+
                    '\n\n\\noindent '+
                    parseInfo['class'] + ' has the following items:\n', 
                    section=parseInfo['class'], sectionLevel=3, 
                    sectionLabel='sec:' + parseInfo['class'].replace(' ',''))
        plr.sRST += '\n' #newline for start of list

        # plr.sLatex += '\mysubsubsection{' + parseInfo['class'] + '} \label{sec:' + parseInfo['class'].replace(' ','') + '}\n'
        # plr.sLatex += Str2Latex(descriptionStr, replaceCurlyBracket=False) + '\\\\ \n'
        # plr.sLatex += '%\n\\noindent '
        # plr.sLatex += parseInfo['class'] + ' has the following items:\n'
        plr.sLatex += '%reference manual TABLE\n'
        plr.sLatex += '\\begin{center}\n'
        plr.sLatex += '  \\footnotesize\n'
        plr.sLatex += '  \\begin{longtable}{| p{4.2cm} | p{2.5cm} | p{0.3cm} | p{3.0cm} | p{6cm} |}\n'
        plr.sLatex += '    \\hline\n'
        plr.sLatex += '    \\bf Name & \\bf type / function return type & \\bf size & \\bf default value / function args & \\bf description \\\\ \\hline\n'
    
        for parameter in parameterListSorted:
            if IsDeprecatedParameter(parameter):
                continue
            if (parameter['lineType'].find('V') != -1 and 
                parameter['cFlags'].find('P') != -1 and
                parameter['type'].find('ResizableVector') == -1): #only if it is a member variable
                
                sString = ''
                if (parameter['type'] == 'String' or parameter['type'] == 'FileName'):
                    sString="'"
                #write latex doc:
                defaultValueStr = parameter['defaultValue']
                paramDescriptionStr = parameter['parameterDescription'].replace('_','\\_')
                if len(defaultValueStr) > 18:
                    paramDescriptionStr = '\\tabnewline ' + paramDescriptionStr
                pythonName = Str2Latex(parameter['pythonName']) 
                typeName = Str2Latex(parameter['type'])
                
                # if len(pythonName)>28:  #inside plr.SystemStructuresWriteDefRow
                #     typeName = '\\tabnewline ' + typeName
                    
                if parameter['type'] != 'String' and parameter['type'] != 'FileName': #don't do this for file names, because 'f' is erased!
                    defaultValueStr = Str2Latex(defaultValueStr, True)

                plr.SystemStructuresWriteDefRow(pythonName, typeName, Str2Latex(parameter['size']), 
                                            sString+defaultValueStr+sString, paramDescriptionStr, 
                                            typicalPaths=typicalPaths, isFunction=False)
                                
                stubStr += spaces4+parameter['pythonName']+': '
                stubStr += TypeConversion(parameter['type'], typeConversionStub) + '\n'
                if ADD_DOCSTRINGS: 
                    stubStr += DocStringGoogleFromPlainText(text=ParameterDescription2DocString(parameter['parameterDescription']),
                                                            addSpaces=' '*4, multiline=False)

            if (parameter['lineType'].find('F') != -1) and (parameter['cFlags'].find('P') != -1): #only if it is a function
                #write latex doc:
                functionName = Str2Latex(parameter['pythonName'])
                argStr = parameter['args']
                if (argStr != ''):
                    #functionName += '(...)' #now added in SystemStructuresWriteDefRow
                    argSplit = argStr.split(',') #split into list of args
                    argStr = ''
                    argSep = '' #no comma for first time
                    for item in argSplit:
                        argName = item.split(' ')[-1] #last word in args is the name of the argument, e.g. in const MainSystem& mainSystem ==> mainSystem
                        argName = Str2Latex(argName)
                        argStr += argSep + argName.replace('=true','=True').replace('=false','=False')
                        argSep = ', '

                functionType = Str2Latex(parameter['type'])
                # if (len(functionName)>28):  #done now in SystemStructuresWriteDefRow
                #     functionType = '\\tabnewline ' + functionType

                # plr.sLatex += '    ' + functionName + ' & '
                # plr.sLatex += '    ' + functionType + ' & '
                # plr.sLatex += '    ' + Str2Latex(parameter['size']) + ' & '

                # plr.sLatex += '    ' + argStr + ' & '
                # plr.sLatex += '    ' + Str2Latex(parameter['parameterDescription'], replaceCurlyBracket=False) + '\\\\ \\hline\n' #Str2Latex not used, must be latex compatible!!!

                plr.SystemStructuresWriteDefRow(functionName, functionType, Str2Latex(parameter['size']), argStr, 
                                            Str2Latex(parameter['parameterDescription'], replaceCurlyBracket=False), isFunction=True)

                stubStr += spaces4+'@overload\n'
                stubStr += spaces4+'def '+functionName+'('+argStr.replace('\\_','_')+')'+' -> '+TypeConversion(parameter['type'], typeConversionStub)+': ...\n'
                
        plr.sLatex += '	  \\end{longtable}\n'
        plr.sLatex += '	\\end{center}\n'


    #************************************
    #class definition:
    strParentClass = ''
    constructorParentClass = ''
    if len(parseInfo['parentClass']) != 0:
        strParentClass = ': public ' + parseInfo['parentClass']
        constructorParentClass = ': '+parseInfo['parentClass']+'()'
    s+='class ' + parseInfo['class'] + strParentClass + ' // AUTO: \n'
    s+='{\n'


    #************************************
    #member variables:
    sPublic = ''
    sPrivate = ''
    sProtected = ''
    for parameter in parameterListSorted:
        if IsDeprecatedParameter(parameter) and not IsStructureParameter(parameter): #for structures, there is no replacement; only for values
            for path in typicalPaths:
                oldParameterStr = path.replace('SC.','') +'.'+ parameter['pythonName']
                baseParameter = oldParameterStr.split('.')[0]
                newParameterStr = baseParameter+'.'+parameter['parameterDescription']
                (version, expDate) = DParameter2VersionExpiration(parameter)
                parameterChangesList.append([oldParameterStr, newParameterStr, version, expDate])

        if ((parameter['lineType'].find('V') != -1) and 
            (parameter['lineType'].find('L') == -1) and 
            (parameter['cplusplusName'].find('.') == -1) and 
            (not IsDeprecatedParameter(parameter) or IsStructureParameter(parameter)) ): #only if it is a member variable, but not linked
            typeStr = TypeConversion(parameter['type'], typeConversion)
            temp = '  ' + typeStr + ' ' + parameter['cplusplusName']+ ';'
            nChar = len(temp)
            alignment = 50
            insertSpaces = ''
            if nChar < alignment:
                insertSpaces = ' '*(alignment-nChar)
            temp += insertSpaces + '//!< AUTO: ' + Str2Doxygen(ParameterDescription(parameter)) + '\n'

            if (parameter['lineType'].find('p') != -1): #make variable private ==> no direct access via C++ or python!
                sPrivate += temp
            else:
                if (parameter['cFlags'].find('P') != -1): #pybind of member variables in this case done via public member variable
                    sPublic += temp
                else:
                    sProtected += temp

    if classHasBackLink:
        #print('class with backlink:',parseInfo['class'])
        sPrivate += '  ' + TopClassName(parseInfo['class']) + '* backlink; //!< AUTO: backlink for global access of structure\n'

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
        if (parameter['lineType'].find('V') != -1): #only if it is a member variable
            strDefault = parameter['defaultValue']
            if len(strDefault): 
                cntDefaultParameters += 1

    #constructor with default initialization:
    if classInitBackLink or cntDefaultParameters or len(parseInfo['addConstructor']) != 0:
        s+='  //! AUTO: default constructor with parameter initialization\n'
        s+='  '+parseInfo['class']+'()'+constructorParentClass+'\n'
        s+='  {\n'
        if classHasBackLink:
            #print('has backlink:',parseInfo['class'])
            s+='    '+'backlink=nullptr;\n'

    
        for parameter in parameterListSorted:
            if (parameter['lineType'].find('V') != -1) and not IsDeprecatedParameter(parameter): #only if it is a member variable
                strDefault = parameter['defaultValue']
                if len(strDefault): #only add initialization if default value exists
                    if parameter['type'] == 'String' or parameter['type'] == 'FileName':
                        strDefault = '"' + strDefault + '"'
                    s+='    ' + parameter['cplusplusName'] + ' = ' + strDefault + ';\n'
        s+=parseInfo['addConstructor'].replace('\\n','\n')
        s+='  };\n'

    #++++++++++++
    #add initializatio nof backlink
    if classInitBackLink:
        s += '  void Init('+TopClassName(parseInfo['class']) + '* backlinkInit) //!< AUTO: called from parent structure\n'
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
#    s+='  virtual ' + parseInfo['class'] + '* GetClone() const { return new '+parseInfo['class']+'(*this); }\n'
#    s+='  \n'
    sDictGet = ''
    sDictGet += '//! AUTO: read access to structure; converting into dictionary\n'
    sDictGet += 'inline py::dict GetDictionaryWithTypeInfo(const ' + parseInfo['class'] + '& data) {\n'
    sDictGet += '    auto structureDict = py::dict();\n'
    sDictGet += '    auto d = py::dict(); //local dict\n'

    sDictGetPure = ''
    sDictGetPure += '//! AUTO: read access to structure; converting into dictionary without type info\n'
    sDictGetPure += 'inline py::dict GetDictionary(const ' + parseInfo['class'] + '& data) {\n'
    sDictGetPure += '    auto structureDict = py::dict();\n'

    sDictSet = ''
    sDictSet += '//! AUTO: write access to data structure; converting dictionary d into structure\n'
    sDictSet += 'inline void SetDictionary(' + parseInfo['class'] + '& data, const py::dict& d) {\n'
    
    #************************************
    #access functions and dictionaries for visualization dialog ...:
    
    if (parseInfo['class'] == 'VisualizationSettings'):
        parameterListSorted2 = copy.deepcopy(parameterList) #unsorted, sorting as in definition file
    else:
        parameterListSorted2 = copy.deepcopy(parameterListSorted) 
        
        
    parameterInfo = [] #list for exporting dictionaries
    for parameter in parameterListSorted2:
        if (parameter['lineType'].find('V') != -1): #only if it is a member variable
            ISP = bool(IsStructureParameter(parameter))
            IDP = bool(IsDeprecatedParameter(parameter))
            IDPNS = bool(IsDeprecatedParameter(parameter)) and not ISP #IDP but not structure

            lineBreakIDP = ''
            deprecationWarning = ''
            if IDPNS:
                lineBreakIDP = '\n    '
                deprecationWarning = 'PyWarning("VisualizationSettings parameter '
                deprecationWarning += ConvertClassName2member(parseInfo['class'])+'.'+parameter['pythonName']
                deprecationWarning += ' is deprecated! use '+parameter['parameterDescription']+' instead!");'+lineBreakIDP
                (version, expDate) = DParameter2VersionExpiration(parameter)
                if expDate <= yearStr:
                    print('parameter outdated '+expDate+':', parseInfo['class']+'::'+parameter['cplusplusName'])
                    continue #not included any more with backlinks!
                
                
            origType = parameter['type']
            typeStr = TypeConversion(parameter['type'], typeConversion)
            paramStr = parameter['cplusplusName']
            paramAccessStr = paramStr if not IDPNS else 'backlink->'+parameter['parameterDescription']

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
            if parameter['cFlags'].find('A') != -1 and not IDPNS:
                accessWritten = True
                s+='  //! AUTO: Read (Reference) access to: ' + Str2Doxygen(ParameterDescription(parameter)) + '\n'
                s+='  const ' + typeStr + refChar + ' '
                s+='Get' + functionStr + '() const { return '+paramAccessStr+'; }\n'
    
                s+='  //! AUTO: Write (Reference) access to: ' + Str2Doxygen(ParameterDescription(parameter)) + '\n'
                s+='  '+typeStr + '&' + ' '
                s+='Get' + functionStr + '() { return ' + paramAccessStr + '; }\n'

            typeWithRangeCheck = IsTypeWithRangeCheck(origType)
            typeWithGetSetFunction = IsTypeWithSetGetFunction(origType) or IDPNS
            
            getFunction = [] #return type, function decl, impl
            setFunction = [] #return type, function decl, impl
                
            typeCastStr = TypeConversion(parameter['type'], typeCasts)
            if (((typeCastStr.find('std::vector') != -1 or typeCastStr.find('std::array') != -1) and 
                 typeCastStr.find('std::ofstream') == -1 and typeCastStr.find('ExuFile::BinaryFileSettings') == -1) or 
                typeWithRangeCheck or typeWithGetSetFunction or
                (parameter['lineType'].find('L') == -1  and parameter['cplusplusName'].find('.') != -1)): #then it must get a set/get function!
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
                        paramInitStr = 'EPyUtils::SetConstMatrixTemplateSafely<'+str(matDim)+', '+str(matDim)+'>('+paramStrPure+'Init, '+ paramStrPure+')'

                    setFunction.append('PySet' + functionStr + '(const ' + typeCastStr + refChar + ' ' + paramStrPure + 'Init) ')
                    setFunction.append('{ ' + lineBreakIDP + deprecationWarning + paramInitStr+'; '+lineBreakIDP+'}\n')
                        
                    if typeStr == 'Matrix3D' or typeStr == 'Matrix6D': #Matrix type (Matrix3D, ...)
                        getReturnStr = 'py::array_t<Real>' #this makes a numpy array instead of list of lists!
                        typeCastStr = 'EPyUtils::Matrix2NumPyTemplate'

                
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
                cn = parseInfo['class']
                implementationGetSetStr += 'inline ' + setFunction[0] + cn + '::'+setFunction[1] + setFunction[2]
                implementationGetSetStr += 'inline ' + getFunction[0] + cn + '::'+getFunction[1] + getFunction[2]
                # print('implementationGetSetStr:',implementationGetSetStr)
            
            #++++++++++++++++++++++++++++++++++++++++++++++++++++++
            #read/write dictionary from hierarchical structure
            if parameter['cFlags'].find('P') != -1 and parameter['cFlags'].find('D') == -1:

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
                    pType = TypeConversion(parameter['type'], convertToDict)
                    #convert size
                    pSize = parameter['size']
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
                    typeCastStr = TypeConversion(parameter['type'], typeCasts)
                    
                    if parameter['type'] != 'KeyPressUserFunction' and not IsDeprecatedParameter(parameter): #this would not work for editing dictionary
                        #get functions:
                        sDictGet += '    d = py::dict(); //reset local dict\n'
                        sDictGet += '    d["itemIdentifier"] = std::string(""); //identifier for item\n'
                        sDictGet += '    d["value"] = data.' + cValueStr + ';\n'
                        sDictGet += '    d["type"] = "' + pType + '";\n'
                        sDictGet += '    d["size"] = std::vector<int>' + pSize + ';\n' #only used for vectors/matrices (e.g. '3') and matrices (e.g. '3x3')
                        sDictGet += '    d["description"] = "' + descrStr + '";\n'
                        sDictGet += '    structureDict["' + parameter['pythonName'] + '"] = d;\n' #keyName used to identify the object
                        sDictGet += '\n'

                        sDictGetPure += '    structureDict["' + parameter['pythonName'] + '"] = '
                        sDictGetPure += 'data.' + cValueStr + ';\n'
                        
                        #set functions:
                        sDictSet += '    data.' + parameter['cplusplusName'] + ' = py::cast<' + typeCastStr + '>(d["' + parameter['pythonName']  + '"]);\n'
                        
                        #dict for exporting
                        pythonTypeStr = typeCastStr
                        if pythonTypeStr in convertToPython:
                            pythonTypeStr = convertToPython[pythonTypeStr]
                        # print(pythonTypeStr)
                            
                        parameterDict={'name': parameter['pythonName'],
                                       'type': pythonTypeStr,
                                       'description': parameter['parameterDescription'].replace("\\_","_"),
                                       'size': pSize,
                                       'default': parameter['defaultValue'],
                            }
                        parameterInfo.append(parameterDict)
                        
            #++++++++++++++++++++++++++++++++++++++++++++++++++++++
    
    
        else: # linked variable
            if parameter['lineType'].find('L') == -1:
                strVirtual = ''
                strOverride = ''
                if (parameter['lineType'].find('v') != -1):
                    strVirtual = 'virtual '
                    strOverride = ' override'
                
                typeStr = TypeConversion(parameter['type'], typeConversion)
                functionStr = parameter['cplusplusName']
                argsStr = parameter['args']
                strConst = ""
                if parameter['cFlags'].find('C') != -1:
                    strConst = " const"
                
                strDef = ''            
                if parameter['cFlags'].find('D') != -1:
                    strDef = ';'
                else:
                    strDef = ' {\n    ' + parameter['defaultValue'] + '\n  }\n' #defaultValue is the function body
    
                s+='  //! AUTO: ' + Str2Doxygen(parameter['parameterDescription']) + '\n'
                s+='  '+strVirtual + typeStr + ' '
                s+=functionStr + '(' + argsStr + ')' + strConst + strOverride + strDef + '\n'
    
        
    sDictGet += '    return structureDict;\n'
    sDictGet += '}\n\n'
    sDictGetPure += '    return structureDict;\n'
    sDictGetPure += '}\n\n'
    sDictSet += '}\n\n'

        
    if ClassHasGetSetDictionary(parseInfo['class']):
        sGetSetDictionarys += sDictGet
        sGetSetDictionarys += sDictGetPure
        sGetSetDictionarys += sDictSet
        #s += '  //! AUTO: read access to structure; converting into dictionary\n'
        #s += '  py::dict GetDictionaryWithTypeInfo() const;\n' #don't do that as we do not want to add pybind to settings files!

    #************************************
    #ostream operator:
#    s+=('  friend std::ostream& operator<<(std::ostream& os, const ' + 
#       parseInfo['class'] + '& object);\n')

    s+='  //! AUTO: print function used in ostream operator (print is virtual and can thus be overloaded)\n'
    s+='  virtual void Print(std::ostream& os) const\n'
    s+='  {\n'
    s+='    os << "' + parseInfo['class'] + '" << ":\\n";\n'
    if len(parseInfo['parentClass']) != 0:
        s+='    os << ":"; \n'
        s+='    ' + parseInfo['parentClass'] + '::Print(os);\n'
        
    #output each parameter
    for parameter in parameterListSorted:
        if ((parameter['lineType'].find('V') != -1) and (parameter['lineType'].find('L') == -1) and 
        (parameter['type']!='TemporaryComputationData') and (parameter['type']!='TemporaryComputationDataArray') and 
        (parameter['type'].find('std::ofstream')==-1) and #(parameter['type'].find('ExuFile::BinaryFileSettings')==-1) and 
        (parameter['type'].find('std::vector<Vector2D>')==-1) and (parameter['type'].find('CrossSectionType')==-1) and
        (parameter['type'].find('userFunction')==-1) and (parameter['type'].find('UserFunction')==-1) and
        not IsDeprecatedParameter(parameter)): #only if it is a member variable; some types not printable
            paramStr = parameter['cplusplusName']
            typeStr = TypeConversion(parameter['type'], typeConversion)
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

    if len(parseInfo['parentClass']) == 0:
        s+=('  friend std::ostream& operator<<(std::ostream& os, const ' + parseInfo['class'] + '& object)\n')
        s+= '  {\n'
        s+= '    object.Print(os);\n'
        s+= '    return os;\n'
        s+= '  }\n\n' # end ostream operator
    
    s+='};\n\n\n' #class

    if len(parameterChangesList) and SHOW_PARAMETER_CHANGES:
        #print('parameter changes:\n',parameterChangesList,sep='')
        for pChange in parameterChangesList:
            print(pChange[0]+' → '+pChange[1])
        
    
    return [s, plr.sLatex, sGetSetDictionarys, plr.sRST, stubStr, 
            parameterInfo, implementationGetSetStr, parameterChangesList]

#**************************************************************************************
#**************************************************************************************
#**************************************************************************************
#create string containing the pybind11 headers/modules for a class
def CreatePybindHeaders(parseInfo, parameterList, typeConversion):
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
    pythonClass = parseInfo['class']
    if parseInfo['pythonClass'] != '':
        pythonClass = parseInfo['pythonClass']
        #print('pythonClass=', pythonClass, ', cClass=', parseInfo['class'])
        
        
#    if len(parseInfo['parentClass']) != 0: #derived class does not work in pybind, if parent class is not defined!
#        parentClass = ', ' + parseInfo['parentClass']
    s += spaces1 + 'py::class_<' + parseInfo['class'] + parentClass + '>(m, "' + pythonClass + '"'
    if addDocuClass:
        s += ', "'+pythonClass+' class"'
    s += ') // AUTO: \n'
    s += spaces2 + '.def(py::init<>())\n'

    #create sorted parameter list; distinguish between structures (cFlags have 'S') and values: adds 0/1 before name for sorting ...
    parameterListSorted=sorted(parameterList, 
                               key=lambda d: str(int(d['cFlags'].find('S')==-1))+d['pythonName'].upper())
    if not sortStructures:
        parameterListSorted = parameterList 

    #************************************
    #member variables access:
    for parameter in parameterListSorted:
        if (parameter['lineType'].find('V') != -1) and (parameter['cFlags'].find('P') != -1): #only if it is a member variable
            ISP = bool(IsStructureParameter(parameter))
            IDP = bool(IsDeprecatedParameter(parameter))

            typeCastStr = TypeConversion(parameter['type'], typeCasts)
            linkedClassStr = ''
            if (len(parseInfo['linkedClass']) != 0):
                linkedClassStr = parseInfo['linkedClass'] + '.'

            if ((typeCastStr.find('std::vector') == -1) and (typeCastStr.find('std::array') == -1) and 
            (parameter['lineType'].find('L') == -1) and (parameter['cplusplusName'].find('.') == -1)
            and not IsTypeWithRangeCheck(parameter['type']) 
            and not (IsTypeWithSetGetFunction(parameter['type']) or (IDP and not ISP))): #then it has a set/get function! e.g. Int2, Int3, Float2, Float3, .... are array structures ==> must be converted
                s += spaces2 + '.def_readwrite("' + parameter['pythonName'] + '", &' + parseInfo['class'] + '::' + linkedClassStr + parameter['cplusplusName']
                if addDocuMember:
                    #s += ', "member: ' + parameter['pythonName'] + '"'
                    s += ', "' + CleanPyDocStrings(parameter['parameterDescription']) + '"'
                s += ')\n' #extend this to incorporate 'read only' and other flags
            else:
                sReturnValueProperty = '' #for structures that should also have write access
                #not needed: done automatically as reference access for such structures with get/set function
                # if parameter['lineType'].find('L') != -1:
                #     sReturnValueProperty += ', py::return_value_policy::reference'
                #access with setter/getter functions and conversions to std::vector
                #functionName = parameter['cplusplusName']
                functionName = parameter['pythonName'] #for linked variables, this is easier to work with linking e.g. to cSolver
                functionName = functionName[0].upper()+functionName[1:]
                s += spaces2 + '.def_property("' + parameter['pythonName'] + '", '
                s += '&' + parseInfo['class'] + '::PyGet' + functionName + ', '
                if parameter['type'] != 'KeyPressUserFunction' or IsDeprecatedParameter(parameter):
                    s += '&' + parseInfo['class'] + '::PySet' + functionName + sReturnValueProperty 
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
    s += spaces2 + '// AUTO: access functions for ' + parseInfo['class'] + '\n'
            
    for parameter in parameterListSorted:
        if (parameter['lineType'].find('F') != -1) and (parameter['cFlags'].find('P') != -1): #only if it is a member function
            s += spaces2 + '.def("' + parameter['pythonName']
            s += '", &' + parseInfo['class'] + '::' + parameter['pythonName']
            if parameter['type'] != 'void': #check return_value_policy if not void
                if parameter['cFlags'].find('V') != -1: #pass by value (copy)
                    s += ', py::return_value_policy::copy'
                else:
                    s += ', py::return_value_policy::reference' #extend this to incorporate 'read only' and other flags
            s += ', "' + RemoveLatexCommands(ParameterDescription(parameter)) + '"'
            if (parameter['cFlags'].find('G') != -1): #add py::arg() in order that type completion shows args in python
                argStr = parameter['args']
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
    
    s += spaces2 + '.def("__repr__", [](const ' + parseInfo['class'] + ' &item) { return "<' + parseInfo['class'] + ':\\n" + EXUstd::ToString(item) + " >"; } ) //!< AUTO: add representation for object based on ostream operator\n'
    
    if parseInfo['addDictionaryAccess'] == 'True':
        s += spaces2 + '.def("GetDictionaryWithTypeInfo", [](const ' + parseInfo['class'] + ' &item) { return EPyUtils::GetDictionaryWithTypeInfo(item); }) //!< AUTO: add read as dictionary with type information access\n'
    if ClassHasGetSetDictionary(parseInfo['class']):
        s += spaces2 + '.def("GetDictionary", [](const ' + parseInfo['class'] + ' &item) { return EPyUtils::GetDictionary(item); }) //!< AUTO: add read for dictionary access\n'
        s += spaces2 + '.def("SetDictionary", [](' + parseInfo['class'] + ' &item, const py::dict& d) { return EPyUtils::SetDictionary(item, d); }) //!< AUTO: add write from dictionary access\n'
        s += pickleDictTemplate.replace('{ClassName}', parseInfo['class'])
#		.def("GetDictionary", [](const VisualizationSettings &item) { return EPyUtils::GetDictionaryWithTypeInfo(item); }) //!< AUTO: add representation for object based on ostream operator

    s += spaces2 + '; // AUTO: end of class definition!!!\n'
    s += '\n'

    s += spaces1 + '//++++++++++++++++++++++++++++++++\n' #end of pybind11 definition


    return s



#************************************************
#MAIN CONVERSION
#************************************************
    
rstFileDict={'SimulationSettings':'',
             'VisualizationSettings':'',
             'CSolverStructures':'',
             'MainSolver':'',
             'PyStructuralElementsDataStructures':'',
             'BeamSectionGeometry':'',
             } #contains available file names and text


#added structures for creating dictionaries for settings; saved via numpy save
dictSystemStructures = {}
dictSystemStructures['structures'] = [] #contains list of modules
dictSystemStructures['version'] = exudynVersionString
dictSystemStructures['name'] = 'systemStructures'

def WriteFileDict(writeFilesDict, fileName, text, fileMode='a'):
    if fileMode=='w' or (fileName not in writeFilesDict):
        writeFilesDict[fileName] = text
    else:
        writeFilesDict[fileName] += text

try: #still close file if crashes
    print('******************************')
    print('Autogenerate system structures')

    #create Python/pybind11 file
    directoryString = '../Autogenerated/'
    pybindFile = directoryString+'pybind_modules.h'
    getSetFile = directoryString+'DictionariesGetSet.h'
    latexFile = '../../../docs/theDoc/interfaces.tex'
    stubFile  = '../../../main/src/pythonGenerator/generated/stubSystemStructures.pyi'
    
    writeFilesDict = {} #this will be written finally, ignoring header section!

    # file=open(pybindFile,'w')  #clear file by one write access
    # file.write('// AUTO:  ++++++++++++++++++++++\n')
    # file.write('// AUTO:  pybind11 module includes; generated by Johannes Gerstmayr\n')
    # file.write('// AUTO:  last modified = '+ GetDateStr() + '\n')
    # file.write('// AUTO:  ++++++++++++++++++++++\n\n')
    # file.close()
    WriteFileDict(writeFilesDict, fileName=pybindFile, text=''+
                  '// AUTO:  ++++++++++++++++++++++\n'+
                  '// AUTO:  pybind11 module includes; generated by Johannes Gerstmayr\n'+
                  '// AUTO:  last modified = '+ GetDateStr() + '\n'+
                  '// AUTO:  ++++++++++++++++++++++\n\n', fileMode='w')
    
    # file=open(getSetFile,'w')  #clear file by one write access
    # file.write('// AUTO:  ++++++++++++++++++++++\n')
    # file.write('// AUTO:  Helper file for dictionaries get/set for system structures; generated by Johannes Gerstmayr\n')
    # file.write('// AUTO:  Generated by Johannes Gerstmayr\n')
    # file.write('// AUTO:  Used for SimulationSettings and VisualizationSettings\n')
    # file.write('// AUTO:  last modified = '+ GetDateStr() + '\n')
    # file.write('// AUTO:  ++++++++++++++++++++++\n\n')
    # file.write('// AUTO:  ++++++++++++++++++++++\n')
    # file.write('// AUTO:  Helper file for dictionaries get/set for system structures; generated by Johannes Gerstmayr\n')
    # file.write('// AUTO:  Generated by Johannes Gerstmayr\n')
    # file.write('// AUTO:  Used for SimulationSettings and VisualizationSettings\n')
    # file.write('// AUTO:  last modified = '+ GetDateStr() + '\n')
    # file.write('// AUTO:  ++++++++++++++++++++++\n\n')

    
    # file.write('  #ifndef DICTIONARIESGETSET__H\n')
    # file.write('  #define DICTIONARIESGETSET__H\n\n')

    # file.write('  #include "Linalg/BasicLinalg.h"\n')
    # file.write('  #include "Main/CSystem.h"\n')
    # file.write('  #include "Autogenerated/SimulationSettings.h"\n')
    # file.write('  #include "Autogenerated/VisualizationSettings.h"\n\n')
    # file.write('  #include <pybind11/pybind11.h>\n')
    # file.write('  #include <pybind11/stl.h>\n')
    # file.write('  #include <pybind11/stl_bind.h>\n')
    # file.write('  namespace py = pybind11;\n\n')
    # file.write('  namespace EPyUtils {\n //add namespace for access to dictionaries')
    # file.close()

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

    #read system definition
    filename = "systemStructuresDefinition.py"
    totalNumberOfLines = 0

    file=open(filename,'r', encoding='utf8') 
    fileLines = file.readlines()
    file.close()
    
    typeConversion = {'Bool':'bool', 'Int':'Index', 'Real':'Real', 'UInt':'Index', 'PInt':'Index', 
                      'UReal':'Real', 'PReal':'Real', 'UFloat':'float',  'PFloat':'float', 
                      'Vector':'Vector', 
                      'Matrix':'Matrix', 'SymmetricMatrix':'Vector', 
                      'NumpyMatrix':'py::array_t<Real>', 'NumpyVector':'py::array_t<Real>', 
                      'String':'std::string', 'FileName':'std::string',
                      'KeyPressUserFunction': 'std::function<bool(int, int, int)>'} #convert parameter types to C++/EXUDYN types

    typeConversionStub = {'Bool':'bool', 'Int':'int', 'Index':'int', 'UInt':'int', 'PInt':'int', 
                      'Real':'float', 'UReal':'float', 'PReal':'float', 'UFloat':'float',  'PFloat':'float', 
                      'Float3':'Tuple[float,float,float]', 
                      'Float4':'Tuple[float,float,float,float]', 'Vector':'List[float]', 
                      'Matrix':'ArrayLike', 'Matrix3D':'ArrayLike', 'Matrix6D':'ArrayLike', 
                      'SymmetricMatrix':'ArrayLike', 
                      'NumpyMatrix':'ArrayLike', 'NumpyVector':'ArrayLike', 'StdArray33F':'ArrayLike',
                      'String':'str', 'FileName':'str', 'Index2':'Tuple[int,int]', 
                      'KeyPressUserFunction': 'Any',
                      'std::string':'str', 'void':'None', 
                      'ArrayIndex':'List[int]','ArrayFloat':'List[float]',
                      } #conversion for stub files

    parseInfo = {'class':'',            # C++ class name
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
    nparam = len(lineDefinition)
    
    
    mode = 0 #1...read parameterlist , 0...read definitions
    linecnt = 1
    
    parameterList = [] #list of dictionaries for parameters
    continueOperation = True #flag to signal that operation shall be terminated
    
    globalLatexStr = '' #this is the whole string for the latex docu
    globalStubStr = ''  #will be written into .pyi file
    globalImplementationGetSetStr = '' #implementation part added at end of each structure (visualization, etc.)
    globalParameterChangesList = []
    
    fileListHeaderOnce = [] #store all opened files, which get a "#endif " at the end for the #ifdef ... at the beginning
    
    for line in fileLines:
        if continueOperation:
            if line[0] != '#':
                pureline = (line.strip('\n')) #eliminate EOL
                if len(pureline.replace(' ','')): #empty lines are ignored
                    if (pureline[0] == 'V') or (pureline[0] == 'F'):
                        #must be definition of parameters
                        info = SplitString(pureline, linecnt)
                        #print(info)
                        if len(info) != len(lineDefinition):
                            continueOperation = False
                        d={} #empty dictionary
                        
                        cnt = 0
                        for item in lineDefinition:
                            d[lineDefinition[cnt]] = info[cnt]
                            cnt+=1
                        if len(d['cplusplusName']) == 0:
                            d['cplusplusName'] = d['pythonName']
                        #print(d)
                        parameterList.append(d) #append parameter dictionary to list
                    elif (pureline.find('=') != -1): #definition
    #                    pureline = pureline.replace(' ','') #eliminate spaces and EOL
                        info = pureline.split('=', maxsplit = 1)
                        #print("info =", info)
                        if (len(info) == 2):
                            defName = info[0].replace(' ','')
                            #print("defname =",defName)
                            RHS = RemoveSpacesTabs(info[1])
                            if RHS != None:
                                if len(RHS) and RHS[0] == "'":
                                    RHS = RHS.strip("'")
                                else:
                                    RHS = RHS.strip('"')
                                if (defName != 'classDescription' and defName != 'latexText' and defName != 'cppText' and defName != 'addConstructor'):
                                    RHS = RHS.replace(' ','')
                                if (defName == 'classDescription' or defName == 'latexText' or defName == 'cppText'):
                                    RHS = RHS.replace('\\n','\n') #enable line breaks!
                                
                            if (defName in parseInfo):
                                parseInfo[defName] = RHS
                                #print(parseinfo)
                            else:
                                print("ERROR: invalid specifier", defName, "in line",linecnt)    
                                continueOperation = False
                            if (defName == "class"):
                                if (mode == 0):
                                    mode = 1
                                else:
                                    print("ERROR: did not expect 'class' keyword in line",linecnt)    
                                    continueOperation = False                       
                            if (defName == "writeFile"):
                                if (mode == 1):
                                    mode = 0
                                    #++++++++++++++++++++++++++++++
                                    #now write C++ header file for defined class
                                    [fileStr, latexStr, getSetDict, rstStr, stubStr, parameterInfo, 
                                     implementationGetSetStr, parameterChangesList] = WriteFile(parseInfo, parameterList, typeConversion)
                                    globalLatexStr += latexStr
                                    globalStubStr += stubStr
                                    globalImplementationGetSetStr += implementationGetSetStr
                                    globalParameterChangesList += parameterChangesList
                                    strFileMode = 'w'
                                    
                                    if parseInfo['appendToFile'] == 'True':
                                        strFileMode = 'a'
                                    else:
                                        fileListHeaderOnce += [directoryString+parseInfo['writeFile']]
                                        
                                    if not HasTopClass(parseInfo['class']) and globalImplementationGetSetStr != '':
                                        fileStr += '\n\n//! implementation:\n'+globalImplementationGetSetStr
                                        #print(parseInfo['class']+'-IMPL:',globalImplementationGetSetStr)
                                        (globalLatexStr, rstStr) = ParameterChanges2LatexRST(globalParameterChangesList, globalLatexStr, rstStr)
                                        globalParameterChangesList = []
                                        globalImplementationGetSetStr = ''
                                        
                                    # file=open(directoryString+parseInfo['writeFile'],strFileMode) 
                                    # file.write(fileStr)
                                    # file.close()
                                    WriteFileDict(writeFilesDict, fileName=directoryString+parseInfo['writeFile'], 
                                                  text=fileStr, fileMode=strFileMode)


                                    fileName = parseInfo['writeFile'].split('.')[0]
                                    if fileName in rstFileDict:
                                       rstFileDict[fileName] += rstStr

                                    #++++++++++++++++++++++++++++++
                                    #write Python/pybind11 includes
                                    if parseInfo['writePybindIncludes'] == 'True':
                                        pybindStr = CreatePybindHeaders(parseInfo, parameterList, typeConversion)
                                        # file=open(pybindFile,'a')  #always append to pybind file
                                        # file.write(pybindStr)
                                        # file.close()
                                        WriteFileDict(writeFilesDict, fileName=pybindFile, 
                                                      text=pybindStr, fileMode='a')
                                        
                                    if parseInfo['writePybindIncludes'] == 'True':
                                        # file=open(getSetFile,'a')  #always append to pybind file
                                        # file.write(getSetDict)
                                        # file.close()
                                        WriteFileDict(writeFilesDict, fileName=getSetFile, 
                                                      text=getSetDict, fileMode='a')
                                    
                                    if len(parameterInfo) != 0:
                                        classDict = {'class':parseInfo['class'],
                                                     'description':parseInfo['classDescription'],
                                                     'typicalPaths':copy.copy(parseInfo['typicalPaths']),
                                                     'parameters':copy.copy(parameterInfo)
                                                     }
                                        dictSystemStructures['structures'].append(classDict)
                                    
                                    #++++++++++++++++++++++++++++++
                                    #reset structures for next file
                                    totalNumberOfLines += CountLines(fileStr)+CountLines(pybindStr)+CountLines(getSetDict)
                                    parameterList = [] #reset list
                                    parseInfo['appendToFile'] = 'False'
                                    parseInfo['writeFile'] = ''
                                    parseInfo['class'] = ''
                                    parseInfo['parentClass'] = ''
                                    parseInfo['pythonClass'] = ''
                                    parseInfo['classDescription'] = ''
                                    parseInfo['addConstructor'] = ''
                                    parseInfo['linkedClass'] = ''
                                    parseInfo['latexText'] = ''
                                    parseInfo['typicalPaths'] = None #not registered ...
                                    parseInfo['cppText'] = ''
                                    parseInfo['writePybindIncludes'] = 'False'
                                    parseInfo['addDictionaryAccess'] = 'False'
                                else:
                                    print("ERROR: did not expect 'writeFile' keyword in line",linecnt)    
                                    continueOperation = False
                                
                        else:
                            print("ERROR: definition mismatch in line",linecnt)    
                            continueOperation = False
                    else: #ERROR
                        print("ERROR: unknown format of line",linecnt)    
            linecnt += 1;
    
    if (continueOperation == False):
        print('\n\nERROR: Parsing terminated unexpectedly in line',linecnt,'\n\n')

    for fileName in fileListHeaderOnce:
        # file=open(fileName,'a') 
        # file.write("\n#endif //#ifdef include once...\n")
        # file.close()
        WriteFileDict(writeFilesDict, fileName=fileName, 
                      text="\n#endif //#ifdef include once...\n", fileMode='a')

    # fileGetSet=open(getSetFile,'a')  #clear file by one write access
    # fileGetSet.write('} //namespace EPyUtils \n\n')
    # fileGetSet.write("\n#endif //#ifdef include once...\n")
    # fileGetSet.close()
    WriteFileDict(writeFilesDict, fileName=getSetFile, text='} //namespace EPyUtils \n\n'+'\n#endif //#ifdef include once...\n',
                       fileMode='a')

    print('total number of lines generated =',totalNumberOfLines)

    totalNumberOfFilesChanged = 0

    #++++++++++++++++++++++++++++++++++++++++
    #now write files and check changes
    for fileName, text in writeFilesDict.items():
        totalNumberOfFilesChanged += int(WriteTextIfDifferent(fileName, text, True) )

    print('total number of files changed =', totalNumberOfFilesChanged)
    
    latexText = """
This section includes the reference manual for structures (such as for solvers, helper structures, etc.) 
and settings which are available in the python interface, e.g., simulation settings, visualization settings. 
The data is auto-generated from the according interfaces in order to keep fully up-to-date with changes.
"""
    globalLatexStr += latexText
    
    fileLatex=open(latexFile,'w')  #clear file by one write access
    fileLatex.write('% definition of structures\n')
    fileLatex.write(globalLatexStr)
    fileLatex.close()

    globalStubStr = """
#This is the stub file for system structures, such as SimulationSettings and VisualizationSettings
#This file will greatly improve autocompletion

""" + globalStubStr

    fileStub=open(stubFile,'w',encoding='utf8')  #clear file by one write access
    fileStub.write(globalStubStr)
    fileStub.close()

    rstDir = '../../../docs/RST/structures/'
    rstIndex = """
=======================
Structures and Settings
=======================
"""
    rstIndex += latexText
    rstIndex += """
.. toctree::
   :maxdepth: 2
    
"""
    
    
    for key, value in rstFileDict.items():
        # print('RST: write '+key)#, ':',value[:200])
        file=io.open(rstDir+key+'.rst','w',encoding='utf8')  #clear file by one write access
        file.write(value+'\n')
        file.close()
        rstIndex += '   '+key+'\n'

    file=io.open(rstDir+'StructuresAndSettingsIndex.rst','w',encoding='utf8')  #clear file by one write access
    file.write(rstIndex+'\n')
    file.close()



    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    structList = dictSystemStructures['structures']
    paramCnt = 0
    for pList in structList: 
        paramCnt+=len(pList['parameters'])
    print('total number of parameters (excl. classes):',paramCnt)

    np.save('generated/systemStructuresData.npy', dictSystemStructures)
    #utilitiesData = np.load('generated/systemStructuresData.npy', allow_pickle=True).item()

finally:    
    file.close()



