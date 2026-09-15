#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits the per-item C++ headers C<Item>.h, Main<Item>.h and Visu<Item>.h, and the two
#           user-function headers PySymbolicUserFunctionSet.h / PythonUserFunctionsTemplates.h,
#           and the item auto-registration objectFactoryAutoReg.h,
#           from definitions/ (revision plan step 33, part 2b). The code was MOVED out of
#           src/pythonGenerator/pythonAutoGenerateObjects.py; the output is byte-identical.
#
#           It reads the members of definitions/ directly through the predicates of itemModel
#           (IsOwnVariable, IsInterfaceParameter, HasFlag, ...) - since plan step 34 no longer the
#           string records of the old representation (lineType, cFlags letters).
#
#           The 7-line header comparison deciding whether a file is rewritten is kept as it was;
#           it hides @brief changes and is fixed by plan step 86 (#2415).
#
# Usage:    python tools/generators/itemHeaderEmitter.py [--output-dir DIR]
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-14 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import os
import re
import sys

toolsDirectory = os.path.dirname(os.path.abspath(__file__))
if toolsDirectory not in sys.path:
    sys.path.insert(0, toolsDirectory)

import itemModel as im                                                              # noqa: E402
from itemModel import *                                                             # noqa: E402,F403
import typeModel as tm                                                              # noqa: E402
from autoGenerateHelper import GenerateHeader, Str2Doxygen, CutLinesFromString, CountLines  # noqa: E402

space4 = '    '
space8 = space4+space4
space12 = space8+space4


# compute destination number of str given from [C|M][P]
# [sParamComp=0, sParamMain=1, sComp=2, sMain=3]
# return -1 if no destination
def DestinationNr(strDest):
    destNr = -1
    if strDest.find('V') != -1: # put into visualization class
        destNr = 4
    elif strDest.find('P') != -1: # put into computational class
        if strDest.find('C') != -1: # put into computational class
            destNr = 0
        if strDest.find('M') != -1: # put into main class
            destNr = 1
    else:
        if strDest.find('C') != -1: # put into computational class
            destNr = 2
        if strDest.find('M') != -1: # put into main class
            destNr = 3

    return destNr

#%%************************************************
def ItemIndexKind(typeName):
    """NodeIndex for NodeIndex, ArrayNodeIndex and NodeIndex2/3/4; the kind an item index is checked against"""
    for kind in ['Node', 'Object', 'Marker', 'Load', 'Sensor']:
        if kind + 'Index' in typeName:
            return kind + 'Index'
    raise ValueError('ItemIndexKind: not an item index type: ' + typeName)


#the range forms of definitions/definitionTypes.py and the check C++ applies to them
rangeCheckForms = {'UReal': 'nonNegative', 'UFloat': 'nonNegative', 'UInt': 'nonNegative',
                   'PReal': 'positive', 'PFloat': 'positive', 'PInt': 'positive'}


def ParameterWriteStatement(parameter, typeCastStr, destStr, pyName, fromDictionary, className):
    """the C++ statement that writes one parameter from Python (revision plan step 34c4): the same
    conversion for SetWithDictionary (source d["name"]) and SetParameter (source value); only
    BodyGraphicsData and OutputVariableType still differ between the two, as before"""
    typeName = TypeName(parameter)
    source = 'd["' + pyName + '"]' if fromDictionary else 'value'
    comment = ' /* AUTO:  read out dictionary and cast to C++ type*/'
    context = className + '.' + pyName #names item and parameter in error messages
    if typeName in ['BodyGraphicsData', 'BodyGraphicsDataList']: #special conversion routines
        function = 'PyWriteBodyGraphicsDataList' if typeName == 'BodyGraphicsData' else 'PyWriteBodyGraphicsDataListOfLists'
        if fromDictionary: #a missing entry is not an error here
            return function + '(d, "' + pyName + '", ' + destStr + '); /*! AUTO: convert dict to ' + typeName + '*/'
        return function + '(value, ' + destStr + ')'
    if IsInternalSetGetParameter(typeName):
        return 'SetInternal' + typeName + '(' + source + '); /*! AUTO:  safely cast to C++ type*/'
    if IsAMatrixVectorSpecial(typeName): #Vector3DList, Matrix3DList, PyMatrixContainer: not yet in PyConversion.h;
        #None stays accepted as empty: it is the default of these parameters in itemInterface.py (step 34c4 c)
        return 'EPyUtils::Set' + typeName + 'Safely(' + source + ', ' + destStr + ');' + comment
    if typeName in ['Matrix3D', 'Matrix6D']: #fixed size: rows and columns are template arguments
        size = typeName[6:-1]
        return 'EPyUtils::FromPython<Real, ' + size + ', ' + size + '>(' + source + ', ' + destStr + ');' + comment
    if IsASetSafelyParameter(typeName): #String, Vector2D ... Vector9D, NumpyVector, NumpyMatrix, NumpyMatrixI
        return 'EPyUtils::FromPython(' + source + ', ' + destStr + ');' + comment
    if IsItemIndex(typeName):
        return 'EPyUtils::ItemIndexFromPython<' + ItemIndexKind(typeName) + '>(' + source + ', ' + destStr + ');' + comment
    if 'PyFunction' in typeName: #py::object can be directly written
        return destStr + ' = ' + source + ';' + comment
    if typeName in rangeCheckForms: #the same check on every write path (step 34c4 b)
        given = '' #must-be-given parameters: Add and dictionaries raise for the placeholder default (step 34c4 e)
        if fromDictionary and HasFlag(parameter, 'Q'):
            given = 'EPyUtils::RequireGiven(' + source + ', ' + DefaultValueString(parameter) + ', "' + context + '"); '
        return (given + 'EPyUtils::FromPython(' + source + ', ' + destStr + ', EPyUtils::RangeCheck::' + rangeCheckForms[typeName]
                + ', "' + context + '");' + comment)
    if typeCastStr == 'OutputVariableType' and fromDictionary:
        return destStr + ' = (OutputVariableType)py::cast<Index>(' + source + ');' + comment
    if typeCastStr in ['bool', 'Real', 'float', 'Index']: #plain scalars: None raises (step 34c4 c)
        return 'EPyUtils::FromPython(' + source + ', ' + destStr + ', "' + context + '");' + comment
    return destStr + ' = py::cast<' + typeCastStr + '>(' + source + ');' + comment


#%%************************************************
#create autogenerated .h files for the list of parameters
def ItemCppHeaders(definition):
    parameterList = definition['members']
    """the C, Main and Visu headers of one item, and its symbolic user functions"""

    
    classStr = definition['className']
    
    #main and computational PARAMETER classes:
    compParamClassStr = "C" + classStr + "Parameters"
    mainParamClassStr = "Main" + classStr + "Parameters"

    #main and computational classes:
    compClassStr = "C" + classStr
    mainClassStr = "Main" + classStr
    visuClassStr = "Visualization" + classStr

    symbolicUserFunctionSet = []
    #symbolicUserFunctionArgs = {}
    
    classNames = [compParamClassStr, mainParamClassStr, compClassStr, mainClassStr, visuClassStr]

    #count parameters and find if there are types which need special treatment or special include files
    cntParameters = [0,0,0,0,0]
    usesPyFunction = False #flag, which shows that PyFunctions are used ==> needs <functional> from C++ std library and pybind11/functional.h
    for parameter in parameterList:
        if (IsOwnVariable(parameter)): #only if it is a member variable
            cntParameters[DestinationNr(Destination(parameter))] += 1
        if TypeName(parameter).find('PyFunction') != -1: 
            usesPyFunction = True

    
    #now start generating strings for classes; the parameter classes go into the same file on top,
    #  therefore the header goes to the parameter classes
    sParamComp = GenerateHeader(compParamClassStr, 'Parameter class for '+compClassStr, author=Header(definition, 'author'))
    sParamMain = GenerateHeader(mainParamClassStr, 'Parameter class for '+mainClassStr, author=Header(definition, 'author'))
    sParamMain += '#include <pybind11/pybind11.h>      //! AUTO: include pybind for dictionary access\n'
    sParamMain += '#include <pybind11/stl.h>           //! AUTO: needed for stl-casts; otherwise py::cast with std::vector<Real> crashes!!!\n'
    sParamMain += 'namespace py = pybind11;            //! AUTO: "py" used throughout in code\n'
    if usesPyFunction:
        sParamMain += '#include <pybind11/functional.h> //! AUTO: for function handling ... otherwise gives a python error (no compilation error in C++ !)\n'
        sParamComp += '#include <functional> //! AUTO: needed for std::function\n'
        
        if useNewUserFunctions:
            sParamComp += '#include "Pymodules/PythonUserFunctions.h" //! AUTO: needed for user functions, without pybind11\n'

            

    #sParamMain += 'using namespace pybind11::literals; //! # enables the "_a" literals\n\n'
    sParamMain += '#include "Autogenerated/' + compClassStr + '.h"\n\n'
    sParamMain += '#include "Autogenerated/Visu' + classStr + '.h"\n'

    sParamComp += Header(definition, 'addIncludesC')
    sParamMain += Header(definition, 'addIncludesMain')
    
    sParamComp += '\n'
    sParamMain += '\n'
#   DONE via addIncludesC
#    cPC = Header(definition, 'cParentClass')
#    if (cPC != 'CNodeODE2') & (cPC != 'CNodeData') & (cPC != 'CObjectBody') & (cPC != 'CObjectConnector') & (cPC != 'CObjectConstraint') & (cPC != 'CMarker') & (cPC != 'CLoad'):
#        sParamComp += '#include "Autogenerated/' + Header(definition, 'cParentClass') + '.h"  //! AUTO: include parent class\n'
#        sParamMain += '#include "Autogenerated/' + Header(definition, 'mainParentClass') + '.h"  //! AUTO: include main parent class\n'

    sVisu = GenerateHeader(visuClassStr, Str2Doxygen(Header(definition, 'classDescription')), author=Header(definition, 'author'))
    #no includes for visualization classes, because they are included at a place, where all necessary headers exist
    #sParamMain += '#include "Graphics/Visualization.h"      //! AUTO: link to visualization class; also includes settings and base classes VisualizationObject/Node/...


    sComp = "" #computation
    sComp = GenerateHeader(compClassStr, Str2Doxygen(Header(definition, 'classDescription')), addModifiedDate=False, addIfdefOnce = False, author=Header(definition, 'author'))
    sMain = "" #main object
    sMain = GenerateHeader(mainClassStr, Str2Doxygen(Header(definition, 'classDescription')), addModifiedDate=False, addIfdefOnce = False, author=Header(definition, 'author'))
    #make list of strings to enable iteration
    sList = [sParamComp, sParamMain, sComp, sMain, sVisu]
    nClasses = 5 #number of different classes
    indexComp = 2 #index in sList
    # indexMain = 3 #index in slist

    #************************************
    #class definition:
    strParentClass = ["", "",""]
    if len(Header(definition, 'cParentClass')) != 0:
        strParentClass[0] = ': public ' + Header(definition, 'cParentClass')

    if len(Header(definition, 'mainParentClass')) != 0:
        strParentClass[1] = ': public ' + Header(definition, 'mainParentClass')

    if len(Header(definition, 'visuParentClass')) != 0:
        strParentClass[2] = ': public ' + Header(definition, 'visuParentClass')

    for i in range (2):
        sList[i] += '//! AUTO: Parameters for class ' + classNames[i] + '\n'
        sList[i+2] += '//! AUTO: ' + classNames[i+2] + '\n' #+ ': ' + Str2Doxygen(Header(definition, 'classDescription')) + '\n'

    for i in range (2):
        sList[i]+='class ' + classNames[i] + ' // AUTO: \n' + '{\n' #parameter classes
        sList[i+2]+='class ' + classNames[i+2] + strParentClass[i] + ' // AUTO: \n' + '{\n' #regular classes
    
    sList[4] += 'class ' + classNames[4] + strParentClass[2] + ' // AUTO: \n' + '{\n' #visualization class

    classTypeStr = Header(definition, 'classType')
    sTypeName = classStr.replace(classTypeStr,'')

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #member variables:
    sList[0]+='public: // AUTO: \n' #parameter classes are just structs
    sList[1]+='public: // AUTO: \n'  
    sList[2]+='protected: // AUTO: \n'
    sList[3]+='protected: // AUTO: \n'
    sList[4]+='protected: // AUTO: \n'

    sList[2]+=Header(definition, 'addProtectedC')
    sList[3]+=Header(definition, 'addProtectedMain')

    #add pointer to computation class in main class
    compClassVariable = compClassStr[0].lower()+compClassStr[1:]  #instance name is lower case
    visuClassVariable = visuClassStr[0].lower()+visuClassStr[1:]  #instance name is lower case
    sList[3]+=space4 + compClassStr + '* ' + compClassVariable + '; //pointer to computational object (initialized in object factory) AUTO:\n'
    sList[3]+=space4 + visuClassStr + '* ' + visuClassVariable + '; //pointer to computational object (initialized in object factory) AUTO:\n'

    #add parameter member variables
    for i in range(2): # 0...comp parameters, 1...main parameters
        if cntParameters[i] != 0:
            sList[i+2]+=space4 + classNames[i] + ' parameters; //! AUTO: contains all parameters for '
            sList[i+2]+=classNames[i+2] + '\n'
            

    #process variables:    
    for parameter in parameterList:
        if (IsOwnVariable(parameter) and
            not IsInternalSetGetParameter(TypeName(parameter)) ): #only if it is a member variable but not special one with conversion
        
            isPyFunction = (TypeName(parameter).find('PyFunction') != -1) 

            typeStr = tm.CppMemberType(TypeName(parameter), 'items')
            
            if HasFlag(parameter, 'U'):
                typeStr = 'mutable ' + typeStr #make this variable changable in GetMassMatrix(), ComputeODE2RHS(), ... functions
                #print(typeStr)
            temp = space4 + typeStr + ' ' + CppName(parameter)+ ';'
            nChar = len(temp)
            alignment = 50
            insertSpaces = ''
            if nChar < alignment:
                insertSpaces = ' '*(alignment-nChar)

            parameterDescription = Description(parameter) #remove symbol from parameter description
            [parameterDescription, latexSymbol] = ExtractLatexSymbol(parameterDescription)

            lineStr = temp + insertSpaces + '//!< AUTO: ' + Str2Doxygen(parameterDescription) + '\n'

            sList[DestinationNr(Destination(parameter))] += lineStr

            
    sList[2]+='\npublic: // AUTO: \n'
    sList[3]+='\npublic: // AUTO: \n'
    sList[2]+=Header(definition, 'addPublicC')
    sList[3]+=Header(definition, 'addPublicMain')

    sList[4]+='\npublic: // AUTO: \n' # visualization

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #default parameters:
    
    #count number of default parameters
    cntDefaultParameters = [0,0,0,0,0]
    for parameter in parameterList:
        if (IsVariable(parameter)): #member variable: also include parent members for default initialization (name and V.show)
            strDefault = DefaultValue(parameter)
            if len(strDefault) or (TypeName(parameter) == 'String'): 
                cntDefaultParameters[DestinationNr(Destination(parameter))] += 1

    #constructor with default initialization:
    for i in range(nClasses):
        if cntDefaultParameters[i]:
            sList[i]+=space4+'//! AUTO: default constructor with parameter initialization\n'
            sList[i]+=space4+classNames[i]+'()\n'
            sList[i]+=space4+'{\n'
        
            for parameter in parameterList:
                if (IsVariable(parameter) and
                    not IsInternalSetGetParameter(TypeName(parameter)) ): #only if it is a variable and not variable with internal conversion; include parent members
                    strDefault = DefaultValue(parameter)
                    if len(strDefault) or (TypeName(parameter) == 'String'): #only add initialization if default value exists
                        if TypeName(parameter) == 'String':
                            strDefault = '"' + strDefault + '"'
                        tempStr=space8 + CppName(parameter) + ' = ' + strDefault + ';\n'
                        if (DestinationNr(Destination(parameter)) == i):
                            sList[i]+=tempStr
        
            sList[i]+=space4+'};\n'
    
    sList[2]+='\n    // AUTO: access functions\n'
    sList[3]+='\n    // AUTO: access functions\n'
    sList[4]+='\n    // AUTO: access functions\n'

    #access functions to compClass pointer:
    sList[3]+=space4+'//! AUTO: Get pointer to computational class\n'
    sList[3]+=space4 + compClassStr + '* Get' + compClassStr + '() { return ' + compClassVariable + '; }\n'
    sList[3]+=space4+'//! AUTO: Get const pointer to computational class\n'
    sList[3]+=space4+'const ' + compClassStr + '* Get' + compClassStr + '() const { return ' + compClassVariable + '; }\n'
    sList[3]+=space4+'//! AUTO: Set pointer to computational class (do this only in object factory!!!)\n'
    sList[3]+=space4+'void Set' + compClassStr + '(' + compClassStr + '* p' + compClassStr + ') { ' + compClassVariable + ' = p' + compClassStr + '; }\n\n'

    #access functions to visuClass pointer:
    sList[3]+=space4+'//! AUTO: Get pointer to visualization class\n'
    sList[3]+=space4 + visuClassStr + '* Get' + visuClassStr + '() { return ' + visuClassVariable + '; }\n'
    sList[3]+=space4+'//! AUTO: Get const pointer to visualization class\n'
    sList[3]+=space4+'const ' + visuClassStr + '* Get' + visuClassStr + '() const { return ' + visuClassVariable + '; }\n'
    sList[3]+=space4+'//! AUTO: Set pointer to visualization class (do this only in object factory!!!)\n'
    sList[3]+=space4+'void Set' + visuClassStr + '(' + visuClassStr + '* p' + visuClassStr + ') { ' + visuClassVariable + ' = p' + visuClassStr + '; }\n\n'

    baseClass = Header(definition, 'classType')
    if len(baseClass) != 0:
        cBaseClass= 'C' + baseClass;
        sList[3]+=space4+'//! AUTO: Get const pointer to computational base class object\n' #added for better generalization of main/comp objects
        sList[3]+=space4+'virtual ' + cBaseClass + '* Get' + cBaseClass + '() const { return ' + compClassVariable + '; }\n'
        sList[3]+=space4+'//! AUTO: Set pointer to computational base class object (do this only in object factory; type is NOT CHECKED!!!)\n'
        sList[3]+=space4+'virtual void Set' + cBaseClass + '(' + cBaseClass + '* p' + cBaseClass + ') { ' + compClassVariable + ' = (' + compClassStr + '*)p' + cBaseClass + '; }\n\n'
        visuBaseClass= 'Visualization' + baseClass;
        sList[3]+=space4+'//! AUTO: Get const pointer to visualization base class object\n' #added for better generalization of main/comp objects
        sList[3]+=space4+'virtual ' + visuBaseClass + '* Get' + visuBaseClass + '() const { return ' + visuClassVariable + '; }\n'
        sList[3]+=space4+'//! AUTO: Set pointer to visualization base class object (do this only in object factory; type is NOT CHECKED!!!)\n'
        sList[3]+=space4+'virtual void Set' + visuBaseClass + '(' + visuBaseClass + '* p' + visuBaseClass + ') { ' + visuClassVariable + ' = (' + visuClassStr + '*)p' + visuBaseClass + '; }\n\n'

    addGraphicsData = ''
    boolAddGraphicsData = ''

    if baseClass == 'Object':
        addGraphicsData = ', addGraphicsData'
        boolAddGraphicsData = 'bool addGraphicsData=false'


    #print(cntParameters)
    #add parameter structures and access functions
    for i in range(2): # 0...comp parameters, 1...main parameters
        if cntParameters[i] != 0:
            sList[i+2]+=space4+'//! AUTO: Write (Reference) access to parameters\n'
            sList[i+2]+=space4+'virtual ' + classNames[i] + '& GetParameters() { return parameters; }\n'
            sList[i+2]+=space4+'//! AUTO: Read access to parameters\n'
            sList[i+2]+=space4+'virtual const ' + classNames[i] + '& GetParameters() const { return parameters; }\n\n'
            
        

    #GetClone() function:
    # should not be done with objects; with parameters it should not be necessary (no ObjectList<parameterObject>)
    #    s+='  //! # clone object; specifically for copying instances of derived class, for automatic memory management e.g. in ObjectContainer\n'
    #    s+='  virtual ' + definition['className'] + '* GetClone() const { return new '+definition['className']+'(*this); }\n'
    #    s+='  \n'

    #************************************
    #create access functions for member variables for Main/Comp objects:
    #  parameterClasses do not have access functions, but the access is transferred to  
    #  Main/Comp objects; also, parameter class itself can be accessed
    #  add also pypind11-access to all parameters
    dictListWrite = ["", "", "", "", ""] # strings to create dict write access for every class
    dictListRead = ["", "", "", "", ""]  # strings to create dict read access for every class

    parameterReadStr = ''  # functions and checks to read (get) parameters
    parameterWriteStr = '' # functions and checks to write (set) parameters

    for parameter in parameterList:
        i = DestinationNr(Destination(parameter)) #sList: [sParamComp, sParamMain, sComp, sMain, sVisu]
        paramStr = CppName(parameter)
        functionStr = paramStr
        c = functionStr[0] #take first character; !remember that the member variable must be lower-case
        functionStr = c.upper()+functionStr[1:]

        #process variables:
        if (IsVariable(parameter)): #only if it is a variable; also include Vp variables - i.e. 'name'
            typeStr = tm.CppMemberType(TypeName(parameter), 'items')
            refChar = '&' #use only '&' in read access, if it is no pointer; 
            if typeStr[len(typeStr)-1] == '*':
                refChar = ''
    
            isPyFunction = (TypeName(parameter).find('PyFunction') != -1) #in case of function, special conversion and tests are necessary (function is either 0 or a python function)

            paramStrAccess = paramStr
            # if isPyFunction:
            #     paramStrAccess += '.userFunction'

            #add function definition for internal conversion functions:
            if IsInternalSetGetParameter(TypeName(parameter)):
                sList[i]+=space4+'void SetInternal' + TypeName(parameter) + '(const py::object& pyObject); //! AUTO: special function which writes pyObject into local data\n'
                sList[i]+=space4+'Py' + TypeName(parameter)+' GetInternal' + TypeName(parameter) + '() const; //! AUTO: special function which returns '+TypeName(parameter) +' converted from local data\n'
            #add Get/Set class function except from members in CItem parameter classes, which are public
            elif (i > 1) and (not FromParent(parameter)): #must be comp, main or visu class; don't do it, if variable of parent class
                #print('add access to:',compClassStr,':',paramStrAccess)
                sList[i]+=space4+'//! AUTO:  Write (Reference) access to:' + Str2Doxygen(Description(parameter)) + '\n'
                sList[i]+=space4+'void Set' + functionStr + '(const ' + tm.CppMemberType(TypeName(parameter), 'items')
                sList[i]+='& value) { ' + paramStrAccess + ' = value; }\n'
        
                sList[i]+=space4+'//! AUTO:  Read (Reference) access to:' + Str2Doxygen(Description(parameter)) + '\n'
                sList[i]+=space4+'const ' + typeStr + refChar + ' '
                sList[i]+='Get' + functionStr + '() const { return '+paramStrAccess+'; }\n'
                if (i == 2) | (i == 4) : #in comp and visu class, also add the Get...() Reference access
                    sList[i]+=space4+'//! AUTO:  Read (Reference) access to:' + Str2Doxygen(Description(parameter)) + '\n'
                    sList[i]+=space4 + typeStr + refChar + ' '
                    sList[i]+='Get' + functionStr + '() { return '+paramStrAccess+'; }\n'
                sList[i]+='\n'

            destFolder = '' #destination folder string (e.g. GetParameters())
            if Destination(parameter).find('C') != -1: #computation
                destFolder+=compClassVariable + '->'

            vPrefix = '' #use 'V' prefix for visualization items
            if Destination(parameter).find('V') != -1: #visualization
                destFolder+=visuClassVariable + '->'
                vPrefix = 'V'
            pyName = vPrefix + parameter['pythonName']
                
            if Destination(parameter).find('P') != -1:
                destFolder+='GetParameters().'
            
            destStr = destFolder + CppName(parameter)
            if i == 2: #comp class
                pstr = CppName(parameter)
                destStr = destFolder + 'Get' + pstr[0].upper() + pstr[1:] + '()'
            
            
            if i == 4: #visualization class
                pstr = CppName(parameter)
                destStr = destFolder + 'Get' + pstr[0].upper() + pstr[1:] + '()'
            
            typeCastStr = tm.Render(TypeName(parameter), 'cppExchange', 'items')

            #dictionary access:
            if IsInterfaceParameter(parameter): #'I' means add dictionary access
                parRead = '' #used for dictionary read and for parameter read
                parWrite = '' #used for dictionary write and for parameter write
                if TypeName(parameter) == 'BodyGraphicsData': #special conversion routine
                    #dictListRead[i] +=space8+'d["' + pyName + '"] = PyGetBodyGraphicsDataList(' + destStr + addGraphicsData+'); //! AUTO: generate dictionary with special function\n'
                    parRead = 'PyGetBodyGraphicsDataList(' + destStr + addGraphicsData+')'
                elif TypeName(parameter) == 'BodyGraphicsDataList': #special conversion routine
                    #dictListRead[i] +=space8+'d["' + pyName + '"] = PyGetBodyGraphicsDataListOfLists(' + destStr + addGraphicsData+'); //! AUTO: generate dictionary with special function\n'                    
                    parRead = 'PyGetBodyGraphicsDataListOfLists(' + destStr + addGraphicsData+')'
                elif IsInternalSetGetParameter(TypeName(parameter)):
                    parRead = 'GetInternal'+ TypeName(parameter) +'()'
                elif TypeName(parameter)[:-2] == 'Matrix' and TypeName(parameter)[-1] == 'D':
                    parRead = 'EPyUtils::ToPython(' + destStr + ')'
                elif TypeName(parameter)[:-2] == 'Vector' and TypeName(parameter)[-1] == 'D': #any Vector2D, Vector3D, ...
                    parRead = 'EPyUtils::ToPython(' + destStr + ')'
                elif TypeName(parameter) == 'Vector': 
                    parRead = 'EPyUtils::ToPython(' + destStr + ')'
                # elif TypeName(parameter) == 'Matrix6D':
                #     parRead = 'EXUmath::Matrix6DToStdArray66(' + destStr + ')'
                # elif TypeName(parameter) == 'Matrix3D':
                #     parRead = 'EXUmath::Matrix3DToStdArray33(' + destStr + ')'
                elif TypeName(parameter) == 'NumpyMatrix':
                    parRead = 'EPyUtils::ToPython(' + destStr + ')'
                elif TypeName(parameter) == 'NumpyMatrixI':
                    parRead = 'EPyUtils::ToPython(' + destStr + ')'
                elif TypeName(parameter) == 'NumpyVector':
                    parRead = 'EPyUtils::ToPython(' + destStr + ')'
                elif IsItemIndex(TypeName(parameter)):
#                    print("typecaststr=", typeCastStr)
#                    print("typestr=", typeStr)
                    if TypeName(parameter).startswith('Array'):
                        parRead = 'EPyUtils::ItemIndexToPython<' + ItemIndexKind(TypeName(parameter)) + '>(' + destStr + ')'
                    elif TypeName(parameter) in ['NodeIndex2', 'NodeIndex3', 'NodeIndex4']:
                        parRead = 'EPyUtils::ItemIndexToPython<NodeIndex>((ArrayIndex)' + destStr + ')'  #Index2/Index3/Index4 as ArrayIndex
                    else:
                        parRead = '(' + typeCastStr + ')' + destStr
                elif isPyFunction:
                    parRead = '(py::object)'+destStr
                else:
                    parRead = '(' + typeCastStr + ')' + destStr

                # isPyFunction = (TypeName(parameter).find('PyFunction') != -1) #in case of function, special conversion and tests are necessary (function is either 0 or a python function)
                    
                #+++++++++++++++++
                #add information for symbolic user function
                if isPyFunction:
                    pyUserFunctionType = TypeName(parameter)
                    stdFunctionType = pyFunctionTypeConversion[pyUserFunctionType]
                    # 'StdVector','StdVector3D','StdVector6D','StdVector','StdMatrix3D','StdMatrix6D',

                    symbolicNewSet = {'itemType':classStr.replace(classTypeStr,''),
                                      'classType':classTypeStr,
                                      'userFunctionName':CppName(parameter),
                                      'pyUserFunctionType':pyUserFunctionType,
                                      'stdFunctionType':stdFunctionType,
                                      }

                    symbolicUserFunctionSet += [symbolicNewSet]

                
                #+++++++++++++++++
                #read from dictionary
                if len(parRead) != 0:
                    dictListRead[i] += space8
                    # if isPyFunction: #old style; now already correct object returned
                    #     dictListRead[i]+='if ('+destStr+')\n            {' #avoid that 'None' is returned in dict due to empty user function
                    # parReadDict = parRead
                    # if isPyFunction: 
                    #     parRead += '.GetGetPythonDictionary()'
                    dictListRead[i] +='d["' + pyName + '"] = ' + parRead + ';'
                    # if isPyFunction: 
                    #     dictListRead[i]+='}\n        else\n'
                    #     dictListRead[i]+=space8+'    {d["' + pyName + '"] = 0;}\n'
                            
                    dictListRead[i] +=' //! AUTO: cast variables into python (not needed for standard types) \n'
                                                    
                #+++++++++++++++++
                #write to dictionary
                if (not HasFlag(parameter, 'R')): #'R' means read only!
                    dictListWrite[i]+=space8
                    if HasFlag(parameter, 'O'): #optional ==> means that we have to check first, if it exists in the dictionary
                        dictListWrite[i]+='if (EPyUtils::DictItemExists(d, "' +  pyName + '")) { '
                    #if (TypeName(parameter) == 'String') | (TypeName(parameter) == 'Vector2D') | (TypeName(parameter) == 'Vector3D') | (TypeName(parameter) == 'Vector4D') | (TypeName(parameter) == 'Vector6D') | (TypeName(parameter) == 'Vector7D'):
                    dictListWrite[i] += ParameterWriteStatement(parameter, typeCastStr, destStr, pyName, fromDictionary=True, className=classStr)
                    if HasFlag(parameter, 'O'): #optional ==> means that we have to check first, if it exists in the dictionary
                        dictListWrite[i]+='} '
                    dictListWrite[i]+='\n'

                    #+++++++++++++++++
                    #parameter write
                    #if (TypeName(parameter) == 'String') | (TypeName(parameter) == 'Vector2D') | (TypeName(parameter) == 'Vector3D') | (TypeName(parameter) == 'Vector4D') | (TypeName(parameter) == 'Vector6D') | (TypeName(parameter) == 'Vector7D'):
                    parWrite += ParameterWriteStatement(parameter, typeCastStr, destStr, pyName, fromDictionary=False, className=classStr)

                #+++++++++++++++++
                #parameter read
                if parRead != '':
                    #if TypeName(parameter).find('Numpy') != -1: #do not add py::cast(...) NumpyMatrix/Vector
                    parameterReadStr += 'if (parameterName.compare("' + pyName + '") == 0) { return '
                    if parRead.startswith('EPyUtils::ToPython('): #already a Python object
                        parameterReadStr += parRead
                    elif isPyFunction:
                        #parameterReadStr += destStr+' ? py::cast('+parRead+') : py::cast((int)0);'
                        parameterReadStr += destStr+'.GetPythonDictionary();'
                    elif 'BodyGraphicsData' in TypeName(parameter):
                        parameterReadStr += parRead.replace(', addGraphicsData',', true')
                    else:
                        parameterReadStr += 'py::cast(' + parRead + ')'
                    
                    parameterReadStr += ';} //! AUTO: get parameter\n        else '
                
                if parWrite != '':
                    parameterWriteStr += 'if (parameterName.compare("' + pyName + '") == 0) { ' + parWrite + '; } //! AUTO: get parameter\n        else '

                #pybind access goes via function in MainSystem/ObjectFactory class, e.g.:
                #   AddMarker(dict) --> return markerNumber
                #   SetMarker(int index (markerNameStr on Python side), dict)
                #   dict& GetMarker(int index (markerNameStr on Python side))
            
        #process member functions:
        else: 
            strVirtual = ''
            strOverride = ''
            if (IsFunction(parameter)):
                if (IsVirtualFunction(parameter)):
                    strVirtual = 'virtual '
                    #every virtual function overrides a parent function; the 'X' flag that used
                    #to suppress this was never set by any definition and has been removed
                    strOverride = ' override'
                if (IsStaticFunction(parameter)): #static
                    strVirtual = 'static ' + strVirtual
                
            typeStr = tm.CppMemberType(TypeName(parameter), 'items')
            argsStr = Args(parameter)
    
            sList[i]+=space4+'//! AUTO:  ' + Str2Doxygen(Description(parameter)) + '\n'
            sList[i]+=space4+strVirtual + typeStr + ' '
            sList[i]+=functionStr + '(' + argsStr + ')' 
            if HasFlag(parameter, 'C'):
                sList[i]+=' const'
                #print('Const found for ' + functionStr)
            sList[i]+=strOverride #all virtual functions should override a parent class function (otherwise add a new option)!
            
            if not IsDeclarationOnly(parameter):
                sList[i]+='\n    {\n        ' + DefaultValue(parameter) + '\n    }'
            else:
                sList[i]+=';'
                
            sList[i]+='\n\n'
    
    #add outputVariableType function automatically if defined:
    outputVariableNames = OutputVariableNames(definition)
    if len(outputVariableNames) != 0:
        if (Header(definition, 'classType') == 'Object') | (Header(definition, 'classType') == 'Node'):
            sList[indexComp] += space4+'virtual OutputVariableType GetOutputVariableTypes() const override\n    {\n        return (OutputVariableType)('
            for outputVariableName in outputVariableNames:
                sList[indexComp] += '\n            (Index64)OutputVariableType::' + outputVariableName + ' +'
            sList[indexComp] = sList[indexComp][0:len(sList[indexComp])-1]
            sList[indexComp] += ');\n    }\n\n'
        else:
            print("ERROR: ",definition['className'], ": output variables only possible for Objects and Nodes")
        
        
    
    typeStr = Header(definition, 'classType')
    c = typeStr[0] #take first character; !remember that the member variable must be lower-case
    typeStr = c.lower()+typeStr[1:]+'Type' #e.g. 'nodeType'

    #+++++++++++++++++++++++++++++++++++++++++++++++
    #now write dictionary read/write functions into main class:
    sList[3] += '\n    //! AUTO:  dictionary write access\n'
    sList[3] += space4+'virtual void SetWithDictionary(const py::dict& d) override\n'
    sList[3] += space4+'{\n'
    for i in range(nClasses):
        sList[3] += dictListWrite[i]
    
    if Header(definition, 'classType') == 'Object': #if parameters have changed (e.g. with ModifyObject(..) ), some functions may be necessary to be reset
        sList[3] += space8+'GetCObject()->ParametersHaveChanged();\n'
        
    sList[3] += space4+'}\n\n'

    #+++++++++++++++++++++++++++++++++++++++++++++++
    sList[3] += space4+'//! AUTO:  dictionary read access\n'
    sList[3] += space4+'virtual py::dict GetDictionary('+boolAddGraphicsData+') const override\n'
    sList[3] += space4+'{\n'
    sList[3] += space8+'auto d = py::dict();\n'
    sList[3] += space8+'d["' + typeStr + '"] = (std::string)GetTypeName();\n'
    for i in range(nClasses):
        sList[3] += dictListRead[i]
        
    sList[3] += space8+'return d; \n'
    sList[3] += space4+'}\n'

    #+++++++++++++++++++++++++++++++++++++++++++++++
    #now write parameter read/write functions into main class:
    sList[3] += '\n    //! AUTO:  parameter read access\n'
    sList[3] += space4+'virtual py::object GetParameter(const STDstring& parameterName) const override \n'
    sList[3] += space4+'{\n        '
    sList[3] += parameterReadStr
    sList[3] += ' {PyError(STDstring("' + classStr + '::GetParameter(...): illegal parameter name ")+parameterName+" cannot be read");} // AUTO: add warning for user\n'
    sList[3] += space8+'return py::object();\n'
#        if Header(definition, 'classType') == 'Object': #if parameters have changed, some functions may be necessary to be reset
#            sList[3] += space8+'GetCObject()->ParametersHaveChanged();\n'
    sList[3] += space4+'}\n\n'

    sList[3] += '\n    //! AUTO:  parameter write access\n'
    sList[3] += space4+'virtual void SetParameter(const STDstring& parameterName, const py::object& value) override \n'
    sList[3] += space4+'{\n        '
    sList[3] += parameterWriteStr
    sList[3] += ' {PyError(STDstring("' + classStr + '::SetParameter(...): illegal parameter name ")+parameterName+" cannot be modified");} // AUTO: add warning for user\n'
    #notify object that parameters have changed
    if Header(definition, 'classType') == 'Object': #if parameters have changed (e.g. with ModifyObject(..) ), some functions may be necessary to be reset
        sList[3] += space8+'GetCObject()->ParametersHaveChanged();\n'
    #sList[3] += space8+'\n'
#        if Header(definition, 'classType') == 'Object': #if parameters have changed, some functions may be necessary to be reset
#            sList[3] += space8+'GetCObject()->ParametersHaveChanged();\n'
    sList[3] += space4+'}\n\n'


    #.def("__repr__", &Vector2::toString);

    for i in range(nClasses):
        sList[i]+='};\n\n\n' #class

    sList[2] += '\n#endif //#ifdef include once...\n'
    sList[3] += '\n#endif //#ifdef include once...\n'
    sList[4] += '\n#endif //#ifdef include once...\n'

    return [sList[0]+sList[2], sList[1]+sList[3], sList[4], symbolicUserFunctionSet]


#%%************************************************
#helper functions for symbolic user functions:

#define types, not available for symbolic user functions:
def FitsSymbolicUF(pyUserFunctionType, stdFunctionType):
    return not (#'StdVector' in stdFunctionType or
                'GraphicsData' in pyUserFunctionType
                or 'NumpyMatrix' in stdFunctionType
                or 'py::object' in stdFunctionType
                or 'StdArrayIndex' in stdFunctionType
                )

    
# NOTE: WILL BE DELETED, because not needed in FUTURE !!!!!!!!!!
def CreateStringSymbolicUserFunctionTransfer(pySymbolicUserFunction):
    # template<typename TItemIndex>
    # void TransferUserFunction2Item(MainSystem& mainSystem, TItemIndex itemIndex, const STDstring& userFunctionName)
    s = """public:
    //! this function realizes the assignment of user function to item fully in C++, to avoid 2 x Python-casts!
    void TransferUserFunction2Item(MainSystem& mainSystem, py::object itemIndex, const STDstring& userFunctionName)
    {
		STDstring sType;
		STDstring itemTypeName;
        Index itemNumber;
		GetItemTypeName(mainSystem, itemIndex, sType, itemTypeName, itemNumber);
        //Index itemNumber = itemIndex.GetIndex();
        //STDstring sType = itemIndex.GetTypeString();
        //STDstring itemTypeName = GetItemTypeName(mainSystem, itemIndex);

"""
    endClassStr = """
            else
            {
                PyError(STDstring("Symbolic::TransferUserFunction2Item<") + itemTypeName + "," + userFunctionName +
                        ">: invalid user object type or user function type; possibly, function is not available as symbolic user function");
            }
        }
"""
    functionTypeList = []
    sFunctionsStr = 'public:\n    //collect all kinds of user functions\n'
    classTypeList = []
    useElse = ''
    useElseClassType = ''

    # print("pySymbolicUserFunction\n",pySymbolicUserFunction)

    for item in pySymbolicUserFunction:
        if len(item) == 0: continue
    
        itemType = item['itemType']     #ConnectorSpringDamper
        classType = item['classType']   #Object, Node
        userFunctionName = item['userFunctionName']
        pyUserFunctionType = item['pyUserFunctionType']
        userFunctionType = pyUserFunctionType.replace('PyFunction','')
        userFunctionType = userFunctionType[0].lower()+userFunctionType[1:]
        stdFunctionType = pyFunctionTypeConversion[pyUserFunctionType]

        cItem = 'C'+classType+itemType

        if userFunctionType not in functionTypeList:
            functionTypeList += [userFunctionType]
            # print(userFunctionType)
            sFunctionsStr += ' '*4+pyFunctionTypeConversion[pyUserFunctionType]+' '
            sFunctionsStr += userFunctionType + ';\n'
        # else:
        #     print('is there:', userFunctionType)
        
        if not FitsSymbolicUF(pyUserFunctionType, stdFunctionType): continue
        
        if itemType != 'MainSystem':
            if classType not in classTypeList:
                if len(classTypeList) != 0:
                    s+=endClassStr
                useElse = ''
    
                classTypeList += [classType]
                s+= ' '*8+useElseClassType+'if (sType == "'+classType+'Index")\n'
                useElseClassType = 'else '
                s+="""        {
            CHECKandTHROW(itemNumber < mainSystem.GetCSystem().GetSystemData().GetCObjects().NumberOfItems(),
                          "Symbolic::TransferUserFunction2Item: illegal objectNumber");

"""
            s+= ' '*12+useElse+'if (itemTypeName == "'+itemType+'" && userFunctionName == "'+userFunctionName+'")\n'
            s+= ' '*12+'{\n'
            s+= ' '*16+cItem+'* cItem = ('+cItem+'*)(mainSystem.GetCSystem().GetSystemData().GetC'+classType+'s()[itemNumber]);\n'
            #s+= ' '*16+'cItem->GetParameters().'+ userFunctionName + ' = '+ userFunctionType + ';\n' #mbsScalarIndexScalar5; //UFlocal;
            s+= ' '*16+'cItem->GetParameters().'+ userFunctionName + '.SetSymbolicUserFunction('+ userFunctionType + ');\n' #mbsScalarIndexScalar5; //UFlocal;
            
            s+= ' '*12+'}\n'
            useElse = 'else '


    s+=endClassStr
    s+= """        else
        {
            PyError(STDstring("Symbolic::TransferUserFunction2Item<") + itemTypeName + "," + userFunctionName + ">: invalid item type (must be Object or Load)");
        }
    }
"""    
    s = sFunctionsStr+'\n'+s
    return s


#create C++ code for included .h file for symbolic user functions
def CreateStringSymbolicUserFunctionSet(pySymbolicUserFunction):
    
    sTemplateInstantiation = '//included into SymbolicUtilities.h\n//templates require explicit instantiations for each template because otherwise linker problems ... (alternative: include everything into .h files ...)\n'
    stdFunctionMember = '//collect all kinds of user functions\npublic:\n'

    #templates for UFT GetSTDfunction()
    sSTDfunctionTemplate="""if constexpr (std::is_same_v<UFT, {stdFunction}>)
		{
			return {varName};
		}
"""
    
    sSTDfunction = """
	//! for specific user function type, get user function by type and name
    template<typename UFT>
    UFT GetSTDfunction() const
    {
"""
    sSTDfunctionEnd = """
		return UFT(0); //will never happen, but avoids errors/warnings
    }

"""
    
    s = """
	//! set up general user function from dictionary:
	void SetUserFunctionFromDict(MainSystem& mainSystem, py::dict pyObject, const STDstring& userFunctionName, py::object itemIndex, STDstring itemTypeName)
	{
        if (itemTypeName == "None")
        {
            STDstring sType;
            Index itemNumber;
            GetItemTypeName(mainSystem, itemIndex, sType, itemTypeName, itemNumber);
        }
        else
        {
            CHECKandTHROW(itemIndex.is_none(), "SetUserFunctionFromDict: if itemTypeName is provided, itemIndex must be None");
        }

		SetupUserFunction(pyObject, itemTypeName, userFunctionName);

		//now cast items to set user function
"""
    functionTypeList = []
    sFunctionsStr = '' #collect all kinds of user functions
    classTypeList = []
    useElse = ''
    stdFunctionElse = ''
    previousList = []
    for item in pySymbolicUserFunction:
        if len(item) == 0: continue
        
        itemType = item['itemType'] #ConnectorSpringDamper
    
        classType = item['classType']   #Object, Node
        userFunctionName = item['userFunctionName']
        pyUserFunctionType = item['pyUserFunctionType']
        userFunctionType = pyUserFunctionType.replace('PyFunction','')
        userFunctionType = userFunctionType[0].lower()+userFunctionType[1:]
        stdFunctionType = pyFunctionTypeConversion[pyUserFunctionType]
        
        if stdFunctionType in previousList: 
            # print('double: ',stdFunctionType)
            continue
        previousList.append(stdFunctionType)
        
        sSTDfunction += '        '+stdFunctionElse+sSTDfunctionTemplate.replace('{varName}',userFunctionType).replace('{stdFunction}',stdFunctionType)
        stdFunctionElse = 'else '
        
        sTemplateInstantiation += 'template class PythonUserFunctionBase<{stdFunction}>;\n'.replace('{stdFunction}',stdFunctionType)
        stdFunctionMember += ' '*4+stdFunctionType+' ' + userFunctionType + ';\n'

        if not FitsSymbolicUF(pyUserFunctionType, stdFunctionType): continue
        
        stdFunctionTypeReturn = stdFunctionType.split('<')[1].split('(')[0].strip()

        returnValueType = stdFunctionTypeReturn.replace('py::object','PyObject').replace('bool','Bool')
                
        cItem = 'C'+classType+itemType

        #create string for function named args
        fcnArgs = stdFunctionType.split('(')[1].split(')')[0].split(',')
        fcnArgsStr = fcnArgs[0]+' mainSystem'
        fcnArgsOnly = 'mainSystem'
        cnt = 0
        for arg in fcnArgs[1:]:
            fcnArgsStr += ', '+arg+' arg'+str(cnt)
            fcnArgsOnly += ', arg'+str(cnt)
            cnt+=1

        #create additional if for specific item / UF case
        s+= ' '*8 + useElse+'if (itemTypeName == "'+classType+itemType+'" && userFunctionName == "'
        s+= userFunctionName+'")\n'
        s+= ' '*8 +'{'
        s+= ' '*12+'//define the user function as lambda function of this\n'
        
        s+= ' '*12+userFunctionType + ' = [this]('+fcnArgsStr+')\n'
        s+= ' '*16+'{\n'
        s+= ' '*20+'return this->Evaluate'+returnValueType+'('+fcnArgsOnly+');\n'
        s+= ' '*16+'};\n'
        s+= ' '*8+'}\n'
        useElse = 'else '

    s+="""		else
		{
			PyError(STDstring("Symbolic::SetUserFunctionFromDict<") + itemTypeName + "," + userFunctionName +
				">: invalid user object type or user function type; possibly, function is not available as symbolic user function");
		}

	}

"""    
    sTemplateInstantiation += 'template class PythonUserFunctionBase<std::function<void(const MainSystem&, Real, Index, Index)>>;\n'
    sTemplateInstantiation += 'template class PythonUserFunctionBase<std::function<PyMatrixContainer(const MainSystem&, Real, Real, Real, Real)>>;\n'


    s = stdFunctionMember + s
    s+= sSTDfunction + sSTDfunctionEnd
    # print(s)
    return [[s,'PySymbolicUserFunctionSet'],[sTemplateInstantiation, 'PythonUserFunctionsTemplates']]



#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def CheckForLeftoverItemHeaders(directoryString):
    """raise if directoryString holds C/Main/Visu item headers of an item that definitions/ no longer
    has: nothing rewrites such a file, so the drift gate cannot see it (#2428)"""
    classNames = set(definition['className'] for definition in im.ItemDefinitions())
    pattern = re.compile(r'^(C|Main|Visu)((' + '|'.join(im.itemTypeOrder) + r')\w+)\.h$')
    leftovers = sorted(fileName for fileName in os.listdir(directoryString)
                       if pattern.match(fileName) and pattern.match(fileName).group(2) not in classNames)
    if leftovers:
        raise ValueError('item headers without a definition in definitions/ (delete them): ' + ', '.join(leftovers))


def WriteItemHeaders(directoryString):
    """write all item headers into directoryString (ending with '/'); returns the number of files changed"""
    totalNumberOfLines = 0
    totalNumberOfFilesChanged = 0
    symbolicUserFunctionSet = list(im.mainSystemUserFunctions)

    for definition in im.ItemDefinitions():
        fileStr = ItemCppHeaders(definition)
        symbolicUserFunctionSet += fileStr[3]

        #+++++++++++++++++++++++++++++++
        #write files if changes apply:
        nLinesHeader = 7 # number of header lines which are ignored in file comparison
        strFileMode = 'w'
        fileName = directoryString + 'C'+definition['className']+'.h'
        fileText = 'INVALID'
        if os.path.isfile(fileName):
            file=open(fileName,'r',encoding='utf8'); fileText = file.read();file.close()

        if (CutLinesFromString(fileText,nLinesHeader) != CutLinesFromString(fileStr[0],nLinesHeader)):
            #write computational 'C' class
            file=open(fileName,strFileMode) 
            file.write(fileStr[0])
            file.close()
            totalNumberOfFilesChanged += 1

        fileName = directoryString + 'Main'+definition['className']+'.h'
        fileText = 'INVALID'
        if os.path.isfile(fileName):
            file=open(fileName,'r',encoding='utf8'); fileText = file.read();file.close()
        if (CutLinesFromString(fileText,nLinesHeader) != CutLinesFromString(fileStr[1],nLinesHeader)):
            #write Main class
            file=open(fileName,strFileMode) 
            file.write(fileStr[1])
            file.close()
            totalNumberOfFilesChanged += 1

        fileName = directoryString + 'Visu'+definition['className']+'.h'
        fileText = 'INVALID'
        if os.path.isfile(fileName):
            file=open(fileName,'r',encoding='utf8'); fileText = file.read();file.close()
        if (CutLinesFromString(fileText,nLinesHeader) != CutLinesFromString(fileStr[2],nLinesHeader)):
            #write Visualization class
            file=open(fileName,strFileMode) 
            file.write(fileStr[2])
            file.close()
            totalNumberOfFilesChanged += 1


        totalNumberOfLines += CountLines(fileStr[0]) + CountLines(fileStr[1]) + CountLines(fileStr[2])

    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #an item header without a definition is left over from a removed item (revision plan step 101)
    CheckForLeftoverItemHeaders(directoryString)

    #%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #write include files (.h) for user functions
    incFiles = []    
    # incFiles += [[CreateStringSymbolicUserFunctionTransfer(symbolicUserFunctionSet), 
    #              'PySymbolicUserFunctionTransfer']]
    incFiles += CreateStringSymbolicUserFunctionSet(symbolicUserFunctionSet)
    for [pyStr, name] in incFiles:
        fileUserFunction = directoryString + name+'.h'
        totalNumberOfLines += pyStr.count('\n')
        with open(fileUserFunction,'w',encoding='utf8') as f:
            f.write('    //include file for '+name+'\n')
            f.write('    //author: Johannes Gerstmayr\n')
            f.write('    //license: see Exudyn license\n')
            f.write('    //AUTO:\n')
            f.write(pyStr+'\n')
            
    
        if os.path.isfile(fileName):
            file=open(fileUserFunction,'r',encoding='utf8'); fileText = file.read();file.close()



    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #the item list for auto-registration, in definition order per item type
    globalItemsDict = {}
    for item in im.itemTypeOrder:
        globalItemsDict[item] = {}
    for definition in im.ItemDefinitions():
        classType = definition['classType']
        globalItemsDict[classType][definition['className'][len(classType):]] = {}

    #files and structures for autoregistration of items
    from autoGenerateHelper import minimalItemsList
    excludeItemsList=[]
    
    templateItemSDAutoReg="""
bool MainObject{classNamePure}IsRegistered = ClassFactoryItemsSystemData<Main{itemType}>::Get().RegisterClass("{classNamePure}", [](CSystemData* cSystemData)
	{ //AUTO: 
		C{itemType}* c{itemType} = new C{itemType}{classNamePure}();
		c{itemType}->SetCSystemData(cSystemData);
		MainObject* object = new MainObject{classNamePure}(); //new main object
		object->SetC{itemType}(c{itemType});
		VisualizationObject{classNamePure}* vObject = new VisualizationObject{classNamePure}();
		object->SetVisualizationObject(vObject);
		return object;
	});
"""
    templateItemAutoReg="""
bool Main{itemType}{classNamePure}IsRegistered = ClassFactoryItem<Main{itemType}>::Get().RegisterClass("{classNamePure}", []
	{ //AUTO: 
		C{itemType}{classNamePure}* cItem = new C{itemType}{classNamePure}();							//new point {itemType}
		Main{itemType}* item = new Main{itemType}{classNamePure}(); //new main item
		item->SetC{itemType}(cItem);
		Visualization{itemType}{classNamePure}* vItem = new Visualization{itemType}{classNamePure}();
		item->SetVisualization{itemType}(vItem);
		return item;
	});
"""
    templateNodeAutoReg="""
bool Main{itemType}{classNamePure}IsRegistered = ClassFactoryItemsSystemData<Main{itemType}>::Get().RegisterClass("{classNamePure}", [](CSystemData* cSystemData)
	{ //AUTO: 
		C{itemType}{classNamePure}* cItem = new C{itemType}{classNamePure}();							//new point {itemType}
		cItem->GetCData() = &(cSystemData->GetCData()); //add CData reference to CNode
		Main{itemType}* item = new Main{itemType}{classNamePure}(); //new main item
		item->SetC{itemType}(cItem);
		Visualization{itemType}{classNamePure}* vItem = new Visualization{itemType}{classNamePure}();
		item->SetVisualization{itemType}(vItem);
		return item;
	});
"""
    sAutoRegMinimal = ''
    sAutoReg = ''
    for key, value in globalItemsDict.items():
        for key1, value1 in value.items():
            fullName = key+key1
            if fullName in excludeItemsList:
                continue
            if key == 'Object':
                code = templateItemSDAutoReg.replace('{classNamePure}',key1).replace('{itemType}',key)
            elif key == 'Node':
                code = templateNodeAutoReg.replace('{classNamePure}',key1).replace('{itemType}',key)
            else:
                code = templateItemAutoReg.replace('{classNamePure}',key1).replace('{itemType}',key)
            if fullName in minimalItemsList:
                sAutoRegMinimal += code
            else:
                sAutoReg += code
            

    fileAutoReg = directoryString+'objectFactoryAutoReg.h'
    with open(fileAutoReg,'w',encoding='utf8') as f:
        f.write('/** **************************************\n')
        f.write('* @brief        autogenerated registration variables for items\n')
        f.write('* @author       Gerstmayr Johannes\n')
        f.write('* @date         2024-02-21 (first created)\n')
        f.write('****************************************** */\n')
        f.write('//AUTO: do not modify\n\n')
        f.write(sAutoRegMinimal+'\n')
        f.write('#ifndef EXUDYN_MINIMAL_COMPILATION\n')
        f.write(sAutoReg+'\n')
        f.write('#endif //EXUDYN_MINIMAL_COMPILATION\n\n')

    print('item headers: total number of lines generated =', totalNumberOfLines)
    print('item headers: total number of files changed =', totalNumberOfFilesChanged)
    return totalNumberOfFilesChanged


def main(argv=None):
    parser = argparse.ArgumentParser(description='Emit the item C++ headers from definitions/.')
    parser.add_argument('--output-dir', default=os.path.join(im.repositoryRoot, 'src', 'Autogenerated'),
                        help='directory to write into (default: src/Autogenerated)')
    args = parser.parse_args(argv)
    WriteItemHeaders(args.output_dir.replace(os.sep, '/').rstrip('/') + '/')
    return 0


if __name__ == '__main__':
    sys.exit(main())
