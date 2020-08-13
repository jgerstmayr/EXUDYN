# -*- coding: utf-8 -*-
"""
Created on Fri May 18 08:53:30 2018

@author: Johannes Gerstmayr

goal: automatically generate interfaces for structures
currently: automatic generate structures with ostream and initialization
"""

from autoGenerateHelper import RemoveSpacesTabs, CountLines, TypeConversion, GenerateHeader, SplitString, Str2Latex, DefaultValue2Python, Str2Doxygen, GetDateStr, GetTypesStringLatex
import os

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

#possible types for certain items (object, marker, node, ...)
possibleTypes = {'Object':['_None','Ground','Connector','Constraint','Body','SingleNoded','MultiNoded','FiniteElement','SuperElement'],
                 'Node':['_None','Ground','Position2D','Orientation2D','Point2DSlope1','Position','Orientation','RigidBody',
                         'RotationEulerParameters','RotationRxyz','RotationRotationVector','RotationLieGroup',
                         'GenericODE2','GenericData'],
                 'Marker':['_None','Node','Object','Body','Position','Orientation','Coordinate','BodyLine','BodySurface',
                           'BodyVolume','BodyMass','BodySurfaceNormal'],
                 'Load':[], 'Sensor':[]}

#conversion list for python functions; names must always start with 'PyFunction'...
pyFunctionTypeConversion = {'PyFunctionGraphicsData': 'std::function<py::object(const MainSystem&, Index)>',
                            'PyFunctionScalar2': 'std::function<Real(Real,Real)>',
                            'PyFunctionScalar6': 'std::function<Real(Real,Real,Real,Real,Real,Real)>', #ConnectorSpringDamper
                            'PyFunctionScalar8': 'std::function<Real(Real,Real,Real,Real,Real,Real,Real,Real)>', #CoordinateSpringDamper
                            'PyFunctionVector3DScalarVector3D': 'std::function<StdVector(Real,StdVector3D)>', #LoadForceVector, LoadTorqueVector, LoadMassProportional
                            'PyFunctionVector6DScalarVector6D': 'std::function<StdVector(Real,StdVector6D)>', #GenericJoint
                            'PyFunctionVector3DScalar5Vector3D': 'std::function<StdVector(Real, StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdVector3D)>', #CartesianSpringDamper
#StdVector3D=std::array<Real,3> does not accept numpy::array                            'PyFunctionVector3DScalarVector3D': 'std::function<StdVector3D(Real,StdVector3D)>', #LoadForceVector, LoadTorqueVector, LoadMassProportional
#                            'PyFunctionVector6DScalarVector6D': 'std::function<StdVector6D(Real,StdVector6D)>', #GenericJoint
#                            'PyFunctionVector3DScalar5Vector3D': 'std::function<StdVector3D(Real, StdVector3D,StdVector3D,StdVector3D,StdVector3D,StdVector3D)>', #CartesianSpringDamper
                            'PyFunctionVectorScalar2Vector': 'std::function<StdVector(Real, StdVector,StdVector)>', #ObjectGenericODE2
                            'PyFunctionMatrixScalar2Vector': 'std::function<NumpyMatrix(Real, StdVector,StdVector)>' #ObjectGenericODE2
                            }

#this function finds out, if a parameter is set with a special Set...Safely function in C++
def IsASetSafelyParameter(parameterType):
    if ((parameterType == 'String') |
        (parameterType == 'Vector2D') | 
        (parameterType == 'Vector3D') | 
        (parameterType == 'Vector4D') | 
        (parameterType == 'Vector6D') | 
        (parameterType == 'Matrix3D') | 
        (parameterType == 'Matrix6D') | 
        (parameterType == 'NumpyMatrix') | 
        (parameterType == 'NumpyMatrixI') | #for index arrays, mesh, ...
        (parameterType == 'PyMatrixContainer') | 
        (parameterType == 'NumpyVector') | 
        (parameterType == 'Vector7D') ):
        return True
    else:
        return False

#extract a latex $...$ code / symbol out of a string
#return [stringWithoutSymbol, stringLatexSymbol]
def ExtractLatexSymbol(s):
    stringLatexSymbol=""
    stringWithoutSymbol=""
    if s[0]=='$':
        splitString = s.split('$')
        n = len(splitString)
        
        if n == 3: #one symbol + text
            stringLatexSymbol = "$" + splitString[1] + "$"
            stringWithoutSymbol=splitString[2]
        elif n%2 != 1:
            print("ERROR: did not find closing $ for description/variable; str =", s)
        else: #several symbols, but one leading
            stringLatexSymbol = "$" + splitString[1] + "$"
            addLatexSign=''
            for i in range(2,n):
                if i%2 == 1:
                    sAdd = splitString[i]
                else:
                    sAdd = splitString[i].replace('_','\\_')                    
                stringWithoutSymbol+=addLatexSign+sAdd
                addLatexSign = '$'

#        print("splitString=",splitString)
#        print("stringLatexSymbol=",stringLatexSymbol)
#        print("stringWithoutSymbol=",stringWithoutSymbol)
    else:
        stringWithoutSymbol=s

    return [stringWithoutSymbol, stringLatexSymbol]
    


#function which writes the mini examples for every item into a separate file
def WriteMiniExample(className, miniExample):
    s = '#+++++++++++++++++++++++++++++++++++++++++++\n'
    s+= '# Mini example for class ' + className + '\n'
    s+= '#+++++++++++++++++++++++++++++++++++++++++++\n\n'

    s+= 'import sys\n'
    s+= "sys.path.append('../../bin/WorkingRelease')\n"
    s+= "sys.path.append('../TestModels')\n"
    s+= 'from modelUnitTests import ExudynTestStructure, exudynTestGlobals\n'
    s+= 'import exudyn as exu\n'
    s+= 'from exudynUtilities import *\n'
    s+= 'from itemInterface import *\n'
    s+= 'import numpy as np\n'
    s+= '\n'
    s+= '#create an environment for mini example\n'
    s+= 'SC = exu.SystemContainer()\n'
    s+= 'mbs = SC.AddSystem()\n'
    s+= '\n'
    s+= 'oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))\n'
    s+= 'nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))\n'
    s+= '\n'
    s+= 'testError=1 #set default error, if failed\n'
    s+= 'exu.Print("start mini example for class ' + className + '")\n'
    s+= 'try: #puts example in safe environment\n'
    s+= miniExample
    s+= '\n'
    s+= 'except BaseException as e:\n'
    s+= '    exu.Print("An error occured in test example for ' + className + ':", e)\n'
    s+= 'else:\n'
    s+= '    exu.Print("example for ' + className + ' completed, test error =", testError)\n'
    s+= '\n'
    
    fileExample=open('../../pythonDev/MiniExamples/'+className+'.py','w') 
    fileExample.write(s)
    fileExample.close()


#************************************************
#create autogenerated .h  files for list of parameters
#creates computational and main class files including parameter classes
def WriteFile(parseInfo, parameterList, typeConversion):
    #print (parseInfo)
    #print('file="'+parseInfo['writeFile']+'"')
    
    #these are the typecasts for the dictionary in the according pybind functions in MainItem
    typeCasts = {'Bool':'bool', 'Int':'int', 'Real':'Real', 'UInt':'Index', 'UReal':'Real', 
                 'Vector':'std::vector<Real>', 'Vector7D':'std::vector<Real>', 'Vector6D':'std::vector<Real>', 
                 'Vector4D':'std::vector<Real>', 'Vector3D':'std::vector<Real>', 'Vector2D':'std::vector<Real>', 
                 'Matrix':'Matrix', 'SymmetricMatrix':'Matrix', 'Matrix6D':'std::array<std::array<Real,6>,6>', 
                 'ArrayIndex':'std::vector<Index>', 'String':'std::string',
                 'NumpyMatrix':'py::array_t<Real>', 
                 'NumpyMatrixI':'py::array_t<Index>', 
                 #'MatrixContainer':'PyMatrixContainer', 
                 'NumpyVector':'py::array_t<Real>',
                 'Float2': 'std::vector<float>', 'Float3': 'std::vector<float>', 'Float4': 'std::vector<float>',  #e.g. for OpenGL vectors
                 'Float9': 'std::vector<float>', 'Float16': 'std::vector<float>', #e.g. for OpenGL rotation matrix and homogenous transformation
                 'Index2': 'std::vector<Index>', 'Index3': 'std::vector<Index>'
                 } #convert parameter types to C++/EXUDYN types
    
    typeCasts.update(pyFunctionTypeConversion)#add the list of python (user) functions
    
    classStr = parseInfo['class']
    
    #main and computational PARAMETER classes:
    compParamClassStr = "C" + classStr + "Parameters"
    mainParamClassStr = "Main" + classStr + "Parameters"

    #main and computational classes:
    compClassStr = "C" + classStr
    mainClassStr = "Main" + classStr
    visuClassStr = "Visualization" + classStr

    
    classNames = [compParamClassStr, mainParamClassStr, compClassStr, mainClassStr, visuClassStr]

    #count parameters and find if there are types which need special treatment or special include files
    cntParameters = [0,0,0,0,0]
    usesPyFunction = False #flag, which shows that PyFunctions are used ==> needs <functional> from C++ std library and pybind11/functional.h
    for parameter in parameterList:
        if (parameter['lineType'] == 'V'): #only if it is a member variable
            cntParameters[DestinationNr(parameter['destination'])] += 1
        if parameter['type'].find('PyFunction') != -1: 
            usesPyFunction = True

    
    #now start generating strings for classes; the parameter classes go into the same file on top,
    #  therefore the header goes to the parameter classes
    sParamComp = GenerateHeader(compParamClassStr, 'Parameter class for '+compClassStr)
    sParamMain = GenerateHeader(mainParamClassStr, 'Parameter class for '+mainClassStr)
    sParamMain += '#include <pybind11/pybind11.h>      //! AUTO: include pybind for dictionary access\n'
    sParamMain += '#include <pybind11/stl.h>           //! AUTO: needed for stl-casts; otherwise py::cast with std::vector<Real> crashes!!!\n'
    sParamMain += 'namespace py = pybind11;            //! AUTO: "py" used throughout in code\n'
    if usesPyFunction:
        sParamMain += '#include <pybind11/functional.h> //! AUTO: for function handling ... otherwise gives a python error (no compilation error in C++ !)\n'
        sParamComp += '#include <functional> //! AUTO: needed for std::function\n'

    #sParamMain += 'using namespace pybind11::literals; //! # enables the "_a" literals\n\n'
    sParamMain += '#include "Autogenerated/' + compClassStr + '.h"\n\n'
    sParamMain += '#include "Autogenerated/Visu' + classStr + '.h"\n'

    sParamComp += parseInfo['addIncludesC']
    sParamMain += parseInfo['addIncludesMain']
    
    sParamComp += '\n'
    sParamMain += '\n'
#   DONE via addIncludesC
#    cPC = parseInfo['cParentClass']
#    if (cPC != 'CNodeODE2') & (cPC != 'CNodeData') & (cPC != 'CObjectBody') & (cPC != 'CObjectConnector') & (cPC != 'CObjectConstraint') & (cPC != 'CMarker') & (cPC != 'CLoad'):
#        sParamComp += '#include "Autogenerated/' + parseInfo['cParentClass'] + '.h"  //! AUTO: include parent class\n'
#        sParamMain += '#include "Autogenerated/' + parseInfo['mainParentClass'] + '.h"  //! AUTO: include main parent class\n'

    sVisu = GenerateHeader(visuClassStr, Str2Doxygen(parseInfo['classDescription']))
    #no includes for visualization classes, because they are included at a place, where all necessary headers exist
    #sParamMain += '#include "Graphics/Visualization.h"      //! AUTO: link to visualization class; also includes settings and base classes VisualizationObject/Node/...

    if len(parseInfo['miniExample']) != 0:
        WriteMiniExample(parseInfo['class'], parseInfo['miniExample'])

    sComp = "" #computation
    sComp = GenerateHeader(compClassStr, Str2Doxygen(parseInfo['classDescription']), addModifiedDate=False, addIfdefOnce = False)
    sMain = "" #main object
    sMain = GenerateHeader(mainClassStr, Str2Doxygen(parseInfo['classDescription']), addModifiedDate=False, addIfdefOnce = False)
    #make list of strings to enable iteration
    sList = [sParamComp, sParamMain, sComp, sMain, sVisu]
    nClasses = 5 #number of different classes
    indexComp = 2 #index in sList
    indexMain = 3 #index in slist

    #************************************
    #class definition:
    strParentClass = ["", "",""]
    if len(parseInfo['cParentClass']) != 0:
        strParentClass[0] = ': public ' + parseInfo['cParentClass']

    if len(parseInfo['mainParentClass']) != 0:
        strParentClass[1] = ': public ' + parseInfo['mainParentClass']

    if len(parseInfo['visuParentClass']) != 0:
        strParentClass[2] = ': public ' + parseInfo['visuParentClass']

    for i in range (2):
        sList[i] += '//! AUTO: Parameters for class ' + classNames[i] + '\n'
        sList[i+2] += '//! AUTO: ' + classNames[i+2] + '\n' #+ ': ' + Str2Doxygen(parseInfo['classDescription']) + '\n'

    for i in range (2):
        sList[i]+='class ' + classNames[i] + ' // AUTO: \n' + '{\n' #parameter classes
        sList[i+2]+='class ' + classNames[i+2] + strParentClass[i] + ' // AUTO: \n' + '{\n' #regular classes
    
    sList[4] += 'class ' + classNames[4] + strParentClass[2] + ' // AUTO: \n' + '{\n' #visualization class

    classTypeStr = parseInfo['classType']
    sTypeName = classStr.replace(classTypeStr,'')

    #************************************
    #Latex doc:
    sLatex = ''
    sLatexItemList = ''

    hasPybindInterface = False
    for parameter in parameterList:
        if (parameter['lineType'] == 'V') & (parameter['cFlags'].find('I') != -1): #only if it is a member variable
            hasPybindInterface = True


    if hasPybindInterface: #otherwise do not include the description into latex doc
        sLatexItemList += '  \\item ' + parseInfo['class'] + '\n'
        
        descriptionStr = parseInfo['classDescription']
        
        sLatex += '\n%+++++++++++++++++++++++++++++++++++\n\mysubsubsection{' + parseInfo['class'] + '}\n'
        sLatex += descriptionStr + '\\vspace{12pt}\n \\\\'


        cLatex  = '\\vspace{12pt} \\noindent The item {\\bf ' + parseInfo['class'] + "} with type = '"
        cLatex += sTypeName + "' has the following parameters:\\vspace{-1cm}\\\\ \n"
        
        vLatex  = 'The item V' + parseInfo['class'] + ' has the following parameters:\\vspace{-1cm}\\\\ \n'
        
        sTemp   = '%reference manual TABLE\n'
        sTemp  += '\\begin{center}\n'
        sTemp  += '  \\footnotesize\n'
        sTemp  += '  \\begin{longtable}{| p{4.5cm} | p{2.5cm} | p{0.5cm} | p{2.5cm} | p{6cm} |}\n'
        sTemp  += '    \\hline\n'
        sTemp  += '    \\bf Name & \\bf type & \\bf size & \\bf default value & \\bf description \\\\ \\hline\n'
        
        cLatex += sTemp
        vLatex += sTemp
    
        #cLatex += '    \\multicolumn{4}{l}{\\parbox{10cm}{type = ' + "'" + sTypeName + "'" + '}} & \\multicolumn{1}{l}{\\parbox{6cm}{\\it item typename for initialization}}\\\\ \\hline\n'

        symbolList = ''
        requestedMarkerString = ''
        itemTypeString = '' #string containing type of item (out of possibleTypes dict)
        requestedNodeString = ''


        for parameter in parameterList:
            if (parameter['lineType'].find('V') != -1) & (parameter['cFlags'].find('I') != -1): #also include parent class members!
                sString = ''
                if (parameter['type'] == 'String'):
                    sString="'"
                #write latex doc:
                parameterDescription = parameter['parameterDescription']
                [parameterDescription, latexSymbol] = ExtractLatexSymbol(parameterDescription)
                if len(latexSymbol) != 0:
                    symbolList+= "\\rowTable{" + parameter['pythonName'].replace('_','\\_') +"}{" + latexSymbol.replace('\n','\\n') + "}{}\n"  #this is the latex symbol string
                
                parameterTypeStr = parameter['type']
                parameterSizeStr = parameter['size']
                parameterDefaultValueStr = parameter['defaultValue']
                if len(parameterTypeStr) > 15:
                    parameterSizeStr = '\\tabnewline ' + parameterSizeStr 
                if len(parameterTypeStr) > 18:
                    parameterDefaultValueStr = '\\tabnewline ' + parameterDefaultValueStr 
                
                sTemp   = '    ' + Str2Latex(parameter['pythonName']) + ' & '
                sTemp  += '    ' + Str2Latex(parameterTypeStr) + ' & '
                sTemp  += '    ' + Str2Latex(parameterSizeStr) + ' & '
                sTemp  += '    ' + sString+Str2Latex(parameterDefaultValueStr, True)+sString + ' & '
                sTemp  += '    ' + parameterDescription + '\\\\ \\hline\n'
                if parameter['destination'].find('V') != -1: #visualization
                    vLatex += sTemp
                else:
                    cLatex += sTemp

            elif (parameter['pythonName'] == 'GetRequestedMarkerType'):
                requestedMarkerString = GetTypesStringLatex(parameter['defaultValue'],'Marker', possibleTypes['Marker'],' +')
            elif (parameter['pythonName'] == 'GetRequestedNodeType'):
                requestedNodeString = GetTypesStringLatex(parameter['defaultValue'],'Node', possibleTypes['Node'],' +')
            elif (parameter['pythonName'] == 'GetType'):
                searchType = parseInfo['classType']
                if parseInfo['classType']=='Object': searchType += 'Type'
                itemTypeString = GetTypesStringLatex(parameter['defaultValue'],searchType, possibleTypes[parseInfo['classType']])
                #print(parseInfo['classType']+':'+itemTypeString)
                

        cLatex += '    visualization & V' + parseInfo['class'] + ' & & & parameters for visualization of item \\\\ \\hline\n'
        cLatex += '	  \\end{longtable}\n'
        cLatex += '	\\end{center}\n'
        vLatex += '	  \\end{longtable}\n'
        vLatex += '	\\end{center}\n'
        #now assemble visualization and computation tables:

        #sLatex += requestedMarkerString
        if len(requestedMarkerString) + len(itemTypeString) + len(parseInfo['pythonShortName']) !=0:
            sLatex += '{\\bf Additional information for ' + parseInfo['class'] + '}:\n'
            sLatex += '\\bi\n'
            if len(itemTypeString) != 0:
                sLatex += '  \\item The ' + parseInfo['classType'] + ' has the following types = ' + itemTypeString + '\n'
            if len(requestedMarkerString) != 0:
                sLatex += '  \\item Requested marker type = ' + requestedMarkerString + '\n'
            if len(requestedNodeString) != 0:
                if requestedNodeString.find('_None') != -1:
                    sLatex += '  \\item Requested node type: read detailed information of item\n'
                else:
                    sLatex += '  \\item Requested node type = ' + requestedNodeString + '\n'
            if len(parseInfo['pythonShortName']) != 0:
                sLatex += '  \\item {\\bf Short name} for Python = {\\bf ' + parseInfo['pythonShortName'] + '}'
                sLatex += '  \\item {\\bf Short name} for Python (visualization object) = {\\bf V' + parseInfo['pythonShortName'] + '}'

            sLatex += '\\ei\n'


        sLatex += cLatex+vLatex

#        if len(parseInfo['outputVariables']) != 0:
#            sLatex += '{\\bf Output variables} (chose type, e.g., OutputVariableType.Position): \n\\begin{itemize}\n'
#            dictOV = eval(parseInfo['outputVariables']) #output variables are given as a string, representing a dictionary with OutputVariables and descriptions
#            for outputVariables in dictOV.items(): 
#                sLatex += '    \\item {\\bf ' + outputVariables[0].replace('_','\_') + '}: ' + outputVariables[1].replace('_','\_') + '\n'
#            
#            sLatex += '\\end{itemize}\n'

        addLatex = ''
        if len(symbolList) != 0: #automatically generated import parameter symbol list
            #addLatex += "\\vspace{6pt}\\\\ \n"
            addLatex += "\\startTable{input parameter}{symbol}{description see tables above}\n"
            addLatex += symbolList
            addLatex += "\\finishTable\n"

        #process outputVariables, including symbols
        if len(parseInfo['outputVariables']) != 0:
            addLatex += "{\\bf The following output parameters are available as OutputVariableType in sensors and other functions}: \n"
            addLatex += "\\startTable{output parameter}{symbol}{description}\n"
            #print("dict=",parseInfo['outputVariables'].replace('\\','\\\\'))
            dictOV = eval(parseInfo['outputVariables'].replace('\n','\\n').replace('\\','\\\\')) #output variables are given as a string, representing a dictionary with OutputVariables and descriptions
            for outputVariables in dictOV.items(): 
                oVariable = outputVariables[0].replace('_','\\_')
                description = outputVariables[1]
                [description, latexSymbol] = ExtractLatexSymbol(description)
                if len(latexSymbol) != 0: 
                    latexSymbol = latexSymbol
                addLatex += "\\rowTable{" + oVariable +"}{" + latexSymbol + "}{" + description + "}\n"  #this is the line for one outputvariable
            addLatex += "\\finishTable\n" #outputvariables

        if len(parseInfo['equations']) != 0:
            addLatex += "{\\bf Description of Item}:\n"
            addLatex +=' \\noindent\n' + parseInfo['equations']
        if len(parseInfo['miniExample']) != 0:
            #addLatex +='\\vspace{12pt}\\\\ \n'
            addLatex +='\\noindent {\\bf Example} for ' + parseInfo['class'] + ':\n'
            addLatex +='\\pythonstyle\n'
            addLatex +='\\begin{lstlisting}[language=Python, firstnumber=1]\n'
            addLatex += parseInfo['miniExample'] + '\n'
            addLatex +='\\end{lstlisting}\n\n'

        if len(addLatex) != 0:
            sLatex += "{\\bf Detailed information on " + parseInfo['class'] + "}:\n"
            sLatex += addLatex


    #************************************
    #Python interface class:
    sPythonClass = '' #the python interface class definition
    sPythonClassInit = '' #the init function body
    sPythonIter = ''  #the iterator member function

    vPythonClass = '' #the python visualization interface class definition
    vPythonClassInit = '' #the init function body
    vPythonIter = ''  #the iterator member function
    sIndent = '    ' #4 spaces indentation for python


    if hasPybindInterface: #otherwise do not include the description into latex doc
        sPythonClass += 'class ' + parseInfo['class'] + ':\n'
        sPythonClass += sIndent+'def __init__(self'
        sPythonIter += sIndent+sIndent+'yield ' + "'" + classTypeStr[0].lower() + classTypeStr[1:] + 'Type' + "'" + ', ' + "'"+sTypeName+"'" + '\n'
        
        vPythonClass += 'class V' + parseInfo['class'] + ':\n'
        vPythonClass += sIndent+'def __init__(self'
        vDefaultDict = '{'
        vDefaultDictEmpty = True
        
        for parameter in parameterList:
            if (parameter['lineType'].find('V') != -1) and (parameter['cFlags'].find('I') != -1) and (parameter['cFlags'].find('R') == -1): #only if it is a variable; also include Vp variables - i.e. 'name'
                sString = ''
                if (parameter['type'] == 'String'):
                    sString="'"

                defaultValueStr = sString+DefaultValue2Python(parameter['defaultValue'])+sString

                #special treatment of BodyGraphicsData
                if parameter['type'] == 'BodyGraphicsData':
                    defaultValueStr = '[]'

                #write item interface class initialization, constructor and iterator doc:
                tempVPythonDict = "'" + parameter['pythonName'] + "': "
                tempPythonClass = ', ' + parameter['pythonName']
                if len(defaultValueStr) != 0:
                    tempPythonClass += ' = ' + defaultValueStr
                    tempVPythonDict += defaultValueStr
                else:
                    tempVPythonDict += "None"
                    
                tempPythonClassInit = sIndent+sIndent+'self.' + parameter['pythonName'] + ' = ' + parameter['pythonName'] + '\n'
                tempPythonIter = sIndent+sIndent+'yield ' + "'" + parameter['pythonName'] + "'" + ', self.' + parameter['pythonName'] + '\n'
                
                if parameter['destination'].find('V') != -1: #visualization
                    vPythonClass += tempPythonClass
                    vPythonClassInit += tempPythonClassInit
                    vPythonIter += tempPythonIter
                    if not(vDefaultDictEmpty): #if already second dict entry added, also add a comma separator
                        vDefaultDict += ", "
                    
                    vDefaultDict += tempVPythonDict
                    vDefaultDictEmpty = False
                    #changed visualization to be a dict by default; this improves the type completion! 
                    #OLD MODE: sPythonIter += sIndent+sIndent+'yield ' + "'V" + parameter['pythonName'] + "'" + ', self.visualization.' + parameter['pythonName'] + '\n'
                    sPythonIter += sIndent+sIndent+'yield ' + "'V" + parameter['pythonName'] + "'" + ', dict(self.visualization)["' + parameter['pythonName'] + '"]\n'
                else: #rest: computational or main
                    sPythonClass += tempPythonClass
                    sPythonClassInit += tempPythonClassInit
                    sPythonIter += tempPythonIter

        vDefaultDict += '}'
        #print(vDefaultDict)
        sPythonClass += ', visualization = ' + vDefaultDict + '):\n' #add visualization structure (must always be there...)
        #sPythonClass += ', visualization = {}):\n' #add visualization structure (must always be there...)
        #OLD MODE: sPythonClass += ', visualization = V' + parseInfo['class'] + '()):\n' #add visualization structure (must always be there...)
        sPythonClass += sPythonClassInit + sIndent+sIndent+'self.visualization = visualization\n\n'
        sPythonClass += sIndent+'def __iter__(self):\n'
        sPythonClass += sPythonIter + '\n'

        vPythonClass += '):\n'
        vPythonClass += vPythonClassInit + '\n'
        vPythonClass += sIndent+'def __iter__(self):\n'
        vPythonClass += vPythonIter + '\n'

        sPythonClass = vPythonClass + sPythonClass #visualization class must be first, otherwise the main class cannot be initialized
        if (len(parseInfo['pythonShortName'])):
            sPythonClass += '#add typedef for short usage:\n'
            sPythonClass += parseInfo['pythonShortName'] + ' = ' + parseInfo['class'] + '\n'
            sPythonClass += 'V'+parseInfo['pythonShortName'] + ' = V' + parseInfo['class'] + '\n\n'
        
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #member variables:
    sList[0]+='public: // AUTO: \n' #parameter classes are just structs
    sList[1]+='public: // AUTO: \n'  
    sList[2]+='protected: // AUTO: \n'
    sList[3]+='protected: // AUTO: \n'
    sList[4]+='protected: // AUTO: \n'

    sList[2]+=parseInfo['addProtectedC']
    sList[3]+=parseInfo['addProtectedMain']

    #add pointer to computation class in main class
    compClassVariable = compClassStr[0].lower()+compClassStr[1:]  #instance name is lower case
    visuClassVariable = visuClassStr[0].lower()+visuClassStr[1:]  #instance name is lower case
    sList[3]+='    ' + compClassStr + '* ' + compClassVariable + '; //pointer to computational object (initialized in object factory) AUTO:\n'
    sList[3]+='    ' + visuClassStr + '* ' + visuClassVariable + '; //pointer to computational object (initialized in object factory) AUTO:\n'

    #add parameter member variables
    for i in range(2): # 0...comp parameters, 1...main parameters
        if cntParameters[i] != 0:
            sList[i+2]+='    ' + classNames[i] + ' parameters; //! AUTO: contains all parameters for '
            sList[i+2]+=classNames[i+2] + '\n'
            

    #process variables:    
    for parameter in parameterList:
        if (parameter['lineType'] == 'V'): #only if it is a member variable
            typeStr = TypeConversion(parameter['type'], typeConversion)
            if parameter['cFlags'].find('U') != -1:
                typeStr = 'mutable ' + typeStr #make this variable changable in GetMassMatrix(), ComputeODE2RHS(), ... functions
                #print(typeStr)
            temp = '    ' + typeStr + ' ' + parameter['cplusplusName']+ ';'
            nChar = len(temp)
            alignment = 50
            insertSpaces = ''
            if nChar < alignment:
                insertSpaces = ' '*(alignment-nChar)

            parameterDescription = parameter['parameterDescription'] #remove symbol from parameter description
            [parameterDescription, latexSymbol] = ExtractLatexSymbol(parameterDescription)

            lineStr = temp + insertSpaces + '//!< AUTO: ' + Str2Doxygen(parameterDescription) + '\n'

            sList[DestinationNr(parameter['destination'])] += lineStr

            
    sList[2]+='\npublic: // AUTO: \n'
    sList[3]+='\npublic: // AUTO: \n'
    sList[2]+=parseInfo['addPublicC']
    sList[3]+=parseInfo['addPublicMain']

    sList[4]+='\npublic: // AUTO: \n' # visualization

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #default parameters:
    
    #count number of default parameters
    cntDefaultParameters = [0,0,0,0,0]
    for parameter in parameterList:
        if (parameter['lineType'].find('V') != -1): #member variable: also include parent members for default initialization (name and V.show)
            strDefault = parameter['defaultValue']
            if len(strDefault) | (parameter['type'] == 'String'): 
                cntDefaultParameters[DestinationNr(parameter['destination'])] += 1

    #constructor with default initialization:
    for i in range(nClasses):
        if cntDefaultParameters[i]:
            sList[i]+='    //! AUTO: default constructor with parameter initialization\n'
            sList[i]+='    '+classNames[i]+'()\n'
            sList[i]+='    {\n'
        
            for parameter in parameterList:
                if (parameter['lineType'].find('V') != -1): #only if it is a variable; include parent members
                    strDefault = parameter['defaultValue']
                    if len(strDefault) | (parameter['type'] == 'String'): #only add initialization if default value exists
                        if parameter['type'] == 'String':
                            strDefault = '"' + strDefault + '"'
                        tempStr='        ' + parameter['cplusplusName'] + ' = ' + strDefault + ';\n'
                        if (DestinationNr(parameter['destination']) == i):
                            sList[i]+=tempStr
        
            sList[i]+='    };\n'
    
    sList[2]+='\n    // AUTO: access functions\n'
    sList[3]+='\n    // AUTO: access functions\n'
    sList[4]+='\n    // AUTO: access functions\n'

    #access functions to compClass pointer:
    sList[3]+='    //! AUTO: Get pointer to computational class\n'
    sList[3]+='    ' + compClassStr + '* Get' + compClassStr + '() { return ' + compClassVariable + '; }\n'
    sList[3]+='    //! AUTO: Get const pointer to computational class\n'
    sList[3]+='    const ' + compClassStr + '* Get' + compClassStr + '() const { return ' + compClassVariable + '; }\n'
    sList[3]+='    //! AUTO: Set pointer to computational class (do this only in object factory!!!)\n'
    sList[3]+='    void Set' + compClassStr + '(' + compClassStr + '* p' + compClassStr + ') { ' + compClassVariable + ' = p' + compClassStr + '; }\n\n'

    #access functions to visuClass pointer:
    sList[3]+='    //! AUTO: Get pointer to visualization class\n'
    sList[3]+='    ' + visuClassStr + '* Get' + visuClassStr + '() { return ' + visuClassVariable + '; }\n'
    sList[3]+='    //! AUTO: Get const pointer to visualization class\n'
    sList[3]+='    const ' + visuClassStr + '* Get' + visuClassStr + '() const { return ' + visuClassVariable + '; }\n'
    sList[3]+='    //! AUTO: Set pointer to visualization class (do this only in object factory!!!)\n'
    sList[3]+='    void Set' + visuClassStr + '(' + visuClassStr + '* p' + visuClassStr + ') { ' + visuClassVariable + ' = p' + visuClassStr + '; }\n\n'

    baseClass = parseInfo['classType']
    if len(baseClass) != 0:
        cBaseClass= 'C' + baseClass;
        sList[3]+='    //! AUTO: Get const pointer to computational base class object\n' #added for better generalization of main/comp objects
        sList[3]+='    virtual ' + cBaseClass + '* Get' + cBaseClass + '() const { return ' + compClassVariable + '; }\n'
        sList[3]+='    //! AUTO: Set pointer to computational base class object (do this only in object factory; type is NOT CHECKED!!!)\n'
        sList[3]+='    virtual void Set' + cBaseClass + '(' + cBaseClass + '* p' + cBaseClass + ') { ' + compClassVariable + ' = (' + compClassStr + '*)p' + cBaseClass + '; }\n\n'
        visuBaseClass= 'Visualization' + baseClass;
        sList[3]+='    //! AUTO: Get const pointer to visualization base class object\n' #added for better generalization of main/comp objects
        sList[3]+='    virtual ' + visuBaseClass + '* Get' + visuBaseClass + '() const { return ' + visuClassVariable + '; }\n'
        sList[3]+='    //! AUTO: Set pointer to visualization base class object (do this only in object factory; type is NOT CHECKED!!!)\n'
        sList[3]+='    virtual void Set' + visuBaseClass + '(' + visuBaseClass + '* p' + visuBaseClass + ') { ' + visuClassVariable + ' = (' + visuClassStr + '*)p' + visuBaseClass + '; }\n\n'

    #print(cntParameters)
    #add parameter structures and access functions
    for i in range(2): # 0...comp parameters, 1...main parameters
        if cntParameters[i] != 0:
            sList[i+2]+='    //! AUTO: Write (Reference) access to parameters\n'
            sList[i+2]+='    virtual ' + classNames[i] + '& GetParameters() { return parameters; }\n'
            sList[i+2]+='    //! AUTO: Read access to parameters\n'
            sList[i+2]+='    virtual const ' + classNames[i] + '& GetParameters() const { return parameters; }\n\n'
            
        

    #GetClone() function:
    # should not be done with objects; with parameters it should not be necessary (no ObjectList<parameterObject>)
    #    s+='  //! # clone object; specifically for copying instances of derived class, for automatic memory management e.g. in ObjectContainer\n'
    #    s+='  virtual ' + parseInfo['class'] + '* GetClone() const { return new '+parseInfo['class']+'(*this); }\n'
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
        i = DestinationNr(parameter['destination']) #sList: [sParamComp, sParamMain, sComp, sMain, sVisu]
        paramStr = parameter['cplusplusName']
        functionStr = paramStr
        c = functionStr[0] #take first character; !remember that the member variable must be lower-case
        functionStr = c.upper()+functionStr[1:]

        #process variables:
        if (parameter['lineType'].find('V') != -1): #only if it is a variable; also include Vp variables - i.e. 'name'
            typeStr = TypeConversion(parameter['type'], typeConversion)
            refChar = '&' #use only '&' in read access, if it is no pointer; 
            if typeStr[len(typeStr)-1] == '*':
                refChar = ''
    
            if (i > 1) & (parameter['lineType'] != 'Vp'): #must be comp, main or visu class; don't do it, if variable of parent class
                sList[i]+='    //! AUTO:  Write (Reference) access to:' + Str2Doxygen(parameter['parameterDescription']) + '\n'
                sList[i]+='    void Set' + functionStr + '(const ' + TypeConversion(parameter['type'], typeConversion)
                sList[i]+='& value) { ' + paramStr + ' = value; }\n'
#                sList[i]+='    //! # Write (Reference) access to:' + parameter['parameterDescription'] + '\n'
#                sList[i]+='    '+typeStr + '&' + ' '
#                sList[i]+='Get' + functionStr + '() { return ' + paramStr + '; }\n'
        
                sList[i]+='    //! AUTO:  Read (Reference) access to:' + Str2Doxygen(parameter['parameterDescription']) + '\n'
                sList[i]+='    const ' + typeStr + refChar + ' '
                sList[i]+='Get' + functionStr + '() const { return '+paramStr+'; }\n'
                if (i == 2) | (i == 4) : #in comp and visu class, also add the Get...() Reference access
                    sList[i]+='    //! AUTO:  Read (Reference) access to:' + Str2Doxygen(parameter['parameterDescription']) + '\n'
                    sList[i]+='    ' + typeStr + refChar + ' '
                    sList[i]+='Get' + functionStr + '() { return '+paramStr+'; }\n'
                sList[i]+='\n'

            destFolder = '' #destination folder string (e.g. GetParameters())
            if parameter['destination'].find('C') != -1: #computation
                destFolder+=compClassVariable + '->'

            vPrefix = '' #use 'V' prefix for visualization items
            if parameter['destination'].find('V') != -1: #visualization
                destFolder+=visuClassVariable + '->'
                vPrefix = 'V'
            pyName = vPrefix + parameter['pythonName']
                
            if parameter['destination'].find('P') != -1:
                destFolder+='GetParameters().'
            
            destStr = destFolder + parameter['cplusplusName']
            if i == 2: #comp class
                pstr = parameter['cplusplusName']
                destStr = destFolder + 'Get' + pstr[0].upper() + pstr[1:] + '()'
            
            
            if i == 4: #visualization class
                pstr = parameter['cplusplusName']
                destStr = destFolder + 'Get' + pstr[0].upper() + pstr[1:] + '()'
            
            typeCastStr = TypeConversion(parameter['type'], typeCasts)
            #dictionary access:
            if parameter['cFlags'].find('I') != -1: #'I' means add dictionary access
                parRead = '' #used for dictionary read and for parameter read
                parWrite = '' #used for dictionary write and for parameter write
                if parameter['type'] == 'BodyGraphicsData': #special conversion routine
                    dictListRead[i] +='        d["' + pyName + '"] = PyGetBodyGraphicsDataDictionary(' + destStr + '); //! AUTO: generate dictionary with special function\n'                    
                elif parameter['type'] == 'Matrix6D':
                    parRead = 'EXUmath::Matrix6DToStdArray66(' + destStr + ')'
                    #dictListRead[i] +='        d["' + pyName + '"] = EXUmath::Matrix6DToStdArray66(' + destStr + '); //! AUTO: generate dictionary with special function\n'                    
                elif parameter['type'] == 'Matrix3D':
                    parRead = 'EXUmath::Matrix3DToStdArray33(' + destStr + ')'
                elif parameter['type'] == 'NumpyMatrix':
                    parRead = 'EPyUtils::Matrix2NumPy(' + destStr + ')'
                elif parameter['type'] == 'NumpyMatrixI':
                    parRead = 'EPyUtils::MatrixI2NumPy(' + destStr + ')'
                elif parameter['type'] == 'NumpyVector':
                    parRead = 'EPyUtils::Vector2NumPy(' + destStr + ')'
                    #dictListRead[i] +='        d["' + pyName + '"] = EXUmath::Matrix3DToStdArray33(' + destStr + '); //! AUTO: generate dictionary with special function\n'                    
#                elif parameter['type'] == 'Matrix6D':
#                    dictListRead[i] +='        d["' + pyName + '"] = EXUmath::Matrix6DToStdArray66(' + destStr + '); //! AUTO: generate dictionary with special function\n'                    
#                elif parameter['type'] == 'Matrix3D':
#                    dictListRead[i] +='        d["' + pyName + '"] = EXUmath::Matrix3DToStdArray33(' + destStr + '); //! AUTO: generate dictionary with special function\n'                    
                else:
                    parRead = '(' + typeCastStr + ')' + destStr

                isPyFunction = False                
                if parameter['type'].find('PyFunction') != -1: #in case of function, special conversion and tests are necessary (function is either 0 or a python function)
                    isPyFunction = True
                    
                if len(parRead) != 0:
                    dictListRead[i] +='        '
                    if isPyFunction: 
                        dictListRead[i]+='if ('+destStr+')\n            {' #avoid that 'None' is returned in dict due to empty user function
                    dictListRead[i] +='d["' + pyName + '"] = ' + parRead + ';'
                    if isPyFunction: 
                        dictListRead[i]+='}\n        else\n'
                        dictListRead[i]+='            {d["' + pyName + '"] = 0;}\n'
                            
                    dictListRead[i] +=' //! AUTO: cast variables into python (not needed for standard types) \n'
                                                    
                #if (parameter['cFlags'].find('C') == -1) & (parameter['cFlags'].find('R') == -1): #'C' ... const access or 'R' means both read only!
                if (parameter['cFlags'].find('R') == -1): #'R' means read only!
                    dictListWrite[i]+='        '
                    if parameter['cFlags'].find('O') != -1: #optional ==> means that we have to check first, if it exists in the dictionary
                        dictListWrite[i]+='if (EPyUtils::DictItemExists(d, "' +  pyName + '")) { '
                    #if (parameter['type'] == 'String') | (parameter['type'] == 'Vector2D') | (parameter['type'] == 'Vector3D') | (parameter['type'] == 'Vector4D') | (parameter['type'] == 'Vector6D') | (parameter['type'] == 'Vector7D'):
                    if IsASetSafelyParameter(parameter['type']):
                        dictListWrite[i]+='EPyUtils::Set' + parameter['type'] + 'Safely(d, "' +  pyName + '", '
                        dictListWrite[i]+=destStr + '); /*! AUTO:  safely cast to C++ type*/'
                    elif parameter['type'] == 'BodyGraphicsData': #special conversion routine
                        dictListWrite[i]+='PyWriteBodyGraphicsData(d, "' +  pyName + '", ' + destStr + '); /*! AUTO: convert dict to BodyGraphicsData*/'
                    else:
                        dictStr = 'd["' + pyName + '"]'
                        if isPyFunction: #in case of function, special conversion and tests are necessary (function is either 0 or a python function)
                            dictListWrite[i]+='if (EPyUtils::CheckForValidFunction(d["'+pyName+'"])) { '
                            dictStr = '(py::function)'+dictStr

                        if typeCastStr != 'OutputVariableType':
                            dictListWrite[i]+=destStr + ' = py::cast<' + typeCastStr + '>'
                        else:
                            dictListWrite[i]+=destStr + ' = (OutputVariableType)py::cast<Index>'
                            
                        dictListWrite[i]+='(' + dictStr + '); /* AUTO:  read out dictionary and cast to C++ type*/'
                        if isPyFunction: 
                            dictListWrite[i]+='}'

                    #if (parameter['type'] == 'String') | (parameter['type'] == 'Vector2D') | (parameter['type'] == 'Vector3D') | (parameter['type'] == 'Vector4D') | (parameter['type'] == 'Vector6D') | (parameter['type'] == 'Vector7D'):
                    if IsASetSafelyParameter(parameter['type']):
                        parWrite+='EPyUtils::Set' + parameter['type'] + 'Safely(value, '
                        parWrite+=destStr + '); /*! AUTO:  safely cast to C++ type*/'
                    elif parameter['type'] == 'BodyGraphicsData': #special conversion routine
                        parWrite+='' #not implemented right now!
                    else:
                        parWrite+=destStr + ' = py::cast<' + typeCastStr + '>'
                        parWrite+='(value); /* AUTO:  read out dictionary and cast to C++ type*/'

                   
                    if parameter['cFlags'].find('O') != -1: #optional ==> means that we have to check first, if it exists in the dictionary
                        dictListWrite[i]+='} '
                    dictListWrite[i]+='\n'

                if parRead != '':
                    if parameter['type'].find('Numpy') != -1: #do not add py::cast(...) NumpyMatrix/Vector
                        parameterReadStr += 'if (parameterName.compare("' + pyName + '") == 0) { return ' + parRead + ';} //! AUTO: get parameter\n        else '
                    else:
                        parameterReadStr += 'if (parameterName.compare("' + pyName + '") == 0) { return py::cast(' + parRead + ');} //! AUTO: get parameter\n        else '
                
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
            if (parameter['lineType'].find('F') != -1):
                if (parameter['lineType'].find('v') != -1):
                    strVirtual = 'virtual '
                    if parameter['cFlags'].find('X') == -1:
                        strOverride = ' override'
                if (parameter['lineType'].find('s') != -1): #static
                    strVirtual = 'static ' + strVirtual
                
            typeStr = TypeConversion(parameter['type'], typeConversion)
            argsStr = parameter['args']
    
            sList[i]+='    //! AUTO:  ' + Str2Doxygen(parameter['parameterDescription']) + '\n'
            sList[i]+='    '+strVirtual + typeStr + ' '
            sList[i]+=functionStr + '(' + argsStr + ')' 
            if parameter['cFlags'].find('C') != -1:
                sList[i]+=' const'
                #print('Const found for ' + functionStr)
            sList[i]+=strOverride #all virtual functions should override a parent class function (otherwise add a new option)!
            
            if parameter['cFlags'].find('D') == -1:
                sList[i]+='\n    {\n        ' + parameter['defaultValue'] + '\n    }'
            else:
                sList[i]+=';'
                
            sList[i]+='\n\n'
    
    #add outputVariableType function automatically if defined:
    if len(parseInfo['outputVariables']) != 0:
        if (parseInfo['classType'] == 'Object') | (parseInfo['classType'] == 'Node'):
            sList[indexComp] += '    virtual OutputVariableType GetOutputVariableTypes() const override\n    {\n        return (OutputVariableType)('
            dictOV = eval(parseInfo['outputVariables'].replace('\\','\\\\').replace('\n','\\n')) #output variables are given as a string, representing a dictionary with OutputVariables and descriptions
            for outputVariables in dictOV.items(): 
                sList[indexComp] += '\n            (Index)OutputVariableType::' + outputVariables[0] + ' +'
            if len(dictOV.items()):
                sList[indexComp] = sList[indexComp][0:len(sList[indexComp])-1]
            sList[indexComp] += ');\n    }\n\n'
        else:
            print("ERROR: ",parseInfo['class'], ": output variables only possible for Objects and Nodes")
        
        
    
    typeStr = parseInfo['classType']
    c = typeStr[0] #take first character; !remember that the member variable must be lower-case
    typeStr = c.lower()+typeStr[1:]+'Type' #e.g. 'nodeType'

    #+++++++++++++++++++++++++++++++++++++++++++++++
    #now write dictionary read/write functions into main class:
    sList[3] += '\n    //! AUTO:  dictionary write access\n'
    sList[3] += '    virtual void SetWithDictionary(const py::dict& d) override\n'
    sList[3] += '    {\n'
    for i in range(nClasses):
        sList[3] += dictListWrite[i]
    
    if parseInfo['classType'] == 'Object': #if parameters have changed (e.g. with ModifyObject(..) ), some functions may be necessary to be reset
        sList[3] += '        GetCObject()->ParametersHaveChanged();\n'
        
    sList[3] += '    }\n\n'

    #+++++++++++++++++++++++++++++++++++++++++++++++
    sList[3] += '    //! AUTO:  dictionary read access\n'
    sList[3] += '    virtual py::dict GetDictionary() const override\n'
    sList[3] += '    {\n'
    sList[3] += '        auto d = py::dict();\n'
    sList[3] += '        d["' + typeStr + '"] = (std::string)GetTypeName();\n'
    for i in range(nClasses):
        sList[3] += dictListRead[i]
        
    sList[3] += '        return d; \n'
    sList[3] += '    }\n'

    #+++++++++++++++++++++++++++++++++++++++++++++++
    #now write parameter read/write functions into main class:
    sList[3] += '\n    //! AUTO:  parameter read access\n'
    sList[3] += '    virtual py::object GetParameter(const STDstring& parameterName) const override \n'
    sList[3] += '    {\n        '
    sList[3] += parameterReadStr
    sList[3] += ' {PyError(STDstring("' + classStr + '::GetParameter(...): illegal parameter name ")+parameterName+" cannot be read");} // AUTO: add warning for user\n'
    sList[3] += '        return py::object();\n'
#        if parseInfo['classType'] == 'Object': #if parameters have changed, some functions may be necessary to be reset
#            sList[3] += '        GetCObject()->ParametersHaveChanged();\n'
    sList[3] += '    }\n\n'

    sList[3] += '\n    //! AUTO:  parameter write access\n'
    sList[3] += '    virtual void SetParameter(const STDstring& parameterName, const py::object& value) override \n'
    sList[3] += '    {\n        '
    sList[3] += parameterWriteStr
    sList[3] += ' {PyError(STDstring("' + classStr + '::SetParameter(...): illegal parameter name ")+parameterName+" cannot be modified");} // AUTO: add warning for user\n'
    #notify object that parameters have changed
    if parseInfo['classType'] == 'Object': #if parameters have changed (e.g. with ModifyObject(..) ), some functions may be necessary to be reset
        sList[3] += '        GetCObject()->ParametersHaveChanged();\n'
    #sList[3] += '        \n'
#        if parseInfo['classType'] == 'Object': #if parameters have changed, some functions may be necessary to be reset
#            sList[3] += '        GetCObject()->ParametersHaveChanged();\n'
    sList[3] += '    }\n\n'


    #.def("__repr__", &Vector2::toString);

    #************************************
    # SIMPLE ostream operator:
#    s+=('  friend std::ostream& operator<<(std::ostream& os, const ' + 
#       parseInfo['class'] + '& object);\n')

    # FULL ostream operator - THIS IS DONE LATERON:
#    s+='  virtual void Print (std::ostream& os) const\n'
#    s+='  {\n'
#    s+='    os << "' + parseInfo['class'] + '";\n'
#    if len(parseInfo['parentClass']) != 0:
#        s+='    os << ":"; \n'
#        s+='    ' + parseInfo['parentClass'] + '::Print(os);\n'
#        
#    #output each parameter
#    for parameter in parameterList:
#        if (parameter['lineType'] == 'V'): #only if it is a member variable
#            paramStr = parameter['cplusplusName']
#            typeStr = TypeConversion(parameter['type'], typeConversion)
#            refChar = ''
#            if typeStr[len(typeStr)-1] == '*':
#                refChar = '*' #print content of object, not the pointer address
#            s+='    os << "  ' + paramStr + ' = " << ' + refChar + paramStr + ' << "\\n";\n'
#         
#    s+='    os << "\\n";\n'
#    s+='  }\n\n' # end ostream operator
#
#    if len(parseInfo['parentClass']) == 0:
#        s+=('  friend std::ostream& operator<<(std::ostream& os, const ' + parseInfo['class'] + '& object)\n')
#        s+= '  {\n'
#        s+= '    object.Print(os);\n'
#        s+= '    return os;\n'
#        s+= '  }\n\n' # end ostream operator
#    
    for i in range(nClasses):
        sList[i]+='};\n\n\n' #class

    sList[2] += '\n#endif //#ifdef include once...\n'
    sList[3] += '\n#endif //#ifdef include once...\n'
    sList[4] += '\n#endif //#ifdef include once...\n'

    s = [sList[0]+sList[2], sList[1]+sList[3], sList[4], sLatex, sLatexItemList, classTypeStr, sPythonClass]

    return s

#**************************************************************************************
#**************************************************************************************
#**************************************************************************************
#create string containing the pybind11 headers/modules for a class
def CreatePybindHeaders(parseInfo, parameterList, typeConversion):
    #print ('Create Pybind11 includes')

    spaces1 = '    '            #first level
    spaces2 = spaces1+'    '    #second level

    s = spaces1 + '//++++++++++++++++++++++++++++++++\n' #create empty string
    #************************************
    #class definition:
    s += spaces1 + 'py::class_<' + parseInfo['class'] + '>(m, "' + parseInfo['class'] + '") // #\n'
    s += spaces2 + '.def(py::init<>())\n'
	
    #************************************
    #member variables access:
    for parameter in parameterList:
        if ((parameter['lineType'] == 'V') | (parameter['lineType'] == 'V')) & (parameter['cFlags'].find('P') != -1): #only if it is a member variable
            s += spaces2 + '.def_readwrite("' + parameter['pythonName'] + '", &' + parseInfo['class'] + '::' + parameter['pythonName'] + ')\n' #extend this to incorporate 'read only' and other flags

    #s += '\n'
    s += spaces2 + '// # access functions for ' + parseInfo['class'] + '\n'
            
    for parameter in parameterList:
        if (parameter['lineType'].find('F') != -1) & (parameter['cFlags'].find('P') != -1): #only if it is a member variable
            s += spaces2 + '.def("' + parameter['pythonName']
            s += '", &' + parseInfo['class'] + '::' + parameter['pythonName']
            s += ', py::return_value_policy::reference)\n' #extend this to incorporate 'read only' and other flags
    
    s += spaces2 + '; // # end of class definition!!!\n'
    s += '\n'

    s += spaces1 + '//++++++++++++++++++++++++++++++++\n' #end of pybind11 definition


    return s

#cut the first 'numberOfCutLines' lines in a string (in order to ignore the header date in comparison of files)
def CutString(theString, numberOfCutLines):
    pos = 0
    for i in range(numberOfCutLines):
        pos = theString.find('\n', pos) + 1

    return theString[pos:]

#************************************************
#MAIN CONVERSION
#************************************************
    

try: #still close file if crashes
    #create Python/pybind11 file; currently not used ...
    #    pybindFile = 'pybind_objects.h'
    #    file=open(pybindFile,'w')  #clear file by one write access
    #    file.write('// # ++++++++++++++++++++++\n')
    #    file.write('// # pybind11 OBJECT includes; generated by Johannes Gerstmayr\n')
    #    file.write('// # ++++++++++++++++++++++\n')
    #    file.close()

    directoryString = '..\\Autogenerated\\'
    
    #read system definition
    filename = "objectDefinition.py"
    totalNumberOfLines = 0        #count number of lines generated automatically ...
    totalNumberOfFilesChanged = 0 #count how many files have been changed

    file=open(filename,'r') 
    
    fileLines = file.readlines()

    #direct type conversion used for in C++ (type casts are in WriteFile(...) function); 
    #types such as UReal shall be used lateron to perform e.g. range checks prior to setting parameters
    typeConversion = {'Bool':'bool', 'Int':'int', 'Real':'Real', 'UInt':'Index', 'UReal':'Real', 
                      'Vector':'Vector', 'Matrix':'Matrix', 'SymmetricMatrix':'Vector', 
                      'NumpyVector':'Vector', 
                      'NumpyMatrix':'Matrix', 
                      'NumpyMatrixI':'MatrixI', 
                      'String':'std::string'} #convert parameter types to C++/EXUDYN types
    typeConversion.update(pyFunctionTypeConversion)#add the list of python (user) functions
    
    #the following commands are recognized:
    parseInfo = {'class':'',            # C++ class name
                 'writeFile':'',        #True; initiates finalization of class definition and file writing
                 #'writePybindIncludes':'',#True, if pybind11 includes shall be written for this class
                 'cParentClass':'',     #name of parent computational object class or empty
                 'cBaseClass':'',       #name of computational object base class or empty
                 'mainParentClass':'',  #name of parent MainObject class or empty
                 'visuParentClass':'',  #name of parent VisualizationItem class or empty
                 'pythonShortName':'',  #short name for python interface
                 'addProtectedC':'',    #code added at protected section (e.g. constants)
                 'addPublicC':'',       #code added at protected section (e.g. constants or functions)
                 'addProtectedMain':'', #code added at protected section (e.g. constants)
                 'addPublicMain':'',    #code added at protected section (e.g. constants or functions)
                 'addIncludesC':'',     #code added at includes section (e.g. special base class)
                 'addIncludesMain':'',     #code added at includes section (e.g. special base class)
                 'classType':'',        #type of class: Object, Node, Sensor, Marker, Load, Sensor
                 'outputVariables':'',  #definition of output variables and description given as dictionary "{'OutputVariableType':'description ...', ...}"
                 'miniExample':'',      #mini python example (without headers and typical setup); code in separate lines, ended with '/end' in separate line
                 'equations':'',        #latex style equations, direct latex code; latex code in separate lines, ended with '/end' in separate line
                 'classDescription':''} #add a (brief, one line) description of class
    #this defines the columns of the line, which is then filled into this structure
    lineDefinition = ['lineType',       #[V|F[v]]P: V...Value (=member variable), F...Function (access via member function); v ... virtual Function; P ... write Pybind11 interface
                      'destination',    #M ... Main object, C ... computational object, V ... visualization object; P ... parameter structure
                      'pythonName',     #name which is used in python
                      'cplusplusName',     #name which is used in DYNALFEX (leave empty if it is the same)
                      'size',           #for size check; leave empty if size is non-constant; e.g. 3 (size of vector), 2x3 (2 rows, 3 columns)  %used for variables and vectors and matrices only!
                      'type',           #variable or return type: Bool, Int, Real, UInt, UReal, Vector, Matrix, SymmetricMatrix
                      'defaultValue',   #default value for member variable or function definition
                      'args',           # arguments in function declaration (empty for variable)
                      'cFlags',         # various flags: R(read only), M(modifiableDuringSimulation), N(parameter change needs object reset), C(onst member function),  D(declaration only; implementation in .cpp file done manually), O ... optional parameter in dictionary (otherwise using default value)
                                        #     P ... write Pybind11 interface, [default is read/write access and that changes are immediately applied and need no reset of the system]
                      'parameterDescription'] #description for parameter used in C++ code
    nparam = len(lineDefinition)
    
    
    mode = 0 #1...read parameterlist , 0...read definitions
    linecnt = 1
    
    parameterList = [] #list of dictionaries for parameters
    continueOperation = True #flag to signal that operation shall be terminated
    
    sLatexGlobal = ['']*5     #gobal Latex string; 'Node','Object','Marker','Load','Sensor'
    sLatexItemList = '' #Latex string containing list of items
    sPythonGlobal = ['']*5  #global python interface class strings; 'Node','Object','Marker','Load','Sensor'
    sPythonGlobalNames = ['Node','Object','Marker','Load','Sensor']  #global python interface class types
    
    miniExamplesList = []    #generate file list for mini examples
    multiLineReading = False #for equations and miniExample
    multiLineString = ''     #stored string from multiline reading
    multiLineType = ''       #equations or miniExample
    cnt = 0
    
    for line in fileLines:
        if continueOperation:
            if multiLineReading:
                pureline = (line.strip('\n')) #eliminate EOL
                if pureline != '/end':
                    multiLineString += line
                else: #store string and finish reading
                    parseInfo[multiLineType] = multiLineString
                    #print('multiline=\n'+multiLineString)
                    multiLineReading = False #end 
                    multiLineString = ''     #stored string from multiline reading
                    multiLineType = ''       #equations or miniExample
                linecnt+=1
                continue
            

            if line[0] != '#':
                pureline = (line.strip('\n')) #eliminate EOL
                pureline = pureline.replace('\\n','\n') #put correct line brake symbols
                if len(pureline.replace(' ','')): #empty lines are ignored
                    if (pureline[0] == 'V') | (pureline[0] == 'F'):
                        #must be definition of parameters
                        info = SplitString(pureline, linecnt)
                        #print(info)
                        if len(info) != len(lineDefinition):
                            continueOperation = False
                        d={} #empty dictionary
                        
                        cnt = 0
                        for item in lineDefinition:
                            if lineDefinition[cnt] != 'parameterDescription':
                                info[cnt] = info[cnt].replace("'",'"')
#                            else:
#                                if info[cnt].find("'"):
#                                    print(info[cnt])

                            d[lineDefinition[cnt]] = info[cnt]
                            cnt+=1
                        if len(d['cplusplusName']) == 0:
                            d['cplusplusName'] = d['pythonName']
                        #print(d)
                        parameterList.append(d) #append parameter dictionary to list
                    elif (pureline.find('=') != -1): #definition
    #                    pureline = pureline.replace(' ','') #eliminate spaces and EOL
                        #info = pureline.split('=') splits into several strings if = occurs in definition
                        eqN = pureline.find('=') #column of FIRST equation character
                        info = [pureline[0:eqN], pureline[eqN+1:]]
                        
                        #print("info =", info)
                        defName = info[0].replace(' ','')
                        #print("defname =",defName)
                        if (defName == 'equations') or (defName == 'miniExample'):
                            multiLineReading = True
                            multiLineType = defName
                            linecnt+=1
                            continue #read next line

                        
                        RHS = RemoveSpacesTabs(info[1])
                        RHS = RHS.strip('"')
                        RHS = RHS.strip("'") #for includes using "..." for include filename
                        if ((defName != 'classDescription') and (defName != 'addProtectedC') and (defName != 'addProtectedMain') and 
                            (defName != 'addPublicC') and (defName != 'addPublicMain') and (defName != 'addIncludesC') and
                            (defName != 'addIncludesMain') and (defName != 'outputVariables')):
                            RHS = RHS.replace(' ','')
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
                                fileStr = WriteFile(parseInfo, parameterList, typeConversion)
                                sLatexItemList += fileStr[4]
                                
                                #find index of python objects
                                typeInd = -1
                                it = 0
                                for item in sPythonGlobalNames: 
                                    if item == fileStr[5]: 
                                        typeInd = it
                                    it+=1

                                if typeInd == -1:
                                    print("ERROR: no valid base name found")
                                else:
                                    sPythonGlobal[typeInd] += fileStr[6]
                                    if len(sLatexGlobal[typeInd]) != 0:
                                        sLatexGlobal[typeInd] += '\\newpage\n' #add newpage after every subsection!
                                        
                                    sLatexGlobal[typeInd] += fileStr[3]

                                #+++++++++++++++++++++++++++++++
                                #write files if changes apply:
                                nLinesHeader = 7 # number of header lines which are ignored in file comparison
                                strFileMode = 'w'
                                fileName = directoryString + 'C'+parseInfo['class']+'.h'
                                fileText = 'INVALID'
                                if os.path.isfile(fileName):
                                    file=open(fileName,'r'); fileText = file.read();file.close()
                                    
                                if (CutString(fileText,nLinesHeader) != CutString(fileStr[0],nLinesHeader)):
                                    #write computational 'C' class
                                    file=open(fileName,strFileMode) 
                                    file.write(fileStr[0])
                                    file.close()
                                    totalNumberOfFilesChanged += 1

                                fileName = directoryString + 'Main'+parseInfo['class']+'.h'
                                fileText = 'INVALID'
                                if os.path.isfile(fileName):
                                    file=open(fileName,'r'); fileText = file.read();file.close()
                                if (CutString(fileText,nLinesHeader) != CutString(fileStr[1],nLinesHeader)):
                                    #write Main class
                                    file=open(fileName,strFileMode) 
                                    file.write(fileStr[1])
                                    file.close()
                                    totalNumberOfFilesChanged += 1

                                fileName = directoryString + 'Visu'+parseInfo['class']+'.h'
                                fileText = 'INVALID'
                                if os.path.isfile(fileName):
                                    file=open(fileName,'r'); fileText = file.read();file.close()
                                if (CutString(fileText,nLinesHeader) != CutString(fileStr[2],nLinesHeader)):
                                    #write Visualization class
                                    file=open(fileName,strFileMode) 
                                    file.write(fileStr[2])
                                    file.close()
                                    totalNumberOfFilesChanged += 1

                                if len(parseInfo['miniExample']) != 0:
                                    miniExamplesList += [parseInfo['class']+'.py']

                                #++++++++++++++++++++++++++++++
                                #write Python/pybind11 includes
# not needed now:
#                                    if parseInfo['writePybindIncludes'] == 'True':
#                                        pybindStr = CreatePybindHeaders(parseInfo, parameterList, typeConversion)
#                                        file=open(pybindFile,'a')  #always append to pybind file
#                                        file.write(pybindStr)
#                                        file.close()
                                
                                #++++++++++++++++++++++++++++++
                                #reset structures for next file
                                totalNumberOfLines += CountLines(fileStr[0]) + CountLines(fileStr[1]) + CountLines(fileStr[2])
                                parameterList = [] #reset list
                                parseInfo['writeFile'] = ''
                                parseInfo['class'] = ''
                                parseInfo['cParentClass'] = ''
                                parseInfo['cBaseClass'] = ''
                                parseInfo['mainParentClass'] = ''
                                parseInfo['visuParentClass'] = ''
                                parseInfo['pythonShortName'] = ''
                                parseInfo['addProtectedC'] = ''
                                parseInfo['addPublicC'] = ''
                                parseInfo['addProtectedMain'] = ''
                                parseInfo['addPublicMain'] = ''
                                parseInfo['addIncludesC'] = ''
                                parseInfo['addIncludesMain'] = ''
                                parseInfo['classType'] = ''
                                parseInfo['outputVariables'] = ''
                                parseInfo['classDescription'] = ''
                                parseInfo['equations'] = ''
                                parseInfo['miniExample'] = ''
                                #parseInfo['writePybindIncludes'] = 'False'
                            else:
                                print("ERROR: did not expect 'writeFile' keyword in line",linecnt)    
                                continueOperation = False
                            
#                       else:
#                           print("ERROR: definition mismatch in line",linecnt)    
#                           continueOperation = False
                    else: #ERROR
                        print("ERROR: unknown format of line",linecnt)    
            linecnt += 1;
    
    if (continueOperation == False):
        print('\n\nERROR: Parsing terminated unexpectedly in line',linecnt,'\n\n')
        
    print("parsed a total of", linecnt, "lines")

    print('total number of lines generated =',totalNumberOfLines)
    print('total number of files changed =',totalNumberOfFilesChanged)

#    sLatexItemList = '\n\\mysubsection{List of Items}\nThe following items are available in \codeName:\n\\begin{itemize}\n' + sLatexItemList
#    sLatexItemList += '\\end{itemize}\n'

    fileLatex=open('..\\..\\..\\docs\\theDoc\\itemDefinition.tex','w') 
#    fileLatex.write(sLatexItemList)
    it = 0
    for item in sPythonGlobalNames: 
        fileLatex.write('\n\\newpage\n%+++++++++++++++++++++++++++++++\n%+++++++++++++++++++++++++++++++\n\\mysubsection{'+item+'s}\n')
        fileLatex.write(sLatexGlobal[it])
        it+=1
    
    fileLatex.close()

    filePython=open('..\\..\\pythonDev\\itemInterface.py','w') 
    s = '#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n'
    s += '#automatically generated file for conversion of item (node, object, marker, ...) data to dictionaries\n'
    s += '#author: Johannes Gerstmayr\n'
    s += '#created: 2019-07-01\n'
    s += '#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n'
    #s += 'from exudyn import OutputVariableType\n\n' #do not import exudyn, causes problems e.g. with exudynFast, ...
    s += '#item interface diagonal matrix creator\n'
    s += 'def IIDiagMatrix(rowsColumns, value):\n'
    s += '    m = []\n'
    s += '    for i in range(rowsColumns):\n'
    s += '        m += [rowsColumns*[0]]\n'
    s += '        m[i][i] = value\n'
    s += '    return m\n\n'
    filePython.write(s)
    
    
    it = 0
    for item in sPythonGlobalNames: 
        filePython.write('#+++++++++++++++++++++++++++++++\n#'+item.upper()+'\n')
        filePython.write(sPythonGlobal[it])
        it+=1
    
    filePython.close()

    fileExampleList=open('../../pythonDev/MiniExamples/miniExamplesFileList.py','w') 
    s = '#this file provides a list of file names for mini examples\n'
    s+= '\n'
    s+= 'miniExamplesFileList = ['
    sepStr = ''
    for item in miniExamplesList:
        s+= sepStr + "'" + item + "'"
        sepStr=',\n'
    s+= ']\n'
    s+= '\n'
    fileExampleList.write(s)
    fileExampleList.close()


finally:    
    file.close()



