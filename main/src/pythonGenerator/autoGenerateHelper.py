# -*- coding: utf-8 -*-
"""
Created on Fri May 18 08:53:30 2018

@author: Johannes Gerstmayr

goal: automatically generate interfaces for structures
currently: automatic generate structures with ostream and initialization
"""
import datetime # for current date


#******************************************************************************************************
def GetDateStr():
    now=datetime.datetime.now()
    monthZero = '' #add leading zero for month
    dayZero = ''   #add leading zero for day
    if now.month < 10:
        monthZero = '0'
    if now.day < 10:
        dayZero = '0'
        
    dateStr = str(now.year) + '-' + monthZero + str(now.month) + '-' + dayZero + str(now.day)

    return dateStr


#************************************************
#convert string to doxygen readable comment --> for formulas in comments and class descriptions
def Str2Doxygen(s, isDefaultValue=False): #replace _ and other symbols to fit into latex code

    s = s.replace('$','\\f$') #$ must be written as \f$ in doxygen
    s = s.replace('\\be','\\f[') #$ must be written as \f$ in doxygen
    s = s.replace('\\ee','\\f]') #$ must be written as \f$ in doxygen

    return s

#************************************************
#convert string to latex readable string --> used in auto-generated docu
def Str2Latex(s, isDefaultValue=False): #replace _ and other symbols to fit into latex code

    if isDefaultValue:
        s = s.replace('true','True') #correct python notation
        s = s.replace('false','False') #correct python notation

        s = s.replace('EXUstd::InvalidIndex','MAXINT') #correct python notation

        
        
        if (s.find('Index') != -1) | (s.find('Float') != -1) | (s.find('Vector') != -1) | (s.find('Matrix') != -1):
            s = s.replace('ArrayIndex','') #correct python notation
            s = s.replace('Vector7D','') #correct python notation; rigid body coordinates
            s = s.replace('Vector6D','') #correct python notation; inertia parameters
            s = s.replace('Vector4D','') #correct python notation
            s = s.replace('Vector3D','') #correct python notation
            s = s.replace('Vector2D','') #correct python notation
            s = s.replace('Vector','') #correct python notation
            s = s.replace('false','False') #correct python notation
            s = s.replace('Index2','')
            s = s.replace('Index3','')
            s = s.replace('Float3','')
            s = s.replace('Float4','')
            s = s.replace('Float9','')
            s = s.replace('Float16','')
            s = s.replace('EXUmath::Matrix3DFToStdArray33','')
            s = s.replace('(','[')
            s = s.replace(')',']')
            #s = s.replace('.f','')
            s = s.replace('{','')
            s = s.replace('}','')
        
        if s.find("'") == -1: #don't do that for strings!
            s = s.replace('f','')

    #s = s.replace('\\','\\\\') #leads to double \\ in latex
    s = s.replace('_','\_')
    s = s.replace('{','\{')
    s = s.replace('}','\}')
    #s = s.replace('$','\$') #do not exclude $ in order to allow latex formulas

    return s

def DefaultValue2Python(s): #replace _ and other symbols to fit into latex code

    s = s.replace('true','True') #correct python notation
    s = s.replace('false','False') #correct python notation

    s = s.replace('EXUstd::InvalidIndex','-1') #as we do not know the value, set it to -1; user needs to overwrite!
    s = s.replace('OutputVariableType::None','0')  #none always corresponds to 0
    s = s.replace('EXUmath::unitMatrix3D','IIDiagMatrix(rowsColumns=3,value=1)')  #replace with itemInterface diagonal matrix

    
    if (s.find('Matrix6D(6,6,') != -1):
        s = s.replace('Matrix6D(6,6,','')
        s = s.replace(')','')
        if s != '0' and s != '0.': print('error: Matrix6D(...) may only initialized with 0s')
        s = 'IIDiagMatrix(rowsColumns=6,value=' + s + ')'
        #
    elif (s.find('Matrix3D(3,3,') != -1):
        s = s.replace('Matrix3D(3,3,','')
        s = s.replace(')','')
        if s != '0' and s != '0.': print('error: Matrix3D(...) may only initialized with 0s')
        s = 'IIDiagMatrix(rowsColumns=3,value=' + s + ')'
        #
    elif (s.find('Index') != -1) | (s.find('Float') != -1) | (s.find('Vector') != -1):
        s = s.replace('ArrayIndex','') #correct python notation
        s = s.replace('Vector7D','') #correct python notation; rigid body coordinates
        s = s.replace('Vector6D','') #correct python notation; inertia parameters
        s = s.replace('Vector4D','') #correct python notation
        s = s.replace('Vector3D','') #correct python notation
        s = s.replace('Vector2D','') #correct python notation
        s = s.replace('Vector','') #Vector(...)-->correct python notation [...]
        s = s.replace('false','False') #correct python notation
        s = s.replace('Index2','')
        s = s.replace('Index3','')
        s = s.replace('Float3','')
        s = s.replace('Float4','')
        s = s.replace('Float9','')
        s = s.replace('Float16','')
        s = s.replace('EXUmath::Matrix3DFToStdArray33','')
        s = s.replace('(','[')
        s = s.replace(')',']')
        s = s.replace('f','')
        s = s.replace('{','')
        s = s.replace('}','')
    
    s = s.replace('f','')

    #not necessary in python:
    #s = s.replace('\\','\\\\')
    #s = s.replace('_','\_') 
    #s = s.replace('{','\{')
    #s = s.replace('}','\}')
    #s = s.replace('$','\$')

    return s


#************************************************
# helper function for reading the structure
def RemoveSpacesTabs(s):
    s = s.replace('\t','')
    s = s.strip(' ') #to not erase interior space (e.g. initialization of vectors!) replace(' ','')

    return s


#************************************************
#count lines to see if changes effect the number of written lines
def CountLines(s):
    location = -1
    strLen = len(s)
    counter = 1 #first line does not have a linebreak!

    while location < strLen:
        location = s.find('\n', location + 1)
        if location == -1: 
            location = strLen
        else:
            counter += 1
    return counter

#************************************************
#convert type to known C++ type or keep it (in case of special class)
def TypeConversion(typeStr, typeConversion):
    newStr = typeStr
    if (typeStr in typeConversion):
        newStr = typeConversion[typeStr]
        
    #print('convert "'+typeStr+'" into "'+newStr+'"')

    return newStr

#************************************************
#convert type to known C++ type or keep it (in case of special class)
def GenerateHeader(classStr, descriptionStr, addModifiedDate = True):

    now=datetime.datetime.now()
    monthZero = '' #add leading zero for month
    dayZero = ''   #add leading zero for day
    hourZero= ''
    minuteZero= ''
    secondZero= ''
    
    if now.month < 10:
        monthZero = '0'
    if now.day < 10:
        dayZero = '0'
    if now.hour < 10:
        hourZero = '0'
    if now.minute < 10:
        minuteZero = '0'
    if now.second < 10:
        secondZero = '0'
        
    dateStr = str(now.year) + '-' + monthZero + str(now.month) + '-' + dayZero + str(now.day)
    timeStr = hourZero+str(now.hour) + ':' + minuteZero+str(now.minute) + ':' + secondZero+str(now.second)
    
    #************************************
    #header
    s='' #generate a string
    #s+='//automatically generated file (pythonAutoGenerateInterfaces.py)\n'
    s+='/** ***********************************************************************************************\n'
    s+='* @class        '+classStr+'\n'
    s+='* @brief        '+descriptionStr+'\n'
    s+='*\n'
    s+='* @author       Gerstmayr Johannes\n'
    s+='* @date         2019-07-01 (generated)\n'
    if addModifiedDate:
        s+='* @date         '+ dateStr+ '  ' + timeStr + ' (last modfied)\n' #this causes all files to change ...
    #s+='* @date         2019-09-12 (last modfied)\n'
    s+='*\n'
    s+='* @copyright    This file is part of Exudyn. Exudyn is free software: you can redistribute it and/or modify it under the terms of the Exudyn license. See "LICENSE.txt" for more details.\n'
    s+='* @note         Bug reports, support and further information:\n'
    s+='                - email: johannes.gerstmayr@uibk.ac.at\n'
    s+='                - weblink: missing\n'
    s+='                \n'
    s+='************************************************************************************************ */\n'
    s+='#pragma once\n'
    s+='\n'
    s+='#include <ostream>\n'
    s+='\n'
    s+='#include "Utilities/ReleaseAssert.h"\n'
    s+='#include "Utilities/BasicDefinitions.h"\n'
    s+='\n'

    return s


#************************************************
#do main part of parameter line parsing
#standard string.split() does not work, because of possible commas in description
def SplitString(string, line): #split comma separated string; commas in "..." are not counted; remove '"' and spaces outside ""

    continueOperation = True #check if parsing shall be terminated
    c = '';
    list=[]
    stringMode = 0 #0=normal mode, 1=string mode ("")
    s=''
#    for i in range[0,len(string)]:
    for c in string:
#        print('c="',c,'"')
        if continueOperation:
            if (c==',') & (stringMode != 1):
                if (stringMode != 2):
                    s = RemoveSpacesTabs(s) #to not erase interior space (e.g. initialization of vectors!) replace(' ','')
                list.append(s)
                s = ''
                stringMode = 0
            elif (c=='"'):
                if (stringMode == 0):
                    if len(s.replace(' ','').replace('\t','')) != 0:
                        print('ERROR in line',line,': invalid characters before ":',s)
                        continueOperation = False
                        list = []
                    stringMode = 1
                    s = '' #start with new string
                elif (stringMode == 1):
                    stringMode = 2 # expect comma or spaces (ignored)
            elif (stringMode != 2):
                s += c


    if (stringMode != 2):
        s = RemoveSpacesTabs(s) #to not erase interior space (e.g. initialization of vectors!) replace(' ','')
    list.append(s) #append last string; 3 commas = 4 strings   
    return list
#************************************************




#************************************************
#helper functions to create manual pybinding to access functions in classes
#pyName = python name, cName=full path of function in C++, description= textual description used in C and in documentation
#argList = [arg1Name, arg2Name, ...]
#defaultArgs = [arg1Default, ...]: either empty or must have same length as argList
#options= additional manual options (e.g. memory model)
#example = string, which is put into latex documentation
#isLambdaFunction = True: cName is intepreted as lambda function and copied into pybind definition
def DefPyFunctionAccess(cClass, pyName, cName, description, argList=[], defaultArgs=[], example='', options='', isLambdaFunction = False): 
    #make some checks:
    if (len(argList) != 0) & (len(defaultArgs) == 0):
        defaultArgs = ['']*len(argList)
    elif len(argList) != len(defaultArgs):
        print('error in command '+pyName+': defaultArgs are inconsistent')
        return ''
    
    s = ''
    if (cClass != ''):
        s += '        .def("'
    else:
        s += '        m.def("'

    s += pyName + '", ' 
    if not(isLambdaFunction): #if lambda function ==> just copy cName as code
        s += '&' 
        if (cClass != ''):
            s += cClass + '::'

    s += cName + ', '
    s += '"' + description +'"'
    if (options != ''):
        s += ', ' + options
    
    
    sLatex = '  ' + Str2Latex(pyName) + '('
    if len(argList):
        for i in range(len(argList)):
            s += ', py::arg("' + argList[i] + '")'
            sLatex += argList[i]
            if (defaultArgs[i] != ''):
                sLatex += ' = ' + defaultArgs[i].replace('py::','').replace('::','.') #replace C-style '::' (e.g. in ConfiguationType) to python-style '.'
                s += ' = ' + defaultArgs[i].replace('exu.','') #remove exudyn 'exu.' for C-code
            sLatex += ', '
        sLatex = sLatex[:-2] #remove last ', '

    sLatex += ')'

    s += ')'
            
    if (cClass == ''):
        s += ';'
    
    s += '\n'


#    sLatex += ' & '
#    if len(defaultArgs):
#        sLatex += '('
#        for argStr in defaultArgs:
#            sLatex += argStr + ' ,'
#        sLatex = sLatex[:-2]+')'
    
    sLatex += ' & ' + description.replace('_','\_')
    if example != '':
        example = Str2Latex(example)
        example = example.replace('\\\\','\\tabnewline\n    ')
        sLatex += '\\tabnewline \n    \\textcolor{steelblue}{{\\bf EXAMPLE}: \\tabnewline \n    ' + example + '}'
    sLatex += '\\\\ \\hline \n'
    
    return [s,sLatex]

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the following functions are used to generate pybinds to classes and add according latex documentation

#start a new table to describe class bindings in latex;
def DefLatexStartClass(sectionName, description, subSection=False):

    sLatex =  "\n%++++++++++++++++++++\n"
    if subSection:
        sLatex += "\\mysubsubsection"
    else:
        sLatex += "\\mysubsection"

    sLatex += "{" + sectionName + "}\n"
    sLatex += description + '\n\n'
    
    sLatex += '\\begin{center}\n'
    sLatex += '\\footnotesize\n'
    sLatex += '\\begin{longtable}{| p{8cm} | p{8cm} |} \n'
    sLatex += '\\hline\n'
    sLatex += '{\\bf function/structure name} & {\\bf description}\\\\ \\hline\n'

    return sLatex

def DefPyStartClass(cClass, pyClass, description):
    s = '\n'
    sectionName = pyClass
    if (cClass == ''): 
        #print("ERROR::DefPyStartClass: cClass must be a string")
        sectionName = '\\codeName' #for EXUDYN, work around
        
    if (cClass != ''):
        s += '    py::class_<' + cClass + '>(m, "' + pyClass + '")\n'
        s += '        .def(py::init<>())\n'

    sLatex = DefLatexStartClass(sectionName, description)        
        
    return [s,sLatex]

#finish latex table for class bindings 
def DefLatexFinishClass():
    sLatex = '\\end{longtable}\n'
    sLatex += '\\end{center}\n'
    
    return sLatex

def DefPyFinishClass(cClass):
    s = ''
    
    if (cClass != ''):
        s += '        ; // end of ' + cClass + ' pybind definitions\n\n'

    sLatex = DefLatexFinishClass()

    return [s,sLatex]


#add a enum value and definition to pybind interface and to latex documentation
def AddEnumValue(className, itemName, description):
    s = '		.value("' + itemName + '", ' + className + '::' + itemName + ')    //' + description + '\n'
    #s = '		.value("' + itemName + '", ' + className + '::' + itemName + ', "' + description + '")\n' #does not work in pybind11
    sLatex = '  ' + Str2Latex(itemName) + ' & ' + Str2Latex(description) + '\\\\ \\hline \n'
        
    return [s,sLatex]












