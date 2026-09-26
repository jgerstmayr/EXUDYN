#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  What a settings structure looks like as Python: which values it has, what each of
#           them is called, what a value looks like in a script, and which of them differ from
#           the defaults. It works on the dictionary GetDictionaryWithTypeInfo() returns, so
#           every function here runs without a window and can be tested without one.
#
#           It was the lower half of exudyn.misc.GUI (#2590),
#           where the settings dialog is its only caller. GUI.py imports tkinter at module
#           scope, so a model script could not use any of it without tkinter installed - which
#           is exactly what a model on a cluster does not have. The dialog imports from here
#           now, and so can a script:
#
#               from exudyn.misc.settingsUtilities import PrintChangedSettings
#               PrintChangedSettings(SC.visualizationSettings)
#
# Author:   Johannes Gerstmayr
# Date:     2020-01-25 (as part of GUI.py), 2026-09-24 (own module)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import ast  #for ast.literal_eval
import copy  #the snapshotted defaults are handed out as a copy
import numpy as np  #for array checks
from numpy import float32
import exudyn


#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'IsFloat', 'IsArrayInt', 'IsVector', 'GetComboBoxListsDict', 'ConvertString2Value',
    'ConvertValue2String', 'CheckType', 'SettingsLeafList', 'EnumDisplayName',
    'EnumFullName', 'ValueLiteral', 'SettingsCodeLines',
    'containerInitialisedSettings', 'CompiledSettingsClass', 'DefaultSettingsDictionary',
    'SettingsValueStrings',
    'FindMatches', 'SettingsPrefix', 'ChangedSettings', 'ChangedSettingsCode',
    'PrintChangedSettings',
    ]

#check if is float:
def IsFloat(v):
    try:
        float(v)
    except ValueError:
        return False
    return True

#check if converts to numpy array
def IsArrayInt(v):
    try:
        np.fromstring(v,dtype=int,sep=',') #frombuffer does not work!
    except ValueError:
        return False
    return True
    
def IsVector(v):
    try:
        np.fromstring(v,dtype=float,sep=',') #frombuffer does not work!
    except ValueError:
        return False
    return True


#create dictionaries for lists in combo box: bool, OutputVariableType, ...
def GetComboBoxListsDict(exu = None):
    """The values a settings item of an enum type may take, as {typeName: [values]}.

    EVERY enum of the module, not a hand-written list of three:
    this named OutputVariableType, LinearSolverType and ItemType, and
    timeIntegration.explicitIntegration.dynamicSolverType - a DynamicSolverType - was therefore
    edited as free text, where a typo is a silent wrong value (#2597). A pybind11 enum is
    recognised by its __members__, so an enum added to the module arrives here by itself.

    Args:
        exu: the exudyn module

    Returns:
        the dictionary the dialog picks its combo box entries from
    """
    dT=dict() #as type

    if exu is not None: #exudyn loaded
        for name in dir(exu):
            if name.startswith('_'):
                continue
            candidate = getattr(exu, name, None)
            members = getattr(candidate, '__members__', None)
            if isinstance(members, dict) and len(members) != 0:
                dT[name] = [members[key] for key in members]
    else:
        exudyn.Print('WARNING: GetComboBoxListsDict: exudyn not loaded as "exu"')

    #d['bool'] = ['True','False']
    dT['bool'] = [True, False]
    return dT
    
#convert string into exudyn type
def ConvertString2Value(value, vType, vSize, dictionaryTypesT):
    errorMsg = ''
    if vType == 'FileName' or vType == 'String':
        return [value, errorMsg]

    if vType == 'bool':
        if value == 'True':
            return [True, errorMsg]
        else:
            return [False, errorMsg]

    if (vType == 'float' 
        or vType == 'PReal' or vType == 'UReal' or vType == 'Real'
        or vType == 'PFloat' or vType == 'UFloat'):
        floatValue = float(value)
        if vType == 'PReal' and floatValue <= 0:
                errorMsg = 'PReal must be > 0'
        if vType == 'UReal' and floatValue < 0:
                errorMsg = 'UReal must be >= 0'
        if vType == 'PFloat' and floatValue <= 0:
                errorMsg = 'PFloat must be > 0'
        if vType == 'UFloat' and floatValue < 0:
                errorMsg = 'UFloat must be >= 0'
        
        return [float(value), errorMsg]

    if vType == 'Index' or vType == 'Int' or vType == 'PInt' or vType == 'UInt':
        intValue = int(value)

        if vType == 'Index' or vType == 'UInt':
            if intValue < 0:
                errorMsg = 'UInt must be >= 0'

        if vType == 'PInt':
            if intValue <= 0:
                errorMsg = 'PInt must be > 0'
                
        return [intValue, errorMsg]

#    print('vType=',vType)
#    print('value=',value)
    
    if vType in dictionaryTypesT:#search for correct type in list
        for iValue in dictionaryTypesT[vType]:
            #the dialog shows and the code writes different spellings of one value (#2640)
            if value in [str(iValue), EnumDisplayName(str(iValue), vType)]:
                return [iValue, errorMsg]

    if (len(vSize) == 2 or                      #must be matrix
        (len(vSize)==1 and vSize[0] > 1) or     #must be vector with fixed size
        (len(vSize)==1 and vSize[0] == -1) ):   #array / vector with undefined size
        return [ast.literal_eval(value), errorMsg]

    return [0, 'unknown type '+vType]

#convert values to string; special treatment of floats (C++ float, single precision)
def ConvertValue2String(value, vType, vSize):
    if (len(vSize)==1 and vSize[0] == 1 and #special treatment for conversion with according number of digits!
        (  vType == 'float'
        or vType == 'PFloat'
        or vType == 'UFloat'
        )):
        return str(float32(value))
    #elif len(vSize)==1 and vType == 'VectorFloat':
    elif vType == 'VectorFloat' or vType == 'MatrixFloat': #special treatment for conversion with according number of digits!
        #return str(np.array(value,dtype=float32).tolist()) #still produces float64 converted numbers
        return str(np.array(value,dtype=float32).astype(str).tolist()).replace("'","") #workaround to produce single-precition numbers ...

    #AN ENUM IS SHOWN WITHOUT ITS TYPE (#2640). This is the one place a value becomes the
    #string that the dialog puts in a cell and that ChangedSettings compares, so shortening
    #it here shortens it everywhere at once - and the full name comes back in ValueLiteral,
    #which is what writes the code. A value that is not an enum cannot start with its own
    #type name, so this costs the others nothing.
    return EnumDisplayName(str(value), vType)

#check if a valueStr corresponds to correct type and size; return True, if correct; False if type incorrect
#returns [isValid, errorMSG]
#isValid=True: everything is ok
def CheckType(valueStr, vType, vSize, dictionaryTypesT=None):
#    print('str=',valueStr)

    #':' belongs in a file name: C:/models/gear.stl is what a Windows user types, and without it
    #the dialog refused even its own default (#2597)
    validFileNameChar = " `'{}()%&-@#$~!_^./\\:"

    #an enum is a value of a fixed list and nothing else. Without this branch the string
    #'LinearSolverType.EXUdense' fell through to the exec() below, raised NameError and was
    #reported as "invalid array or matrix" - which the combo box hid, because it never asks
    #CheckType (#2597)
    if dictionaryTypesT is not None and vType in dictionaryTypesT:
        allowed = [EnumDisplayName(str(value), vType)
                   for value in dictionaryTypesT[vType]]
        if EnumDisplayName(valueStr, vType) in allowed:
            return [True, '']
        return [False, vType + ' must be one of: ' + ', '.join(allowed)]
    
#    if vType == 'bool':
#        if valueStr=='False' or valueStr=='True':
#            return [True, '']
#        else:
#            return [False, 'bool may only be True or False']

    if vType == 'FileName':
        if len(valueStr) == 0 or valueStr[0]==' ': #space at first position may be possible on file systems, but is not recommended
            return [False, 'filename may neither be empty nor begin with a SPACE character']
        for x in valueStr: #this is inefficient but should not delay too much
            if not ((x in validFileNameChar)  or x.isalpha() or x.isnumeric()):
                return [False, 'invalid character in file name: may only be A-Z, a-z, 0-9, "'+validFileNameChar +'"']
        return [True, '']

    if vType == 'String':
        return [True, '']
    if vType == 'float':
        rv = IsFloat(valueStr)
        if rv:
            return [True, '']
        else:
            return [False, 'invalid float number']
    if vType == 'Index' and not valueStr.isdigit():
        return [False, 'invalid integer (must be positive)']
    
    #Now check vectors, matrices, ...: try if value can be converted ...
    x=[0]
    try:
        s = 'locx='+str(valueStr)# + '\nprint(x)'
        mylocals={'locx':[]}
        exec(s,globals(),mylocals)
        x=mylocals['locx']
    except Exception:
        return [False, 'invalid array or matrix: check brackets and types']

    
    if len(vSize) == 1 and vSize[0] > 1: #vector/array
        if len(x) != vSize[0]:
            return [False, 'vector/array must have length '+str(vSize[0])]
        
        if vType == 'IndexArray':
            for i in x:
                if int(i) != i or i < 0: #not an integer
                    return [False, 'array values must be positive integer (including 0)']
    if len(vSize) == 2:
        if len(x) != vSize[0]:
            return [False, 'matrix must have '+str(vSize[0]) + ' rows']
        for row in x:
            if len(row) != vSize[1]:
                return [False, 'matrix must have '+str(vSize[1]) + ' columns']
    
    return [True, '']

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#WHAT THE DIALOG SHOWS, WITHOUT A WINDOW.
#The tree, the code line, the marking of a changed value and the find ask the same questions -
#which leaves are there, and what does a leaf look like as Python - so they are asked here, on
#dictionaries, where a test can reach them without opening anything.

def SettingsLeafList(dictionaryWithTypeInfo, path=''):
    """every editable value of a settings structure, in tree order

    Args:
        dictionaryWithTypeInfo: what GetDictionaryWithTypeInfo() returns, or a part of it
        path: the dotted path the given dictionary sits at, '' for the whole structure

    Returns:
        list of (path, value, valueString, vType, vSize, description); valueString is what the
        dialog shows in the cell, which is what everything else compares and copies, and value is
        what the settings structure holds
    """
    leaves = []
    for (key, value) in dictionaryWithTypeInfo.items():
        if not isinstance(value, dict):
            continue
        if 'itemIdentifier' in value:
            leaves.append((path + key, value['value'],
                           ConvertValue2String(value['value'], value['type'], value['size']),
                           value['type'], value['size'], value['description']))
        else:
            leaves += SettingsLeafList(value, path + key + '.')
    return leaves


def EnumDisplayName(valueStr, vType):
    """the enum value without the type in front of it: Displacement, not
    OutputVariableType.Displacement

    An enum is edited in a combo box as wide as the value column, and every entry of a list
    begins with the same type name - which is already in the type column beside it - so the
    part that tells the entries apart was pushed out of sight (#2635). Since #2640 this is
    **the** value string of an enum - what `ConvertValue2String` produces, what the cell
    shows, and what `ChangedSettings` compares - because shortening only the list left the
    cell unreadable the moment the box collapsed. The full name lives in exactly one place:
    `ValueLiteral`, which writes the Python.

    Args:
        valueStr: the value as str() writes it
        vType: the type name of the setting

    Returns:
        the value without its type prefix; anything else unchanged, bool included
    """
    prefix = vType + '.'
    return valueStr[len(prefix):] if valueStr.startswith(prefix) else valueStr


def EnumFullName(displayName, vType):
    """the inverse of EnumDisplayName: the name Python needs, which `ValueLiteral` writes

    Args:
        displayName: the value as the combo box shows it
        vType: the type name of the setting

    Returns:
        the value with its type prefix; an empty string, a bool and an already complete name
        unchanged
    """
    if displayName in ['', 'True', 'False'] or displayName.startswith(vType + '.'):
        return displayName
    return vType + '.' + displayName


def ValueLiteral(valueStr, vType, dictionaryTypesT=None):
    """the value as PYTHON writes it: a string is quoted, an enum carries its module

    Args:
        valueStr: the value as the dialog shows it
        vType: the type name of the setting
        dictionaryTypesT: the lists of the types that have a fixed set of values

    Returns:
        a string that can stand on the right hand side of an assignment
    """
    if vType in ['String', 'FileName']:
        return repr(valueStr)
    if vType != 'bool' and dictionaryTypesT is not None and vType in dictionaryTypesT:
        #the full name, which is what Python needs: the dialog shows the short one (#2640)
        return 'exu.' + EnumFullName(valueStr, vType)
    return valueStr


def SettingsCodeLines(currentLeaves, referenceValueStrings, prefix, dictionaryTypesT=None):
    """the settings that differ from a reference, as the lines that set them

    The comparison is on the string the dialog SHOWS, not on the value: that is what makes a float
    and an enum comparable at all, and it marks exactly what a user sees in the cell.

    Args:
        currentLeaves: SettingsLeafList(...), or the same six fields taken from the dialog
        referenceValueStrings: {path: valueString} of what is compared against - the defaults, or
            the values a dialog opened with; a path that is not in it counts as unchanged
        prefix: SettingsPrefix(...) of the structure
        dictionaryTypesT: the lists of the types that have a fixed set of values

    Returns:
        list of (path, line), in tree order
    """
    lines = []
    for (path, _, valueStr, vType, _, _) in currentLeaves:
        if referenceValueStrings.get(path, valueStr) != valueStr:
            lines.append((path, prefix + '.' + path + ' = '
                          + ValueLiteral(valueStr, vType, dictionaryTypesT)))
    return lines


#the settings a SystemContainer initialises beyond the defaults of the structure itself - and
#(#2626) there are NONE: the three dimmed lights and the ten
#raytracer materials are defaults of the structure now, written in
#definitions/structureDefsVisualizationSettings.py, so the constructor is the truth and a
#difference shown by the dialog is a difference a user made. The list stays as the place to name
#an exception, and a test requires it to remain empty.
containerInitialisedSettings = []


def CompiledSettingsClass(settingsStructure):
    """the class the COMPILED module defines for this structure, which is not always its own

    A settings structure can be an instance of a Python subclass: `import exudyn` installs one for
    VisualizationSettings when ~/.exudyn/config.json holds any, so that a stored setting reaches
    every structure that is created (revision2026b step RG12.10, #2684). Constructing that subclass
    to find the DEFAULTS would apply the overrides to it and report them as the defaults - measured
    on the first attempt: openGL.multiSampling came back with 4 as its own default - so everything
    that shows a difference has to construct the compiled class instead.

    Args:
        settingsStructure: the structure being edited

    Returns:
        the first class of its mro that the compiled module defines; the structure's own class when
        it is not a subclass, which is the normal case
    """
    compiledModuleName = exudyn._compiledModule.__name__   #'exudyn.exudynCPP', or the fast one
    for candidate in type(settingsStructure).__mro__:
        if candidate.__module__ == compiledModuleName:
            return candidate
    return type(settingsStructure)


def DefaultSettingsDictionary(settingsStructure):
    """the defaults of a settings structure, as its own constructor produces them

    NOT from a SystemContainer, although that is the state a user really starts from: creating one
    ATTACHES IT TO THE RUNNING RENDER ENGINE (MainSystemContainer() calls
    AttachToRenderEngineInternal) and destroying one DETACHES it (Reset() ->
    DetachFromRenderEngine), so a temporary container opened for a moment takes the render window
    away from the container that owns it - the window closes (#2625). The settings a container
    initialises are listed in containerInitialisedSettings above, and RG6.2.20 moves them where
    this function can see them.

    AND NOT BY CONSTRUCTING ANYTHING when the override settings are in use: since revision2026b
    step RG12.17 the constructor of the compiled class itself applies a stored setting, so
    `exudyn.misc.overrideSettings.structureDefaults` holds what it produced BEFORE it was wrapped,
    and that is used when it is there.

    Args:
        settingsStructure: the structure being edited

    Returns:
        the dictionary with type info of a fresh structure of the same kind
    """
    compiledClass = CompiledSettingsClass(settingsStructure)
    try:
        from exudyn.misc.overrideSettings import structureDefaults              # noqa: PLC0415
        if compiledClass.__name__ in structureDefaults:
            return copy.deepcopy(structureDefaults[compiledClass.__name__])
    except ImportError:            #a package without the module: constructing is still right
        pass
    return compiledClass().GetDictionaryWithTypeInfo()


def SettingsValueStrings(dictionaryWithTypeInfo):
    """{path: valueString} of a settings structure - what SettingsCodeLines compares against"""
    return {path: valueStr
            for (path, _, valueStr, _, _, _) in SettingsLeafList(dictionaryWithTypeInfo)}


def FindMatches(leaves, searchText):
    """the settings a search text finds: the NAMES first, the descriptions second

    Several hundred values in a tree of folders, and the only
    way to a setting was knowing which folder it sits in.

    Args:
        leaves: SettingsLeafList(...) of the settings being searched
        searchText: what the user typed; case does not matter

    Returns:
        list of (path, label), name hits first, then hits in the path, then hits that are only in
        the description - those labelled with the part of the description that matched
    """
    searchText = searchText.strip().lower()
    if searchText == '':
        return []
    (nameHits, pathHits, descriptionHits) = ([], [], [])
    for (path, _, _, _, _, description) in leaves:
        name = path.split('.')[-1]
        if searchText in name.lower():
            nameHits.append((path, path))
        elif searchText in path.lower():
            pathHits.append((path, path))
        elif searchText in description.lower():
            start = max(0, description.lower().find(searchText) - 20)
            snippet = ' '.join(description[start:start + 70].split())
            descriptionHits.append((path, path + '  -  ...' + snippet + '...'))
    return nameHits + pathHits + descriptionHits


def SettingsPrefix(settingsStructure):
    """the name a script uses for this settings structure, e.g. SC.visualizationSettings"""
    structure = type(settingsStructure).__name__
    if structure == 'VisualizationSettings':
        return 'SC.visualizationSettings'
    if structure == 'SimulationSettings':
        return 'simulationSettings'
    return structure[:1].lower() + structure[1:]

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#WHAT A MODEL CHANGED (#2590). The dialog could always show it for
#one session; these three say it for a whole script, which is what makes a set of settings
#reproducible - the answer is the code that produces them.

def ChangedSettings(settingsStructure, reference=None):
    """every setting that differs from the defaults, as (path, the line that sets it)

    Args:
        settingsStructure: SC.visualizationSettings or a simulationSettings object
        reference: {path: valueString} to compare against; the defaults of the structure by
            default. SettingsValueStrings(structure.GetDictionaryWithTypeInfo()) taken earlier
            gives the changes since THAT moment instead

    Returns:
        list of (path, line), in tree order; empty if nothing differs

    Example:
        SC.visualizationSettings.openGL.lineWidth = 2
        ChangedSettings(SC.visualizationSettings)
        #[('openGL.lineWidth', 'SC.visualizationSettings.openGL.lineWidth = 2.0')]
    """
    leaves = SettingsLeafList(settingsStructure.GetDictionaryWithTypeInfo())
    if reference is None:
        reference = SettingsValueStrings(DefaultSettingsDictionary(settingsStructure))
    return SettingsCodeLines(leaves, reference, SettingsPrefix(settingsStructure),
                             GetComboBoxListsDict(exudyn))


def ChangedSettingsCode(settingsStructure, reference=None, comment=True):
    """what ChangedSettings found, as one pastable block of Python

    Args:
        settingsStructure: SC.visualizationSettings or a simulationSettings object
        reference: see ChangedSettings
        comment: True writes one '#' line saying how many settings differ, which is what makes
            the block readable when it is stored somewhere

    Returns:
        the lines as one string, '' if nothing differs and comment is False

    Note:
        This is also what belongs in a solution file that has to be reproducible:
        simulationSettings.solutionSettings.solutionInformation is written into the header of
        the solution file, and it takes any string.
    """
    changes = ChangedSettings(settingsStructure, reference)
    lines = [line for (_, line) in changes]
    if comment:
        what = SettingsPrefix(settingsStructure)
        one = (len(changes) == 1)
        lines = ['#' + str(len(changes)) + (' setting of ' if one else ' settings of ')
                 + what + (' differs' if one else ' differ') + ' from the defaults'] + lines
    return '\n'.join(lines)


def PrintChangedSettings(settingsStructure, reference=None):
    """print what a model changed, as the code that changes it

    Args:
        settingsStructure: SC.visualizationSettings or a simulationSettings object
        reference: see ChangedSettings

    Returns:
        None
    """
    exudyn.Print(ChangedSettingsCode(settingsStructure, reference))
