#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# File:     Exudyn GUI helper files (Tkinter)
#
# Details:  Helper functions and classes for graphical interaction with Exudyn
#
# Author:   Johannes Gerstmayr
# Date:     2020-01-25
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# Notes:    This is an internal library, which is only used inside Exudyn for modifying settings.
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import tkinter as tk
import tkinter.messagebox
import tkinter.ttk as ttk
import numpy as np #for array checks
from numpy import float32
import ast #for ast.literal_eval
import sys
import exudyn
from exudyn.misc.keyBindings import RendererHelpText

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'useRenderWindowDisplayScaling', 'treeviewDefaultFontSize', 'textHeightFactor',
    'treeEditDefaultWidth', 'treeEditDefaultHeight', 'treeEditMaxInitialHeight',
    'dialogDefaultWidth', 'dialogDefaultHeight', 'treeEditOpenItems', 'treeEditLastOpenItems',
    'codeLineBackground', 'changedValueColor', 'IsApple', 'GetRendererSystemContainer',
    'GetTkRootAndNewWindow', 'TkRootExists', 'TkTextHeight', 'IsFloat', 'IsArrayInt', 'IsVector',
    'GetExudynDisplayScaling', 'GetGUIContentScaling', 'DialogScaling', 'GetComboBoxListsDict',
    'ConvertString2Value', 'ConvertValue2String', 'CheckType', 'SettingsLeafList', 'ValueLiteral',
    'SettingsCodeLines', 'containerInitialisedSettings', 'DefaultSettingsDictionary',
    'SettingsValueStrings', 'FindMatches', 'SettingsPrefix', 'Tooltip',
    'TkinterEditDictionaryWithTypeInfo', 'EditDictionaryWithTypeInfo', 'TkinterEditDictionary',
    'EditDictionary', 'ApplyDialogWindowSettings', 'rendererHelpText', 'ShowHelpDialog',
    'pythonCommandExamples', 'ShowPythonCommandDialog', 'ShowVisualizationSettingsDialog',
    'ShowRightMouseSelectionDialog', 'AskQuitDialog',
    ]

useRenderWindowDisplayScaling = True #using this, scaling will change with render window

treeviewDefaultFontSize = 9 #this is then scaled; but it could be changed to make fonts smaller
textHeightFactor = 1.45 #this is the factor between font size and text height; larger values leading to more space between lines

treeEditDefaultWidth = 1024     #unscaled width of e.g. visualizationSettings
treeEditDefaultHeight = 800     #unscaled height of e.g. visualizationSettings
treeEditMaxInitialHeight = 1440 #larger height, if screen resolution admits
dialogDefaultWidth = 800        #unscaled width of e.g. right mouse edit
dialogDefaultHeight = 600       #unscaled height of e.g. right mouse edit
#the folders that are open when a settings dialog is opened; a user sets this, and nothing
#else writes it (revision2026b step RG6.2.7, #2591 - the dialog used to APPEND to it and remove
#from it on every click, so that clicking in the dialog silently rewrote the configuration)
treeEditOpenItems = ['bodies','connectors','nodes','general']
#which folders were open when a dialog was last used in this process; None until one was, and
#then it is what the next dialog opens with. This is the remembering the clicks used to do, with
#a name that says it is session state and not configuration.
treeEditLastOpenItems = None
codeLineBackground = '#eef1f6'  #the box holding the line that sets a setting (#2605)
changedValueColor = '#1a3fb0'   #a setting that differs from its default (#2606)

def IsApple():
    if sys.platform == 'darwin':
        return True
    else:
        return False

def GetRendererSystemContainer():
    try:
        if 'currentRendererSystemContainer' in exudyn.sys: 
            guiSC = exudyn.sys['currentRendererSystemContainer']
            if guiSC != 0 and type(guiSC) == exudyn.SystemContainer:
                return guiSC
    #RuntimeError is the access violation of a container that was destroyed while the entry
    #still names it (#2623); a dialog must not die of that
    except (KeyError, AttributeError, RuntimeError): 
        pass
    return None

def GetTkRootAndNewWindow():
    """get new or current root and new window app; return list of [tkRoot, tkWindow, tkRuns]
    """
    if tk._default_root is None:
        root = tk.Tk()
        tkWindow = root
        tkRuns = False
    else:
        root = tk._default_root
        tkWindow = tk.Toplevel(root)
        tkRuns = True
    return [root, tkWindow, tkRuns]

def TkRootExists():
    """this function returns True, if tkinter has already a root window (which is assumed to have already a mainloop running)
    """
    return (tk._default_root is not None)



#unique text height for tk with given scaling
def TkTextHeight(systemScaling):
    #OLD, without     style.configure("Treeview", font=(None, treeviewDefaultFontSize ) ):
    #return int(13*systemScaling) #must be int; 13 is good; 16 is too big on surface
    
    return int((treeviewDefaultFontSize*textHeightFactor)*systemScaling) #must be int; 13 is good with treeviewDefaultFontSize = 9; 12 leads to some cuts of 'g'

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

#safely request scaling factor from exudyn
def GetExudynDisplayScaling():
    try:
        if 'currentRendererSystemContainer' in exudyn.sys: 
            guiSC = exudyn.sys['currentRendererSystemContainer']
            if guiSC != 0: #this would mean that renderer is detached
                rs = guiSC.renderer.GetState()
                return rs['displayScaling']
        
        return 1

    except (KeyError, AttributeError, tk.TclError): 
        return 1

#return either exudyn or tkinter scaling, unified approach
def GetGUIContentScaling(root):
    try:
        if useRenderWindowDisplayScaling: #would also work under linux
            s = 1.4*GetExudynDisplayScaling() #gives similar size as other programs; factor 1.4 is empirical
            root.tk.call('tk', 'scaling', s) #needed to update font size internally ...
            return s
        else:
            return root.tk.call('tk', 'scaling') #obtains current scaling?
    except tk.TclError:
        return 1
    
def DialogScaling(root):
    """How tall a row of a dialog is and how large its font is, as [systemScaling, fontFactor].

    dialogs.fontScaling is 0 by default, and 0 means what every platform did before the setting
    existed: a fixed factor on MacOS, the system display scaling on Windows and Linux. A value
    > 0 sets the font on EVERY platform, which is what makes the dialogs readable on a Linux
    desktop - off MacOS the font factor used to be forced to 1 and nothing could change it
    (#2602, revision2026b step RG6.2.3.1).

    Args:
        root: the tkinter root window, which knows the display scaling

    Returns:
        [systemScaling, fontFactor]
    """
    fontScaling = 0.
    guiSC = GetRendererSystemContainer()
    if guiSC is not None:
        #the deprecated fontScalingMacOS forwards to this one in the C++, so it is read once
        fontScaling = guiSC.visualizationSettings.dialogs.fontScaling

    systemScaling = 1.35                #ideal for MacOS
    fontFactor = systemScaling
    if not IsApple():
        #the font size is not changed here; the content scaling does it internally
        fontFactor = 1
        systemScaling = GetGUIContentScaling(root)

    if fontScaling > 0:                 #explicit, and it wins on every platform
        fontFactor = fontScaling
        systemScaling = fontScaling

    return [systemScaling, fontFactor]

#create dictionaries for lists in combo box: bool, OutputVariableType, ...
def GetComboBoxListsDict(exu = None):
    """The values a settings item of an enum type may take, as {typeName: [values]}.

    EVERY enum of the module, not a hand-written list of three: until revision2026b step RG6.2.3
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
            if str(iValue) == value:
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
    return str(value)

#check if a valueStr corresponds to correct type and size; return True, if correct; False if type incorrect
#returns [isValid, errorMSG]
#isValid=True: everything is ok
def CheckType(valueStr, vType, vSize, dictionaryTypesT=None):
#    print('str=',valueStr)

    #':' belongs in a file name: C:/models/gear.stl is what a Windows user types, and without it
    #the dialog refused even its own default (#2597, revision2026b step RG6.2.3)
    validFileNameChar = " `'{}()%&-@#$~!_^./\\:"

    #an enum is a value of a fixed list and nothing else. Without this branch the string
    #'LinearSolverType.EXUdense' fell through to the exec() below, raised NameError and was
    #reported as "invalid array or matrix" - which the combo box hid, because it never asks
    #CheckType (#2597)
    if dictionaryTypesT is not None and vType in dictionaryTypesT:
        allowed = [str(value) for value in dictionaryTypesT[vType]]
        if valueStr in allowed:
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
#WHAT THE DIALOG SHOWS, WITHOUT A WINDOW (revision2026b steps RG6.2.8 to RG6.2.10).
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
        return 'exu.' + valueStr           #an enum needs the module it lives in
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
#since revision2026b step RG6.2.20 (#2626) there are NONE: the three dimmed lights and the ten
#raytracer materials are defaults of the structure now, written in
#definitions/structureDefsVisualizationSettings.py, so the constructor is the truth and a
#difference shown by the dialog is a difference a user made. The list stays as the place to name
#an exception, and a test requires it to remain empty.
containerInitialisedSettings = []


def DefaultSettingsDictionary(settingsStructure):
    """the defaults of a settings structure, as its own constructor produces them

    NOT from a SystemContainer, although that is the state a user really starts from: creating one
    ATTACHES IT TO THE RUNNING RENDER ENGINE (MainSystemContainer() calls
    AttachToRenderEngineInternal) and destroying one DETACHES it (Reset() ->
    DetachFromRenderEngine), so a temporary container opened for a moment takes the render window
    away from the container that owns it - the window closes (#2625). The settings a container
    initialises are listed in containerInitialisedSettings above, and RG6.2.20 moves them where
    this function can see them.

    Args:
        settingsStructure: the structure being edited

    Returns:
        the dictionary with type info of a fresh structure of the same kind
    """
    return type(settingsStructure)().GetDictionaryWithTypeInfo()


def SettingsValueStrings(dictionaryWithTypeInfo):
    """{path: valueString} of a settings structure - what SettingsCodeLines compares against"""
    return {path: valueStr
            for (path, _, valueStr, _, _, _) in SettingsLeafList(dictionaryWithTypeInfo)}


def FindMatches(leaves, searchText):
    """the settings a search text finds: the NAMES first, the descriptions second

    Several hundred values in a tree of folders, and until revision2026b step RG6.2.10 the only
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


class Tooltip:
    """The small yellow window that shows the description of the row under the mouse.

    tkinter has none, and the description used to be behind the key 'h' and a modal message box -
    which is not where a reader looks for it (#2601, revision2026b step RG6.2.3). It is a
    borderless Toplevel that is created when it is first needed and hidden afterwards, so a
    dialog that is never hovered never builds one.
    """

    def __init__(self, widget, wrapLength=520, delay=500):
        self.widget = widget
        self.wrapLength = wrapLength
        self.delay = delay          #ms before it appears; a tooltip that is instant is in the way
        self.window = None
        self.label = None
        self.pending = None

    def Show(self, text, x, y):
        """show the tooltip after the delay, at the screen position (x, y)

        The delay is the maintainer's (#2614): a description that appears the moment the pointer
        crosses a row is annoying, and since RG6.2.10 nobody has to sweep the tree to find a
        setting any more.
        """
        self.Cancel()
        if self.delay > 0:
            self.pending = self.widget.after(self.delay, lambda: self.Place(text, x, y))
        else:
            self.Place(text, x, y)

    def Cancel(self):
        """forget a tooltip that was scheduled and has not appeared yet"""
        if self.pending is not None:
            try:
                self.widget.after_cancel(self.pending)
            except (tk.TclError, ValueError):
                pass
            self.pending = None

    def Bind(self, widget, text):
        """let a widget - a button, say - show this tooltip while the pointer rests on it"""
        widget.bind('<Enter>', lambda event: self.Show(text, event.x_root, event.y_root))
        widget.bind('<Leave>', lambda event: self.Hide())

    def Place(self, text, x, y):
        """place the tooltip at the screen position (x, y), a little below the pointer"""
        self.pending = None
        if self.window is None:
            self.window = tk.Toplevel(self.widget)
            self.window.wm_overrideredirect(True)   #no title bar, no border
            self.label = tk.Label(self.window, justify=tk.LEFT, background='#ffffe0',
                                  relief=tk.SOLID, borderwidth=1, wraplength=self.wrapLength)
            self.label.pack(ipadx=3, ipady=2)
        self.label.configure(text=text)
        self.window.wm_geometry('+' + str(int(x) + 16) + '+' + str(int(y) + 18))
        self.window.deiconify()

    def Hide(self):
        self.Cancel()
        if self.window is not None:
            self.window.withdraw()


#this class gets a dictionary with type information structure in, but a plain dictionary with types (int, float, string, list, ...) out
#settingsStructure: contains hierarchical settings structure with function GetDictionaryWithTypeInfo() to obtain dictionary for editing
#dictionaryTypes: contains a dictionary with the available types, e.g. bool, etc.
#updateOnChange: every change is directly applied to the settingsStructure and redraw is signaled in stored renderer
class TkinterEditDictionaryWithTypeInfo(tk.Frame):
    def __init__(self, parent, settingsStructure, dictionaryTypesT, updateOnChange=False, treeOpen=False, textHeight = 15, systemScaling = 1, fontFactor = 1):
        tk.Frame.__init__(self, parent)
        
        self.parentFrame = parent #parent frame stored for member functions
        self.settingsStructure = settingsStructure
        self.dictionaryTypesT = dictionaryTypesT #as type
        self.updateOnChange = updateOnChange
        self.treeOpen = treeOpen
        self.textHeight = textHeight
        self.systemScaling = systemScaling
        self.fontFactor = fontFactor
        #the code line is read, not edited: one size below the cells, which is what the
        #maintainer asked for after using it (revision2026b step RG6.2.8, #2605)
        self.codeFontSize = max(6, int(treeviewDefaultFontSize*fontFactor) - 1)

        self.dictionaryData = settingsStructure.GetDictionaryWithTypeInfo()
        #the folders this dialog opens with: the configuration, or what was open when a dialog
        #was last used in this process (revision2026b step RG6.2.7, #2591)
        self.openItems = list(treeEditOpenItems if treeEditLastOpenItems is None
                              else treeEditLastOpenItems)

        #additional storage for type, size, etc.
        self.typeStorage = dict() #dictionary is stored as {'ID1': 'type1', 'ID2': 'type2', ...}
        self.sizeStorage = dict()
        self.descriptionStorage = dict()

        #create treeview:
        self.tree = ttk.Treeview(self, columns=("value","type","description"), selectmode='browse',
                                 height=self.textHeight)
        self.vertivalScrollbar = ttk.Scrollbar(self, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=self.vertivalScrollbar.set)

        #THE FIND BAR (revision2026b step RG6.2.10, #2607) stands above the tree, which is
        #where CTRL-F puts one. It searches the NAMES first and the descriptions second, and the
        #drop-down holds the hits, so that one can be picked instead of stepped to.
        self.findFrame = tk.Frame(self)
        #columnspan 3, not 4: column 3 is the vertical scroll bar of the tree, and a button
        #that reaches under it looks misplaced (#2621)
        self.findFrame.grid(row=0, column=0, columnspan=3, sticky=tk.E+tk.W)
        self.findFrame.grid_columnconfigure(2, weight=1)
        tk.Label(self.findFrame, text='find:').grid(row=0, column=0, padx=(4, 2))
        self.findVar = tk.StringVar()
        self.findEntry = tk.Entry(self.findFrame, textvariable=self.findVar, width=24)
        self.findEntry.grid(row=0, column=1, pady=2)
        self.findEntry.bind('<Return>', self.OnFindNext)
        self.findEntry.bind('<Escape>', lambda event: self.tree.focus_set())
        self.findVar.trace_add('write', lambda *arguments: self.UpdateFindHits())
        #no find button: the search runs while the text is typed, which is enough (#2613);
        #and the drop-down is disabled until there is something to pick, so that it does not
        #look like a control that does nothing
        self.findCombo = ttk.Combobox(self.findFrame, state='disabled', values=())
        self.findCombo.grid(row=0, column=2, sticky=tk.E+tk.W, padx=4)
        self.findCombo.bind('<<ComboboxSelected>>', self.OnFindPick)
        self.findHits = []
        self.findIndex = -1

        self.tree.grid(row=1, column=0, columnspan=3, sticky="nsew")
        self.vertivalScrollbar.grid(row=1, column=3, sticky='nse')
        self.vertivalScrollbar.configure(command=self.tree.yview)


        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0,weight=1)
        self.grid_columnconfigure(1,weight=2)
        self.grid_columnconfigure(2,weight=2)
        self.grid_columnconfigure(3,weight=0)
        
        self.tree.heading("#0",text="Name",anchor=tk.W)
        self.tree.heading("value",text="Value",anchor=tk.W)
        self.tree.heading("type",text="Type",anchor=tk.W)
        self.tree.heading("description",text="Description",anchor=tk.W)

        #the columns had no width at all until revision2026b step RG6.2.3 (#2601): tree.column()
        #was never called, so every one of them kept tkinter's 200 px default - the name was cut,
        #the description was unreadable, and dragging one moved the others. Only the description
        #stretches with the window; the rest keep what they are given.
        scale = max(1, int(round(self.systemScaling)))
        self.tree.column("#0", width=260*scale, minwidth=120, stretch=False)
        self.tree.column("value", width=150*scale, minwidth=60, stretch=False)
        self.tree.column("type", width=90*scale, minwidth=50, stretch=False, anchor=tk.W)
        self.tree.column("description", width=420*scale, minwidth=120, stretch=True)


        #a row that differs from the default is written in colour and bold (#2606); the size
        #has to be given here, because a tag font does not follow the style of the tree
        self.tree.tag_configure('changed', foreground=changedValueColor,
                                font=(None, int(treeviewDefaultFontSize*fontFactor), 'bold'))

        self.AddNodeFromDictionaryWithTypeInfo(value=self.dictionaryData, parentNode="")
        self.tree.bind('<<TreeviewSelect>>', self.TreeviewSelect) #selection changed
        self.tree.bind("<Double-1>", self.OnTreeDoubleClick) #an item has been selected for change
        self.tree.bind("<Return>", self.OnTreeEdit) #an item has been selected for change
        self.tree.bind("<ButtonRelease-1>", self.OnTreeEdit) #an item has been selected for change #1357
        self.tree.bind("h", self.OnTreeHelp) #show description for selected item; the tooltip
                                             #below is the ordinary way since #2601
        self.tooltip = Tooltip(self.tree)
        self.tooltipRow = ''
        self.tree.bind("<Motion>", self.OnTreeHover)
        self.tree.bind("<Leave>", lambda event: self.tooltip.Hide())
        self.tree.bind("<Escape>", self.OnQuit) 
        self.tree.bind("q", self.OnQuit) 
        
        #+++++++++++++++++++++++++++++++++++++++++
        #THE EDITOR SITS IN THE CELL (revision2026b step RG6.2.4, #2604). It used to be an
        #Entry and a Combobox at the bottom of the window that swapped places by z-order, so a
        #value was typed far away from the row it belonged to. These two are children of the
        #tree and are placed over the value cell while an edit is running.
        self.selectedItem = ''          #the item being edited, '' when nothing is
        self.editItemVar = tk.StringVar()
        self.cellEditor = tk.Entry(self.tree, textvariable=self.editItemVar)
        self.cellEditor.bind('<Return>', self.OnCellCommit)
        self.cellEditor.bind('<FocusOut>', self.OnCellCommit)
        self.cellEditor.bind('<Escape>', self.OnCellCancel)

        self.cellCombo = ttk.Combobox(self.tree, values=['True','False'], state='readonly')
        self.cellCombo.bind('<<ComboboxSelected>>', self.OnCellCommit)
        self.cellCombo.bind('<Escape>', self.OnCellCancel)

        #+++++++++++++++++++++++++++++++++++++++++
        #the bottom row is not an editor any more: it says what the selected item IS, as the line
        #that sets it - which is what the maintainer asked for, a line to copy into a script.
        #It sits in a BOX of its own since revision2026b step RG6.2.8 (#2605): the plain entry on
        #the window background did not look like something one copies. The label that stood under
        #it went with the same step - it repeated the description that the tooltip already shows,
        #and the size it also carried is in the tooltip now.
        self.bottomFrame = tk.Frame(self)
        self.bottomFrame.grid(row=2, column=0, columnspan=3, sticky=tk.E+tk.W)
        self.bottomFrame.grid_columnconfigure(0, weight=1)

        codeBox = tk.Frame(self.bottomFrame, relief=tk.SOLID, borderwidth=1,
                           background=codeLineBackground)
        codeBox.grid(row=0, column=0, sticky=tk.E+tk.W, padx=4, pady=4)
        self.codeLineVar = tk.StringVar()
        self.codeLine = tk.Entry(codeBox, textvariable=self.codeLineVar, state='readonly',
                                 readonlybackground=codeLineBackground, borderwidth=0,
                                 highlightthickness=0, font=(None, self.codeFontSize))
        self.codeLine.pack(fill=tk.X, expand=True, padx=4, pady=2)

        self.copyButton = tk.Button(self.bottomFrame, text='copy line',
                                    command=self.OnCopyCodeLine)
        self.copyButton.grid(row=0, column=1, padx=(2, 4))

        #THE BUTTON ROW (revision2026b step RG6.2.14, #2614): the two windows of RG6.2.9 on the
        #left, and what a user does with a dialog - take it back, or leave it - on the right.
        #Every button says what it does in a tooltip; none of them fits in two words.
        self.buttonFrame = tk.Frame(self)
        self.buttonFrame.grid(row=3, column=0, columnspan=3, sticky=tk.E+tk.W)
        self.buttonFrame.grid_columnconfigure(2, weight=1)      #the gap between the two groups
        self.buttonTooltip = Tooltip(self, wrapLength=420)

        self.diffButton = tk.Button(self.buttonFrame, text='diff to default',
                                    command=self.OnShowDiffToDefault)
        self.diffButton.grid(row=0, column=0, padx=(4, 2), pady=(0, 4))
        self.sessionButton = tk.Button(self.buttonFrame, text='changes since start',
                                       command=self.OnShowSessionChanges)
        self.sessionButton.grid(row=0, column=1, padx=2, pady=(0, 4))

        self.resetButton = tk.Button(self.buttonFrame, text='reset', command=self.OnReset)
        self.resetButton.grid(row=0, column=3, padx=2, pady=(0, 4))
        self.revertButton = tk.Button(self.buttonFrame, text='revert', command=self.OnRevert)
        self.revertButton.grid(row=0, column=4, padx=2, pady=(0, 4))
        self.undoButton = tk.Button(self.buttonFrame, text='undo', command=self.OnUndo,
                                    state=tk.DISABLED)
        self.undoButton.grid(row=0, column=5, padx=2, pady=(0, 4))
        self.closeButton = tk.Button(self.buttonFrame, text='close',
                                     command=lambda: self.parentFrame.destroy())
        self.closeButton.grid(row=0, column=6, padx=(2, 4), pady=(0, 4))

        for (button, description) in [
                (self.copyButton, 'copy the line above, which sets the selected setting'),
                (self.diffButton, 'show diffs to default'),
                (self.sessionButton, 'show changes since dialog opened'),
                (self.resetButton, 'reset to default'),
                (self.revertButton, 'revert to state when dialog opened'),
                (self.undoButton, 'undo the last change, a reset or a revert'),
                (self.closeButton, 'close the dialog (same as ESCAPE)')]:
            self.buttonTooltip.Bind(button, description)

        #one WHOLE state back per entry, so that undo takes a reset and a revert back too
        self.undoStack = []

        #+++++++++++++++++++++++++++++++++++++++++
        #pre-select item
        first = self.tree.get_children('')[0]
        self.tree.focus_set()
        self.tree.focus(first)
        self.tree.selection_set((first))
        
        self.modifiedDictionary = self.GetDictionary('')

        #WHAT DIFFERS FROM THE DEFAULTS IS MARKED (revision2026b step RG6.2.9, #2606). The
        #defaults are one constructor call away, and a settings structure Python builds on its own
        #is safe to read as long as no DEPRECATED member is touched, which none of this does
        #(#2603). If it ever fails, the marking stays off rather than the dialog.
        self.defaultValueStrings = {}
        try:
            self.defaultValueStrings = SettingsValueStrings(
                DefaultSettingsDictionary(self.settingsStructure))
        except Exception as exception:                                       # noqa: BLE001
            exudyn.Print('WARNING: the settings defaults are not available, so nothing is'
                         ' marked as changed: ' + str(exception))
        #and what THIS dialog started with, which is the other thing a user calls "changed"
        self.openingValueStrings = {path: valueStr
                                    for (path, _, valueStr, _, _, _) in self.TreeLeaves()}
        self.MarkChangedValues()

        #what the find searches and where it jumps to (revision2026b step RG6.2.10, #2607); the
        #names and descriptions do not change while the dialog is open, so this is built once
        self.findLeaves = self.TreeLeaves()
        self.itemByPath = {self.ItemPath(item): item for item in self.LeafItems()}
        for widget in [self.parentFrame, self.tree]:
            widget.bind('<Control-f>', self.OnFindFocus)
            widget.bind('<F3>', self.OnFindNext)

    #create treeview from dictionary with type info
    def AddNodeFromDictionaryWithTypeInfo(self, value, parentNode="", key=None, level=0):
        if key is None:
            id = ""
        else:
            id = self.tree.insert(parentNode, "end", text=key)
        isOpen = self.treeOpen and (level<=1)

        if isinstance(value, dict):
            itemIsOpen = isOpen
            if ('itemIdentifier' not in value) and (key in self.openItems): 
                itemIsOpen = True
                
            self.tree.item(id, open=itemIsOpen)
            if 'itemIdentifier' in value: #is a value with types:
                strValue = ConvertValue2String(value['value'], value['type'], value['size'])
                
                #(value, type, description): the type is what tells a reader whether to
                #type 3, 3.0, True or [1,2,3], and it was read and never shown (#2601)
                self.tree.item(id, values=(strValue, value['type'], value['description']))
                #store additional data in dictionaries (could also be tuples ...)
                self.typeStorage[id] = value['type']
                self.sizeStorage[id] = value['size']
                self.descriptionStorage[id] = value['description']
            else: #must be another dictionary:
                #what the FOLDER is: the class description of the settings structure, which
                #reaches the dictionary since revision2026b step RG6.2.15 (#2615) and is what the
                #tooltip shows over a folder - it had nothing to show there at all
                self.descriptionStorage[id] = str(value.get('structureDescription', ''))
                for (key, value) in value.items():
                    if key != 'structureDescription':    #a description, not a settings value
                        self.AddNodeFromDictionaryWithTypeInfo(value, id, key, level=level+1)
        else:
            exudyn.Print('ERROR: AddNodeFromDictionaryWithTypeInfo: item ' + str(value)
                         + ' (parent=' + str(parentNode) + ', key=' + str(key)
                         + ') is no settings value and no sub-structure')

    #create treeview from plain dictionary
    def AddNodeFromDictionary(self, value, parentNode="", key=None):
        if key is None:
            id = ""
        else:
            id = self.tree.insert(parentNode, "end", text=key)

        if isinstance(value, dict):
            self.tree.item(id, open=True)
            for (key, value) in value.items():
                self.AddNodeFromDictionary(value, id, key)
        else:
            self.tree.item(id, values=(value))

    def GetDictionary(self, item):
        d=dict()
        kids = self.tree.get_children(item)
        for i in kids:
            nchilds = len(self.tree.get_children(i))
            if nchilds == 0:
                if len(self.tree.item(i,'values')) != 0:
                    valStr = self.tree.item(i,'values')[0]
                    [val, errorMsg] = ConvertString2Value(valStr, str(self.typeStorage[i]), self.sizeStorage[i], self.dictionaryTypesT)
                    if errorMsg == '':
                        d.update({self.tree.item(i,'text'): val})
                    else:
                        exudyn.Print('ERROR: item ' + str(self.tree.item(i,'text'))
                                     + ' has illegal value "' + valStr + '": ' + errorMsg)
                else:
                    d.update({self.tree.item(i,'text'): ''})
            else:
                d.update({self.tree.item(i,'text'): self.GetDictionary(i)})
        return d

    def OnTreeHover(self,event):
        """the description of the row under the mouse, as a tooltip: it was behind the key 'h'
        and a modal message box, which is not where anybody looks for it (#2601)"""
        row = self.tree.identify_row(event.y)
        if row == self.tooltipRow:
            return                          #same row: leave the tooltip where it is
        self.tooltipRow = row
        self.tooltip.Hide()
        description = self.descriptionStorage.get(row, '')
        if description.strip() != '':
            name = self.tree.item(row,'text')
            vType = self.typeStorage.get(row, '')
            #the size stands here since revision2026b step RG6.2.8 (#2605): it was the only fact
            #of the label under the tree that nothing else said
            size = self.sizeStorage.get(row, '')
            if str(size) not in ['[1]', '1', '']:
                vType += ', size ' + str(size)
            self.tooltip.Show(name + ('  [' + vType + ']' if vType else '') + '\n' + description,
                              event.x_root, event.y_root)

    def OnTreeHelp(self,event):
        item = self.tree.selection()[0]
        if item in self.descriptionStorage:
            d = self.descriptionStorage[item]
            s = self.tree.item(item,'text')
            tk.messagebox.showinfo(s, d)

    def OnTreeDoubleClick(self,event):
        self.OnTreeEditOrDoubleClick(event, True)
        
    def OnTreeEdit(self,event):
        self.OnTreeEditOrDoubleClick(event)
        
    def OnTreeEditOrDoubleClick(self,event,doubleClick=False):
        item = self.tree.selection()[0]
        nchilds = len(self.tree.get_children(item))

        #only edit items which have no subitems (no folders!)
        if nchilds == 0:
            
            selectedType=self.typeStorage[item]
            #+++++++++++++++++++                
            if selectedType=='bool' and doubleClick: #just toggle value #1354
                value=self.tree.item(item,'values')[0]
                if value=='True': value='False'
                else: value='True'

                self.PushUndoState()
                self.tree.item(item, values=(value, self.typeStorage[item],
                                             self.descriptionStorage[item]))
                self.MarkChangedValue(item)
                self.modifiedDictionary = self.GetDictionary('') #update stored dictionary
                self.UpdateSettingsStructure() #only if according flag set in visualizationSettings
                return

            #the path of the item is built by ItemPath() now, which ShowInfo and the code line
            #both need (revision2026b step RG6.2.4)
            self.ShowInfo(item)
            self.StartCellEdit(item)
        else: #folders (may be opened/closed)
            openState = self.tree.item(item, 'open')
            s = self.tree.item(item,'text')
            global treeEditLastOpenItems                                     # noqa: PLW0603
            if openState and (s not in self.openItems):
                self.openItems.append(s)
            elif not openState and (s in self.openItems):
                self.openItems.remove(s)
            #remembered for the next dialog of this process, without touching the configuration
            treeEditLastOpenItems = list(self.openItems)
        

    def OnQuit(self,event): #new selection --> nothing to edit for now
        self.parentFrame.destroy()
        
    def TreeviewSelect(self,event):
        """a new row is selected: the bottom row follows it, whether it is edited or not"""
        self.CancelCellEdit()
        selection = self.tree.selection()
        if selection:
            self.ShowInfo(selection[0])
        
    #if visualizationSettings are accordingly, the renderer will obtain an update signal
    def UpdateSettingsStructure(self):
        #++++++++++++++++++++++++++++++++++++++++++++++++
        #update changes
        self.settingsStructure.SetDictionary(self.modifiedDictionary)  #this may also change dialogs.multiThreadedDialogs itself
        if 'currentRendererSystemContainer' in exudyn.sys:
            guiSC = exudyn.sys['currentRendererSystemContainer']
            if guiSC != 0:
                if guiSC.visualizationSettings.dialogs.multiThreadedDialogs:
                    guiSC.renderer.SendRedrawSignal()
                    guiSC.renderer.DoIdleTasks(0) #do not wait
        #++++++++++++++++++++++++++++++++++++++++++++++++

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++        
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #THE CELL EDITOR (revision2026b step RG6.2.4, #2604)

    def ItemPath(self, item):
        """the dotted path of a row, as a script writes it: openGL.lineWidth"""
        path = self.tree.item(item,'text')
        parent = self.tree.parent(item)
        depth = 0
        while depth < 10 and parent != '':      #limits to 10 levels ...
            depth += 1
            path = self.tree.item(parent,'text') + '.' + path
            parent = self.tree.parent(parent)
        return path

    def CodeLine(self, item):
        """the selected setting as the LINE THAT SETS IT, ready to paste into a script:

            SC.visualizationSettings.openGL.lineWidth = 2.0

        which is what makes a session in this dialog reusable (maintainer, 2026-09-23). The
        prefix follows the structure being edited, the value is written as a Python literal."""
        literal = ValueLiteral(self.tree.item(item,'values')[0],
                               self.typeStorage.get(item, ''), self.dictionaryTypesT)
        return SettingsPrefix(self.settingsStructure) + '.' + self.ItemPath(item) + ' = ' + literal

    def ShowInfo(self, item):
        """the bottom row: the line that sets the selected item"""
        if item == '' or item not in self.typeStorage:
            self.codeLineVar.set('')
            return
        self.codeLineVar.set(self.CodeLine(item))

    def OnCopyCodeLine(self):
        """the line into the clipboard; tkinter keeps it only while it runs, so it is flushed"""
        line = self.codeLineVar.get()
        if line == '':
            return
        self.clipboard_clear()
        self.clipboard_append(line)
        self.update()                         #without this the clipboard is empty after closing

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #WHAT DIFFERS, AND FROM WHAT (revision2026b step RG6.2.9, #2606)

    def LeafItems(self, item=''):
        """the rows that hold a value, in tree order - folders are not settings"""
        items = []
        for child in self.tree.get_children(item):
            if self.tree.get_children(child):
                items += self.LeafItems(child)
            elif child in self.typeStorage:
                items.append(child)
        return items

    def TreeLeaves(self):
        """the rows of the tree in the shape of SettingsLeafList, so that one comparison serves
        the marking, the two windows and the tests. The TREE is asked, not the settings structure:
        what a user sees is what is copied."""
        return [(self.ItemPath(item), None, self.tree.item(item,'values')[0],
                 self.typeStorage[item], self.sizeStorage[item], self.descriptionStorage[item])
                for item in self.LeafItems()]

    def MarkChangedValues(self, item=''):
        """colour every row that differs from the default - and every FOLDER that holds such a
        row somewhere below it, because a folded folder hides the mark otherwise (#2612)

        Returns:
            True if anything below the given item differs from the defaults
        """
        if self.defaultValueStrings == {}:
            return False
        anyChanged = False
        for child in self.tree.get_children(item):
            if self.tree.get_children(child):
                isChanged = self.MarkChangedValues(child)
            else:
                isChanged = self.IsChangedValue(child)
            self.SetChangedTag(child, isChanged)
            anyChanged = anyChanged or isChanged
        return anyChanged

    def IsChangedValue(self, item):
        """does this row differ from the value a user starts with"""
        path = self.ItemPath(item)
        return (path in self.defaultValueStrings
                and self.defaultValueStrings[path] != self.tree.item(item,'values')[0])

    def SetChangedTag(self, item, isChanged):
        self.tree.item(item, tags=('changed',) if isChanged else ())

    def MarkChangedValue(self, item):
        """one row after it was edited, and the folders it sits in"""
        if self.defaultValueStrings == {}:
            return
        self.SetChangedTag(item, self.IsChangedValue(item))
        parent = self.tree.parent(item)
        while parent != '':
            childIsChanged = any('changed' in self.tree.item(child, 'tags')
                                 for child in self.tree.get_children(parent))
            self.SetChangedTag(parent, childIsChanged)
            parent = self.tree.parent(parent)

    def ChangedCodeLines(self, referenceValueStrings):
        return SettingsCodeLines(self.TreeLeaves(), referenceValueStrings,
                                 SettingsPrefix(self.settingsStructure), self.dictionaryTypesT)

    def OnShowDiffToDefault(self):
        description = 'every setting that differs from the Exudyn defaults'
        if self.defaultValueStrings == {}:
            description = 'the defaults are not available in this session'
        self.ShowCodeLines('settings differing from the defaults',
                           self.ChangedCodeLines(self.defaultValueStrings), description)

    def OnShowSessionChanges(self):
        self.ShowCodeLines('settings changed in this dialog',
                           self.ChangedCodeLines(self.openingValueStrings),
                           'what was changed since this dialog was opened')

    def CurrentValueStrings(self):
        """what every row of the tree shows now, as {path: valueString}"""
        return {self.ItemPath(item): self.tree.item(item, 'values')[0]
                for item in self.LeafItems()}

    def PushUndoState(self):
        """remember the WHOLE state before a change, which is what makes undo always work

        A single edited value, a reset and a revert all go through this, so undo takes any of
        them back, and a chain of them one by one (maintainer, 2026-09-23, RG6.2.21). The state
        is ~470 short strings, which costs nothing next to the redraw it triggers.
        """
        self.undoStack.append(self.CurrentValueStrings())
        self.undoButton.configure(state=tk.NORMAL)

    def WriteValue(self, item, valueStr):
        """put a value into a row and let everything that follows from it happen"""
        self.tree.item(item, values=(valueStr, self.typeStorage[item],
                                     self.descriptionStorage[item]))
        self.MarkChangedValue(item)

    def ApplyValues(self, valueStrings, pushUndo=True):
        """write a whole set of {path: valueString} into the tree and into the settings

        Args:
            valueStrings: what to write; a path the tree does not hold is ignored
            pushUndo: remember the state before, so that undo takes the whole set back;
                False only for the undo itself, which must not push what it is popping

        Returns:
            the number of rows that changed
        """
        changed = 0
        if pushUndo:
            self.PushUndoState()
        for item in self.LeafItems():
            path = self.ItemPath(item)
            if path in valueStrings and valueStrings[path] != self.tree.item(item,'values')[0]:
                self.WriteValue(item, valueStrings[path])
                changed += 1
        if changed != 0:
            self.modifiedDictionary = self.GetDictionary('')
            self.UpdateSettingsStructure()
            self.ShowInfo(self.tree.focus())
        elif pushUndo:
            self.undoStack.pop()                    #nothing changed, so there is nothing to undo
            self.undoButton.configure(state=tk.NORMAL if self.undoStack != [] else tk.DISABLED)
        return changed

    def OnReset(self):
        """every setting back to what a user starts with

        It does NOT ask (maintainer, 2026-09-23): a wrong click is taken back by undo, or by
        revert, and a question that is always answered with yes is only in the way.
        """
        if self.defaultValueStrings == {}:
            tk.messagebox.showinfo('reset', 'the defaults are not available in this session')
            return
        self.ApplyValues(self.defaultValueStrings)

    def OnRevert(self):
        """back to the state the dialog opened with; it does not ask either"""
        self.ApplyValues(self.openingValueStrings)

    def OnUndo(self):
        """one whole state back, whether that was one value, a reset or a revert"""
        if self.undoStack == []:
            return
        previousState = self.undoStack.pop()
        self.undoButton.configure(state=tk.NORMAL if self.undoStack != [] else tk.DISABLED)
        self.ApplyValues(previousState, pushUndo=False)

    def ShowCodeLines(self, title, lines, description):
        """the changes as the code that makes them, in a window that shows AND copies: a dialog
        session that can be pasted into a script (maintainer, 2026-09-23)"""
        window = tk.Toplevel(self)
        window.title(title)
        #THE WINDOW HAS TO BE SEEN (#2621). The settings dialog is topmost - it has to be, it
        #blocks the render window - and a Toplevel of it opens at the same place BEHIND it, which
        #looks exactly like a button that does nothing: the content was right all along. Four
        #things together make it visible, and none of them alone was enough: transient ties it to
        #the dialog, topmost puts it in front of a topmost parent, an offset means it cannot be
        #hidden by the dialog even if the stacking fails, and grab_set makes it modal, which is
        #what a window manager never puts behind.
        window.transient(self.parentFrame)
        ApplyDialogWindowSettings(window, alwaysTopmost=True)
        #THE DIALOG STEPS ASIDE (maintainer, 2026-09-23): it keeps itself -topmost, which is why
        #this window came up behind it whatever was done to the window. The flag is taken off the
        #dialog - and off its root, where EditDictionaryWithTypeInfo sets it - while the window is
        #open, and put back when it closes.
        topmostWindows = []
        for candidate in [self.parentFrame, self.parentFrame.master]:
            try:
                if candidate is not None and candidate.attributes('-topmost'):
                    candidate.attributes('-topmost', False)
                    topmostWindows.append(candidate)
            except (tk.TclError, AttributeError):
                pass

        def RestoreTopmost():
            for candidate in topmostWindows:
                try:
                    candidate.attributes('-topmost', True)
                except tk.TclError:
                    pass

        window.bind('<Destroy>', lambda event: RestoreTopmost() if event.widget is window
                    else None)
        self.parentFrame.update_idletasks()
        window.geometry('+' + str(self.parentFrame.winfo_rootx() + 60) + '+'
                        + str(self.parentFrame.winfo_rooty() + 60))
        window.lift()
        window.focus_force()
        window.grab_set()               #released when the window is destroyed
        window.grid_columnconfigure(0, weight=1)
        window.grid_rowconfigure(1, weight=1)

        tk.Label(window, text=description, justify=tk.LEFT, anchor=tk.W).grid(
            row=0, column=0, columnspan=2, sticky=tk.E+tk.W, padx=8, pady=(8, 2))

        textArea = tk.Text(window, wrap=tk.NONE, width=80, height=min(25, max(4, len(lines) + 1)),
                           background=codeLineBackground, font=(None, self.codeFontSize))
        textArea.grid(row=1, column=0, columnspan=2, sticky=tk.NSEW, padx=(8, 0))
        #a structure can differ from the defaults in more lines than fit on a screen
        scrollbar = ttk.Scrollbar(window, orient='vertical', command=textArea.yview)
        scrollbar.grid(row=1, column=2, sticky=tk.N+tk.S, padx=(0, 8))
        textArea.configure(yscrollcommand=scrollbar.set)
        code = '\n'.join([line for (_, line) in lines])
        textArea.insert(tk.END, code if lines else '#nothing')
        textArea.configure(state='disabled')

        def CopyAll():
            if lines:
                self.clipboard_clear()
                self.clipboard_append(code)
                self.update()       #without this the clipboard is empty once the window closes

        tk.Button(window, text='copy all', command=CopyAll).grid(row=2, column=0, pady=6)
        tk.Button(window, text='close', command=window.destroy).grid(row=2, column=1, pady=6)
        window.bind('<Escape>', lambda event: window.destroy())

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #FIND A SETTING (revision2026b step RG6.2.10, #2607)

    def OnFindFocus(self, event=None):
        """CTRL-F: the find entry takes the focus and its text is selected, ready to be replaced"""
        self.findEntry.focus_set()
        self.findEntry.select_range(0, tk.END)
        return 'break'

    def UpdateFindHits(self):
        """as the text is typed: the hits, in the drop-down, names before descriptions"""
        self.findHits = FindMatches(self.findLeaves, self.findVar.get())
        self.findIndex = -1
        self.findCombo['values'] = tuple(label for (_, label) in self.findHits)
        self.findCombo.configure(state='readonly' if self.findHits else 'disabled')
        self.findCombo.set(str(len(self.findHits)) + ' found' if self.findHits
                           else ('nothing found' if self.findVar.get().strip() != '' else ''))

    def OnFindPick(self, event=None):
        """a hit is picked from the drop-down: jump to it"""
        index = self.findCombo.current()
        if 0 <= index < len(self.findHits):
            self.findIndex = index
            self.JumpToPath(self.findHits[index][0])

    def OnFindNext(self, event=None):
        """RETURN, F3 or the find button: the next hit, and around at the end"""
        if self.findHits == []:
            return 'break'
        self.findIndex = (self.findIndex + 1) % len(self.findHits)
        self.findCombo.current(self.findIndex)
        self.JumpToPath(self.findHits[self.findIndex][0])
        return 'break'

    def JumpToPath(self, path):
        """show the row a path names: see() opens the folders it sits in and scrolls it into view.
        The focus stays in the find entry, so that RETURN steps on."""
        item = self.itemByPath.get(path, '')
        if item == '':
            return
        self.CancelCellEdit()
        self.tree.see(item)
        self.tree.selection_set(item)
        self.tree.focus(item)
        self.ShowInfo(item)

    def CellBox(self, item):
        """where the value cell of a row is, in the coordinates of the tree"""
        self.tree.see(item)                   #a row that is scrolled away has no box
        box = self.tree.bbox(item, 'value')
        return box if box != '' else None

    def StartCellEdit(self, item):
        """an Entry, or a Combobox for a type with a fixed set of values, over the value cell"""
        self.CancelCellEdit()
        box = self.CellBox(item)
        if box is None:
            return
        (x, y, width, height) = box
        self.selectedItem = item
        vType = self.typeStorage[item]
        value = self.tree.item(item,'values')[0]

        if vType in self.dictionaryTypesT:    #bool and the enums: pick, do not type
            values = tuple(str(entry) for entry in self.dictionaryTypesT[vType])
            self.cellCombo['values'] = values
            self.cellCombo.set(value if value in values else (values[0] if values else ''))
            self.cellCombo.place(x=x, y=y, width=width, height=height)
            self.cellCombo.focus_set()
        else:
            self.editItemVar.set(value)
            self.cellEditor.place(x=x, y=y, width=width, height=height)
            self.cellEditor.focus_set()
            self.cellEditor.select_range(0, tk.END)

    def CancelCellEdit(self, event=None):
        """take the editor away without writing anything"""
        self.selectedItem = ''
        self.cellEditor.place_forget()
        self.cellCombo.place_forget()

    def OnCellCancel(self, event):
        self.CancelCellEdit()
        self.tree.focus_set()

    def OnCellCommit(self, event=None):
        """write the edited value back, if it is one the setting can take"""
        item = self.selectedItem
        if item == '':
            return
        vType = self.typeStorage[item]
        vSize = self.sizeStorage[item]
        if vType in self.dictionaryTypesT:
            valueStr = self.cellCombo.get()
        else:
            valueStr = self.editItemVar.get()

        [isValid, message] = CheckType(valueStr, vType, vSize, self.dictionaryTypesT)
        if isValid:
            #CheckType says the SHAPE is right; the range lives in the type name and only
            #ConvertString2Value knows it (#2597)
            try:
                [_, rangeMessage] = ConvertString2Value(valueStr, vType, vSize,
                                                        self.dictionaryTypesT)
            except (ValueError, SyntaxError) as exception:
                rangeMessage = str(exception)
            if rangeMessage != '':
                (isValid, message) = (False, rangeMessage)

        if not isValid:
            self.cellEditor.unbind('<FocusOut>')    #otherwise OnCellCommit is called twice
            tk.messagebox.showerror("Error", self.ItemPath(item) + ' expects ' + vType + ':\n'
                                    + message + '\npress ESCAPE to keep the original value')
            self.cellEditor.bind('<FocusOut>', self.OnCellCommit)
            return

        self.CancelCellEdit()
        self.PushUndoState()
        self.tree.item(item, values=(valueStr, vType, self.descriptionStorage[item]))
        self.MarkChangedValue(item)
        self.tree.focus_set()
        self.tree.focus(item)
        self.ShowInfo(item)
        self.modifiedDictionary = self.GetDictionary('')
        self.UpdateSettingsStructure()      #only if according flag set in visualizationSettings


def EditDictionaryWithTypeInfo(settingsStructure, exu=None, dictionaryName='edit'):
    """edit dictionaryData and return modified (new) dictionary

    Args:
        settingsStructure: hierarchical settings structure, e.g., SC.visualizationSettings
        exu: exudyn module
        dictionaryName: name displayed in dialog

    Returns:
        returns modified dictionary, which can be used, e.g., for SC.visualizationSettings.SetDictionary(...)
    """

    [root, tkWindow, tkinterAlreadyRunning] = GetTkRootAndNewWindow()
    
    windowHeight = treeEditDefaultHeight
    if treeEditMaxInitialHeight > treeEditDefaultHeight:
        try:
            screen_height = root.winfo_screenheight()
            if screen_height > 1.2*treeEditDefaultHeight:
                windowHeight = int(min(treeEditMaxInitialHeight, 0.85*screen_height))
        except tk.TclError:
            exudyn.Print('WARNING: EditDictionaryWithTypeInfo could not determine the screen'
                         ' size; please report this with your Python version and platform as a'
                         ' github issue')

    tkWindow.geometry(str(treeEditDefaultWidth)+'x'+str(windowHeight))

    guiSC = GetRendererSystemContainer()
    updateOnChange = False
    topmost = True
    alphaTransparency = 1 #<1 means transparency
    treeOpen = True
    if guiSC is not None:
        updateOnChange = guiSC.visualizationSettings.dialogs.multiThreadedDialogs
        topmost = guiSC.visualizationSettings.dialogs.alwaysTopmost
        if guiSC.visualizationSettings.dialogs.alphaTransparency <= 1:
            alphaTransparency = guiSC.visualizationSettings.dialogs.alphaTransparency
        treeOpen = guiSC.visualizationSettings.dialogs.openTreeView

    [systemScaling, fontFactor] = DialogScaling(root)

    tkWindow.lift() #brings it to front of other; not always "strong" enough
    if topmost:
        root.attributes("-topmost", True) #puts window topmost (permanent)
    if alphaTransparency <= 1:
        tkWindow.attributes("-alpha", alphaTransparency) 
        
    tkWindow.title(dictionaryName)
    tkWindow.focus_force() #window has focus

    textHeight = TkTextHeight(systemScaling)

    #no effect:
    # defaultFont = tkFont.Font(root=root, family = "TkDefaultFont")
    # defaultFont.configure(size=treeviewDefaultFontSize*fontFactor)
        
    style = ttk.Style(tkWindow)
    style.configure('Treeview', rowheight=textHeight) 
    
    style.configure("Treeview.Heading", font=(None, int(treeviewDefaultFontSize*fontFactor) ) )
    style.configure("Treeview", font=(None, int(treeviewDefaultFontSize*fontFactor) ))
    
    
    comboListsT = GetComboBoxListsDict(exu)
    ex=TkinterEditDictionaryWithTypeInfo(parent=tkWindow, settingsStructure=settingsStructure, dictionaryTypesT=comboListsT, 
                                         updateOnChange=updateOnChange, treeOpen=treeOpen, textHeight = textHeight,
                                         systemScaling = systemScaling, fontFactor = fontFactor)
    ex.pack(fill="both", expand=True)

    if not tkinterAlreadyRunning:
        tk.mainloop()
    else:
        root.wait_window(tkWindow)
    
    settingsStructure.SetDictionary(ex.modifiedDictionary)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#hierarchical lists, without type info:
class TkinterEditDictionary(tk.Frame):
    def __init__(self, parent, dictionaryData, dictionaryIsEditable=True, textHeight = 15):
        tk.Frame.__init__(self, parent)
        
        self.parent = parent
        self.dictionaryIsEditable = dictionaryIsEditable
        self.longestColumn = 20 #min value
        self.textHeight = textHeight

        #create treeview:
        self.tree = ttk.Treeview(self, columns=("value"), height=textHeight)
        self.AddNodeFromDictionary(value=dictionaryData, parentNode="")

        self.vsb = ttk.Scrollbar(self, orient="vertical", command=self.tree.yview, )
        self.hsb = ttk.Scrollbar(self, orient="horizontal", command=self.tree.xview)
        self.tree.configure(yscrollcommand=self.vsb.set, xscrollcommand=self.hsb.set)

        self.vsb.pack(side="right", fill="y")
        self.tree.pack(side="top", fill="both", expand=True)

        self.hsb.pack(side="bottom",anchor='w', fill="x",padx=10,pady=5)

        self.tree.bind('<<TreeviewSelect>>', self.TreeviewSelect) #selection changed
        self.tree.bind("<Double-1>", self.OnTreeDoubleClick) #an item has been selected for change
        self.tree.bind("<Return>", self.OnTreeDoubleClick) #an item has been selected for change
        self.tree.bind("<Escape>", self.OnQuit) 
        self.tree.bind("q", self.OnQuit) 
        
        #create the entry field for editing the treeview value

        if True:
            self.selectedItem = '' #will change to valid item in order to change value
            self.editItemName = tk.StringVar()
            self.editItemVar = tk.StringVar()
    
            self.editName = tk.Label(self, textvariable=self.editItemName)#, width=10)
            self.editName.pack(side="left", fill="both", expand=True)
    
            self.editItem = tk.Entry(self, textvariable=self.editItemVar)#, width=40)
    #        self.editItem.grid(row=1, column=1)
            self.editItem.pack(side="right", fill="both", expand=True)
            self.editItem.bind('<Return>', self.OnEditEntryItem)
            self.editItem.bind('<FocusOut>', self.OnEditEntryItem)
            self.editItem.bind('<Escape>', self.OnEditEscapeItem)

        first = self.tree.get_children('')[0]
        self.tree.focus_set()
        self.tree.focus(first)
        self.tree.selection_set((first))
        self.tree.heading('#0', text='variable', anchor='w')
        self.tree.column("#0",minwidth=300, stretch=True)        
        self.tree.heading('#1', text='value', anchor='w')
        self.tree.column("#1",anchor='w', #width=200,
                         minwidth=self.longestColumn*12,
                         stretch=True)
        self.hsb.config(command = self.tree.xview)

        self.modifiedDictionary = self.GetDictionary('')

    def IsItemIndex(self, var):
        return (isinstance(var, exudyn.NodeIndex) or
                isinstance(var, exudyn.ObjectIndex) or
                isinstance(var, exudyn.MarkerIndex) or
                isinstance(var, exudyn.LoadIndex) or
                isinstance(var, exudyn.SensorIndex))
    #create dictionary
    #this is used e.g. for mouse right-button dialog
    def AddNodeFromDictionary(self, value, parentNode="", key=None):
        if key is None:
            id = ""
        else:
            if key != 'TODO':
                id = self.tree.insert(parentNode, "end", text=key)
            else:
                id = self.tree.insert(parentNode, "end", text='<unavailable>')

        if isinstance(value, dict):
            self.tree.item(id, open=True)
            for (key, value) in value.items():
                self.AddNodeFromDictionary(value, id, key)
        else:
            if isinstance(value, bool) : #bool first, bool is also int
                self.tree.item(id, values=(str(value)))
            elif isinstance(value, int) or isinstance(value, float):
                self.tree.item(id, values=(value))
            elif isinstance(value, np.ndarray):
                if (value.size < 5000):
                    valueList = value.tolist()
                    valueListStr = str(valueList)
                    self.longestColumn = max(self.longestColumn,len(valueListStr))
                    valueStr = valueListStr.replace(' ','\\ ').replace(',','\\,')
                    self.tree.item(id, values=(valueStr))
                else:
                    self.tree.item(id, values=('<unavailable>'))
            elif isinstance(value, list):
                if np.array(value).size < 5000:
                    valueListStr = str(value)
                    self.longestColumn = max(self.longestColumn,len(valueListStr))
                    valueStr = valueListStr.replace(' ','\\ ').replace(',','\\,')
                    self.tree.item(id, values=(valueStr))
                else:
                    self.tree.item(id, values=('<unavailable>'))
            elif isinstance(value, str) and value!='Get graphics data to be implemented':
                self.longestColumn = max(self.longestColumn,len(value))
                self.tree.item(id, values=(value.replace(' ','\\ ')))
            elif self.IsItemIndex(value):
                self.tree.item(id, values=(int(value)))
            else:
                self.tree.item(id, values=('<unavailable>'))

    def GetDictionary(self, item):
        d=dict()
        kids = self.tree.get_children(item)
        for i in kids:
            nchilds = len(self.tree.get_children(i))
            if nchilds == 0:
                d.update({self.tree.item(i,'text'): self.tree.item(i,'values')[0]})
            else:
                d.update({self.tree.item(i,'text'): self.GetDictionary(i)})
        return d

    def OnTreeDoubleClick(self,event):
        item = self.tree.selection()[0]
        nchilds = len(self.tree.get_children(item))

        if nchilds == 0:
            self.editItemVar.set(self.tree.item(item,'values')[0])
            
            s = self.tree.item(item,'text')
            i=0
            pItem=self.tree.parent(item)
            while i < 3 and pItem != '':
                i+=1
                s = self.tree.item(pItem,'text') + '.' + s
                pItem=self.tree.parent(pItem)
                
                
            self.editItemName.set(s)
            self.selectedItem = item #now item can be modified
            self.editItem.focus_set()
        else: #move to next item
            next = self.tree.get_children(item)[0]
            if next != '':
                self.tree.selection_set((next))
                self.tree.focus(next)
                self.tree.see(next)
            

    def OnQuit(self,event): #new selection --> nothing to edit for now
        self.parent.destroy()
        
    def TreeviewSelect(self,event): #new selection --> nothing to edit for now
        self.selectedItem = '' #now item can be modified
        self.editItemVar.set('')
        self.editItemName.set('')
        
        
    def OnEditEntryItem(self,event):
        if self.selectedItem != '':
            if self.dictionaryIsEditable:
                valueStr = self.editItemVar.get()
                valueStr = str(valueStr).replace(' ','\\ ')
                self.tree.item(self.selectedItem, values=(valueStr))
                currentItem = self.selectedItem
                self.selectedItem = '' #now item can be modified
                self.editItemVar.set('')
                self.editItemName.set('')
                self.tree.focus_set()
    
                #as return is pressed, move to next item
                next = self.tree.next(currentItem)
                if next != '':
                    self.tree.selection_set((next))
                    self.tree.focus(next)
                    self.tree.see(next)
                else:
                    par = self.tree.parent(currentItem)
                    if par != '' and self.tree.next(par) != '':
                        next = self.tree.next(par)
                        self.tree.selection_set(next)
                        self.tree.focus(next)
                        self.tree.see(next)
                self.modifiedDictionary = self.GetDictionary('') #update stored dictionary

    def OnEditEscapeItem(self,event):
        if self.selectedItem != '':
            valueStr = self.tree.item(self.selectedItem,'values')[0]
            valueStr = str(valueStr).replace(' ','\\ ')
            self.editItemVar.set(valueStr)
            self.selectedItem = '' #now item can be modified
#            self.editItemVar.set('')
#            self.editItemName.set('')
            self.tree.focus_set()
    

#edit dictionaryData and return modified (new) dictionary
def EditDictionary(dictionaryData, dictionaryIsEditable=True, dialogName=''):
    [root, tkWindow, tkinterAlreadyRunning] = GetTkRootAndNewWindow()

    tkWindow.geometry(str(dialogDefaultWidth)+'x'+str(dialogDefaultHeight))

    guiSC = GetRendererSystemContainer()
    topmost = True
    alphaTransparency = 1 #<1 means transparency
    if guiSC is not None:
        topmost = guiSC.visualizationSettings.dialogs.alwaysTopmost
        if guiSC.visualizationSettings.dialogs.alphaTransparency <= 1:
            alphaTransparency = guiSC.visualizationSettings.dialogs.alphaTransparency

    [systemScaling, fontFactor] = DialogScaling(root)

    tkWindow.lift() #brings it to front of other; not always "strong" enough
    if topmost:
        tkWindow.attributes("-topmost", True) #puts window topmost (permanent)
    if alphaTransparency:
        tkWindow.attributes("-alpha", alphaTransparency) 
    

    textHeight = TkTextHeight(systemScaling)

    tkWindow.title(dialogName)
    tkWindow.focus_force() #window has focus

    style = ttk.Style(tkWindow)
    style.configure('Treeview', rowheight=textHeight) 
    #it seems that the font size should not be changed (what is done due to scaling internally ...)

    style.configure("Treeview.Heading", font=(None, int(treeviewDefaultFontSize*fontFactor) ) )
    style.configure("Treeview", font=(None, int(treeviewDefaultFontSize*fontFactor) ) )

    #this has no effect style.configure("Vertical.TScrollbar", width=4)

    ex=TkinterEditDictionary(tkWindow, dictionaryData, dictionaryIsEditable, textHeight)
    ex.pack(fill="both", expand=True)

    if not tkinterAlreadyRunning:
        tk.mainloop() #run second main loop? will crash
    else:
        root.wait_window(tkWindow)
    
    if dictionaryIsEditable:
        return ex.modifiedDictionary
    else:
        return {}


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#THE DIALOGS THE RENDERER OPENS (revision2026b step RG6.2.1, #2595)
#
#They were written as Python inside src/Main/rendererPythonInterface.cpp - 220 of its 775 lines were
#raw string literals holding tkinter code, where no syntax check, no ruff and no import test ever
#saw them. Each one is a function here now, and the C++ calls it. The window setup that the C++
#assembled by string concatenation from visualizationSettings.dialogs is ApplyDialogWindowSettings.

def ApplyDialogWindowSettings(tkWindow, alwaysTopmost=None, alphaTransparency=None):
    """Apply what visualizationSettings.dialogs says about a dialog window.

    Args:
        tkWindow: the window to configure
        alwaysTopmost: None takes dialogs.alwaysTopmost; True or False overrides it
        alphaTransparency: None takes dialogs.alphaTransparency; a float overrides it

    Returns:
        None
    """
    guiSC = GetRendererSystemContainer()
    if guiSC is not None:
        if alwaysTopmost is None:
            alwaysTopmost = guiSC.visualizationSettings.dialogs.alwaysTopmost
        if alphaTransparency is None:
            alphaTransparency = guiSC.visualizationSettings.dialogs.alphaTransparency

    if alwaysTopmost:
        tkWindow.attributes('-topmost', True)   #permanent, otherwise it hides behind the renderer
    if alphaTransparency is not None and alphaTransparency < 1:
        tkWindow.attributes('-alpha', alphaTransparency)


#the keyboard and mouse commands of the renderer, from the ONE table that also feeds the
#tables of docs/manual/GUI.md through tools/generators/keyBindingsEmitter.py. This text
#was a third copy of them, and it had drifted (revision2026b step RG6.2.6, #2591).
rendererHelpText = RendererHelpText()


def ShowHelpDialog():
    """The keyboard and mouse commands of the renderer, in a read-only window; opened with H in
    the render window.

    Returns:
        None
    """
    [root, tkWindow, tkRuns] = GetTkRootAndNewWindow()
    ApplyDialogWindowSettings(tkWindow)

    tkWindow.title("Help on keyboard commands and mouse")
    tkWindow.lift()                          #window has focus
    tkWindow.bind("<Escape>", lambda event: tkWindow.destroy())
    tkWindow.focus_force()

    scrollW = tk.Scrollbar(tkWindow)
    #resize grid columns/rows if window is resized:
    tkWindow.grid_columnconfigure(0, weight=1)
    tkWindow.grid_rowconfigure(0, weight=1)

    textW = tk.Text(tkWindow, height=30, width=90, background='gray98')
    textW.focus_set()
    textW.grid(row=0, column=0, padx=10, pady=10, sticky=tk.NSEW)
    scrollW.grid(row=0, column=1, pady=10, sticky=tk.NSEW)
    scrollW.config(command=textW.yview)
    textW.config(yscrollcommand=scrollW.set)

    textW.insert(tk.END, rendererHelpText)
    textW.configure(state='disabled')        #unable to edit

    if tkRuns:
        root.wait_window(tkWindow)
    else:
        tk.mainloop()


#the examples shown under the command input; they are what people ask for most often while a
#simulation runs
pythonCommandExamples = ('helpful examples:\n'
                         'show overall info of mbs:\n'
                         'print(mbs)\n'
                         '#change current dynamic solver end time:\n'
                         "mbs.sys['dynamicSolver'].it.endTime=10 \n"
                         '#change verbose mode of dynamic solver:\n'
                         "mbs.sys['dynamicSolver'].output.verboseMode=1\n"
                         '#stop file writing:\n'
                         "mbs.sys['dynamicSolver'].output.writeToSolutionFile=False\n"
                         '#print values of sensor 0:\n'
                         'print(mbs.GetSensorValues(0))\n'
                         '#pause after each step:\n'
                         'simulationSettings.pauseAfterEachStep=True\n'
                         '\n#==>BUT changing simulationSettings is dangerous!')


def ShowPythonCommandDialog():
    """A window that executes a Python command in the global scope of the running model; opened
    with X in the render window. CTRL+RETURN runs what is in the text area.

    Returns:
        None
    """
    import traceback                                                            # noqa: PLC0415
    from tkinter import scrolledtext                                            # noqa: PLC0415

    [root, tkWindow, tkRuns] = GetTkRootAndNewWindow()
    tkWindow.title("Exudyn command window")
    ApplyDialogWindowSettings(tkWindow)

    #resize grid columns/rows if window is resized:
    tkWindow.grid_columnconfigure(0, weight=1)
    tkWindow.grid_rowconfigure(1, weight=1)

    description = ('Enter Python command which operates in global scope of you Python model;\n'
                   'Evaluate or CHANGE your current model (parameters) during simulation;\n'
                   'Press CRTL+RETURN to execute, escape to close:')

    label = tk.Label(tkWindow, text=description, justify=tk.LEFT,
                     relief=tk.SUNKEN, background='gray94')
    label.grid(row=0, column=0, padx=15, pady=(15, 0), sticky='W')

    textArea = scrolledtext.ScrolledText(tkWindow, wrap=tk.WORD, width=60, height=8)
    #configure tab size:
    font = tk.font.Font(font=textArea['font'])
    textArea.config(tabs=font.measure(' '*4))   #in pixels

    def OnRunCode(event):
        commandString = textArea.get('1.0', tk.END)
        print('command window execute:\n', commandString.strip(), sep='')  #printout the command
        print('output:')

        if commandString.strip() == '':      #empty command causes exception
            return None
        commandString = commandString.replace('\t', ' '*4)  #tabs may cause problems

        try:
            exec(commandString, globals(), locals())        # noqa: S102 - this IS the feature
        except Exception:                    #whatever the user typed; it must not kill the dialog
            print("Execution of command failed; error:")
            for line in traceback.format_exc().split('\n'):
                line = line.replace('  File "<string>", ', '')
                if ('File "' not in line) and ('exec(commandString' not in line):
                    print(line)

        if event is not None:
            return "break"                   #prevent from passing Return key to text ...
        return None

    def OnClose(event):
        tkWindow.destroy()

    textArea.grid(row=1, column=0, pady=15, padx=10, sticky=tk.NSEW)
    textArea.bind('<Control-Return>', OnRunCode)
    textArea.bind('<Escape>', OnClose)
    tkWindow.bind('<Escape>', OnClose)

    frame = tk.Frame(tkWindow)
    runButton = tk.Button(frame, text="    Run code    ", command=lambda: OnRunCode(None))
    closeButton = tk.Button(frame, text="    Close    ", command=lambda: OnClose(None))

    frame.grid(row=2, column=0, padx=15, pady=(0, 15), sticky='', columnspan=3)
    runButton.grid(row=0, column=0, padx=80, sticky='')
    closeButton.grid(row=0, column=1, padx=80, sticky='')

    textExample = scrolledtext.ScrolledText(tkWindow, wrap=tk.WORD, width=60,
                                            height=pythonCommandExamples.count('\n')+1,
                                            background='gray94')
    textExample.grid(row=3, column=0, padx=15, pady=(0, 15), sticky=tk.NSEW)
    textExample.insert(tk.END, pythonCommandExamples)
    textExample.configure(state='disabled')  #unable to edit

    textArea.focus_set()                     #placing cursor in text area
    tkWindow.focus_force()

    if tkRuns:
        root.wait_window(tkWindow)
    else:
        tk.mainloop()


def ShowVisualizationSettingsDialog():
    """The settings tree of the renderer; opened with V in the render window.

    Returns:
        None
    """
    guiSC = GetRendererSystemContainer()
    if guiSC is None:
        exudyn.Print('ERROR: ShowRightMouseSelectionDialog: problems with the'
                     ' SystemContainer, probably not attached to the renderer yet')
        return

    EditDictionaryWithTypeInfo(guiSC.visualizationSettings, exudyn, 'Visualization Settings')


def ShowRightMouseSelectionDialog():
    """The properties of the item the right mouse button selected, read-only; the renderer has
    put them into exudyn.sys['currentRendererSelectionDict'] before calling this.

    Returns:
        None
    """
    try:
        d = exudyn.sys['currentRendererSelectionDict']
        EditDictionary(d, False, dialogName='properties of <' + d['name'] + '>')
    except Exception:                        #a dict without 'name', or no dict at all
        exudyn.Print('ERROR: ShowRightMouseSelectionDialog: showing the dictionary failed')


def AskQuitDialog():
    """Ask whether a long running simulation really shall be stopped; the answer goes back to the
    renderer in exudyn.sys['quitResponse'], as 2 (do not quit) or 3 (quit).

    Returns:
        None
    """
    response = False                         #if the user just shuts the window

    [root, tkWindow, tkRuns] = GetTkRootAndNewWindow()
    #topmost unconditionally: this question is the reason the renderer is waiting
    ApplyDialogWindowSettings(tkWindow, alwaysTopmost=True, alphaTransparency=1)
    tkWindow.bind("<Escape>", lambda event: tkWindow.destroy())
    tkWindow.title("WARNING - long running simulation!")

    def QuitResponse(clickResponse):
        nonlocal response
        response = clickResponse
        tkWindow.destroy()

    label = tk.Label(tkWindow, text="Do you really want to stop simulation and close renderer?",
                     justify=tk.LEFT)
    yesButton = tk.Button(tkWindow, text="        Yes        ",
                          command=lambda: QuitResponse(True))
    noButton = tk.Button(tkWindow, text="        No        ",
                         command=lambda: QuitResponse(False))

    label.grid(row=0, column=0, pady=(20, 0), padx=50, columnspan=5)
    yesButton.grid(row=1, column=1, pady=20)
    noButton.grid(row=1, column=3, pady=20)

    tkWindow.focus_force()

    if tkRuns:
        root.wait_window(tkWindow)
    else:
        tk.mainloop()

    exudyn.sys['quitResponse'] = response + 2   #2=do not quit, 3=quit
