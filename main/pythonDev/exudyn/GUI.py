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
import tkinter.font as tkFont
import numpy as np #for array checks
from numpy import float32
import ast #for ast.literal_eval
import sys
import exudyn

useRenderWindowDisplayScaling = True #using this, scaling will change with render window

treeviewDefaultFontSize = 9 #this is then scaled; but it could be changed to make fonts smaller
textHeightFactor = 1.45 #this is the factor between font size and text height; larger values leading to more space between lines

treeEditDefaultWidth = 1024     #unscaled width of e.g. visualizationSettings
treeEditDefaultHeight = 800     #unscaled height of e.g. visualizationSettings
treeEditMaxInitialHeight = 1440 #larger height, if screen resolution admits
dialogDefaultWidth = 800        #unscaled width of e.g. right mouse edit
dialogDefaultHeight = 600       #unscaled height of e.g. right mouse edit
treeEditOpenItems = ['bodies','connectors','nodes','general'] #these items are opened at the beginning

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
    except: 
        pass
    return None

#**function: get new or current root and new window app; return list of [tkRoot, tkWindow, tkRuns]
def GetTkRootAndNewWindow():
    if tk._default_root == None:
        root = tk.Tk()
        tkWindow = root
        tkRuns = False
    else:
        root = tk._default_root
        tkWindow = tk.Toplevel(root)
        tkRuns = True
    return [root, tkWindow, tkRuns]

#**function: this function returns True, if tkinter has already a root window (which is assumed to have already a mainloop running)
def TkRootExists():
    return (tk._default_root != None)



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
                rs = guiSC.GetRenderState()
                #print('return render state')
                return rs['displayScaling']
        
        return 1

    except: 
        #print('except!')
        return 1

#return either exudyn or tkinter scaling, unified approach
def GetGUIContentScaling(root):
    try:
        if useRenderWindowDisplayScaling: #would also work under linux
            #print('exudyn scaling      =',GetExudynDisplayScaling())
            #print('tkinter scaling(OLD)=',root.tk.call('tk', 'scaling'))
            s = 1.4*GetExudynDisplayScaling() #gives similar size as other programs; factor 1.4 is empirical
            root.tk.call('tk', 'scaling', s) #needed to update font size internally ...
            #print('tkinter scaling(NEW)=',root.tk.call('tk', 'scaling'))
            return s
        else:
            return root.tk.call('tk', 'scaling') #obtains current scaling?
    except:
        return 1
    
#create dictionaries for lists in combo box: bool, OutputVariableType, ...
def GetComboBoxListsDict(exu = None):
    d=dict()  #as string
    dT=dict() #as type
    
    if exu != None: #exudyn loaded
        listOfTypes = []
        listOfTypesT = []
        dTypes = exu.OutputVariableType.__members__
        for i in dTypes: 
            listOfTypes+=[str(dTypes[i])]
            listOfTypesT+=[dTypes[i]]
        d['OutputVariableType'] = listOfTypes
        dT['OutputVariableType'] = listOfTypesT

        
        listOfTypes = []
        listOfTypesT = []
        dTypes = exu.LinearSolverType.__members__
        for i in dTypes: 
            listOfTypes+=[str(dTypes[i])]
            listOfTypesT+=[dTypes[i]]
        d['LinearSolverType'] = listOfTypes
        dT['LinearSolverType'] = listOfTypesT

        listOfTypes = []
        listOfTypesT = []
        dTypes = exu.ItemType.__members__
        for i in dTypes: 
            listOfTypes+=[str(dTypes[i])]
            listOfTypesT+=[dTypes[i]]
        d['ItemType'] = listOfTypes
        dT['ItemType'] = listOfTypesT

    else:
        print('WARNING: exudyn not loaded as "exu"')

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

    #print("Error in ConvertString2Value: unknown type",vType, "value=", value)
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
def CheckType(valueStr, vType, vSize):
#    print('str=',valueStr)
    
    validFileNameChar = " `'{}()%&-@#$~!_^./\\"
    
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
                #print('(x in validFileNameChar)=',(x in validFileNameChar),',x.isalpha()=',x.isalpha(),',x.isnumeric()=',x.isnumeric())
                return [False, 'invalid character in file name: may only be A-Z, a-z, 0-9, "'+validFileNameChar +'"']
        return [True, '']

    if vType == 'String':
        return [True, '']
    if vType == 'float':
        #print('float=',IsFloat(valueStr))
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
        #s = 'global x\nx='+str(valueStr) + '\nprint(x)'
        s = 'locx='+str(valueStr)# + '\nprint(x)'
        mylocals={'locx':[]}
        exec(s,globals(),mylocals)
        x=mylocals['locx']
        #print('mylocals=',mylocals)
    except:
        #print("entered text does not comply with the value's type")
        return [False, 'invalid array or matrix: check brackets and types']

    #print('x=',x)
    
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

#this class gets a dictionary with type information structure in, but a plain dictionary with types (int, float, string, list, ...) out
#settingsStructure: contains hierarchical settings structure with function GetDictionaryWithTypeInfo() to obtain dictionary for editing
#dictionaryTypes: contains a dictionary with the available types, e.g. bool, etc.
#updateOnChange: every change is directly applied to the settingsStructure and redraw is signaled in stored renderer
class TkinterEditDictionaryWithTypeInfo(tk.Frame):
    def __init__(self, parent, settingsStructure, dictionaryTypesT, updateOnChange=False, treeOpen=False, textHeight = 15):
        tk.Frame.__init__(self, parent)
        
        self.parentFrame = parent #parent frame stored for member functions
        self.settingsStructure = settingsStructure
        self.dictionaryTypesT = dictionaryTypesT #as type
        self.updateOnChange = updateOnChange
        self.treeOpen = treeOpen
        self.textHeight = textHeight

        self.dictionaryData = settingsStructure.GetDictionaryWithTypeInfo()

        #additional storage for type, size, etc.
        self.typeStorage = dict() #dictionary is stored as {'ID1': 'type1', 'ID2': 'type2', ...}
        self.sizeStorage = dict()
        self.descriptionStorage = dict()

        #create treeview:
        self.tree = ttk.Treeview(self, columns=("value","description"), selectmode='browse', height=self.textHeight)
        self.vertivalScrollbar = ttk.Scrollbar(self, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=self.vertivalScrollbar.set)

        self.tree.grid(row=0, column=0, columnspan=3, sticky="nsew")
        self.vertivalScrollbar.grid(row=0, column=3, sticky='nse')
        self.vertivalScrollbar.configure(command=self.tree.yview)


        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0,weight=1)
        self.grid_columnconfigure(1,weight=2)
        self.grid_columnconfigure(2,weight=2)
        self.grid_columnconfigure(3,weight=0)
        
        self.tree.heading("#0",text="Name",anchor=tk.W)
        self.tree.heading("value",text="Value",anchor=tk.W)
        self.tree.heading("description",text="Description (press H to show)",anchor=tk.W)


        self.AddNodeFromDictionaryWithTypeInfo(value=self.dictionaryData, parentNode="")
        self.tree.bind('<<TreeviewSelect>>', self.TreeviewSelect) #selection changed
        self.tree.bind("<Double-1>", self.OnTreeDoubleClick) #an item has been selected for change
        self.tree.bind("<Return>", self.OnTreeEdit) #an item has been selected for change
        self.tree.bind("<ButtonRelease-1>", self.OnTreeEdit) #an item has been selected for change #1357
        self.tree.bind("h", self.OnTreeHelp) #show description for selected item
        self.tree.bind("<Escape>", self.OnQuit) 
        self.tree.bind("q", self.OnQuit) 
        
        #+++++++++++++++++++++++++++++++++++++++++
        #create the entry field for editing the treeview value
        self.selectedItem = '' #will change to valid item in order to change value
        self.editItemName = tk.StringVar()
        self.editItemVar = tk.StringVar()

        #self.editName = tk.Label(self, textvariable=self.editItemName) #label expands itself
        self.editName = tk.Entry(self, textvariable=self.editItemName) #label expands itself
        self.editName.configure(background='gray95', borderwidth=0)
        self.editName.grid(row=1, column=0, columnspan=1, sticky=tk.N+tk.E+tk.S+tk.W)

        self.editItem = tk.Entry(self, textvariable=self.editItemVar)
        self.editItem.grid(row=1, column=1, columnspan=2, sticky=tk.N+tk.E+tk.S+tk.W)

            
        self.editItem.bind('<Return>', self.OnEditEntryItem)
        self.editItem.bind('<FocusOut>', self.OnEditEntryItem)
        self.editItem.bind('<Escape>', self.OnEscapeEntryItem)

        #+++++++++++++++++++++++++++++++++++++++++
        #combo box for special items (bool, enums, ...)
        self.comboItem = ttk.Combobox(self, values=['True','False'] )
        self.comboItem.current(0)
        self.comboItem.grid(row=1, column=1, columnspan=2, sticky=tk.N+tk.E+tk.S+tk.W)
        self.comboItem.lower(self.editItem)

        #self.comboItem.bind('<Return>', self.OnEditComboItem)
        self.comboItem.bind('<<ComboboxSelected>>', self.OnEditComboItem)
        self.comboItem.bind('<Escape>', self.OnEscapeComboItem)

        #+++++++++++++++++++++++++++++++++++++++++
        #pre-select item
        first = self.tree.get_children('')[0]
        self.tree.focus_set()
        self.tree.focus(first)
        self.tree.selection_set((first))
        
        self.modifiedDictionary = self.GetDictionary('')

    #create treeview from dictionary with type info
    def AddNodeFromDictionaryWithTypeInfo(self, value, parentNode="", key=None, level=0):
        if key is None:
            id = ""
        else:
            id = self.tree.insert(parentNode, "end", text=key)
        #print("key =", key)
        isOpen = self.treeOpen and (level<=1)

        if isinstance(value, dict):
            itemIsOpen = isOpen
            if ('itemIdentifier' not in value) and (key in treeEditOpenItems): 
                itemIsOpen = True
                
            self.tree.item(id, open=itemIsOpen)
            if 'itemIdentifier' in value: #is a value with types:
                #strValue = ConvertValue2String(value['value'], value['type'], value['size'])
                #print('v=',value['value'],',t=',str(value['type']),'s=', value['size'])
                #strValue = str(value['value'])
                strValue = ConvertValue2String(value['value'], value['type'], value['size'])
                
                self.tree.item(id, values=(strValue, value['description']))
                #store additional data in dictionaries (could also be tuples ...)
                self.typeStorage[id] = value['type']
                self.sizeStorage[id] = value['size']
                self.descriptionStorage[id] = value['description']
            else: #must be another dictionary:
                for (key, value) in value.items():
                    self.AddNodeFromDictionaryWithTypeInfo(value, id, key, level=level+1)
        else:
            print("Error in AddNodeFromDictionaryWithTypeInfo with item:", value, ", parent=", parentNode, ", key=", key)
            #self.tree.item(id, values=(value))

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
        #print(kids)
        for i in kids:
            nchilds = len(self.tree.get_children(i))
            if nchilds == 0:
                if len(self.tree.item(i,'values')) != 0:
                    valStr = self.tree.item(i,'values')[0]
                    [val, errorMsg] = ConvertString2Value(valStr, str(self.typeStorage[i]), self.sizeStorage[i], self.dictionaryTypesT)
                    if errorMsg == '':
                        d.update({self.tree.item(i,'text'): val})
                    else:
                        print('item '+ str(self.tree.item(i,'text')) + ' has illegal value "'+valStr + '": '+errorMsg)
                else:
                    d.update({self.tree.item(i,'text'): ''})
            else:
                d.update({self.tree.item(i,'text'): self.GetDictionary(i)})
        return d

    def OnTreeHelp(self,event):
        item = self.tree.selection()[0]
        if item in self.descriptionStorage:
            d = self.descriptionStorage[item]
            s = self.tree.item(item,'text')
            tk.messagebox.showinfo(s, d)

    def OnTreeDoubleClick(self,event):
        #print('tree double clicked')
        self.OnTreeEditOrDoubleClick(event, True)
        
    def OnTreeEdit(self,event):
        self.OnTreeEditOrDoubleClick(event)
        
    def OnTreeEditOrDoubleClick(self,event,doubleClick=False):
        item = self.tree.selection()[0]
        nchilds = len(self.tree.get_children(item))
        #print('tree edit: item=',item, ', childs=', nchilds)

        #only edit items which have no subitems (no folders!)
        if nchilds == 0:
            
            selectedType=self.typeStorage[item]
            #+++++++++++++++++++                
            if selectedType=='bool' and doubleClick: #just toggle value #1354
                value=self.tree.item(item,'values')[0]
                if value=='True': value='False'
                else: value='True'

                self.tree.item(item, values=(value, self.descriptionStorage[item]))
                self.modifiedDictionary = self.GetDictionary('') #update stored dictionary
                self.UpdateSettingsStructure() #only if according flag set in visualizationSettings
                return

            #+++++++++++++++++++                
            s = self.tree.item(item,'text')
            #print('text=',s)
            i=0
            pItem=self.tree.parent(item)
            while i < 10 and pItem != '': #limits to 10 levels ...
                i+=1
                s = self.tree.item(pItem,'text') + '.' + s
                pItem=self.tree.parent(pItem)
            
            self.editItemName.set(s)
            self.selectedItem = item #now item can be modified
            
            if selectedType in self.dictionaryTypesT:
                #print('type =',selectedType)
                
                value = self.tree.item(item,'values')[0]
                valueTypes = []
                for iType in self.dictionaryTypesT[selectedType]:
                    valueTypes += [str(iType)]
                valueTypes = tuple(valueTypes)

                self.comboItem['values'] = valueTypes
                #print(self.comboItem['values'])
                #print(type(self.comboItem['values']))
                
                #find current index:
                i = 0 #default value
                if value in valueTypes:
                    i = valueTypes.index(value)
                self.comboItem.current(i)
                
                self.comboItem.focus_set()
                self.editItem.lower(self.comboItem)
                
            else:
                #print('general value')
                self.editItemVar.set(self.tree.item(item,'values')[0])
                self.editItem.focus_set()
                self.comboItem.lower(self.editItem)
        else: #folders (may be opened/closed)
            openState = self.tree.item(item, 'open')
            s = self.tree.item(item,'text')
            #print('  openState=', openState, ', text=',s)
            if openState and (s not in treeEditOpenItems):
                treeEditOpenItems.append(s)
            elif not openState and (s in treeEditOpenItems):
                treeEditOpenItems.remove(s)
        

    def OnQuit(self,event): #new selection --> nothing to edit for now
        self.parentFrame.destroy()
        
    def TreeviewSelect(self,event): #new selection --> nothing to edit for now
        #print('select')
        self.editItemVar.set('')
        self.editItemName.set('')
        self.comboItem.lower(self.editItem) #bring entry item to front
        
    #if visualizationSettings are accordingly, the renderer will obtain an update signal
    def UpdateSettingsStructure(self):
        #++++++++++++++++++++++++++++++++++++++++++++++++
        #update changes
        self.settingsStructure.SetDictionary(self.modifiedDictionary)  #this may also change dialogs.multiThreadedDialogs itself
        if 'currentRendererSystemContainer' in exudyn.sys:
            guiSC = exudyn.sys['currentRendererSystemContainer']
            if guiSC != 0:
                if guiSC.visualizationSettings.dialogs.multiThreadedDialogs:
                    guiSC.SendRedrawSignal()
                    exudyn.DoRendererIdleTasks()
        #++++++++++++++++++++++++++++++++++++++++++++++++

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++        
    #user has now edited the Entry dialog and valid updates are copied to treeview
    def OnEditEntryItem(self,event):
        if self.selectedItem != '':
            valueStr = self.editItemVar.get()
            [rv, errorMSG] = CheckType(valueStr, self.typeStorage[self.selectedItem], self.sizeStorage[self.selectedItem])
            if rv:
                self.tree.item(self.selectedItem, values=(self.editItemVar.get(), self.descriptionStorage[self.selectedItem]))
                currentItem = self.selectedItem
                self.selectedItem = '' #modification finished
                self.editItemVar.set('')
                self.editItemName.set('')
                self.tree.focus_set()
                self.tree.focus(currentItem)
            else:
                self.editItem.unbind('<FocusOut>') #otherwise OnEditEntryItem called twice
                tk.messagebox.showerror("Error", errorMSG+'\npress ESCAPE to reset to original values')
                self.editItem.bind('<FocusOut>', self.OnEditEntryItem) #bind again
            
            self.modifiedDictionary = self.GetDictionary('') #update stored dictionary
            self.UpdateSettingsStructure() #only if according flag set in visualizationSettings

    def OnEscapeEntryItem(self,event):
        #print('escape')
        if self.selectedItem != '':
            self.editItemVar.set(self.tree.item(self.selectedItem,'values')[0])
            self.selectedItem = '' #now item can be modified
#            self.editItemVar.set('')
#            self.editItemName.set('')
            self.tree.focus_set()

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++        
    #user has now edited the Entry dialog and valid updates are copied to treeview
    def OnEditComboItem(self,event):
        if self.selectedItem != '':
            
            value = self.comboItem.current()
            valueStr = self.comboItem['values'][value]
            
            self.tree.item(self.selectedItem, values=(valueStr, self.descriptionStorage[self.selectedItem]))
            currentItem = self.selectedItem
            self.selectedItem = '' #modification finished
            self.editItemVar.set('')
            self.editItemName.set('')
            self.comboItem.lower(self.editItem) #bring entry item to front
            
            self.tree.focus_set()
            self.tree.focus(currentItem)

            self.modifiedDictionary = self.GetDictionary('') #update stored dictionary
            self.UpdateSettingsStructure() #only if according flag set in visualizationSettings

    def OnEscapeComboItem(self,event):
        #print('escape')
        if self.selectedItem != '':
            self.editItemVar.set(self.tree.item(self.selectedItem,'values')[0])
            self.selectedItem = '' #now item can be modified
#            self.editItemVar.set('')
#            self.editItemName.set('')
            self.tree.focus_set()
            self.comboItem.lower(self.editItem) #bring entry item to front


#**function: edit dictionaryData and return modified (new) dictionary
#**input: 
#  settingsStructure: hierarchical settings structure, e.g., SC.visualizationSettings
#  exu: exudyn module
#  dictionaryName: name displayed in dialog
#**output: returns modified dictionary, which can be used, e.g., for SC.visualizationSettings.SetDictionary(...)
def EditDictionaryWithTypeInfo(settingsStructure, exu=None, dictionaryName='edit'):

    [root, tkWindow, tkinterAlreadyRunning] = GetTkRootAndNewWindow()
    
    windowHeight = treeEditDefaultHeight
    if treeEditMaxInitialHeight > treeEditDefaultHeight:
        try:
            #screen_width = root.winfo_screenwidth()
            screen_height = root.winfo_screenheight()
            #print('screen height=', screen_height)
            if screen_height > 1.2*treeEditDefaultHeight:
                windowHeight = int(min(treeEditMaxInitialHeight, 0.85*screen_height))
        except:
            print('WARNING: EditDictionaryWithTypeInfo could not determine screen size; please report error, Python version and platform as github issue')

    tkWindow.geometry(str(treeEditDefaultWidth)+'x'+str(windowHeight))

    guiSC = GetRendererSystemContainer()
    updateOnChange = False
    systemScaling = 1.35 #ideal for MacOS 
    topmost = True
    alphaTransparency = 1 #<1 means transparency
    treeOpen = True
    if guiSC != None:
        updateOnChange = guiSC.visualizationSettings.dialogs.multiThreadedDialogs
        systemScaling = guiSC.visualizationSettings.dialogs.fontScalingMacOS
        topmost = guiSC.visualizationSettings.dialogs.alwaysTopmost
        if guiSC.visualizationSettings.dialogs.alphaTransparency <= 1:
            alphaTransparency = guiSC.visualizationSettings.dialogs.alphaTransparency
        treeOpen = guiSC.visualizationSettings.dialogs.openTreeView
        
    fontFactor = systemScaling
    if not IsApple():
        #it seems that the font size should not be changed (what is done due to scaling internally ...)
        fontFactor = 1
        systemScaling = GetGUIContentScaling(root)

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
    
    #style.configure("Vertical.TScrollbar", width=4)
    
    comboListsT = GetComboBoxListsDict(exu)
    ex=TkinterEditDictionaryWithTypeInfo(parent=tkWindow, settingsStructure=settingsStructure, dictionaryTypesT=comboListsT, 
                                         updateOnChange=updateOnChange, treeOpen=treeOpen, textHeight = textHeight)
    ex.pack(fill="both", expand=True)

    if not tkinterAlreadyRunning:
        tk.mainloop()
    else:
        root.wait_window(tkWindow)
    
    settingsStructure.SetDictionary(ex.modifiedDictionary)
    #return ex.modifiedDictionary


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
            #print("value =", value)
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
        #print(kids)
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
        #print('select')
        self.selectedItem = '' #now item can be modified
        self.editItemVar.set('')
        self.editItemName.set('')
        
        
    def OnEditEntryItem(self,event):
        if self.selectedItem != '':
            #print('edit')
            if self.dictionaryIsEditable:
                valueStr = self.editItemVar.get()
                valueStr = str(valueStr).replace(' ','\\ ')
                #print(valueStr)
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
        #print('escape')
        if self.selectedItem != '':
            valueStr = self.tree.item(self.selectedItem,'values')[0]
            #print(valueStr)
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
    systemScaling = 1.35 #ideal for MacOS 
    topmost = True
    alphaTransparency = 1 #<1 means transparency
    if guiSC != None:
        systemScaling = guiSC.visualizationSettings.dialogs.fontScalingMacOS
        topmost = guiSC.visualizationSettings.dialogs.alwaysTopmost
        if guiSC.visualizationSettings.dialogs.alphaTransparency <= 1:
            alphaTransparency = guiSC.visualizationSettings.dialogs.alphaTransparency
        
    fontFactor = systemScaling
    if not IsApple():
        #it seems that the font size should not be changed (what is done due to scaling internally ...)
        fontFactor = 1
        systemScaling = GetGUIContentScaling(root)

    tkWindow.lift() #brings it to front of other; not always "strong" enough
    if topmost:
        tkWindow.attributes("-topmost", True) #puts window topmost (permanent)
    if alphaTransparency:
        tkWindow.attributes("-alpha", alphaTransparency) 
    

    textHeight = TkTextHeight(systemScaling)

    #no effect:
    # defaultFont = tkFont.Font(root=root, family = "TkDefaultFont")
    # defaultFont.configure(size=treeviewDefaultFontSize*fontFactor)

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



        
##+++++++++++++++++++++++++++++++++++++++
##EXAMPLE        

#DATA2={'objectType': 'ConnectorSpringDamper',
# 'markerNumbers': [1, 3],
# 'referenceLength': 1.0,
# 'stiffness': 1000.0,
# 'damping': 5.0,
# 'force': 0.0,
# 'activeConnector': True,
# 'springForceUserFunction': None,
# 'name': 'spring damper0',
# 'Vshow': True,
# 'VdrawSize': 0.0,
# 'Vcolor': [-1.0, -1.0, -1.0, -1.0]}
#
#x=EditDictionaryWithTypeInfo(DATA2)

