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
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import tkinter as tk
import tkinter.messagebox
import tkinter.ttk as ttk
import numpy as np #for array checks
import ast #for ast.literal_eval

#unique text height for tk with given scaling
def TkTextHeight(systemScaling):
    return int(13*systemScaling) #must be int #16 is too big on surface
    
##convert a given value with type information and size to a string; depending on size, it converts from matrix, vector, scalar, ...
#def ConvertValue2String(value, vType, vSize):
#    s= str(value)
#    if len(vSize) == 2 or (len(vSize)==1 and vSize[0] > 1): #must be matrix
#        s.s()
#    return s

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

    else:
        print('WARNING: exudyn not loaded as "exu"')

    #d['bool'] = ['True','False']
    dT['bool'] = [True, False]
    return dT
    
#convert string into exudyn type
def ConvertString2Value(value, vType, vSize, dictionaryTypesT):
    
    if vType == 'FileName' or vType == 'String':
        return value

    if vType == 'bool':
        if value == 'True':
            return True
        else:
            return False

    if vType == 'float':
        return float(value)

    if vType == 'Index':
        return int(value)
#    print('vType=',vType)
#    print('value=',value)
    
    if vType in dictionaryTypesT:#search for correct type in list
        for iValue in dictionaryTypesT[vType]:
            if str(iValue) == value:
                return iValue

    if len(vSize) == 2 or (len(vSize)==1 and vSize[0] > 1): #must be matrix
        return ast.literal_eval(value)

    print("Error in ConvertString2Value: unknown type",vType, "value=", value)
    return 0

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
            if len(row) != len(vSize[1]):
                return [False, 'matrix must have '+str(vSize[1]) + ' columns']
    
    return [True, '']

#this class gets a dictionary with type information structure in, but a plain dictionary with types (int, float, string, list, ...) out
#dictionaryTypes contains a dictionary which the available types, e.g. bool, etc.
class TkinterEditDictionaryWithTypeInfo(tk.Frame):
    def __init__(self, parent, dictionaryData, dictionaryTypesT, treeOpen=False):
        tk.Frame.__init__(self, parent)
        
        self.parentFrame = parent #parent frame stored for member functions
        #self.dictionaryTypes = dictionaryTypes #as string
        self.dictionaryTypesT = dictionaryTypesT #as type
        self.treeOpen = treeOpen

        #additional storage for type, size, etc.
        self.typeStorage = dict() #dictionary is stored as {'ID1': 'type1', 'ID2': 'type2', ...}
        self.sizeStorage = dict()
        self.descriptionStorage = dict()

        #create treeview:
        self.tree = ttk.Treeview(self, columns=("value","description"), selectmode='browse', height=15)
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


        self.AddNodeFromDictionaryWithTypeInfo(value=dictionaryData, parentNode="")
        self.tree.bind('<<TreeviewSelect>>', self.TreeviewSelect) #selection changed
        self.tree.bind("<Double-1>", self.OnTreeDoubleClick) #an item has been selected for change
        self.tree.bind("<Return>", self.OnTreeDoubleClick) #an item has been selected for change
        self.tree.bind("h", self.OnTreeHelp) #show description for selected item
        self.tree.bind("<Escape>", self.OnQuit) #an item has been selected for change
        
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
    def AddNodeFromDictionaryWithTypeInfo(self, value, parentNode="", key=None):
        if key is None:
            id = ""
        else:
            id = self.tree.insert(parentNode, "end", text=key)

        if isinstance(value, dict):
            self.tree.item(id, open=self.treeOpen)
            if 'itemIdentifier' in value: #is a value with types:
                #strValue = ConvertValue2String(value['value'], value['type'], value['size'])
                strValue = str(value['value'])
                
                self.tree.item(id, values=(strValue, value['description']))
                #store additional data in dictionaries (could also be tuples ...)
                self.typeStorage[id] = value['type']
                self.sizeStorage[id] = value['size']
                self.descriptionStorage[id] = value['description']
            else: #must be another dictionary:
                for (key, value) in value.items():
                    self.AddNodeFromDictionaryWithTypeInfo(value, id, key)
        else:
            print("Error in AddNodeFromDictionaryWithTypeInfo")
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
                    val = ConvertString2Value(valStr, str(self.typeStorage[i]), self.sizeStorage[i], self.dictionaryTypesT)
                    d.update({self.tree.item(i,'text'): val})
                else:
                    d.update({self.tree.item(i,'text'): ''})
            else:
                d.update({self.tree.item(i,'text'): self.GetDictionary(i)})
        return d

    def OnTreeHelp(self,event):
        item = self.tree.selection()[0]
        s = self.tree.item(item,'text')
        d = self.descriptionStorage[item]
        tk.messagebox.showinfo(s, d)

    def OnTreeDoubleClick(self,event):
        item = self.tree.selection()[0]
        nchilds = len(self.tree.get_children(item))
        #print('item=',item)
        #print('nchilds=',nchilds)

        #only edit items which have no subitems (no folders!)
        if nchilds == 0:
            
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
            
            selectedType=self.typeStorage[item]
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
            

    def OnQuit(self,event): #new selection --> nothing to edit for now
        self.parentFrame.destroy()
        
    def TreeviewSelect(self,event): #new selection --> nothing to edit for now
        #print('select')
        self.editItemVar.set('')
        self.editItemName.set('')
        self.comboItem.lower(self.editItem) #bring entry item to front
        
#        if self.selectedItem != '':
#            self.editItemVar.set(self.tree.item(self.selectedItem,'values')[0])
#            self.selectedItem = '' #now item can be modified

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

    def OnEscapeComboItem(self,event):
        #print('escape')
        if self.selectedItem != '':
            self.editItemVar.set(self.tree.item(self.selectedItem,'values')[0])
            self.selectedItem = '' #now item can be modified
#            self.editItemVar.set('')
#            self.editItemName.set('')
            self.tree.focus_set()
            self.comboItem.lower(self.editItem) #bring entry item to front


#edit dictionaryData and return modified (new) dictionary
def EditDictionaryWithTypeInfo(dictionaryData, exu=None, dictionaryName='edit'):
    master = tk.Tk()
    master.title(dictionaryName)
    systemScaling = master.call('tk', 'scaling') #obtains current scaling?
    #print('display scaling=',systemScaling)
    textHeight = TkTextHeight(systemScaling)
    
    style = ttk.Style(master)
    style.configure('Treeview', rowheight=textHeight) 
    
    def closeEditDictionary(event):
        master.destroy() # if you want to bring it back

    comboListsT = GetComboBoxListsDict(exu)
    ex=TkinterEditDictionaryWithTypeInfo(parent=master, dictionaryData=dictionaryData, dictionaryTypesT=comboListsT, treeOpen=True)
    ex.pack(fill="both", expand=True)

    master.mainloop()
    
    return ex.modifiedDictionary



#hierarchical lists:
class TkinterEditDictionary(tk.Frame):
    def __init__(self, parent, dictionaryData):
        tk.Frame.__init__(self, parent)
        
        self.parent = parent

        #create treeview:
        self.tree = ttk.Treeview(self, columns=("value",), height=15)
        self.vsb = ttk.Scrollbar(self, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=self.vsb.set)

        self.vsb.pack(side="right", fill="y")
        self.tree.pack(side="top", fill="both", expand=True)
#        self.vsb.grid(row=0, column=1)
#        self.tree.grid(row=0, column=0)


        self.AddNodeFromDictionary(value=dictionaryData, parentNode="")
        self.tree.bind('<<TreeviewSelect>>', self.TreeviewSelect) #selection changed
        self.tree.bind("<Double-1>", self.OnTreeDoubleClick) #an item has been selected for change
        self.tree.bind("<Return>", self.OnTreeDoubleClick) #an item has been selected for change
        self.tree.bind("<Escape>", self.OnQuit) #an item has been selected for change
        
        #create the entry field for editing the treeview value

        self.selectedItem = '' #will change to valid item in order to change value
        self.editItemName = tk.StringVar()
        self.editItemVar = tk.StringVar()

        self.editName = tk.Label(self, textvariable=self.editItemName)
        self.editName.pack(side="left", fill="both", expand=True)

        self.editItem = tk.Entry(self, textvariable=self.editItemVar)
#        self.editItem.grid(row=1, column=1)
        self.editItem.pack(side="left", fill="both", expand=False)
        self.editItem.bind('<Return>', self.OnEditEntryItem)
        self.editItem.bind('<FocusOut>', self.OnEditEntryItem)
        self.editItem.bind('<Escape>', self.OnEditEscapeItem)

        first = self.tree.get_children('')[0]
        self.tree.focus_set()
        self.tree.focus(first)
        self.tree.selection_set((first))
        
        self.modifiedDictionary = self.GetDictionary('')

    #create dictionary
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
            #self.tree.item(id, text=str(value))

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
            self.tree.item(self.selectedItem, values=(self.editItemVar.get()))
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

    def OnEditEscapeItem(self,event):
        #print('escape')
        if self.selectedItem != '':
            self.editItemVar.set(self.tree.item(self.selectedItem,'values')[0])
            self.selectedItem = '' #now item can be modified
#            self.editItemVar.set('')
#            self.editItemName.set('')
            self.tree.focus_set()

#edit dictionaryData and return modified (new) dictionary
def EditDictionary(dictionaryData):
    master = tk.Tk()
    systemScaling = master.tk.call('tk', 'scaling') #obtains current scaling?
    #print('display scaling=',systemScaling)
    textHeight = TkTextHeight(systemScaling)
    
    style = ttk.Style(master)
    style.configure('Treeview', rowheight=textHeight) 
    
    def closeEditDictionary(event):
        master.destroy() # if you want to bring it back

    ex=TkinterEditDictionary(master, dictionaryData)
    ex.pack(fill="both", expand=True)

    master.mainloop()
    
    return ex.modifiedDictionary

        
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

##convert a given value with type information and size to a string; depending on size, it converts from matrix, vector, scalar, ...
##NOT NEEDED: just use str(value) ...
#def ConvertValue2String(value, vType, vSize):
#    s= ''
#    if len(vSize) == 2: #must be matrix
#        s='['
#        delimiterRow = ''
#        for row in value:
#            delimiterCol = ''
#            s += delimiterRow + '['
#            for col in row:
#                s += delimiterCol + str(col)
#                delimiterCol = ','
#            s+=']'
#            delimiterRow = ','
#    elif vSize[0] > 1: #vector/array
#        delimiterCol = ''
#        s = '['
#        for col in value:
#            s += delimiterCol + str(col)
#            delimiterCol = ','
#        s+=']'
#    else: #scalar, string, int, type, ...
#        s=str(value)
#    return s