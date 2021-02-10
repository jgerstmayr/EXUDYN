#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Utilities for interactive simulation and results monitoring
#
# Author:   Johannes Gerstmayr
# Date:     2021-01-17 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++import sys
# Utility functions and structures for Exudyn

import numpy as np #LoadSolutionFile
import time        #AnimateSolution
import tkinter
import tkinter.font as tkFont
import matplotlib.pyplot as plt
import copy		   #copy numpy objects
import exudyn

#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#**class: create an interactive dialog, which allows to interact with simulations
#the dialog has a 'Run' button, which initiates the simulation and a 'Stop' button which stops/pauses simulation; 'Quit' closes the simulation model
#for examples, see \texttt{simulateInteractively.py} and \texttt{massSpringFrictionInteractive.py}
#use \_\_init\_\_ method to setup this class with certain buttons, edit boxes and sliders
#**example:
# #the following example is only demonstrating the structure of dialogItems and plots
# #dialogItems structure:
# #general items: 
# #    'type' can be out of:
# #               'label' (simple text), 
# #               'button' (button with callback function),
# #               'radio' (a radio button with several alternative options),
# #               'slider' (with an adjustable range to choose a value)
# #    'grid': (row, col, colspan) specifies the row, column and (optionally) the span of columns the item is placed at;
# #            exception in 'radio', where grid is a list of (row, col) for every choice
# #    'options': text options, where 'L' means flush left, 'R' means flush right
# #suboptions of 'label':
# #               'text': a text to be drawn
# #suboptions of 'button':
# #               'text': a text to be drawn on button
# #               'callFunction': function which is called on button-press
# #suboptions of 'radio':
# #               'textValueList': [('text1',0),('text2',1)] a list of texts with according values
# #               'value': default value (choice) of radio buttons
# #               'variable': according variable in mbs.variables, which is set to current radio button value
# #suboptions of 'slider':
# #               'range': (min, max) a tuple containing minimum and maximum value of slider
# #               'value': default value of slider
# #               'steps': number of steps in slider
# #               'variable': according variable in mbs.variables, which is set to current slider value
# #example:
# dialogItems = [{'type':'label', 'text':'Nonlinear oscillation simulator', 'grid':(0,0,2), 'options':['L']},
#                {'type':'button', 'text':'test button','callFunction':ButtonCall, 'grid':(1,0,2)},
#                {'type':'radio', 'textValueList':[('linear',0),('nonlinear',1)], 'value':0, 'variable':'mode', 'grid': [(2,0),(2,1)]},
#                {'type':'label', 'text':'excitation frequency (Hz):', 'grid':(5,0)},
#                {'type':'slider', 'range':(3*f1/800, 3*f1), 'value':omegaInit/(2*pi), 'steps':800, 'variable':'frequency', 'grid':(5,1)},
#                {'type':'label', 'text':'damping:', 'grid':(6,0)},
#                {'type':'slider', 'range': (0, 40), 'value':damper, 'steps':800, 'variable':'damping', 'grid':(6,1)},
#                {'type':'label', 'text':'stiffness:', 'grid':(7,0)},
#                {'type':'slider', 'range':(0, 10000), 'value':spring, 'steps':800, 'variable':'stiffness', 'grid':(7,1)}]
#
# #plots structure:
# plots={'nPoints':500,               #number of stored points in subplots (higher means slower drawing)
#        'subplots':(2,1),           #(rows, columns) arrangement of subplots
#        #sensors defines per subplot (sensor, coordinate), xlabel and ylabel; if coordinate=0, time is used:
#        'sensors':[[(sensPos,0),(sensPos,1),'time','mass position'], 
#                   [(sensFreq,0),(sensFreq,1),'time','excitation frequency']],
#        'limitsX':[(0,2),(-5,5)],   #x-range per subplot; if not provided, autoscale is applied
#        'limitsY':[(-5,5),(0,10),], #y-range per subplot; if not provided, autoscale is applied
#        'fontSize':16,              #custom font size for figure
#        'sizeInches':(12,12)}        #specific x and y size of figure in inches (using 100 dpi)
class InteractiveDialog:
    #**classFunction: initialize an InteractiveDialog
    #**input: 
    #  mbs: a multibody system to be simulated
    #  simulationSettings: exu.SimulationSettings() according to user settings
    #  simulationFunction: a function which is called before a simulation for the short period is started (e.g, assign special values, etc.)
    #  dialogItems: a list of dictionaries, which describe the contents of the interactive items, where every dict has the structure {'type':[label, entry, button, slider, check] ... according to tkinter widgets, 'callFunction': a function to be called, if item is changed/button pressed, 'grid': (row,col) of item to be placed, 'rowSpan': number of rows to be used, 'columnSpan': number of columns to be used; for special item options see notes}
    #  plots: list of dictionaries to specify a sensor to be plotted live, see example
    #  period: a simulation time span in seconds which is simulated with the simulationFunction in every iteration
    #  realtimeFactor: if 1, the simulation is nearly performed in realtime (except for computation time); if > 1, it runs faster than realtime, if < 1, than it is slower
    #  userStartFunction: a function F(flag) which is called every time after Run/Stop is pressed. The argument flag = False if button "Run" has been pressed, flag = True, if "Stop" has been pressed
    #  title: title text for interactive dialog
    #  showTime: shows current time in dialog
    #  fontSize: adjust font size for all dialog items
    #**notes: detailed description of dialogItems and plots list/dictionary is given in commented the example below
    def __init__(self, mbs, simulationSettings, simulationFunction, 
                 dialogItems, plots = [], period = 0.04, 
                 realtimeFactor = 1, userStartSimulation=False,
                 title='',  showTime=False, fontSize = 12):
        #store init arguments
        self.mbs = mbs
        self.simulationFunction = simulationFunction
        self.simulationSettings = simulationSettings
        self.title = title
        self.dialogItems = dialogItems
        self.period = period
        self.realtimeFactor = realtimeFactor
        self.stepSize = simulationSettings.timeIntegration.endTime/simulationSettings.timeIntegration.numberOfSteps

        self.plots = plots
        self.userStartSimulation = userStartSimulation
        self.showTime = showTime
        self.fontSize = fontSize

        #create tkinter instance
        root = self.root = tkinter.Tk()
        root.protocol("WM_DELETE_WINDOW", self.OnQuit) #always leave app with OnQuit
        root.title(title)
        systemScaling = root.call('tk', 'scaling') #obtains current scaling?
        #print('systemScaling=',systemScaling)
        systemScaling = 1

        #change global font size
        if True:
            defaultFont = tkFont.Font(root=root, family = "TkDefaultFont")#,weight = "bold")
            defaultFont.configure(size=int(systemScaling*self.fontSize))
            root.option_add("*Font", "TkDefaultFont") #all widgets should use TkDefaultFont; does not work
        
        self.root = root
        self.counter = 0 #counter for simulationFunction
        self.simulationStopped = True
        self.variableList = [] #list of tuples: (widget, mbs variable name); value obtained with widget.get()

        self.itemBorder = 2 #common border to items

        tkinterNESW = tkinter.N+tkinter.E+tkinter.S+tkinter.W
        
        self.widgets = [] #store tk widgets as list, for later access
        firstItem = True
        for item in dialogItems:
            callFunction = 0
            text = ''
            setGrid = False
            # sticky = 'NSEW'
           
            if 'callFunction' in item:
                callFunction = item['callFunction']
            if 'text' in item:
                text = item['text']
            # if 'sticky' in item:
            #     sticky = item['sticky']
            #++++++++++++++++++++++++++++++++++
            if item['type'] == 'label':
                setGrid = True
                widget = tkinter.Label(root, text = text, 
                                       borderwidth = self.itemBorder, 
                                       font=defaultFont)
            #++++++++++++++++++++++++++++++++++
            elif item['type'] == 'button':
                setGrid = True
                widget = tkinter.Button(root, text = text, 
                                       borderwidth = self.itemBorder, 
                                       font=defaultFont)
                if 'callFunction' in item:
                    widget['command'] = item['callFunction']
            #++++++++++++++++++++++++++++++++++
            elif item['type'] == 'slider':
                setGrid = True
                steps = 400
                minValue = 0
                maxValue = 1
                initialValue = 0.5
                if 'steps' in item:
                    steps = item['steps']
                if 'range' in item:
                    minValue = item['range'][0]
                    maxValue = item['range'][1]
                if 'value' in item:
                    initialValue = item['value']
                if initialValue < minValue or initialValue > maxValue:
                    initialValue = 0.5*(minValue+maxValue)

                widget = tkinter.Scale(root, from_=minValue, to=maxValue,
                                       length = steps, digits=4, resolution=(maxValue-minValue)/steps,
                                       orient=tkinter.HORIZONTAL,
                                       font=defaultFont)
                widget.set(initialValue)
                if 'callFunction' in item:
                    widget['command'] = self.item['callFunction']
                if 'variable' in item:
                    self.variableList += [(widget, item['variable'])]

            #++++++++++++++++++++++++++++++++++
            elif item['type'] == 'radio':
                var = tkinter.IntVar()
                var.set(item['value'])
                cnt = 0
                for opt in item['textValueList']:
                    widget = tkinter.Radiobutton(root, 
                                                text=opt[0],
                                                padx = 20, 
                                                variable=var, 
                                                value=opt[1], 
                                                #height=1, width=1, 
                                                indicatoron=0, #use highlighting instead of checkboxes
                                                font=defaultFont)
                    if 'grid' in item:
                        grid = item['grid'][cnt] #must be list of grid values
                        if len(grid) == 2:
                            widget.grid(row=grid[0], column=grid[1], sticky=tkinterNESW)
                        elif len(grid) == 3:
                            widget.grid(row=grid[0], column=grid[1], columnspan=grid[2], sticky=tkinterNESW)
                    # not needed, all flush left
                    # if 'options' in item:
                    #     if 'L' in item['options']:
                    #         widget['anchor'] = 'w'
                    #     if 'R' in item['options']:
                    #         widget['anchor'] = 'e'
                    else:
                        widget.grid(column=0, sticky=tkinter.W)
                    cnt+=1
                self.variableList += [(var, item['variable'])] #store widget variable and mbs variable
            #++++++++++++++++++++++++++++++++++
            if setGrid:
                if 'grid' in item:
                    grid = item['grid']
                    if len(grid) == 2:
                        widget.grid(row=grid[0], column=grid[1], sticky=tkinterNESW)
                    elif len(grid) == 3:
                        widget.grid(row=grid[0], column=grid[1], columnspan=grid[2], sticky=tkinterNESW)
                    elif len(grid) == 4:
                        widget.grid(row=grid[0], column=grid[1], columnspan=grid[2], rowspan=grid[3], sticky=tkinterNESW)
                    else:
                        raise ValueError("ERROR: InteractiveDialog: grid must have 2, 3, or 4 components")
                else:
                    widget.grid(column=0, sticky=tkinterNESW)
                if 'options' in item:
                    if 'L' in item['options']:
                        widget['anchor'] = 'w'
                    if 'R' in item['options']:
                        widget['anchor'] = 'e'
            self.widgets += [widget]
            # if item['type'] == 'entry' and firstEntry: #entry may get focus
            #     firstEntry = False
            #     widget.focus_set() #set focus to first item
                
        #show current time 
        if self.showTime:
            self.currentTime = tkinter.StringVar()
            widget = tkinter.Label(root, textvariable = self.currentTime, 
                                   borderwidth = self.itemBorder, justify=tkinter.LEFT,
                                   font=defaultFont)
            self.currentTime.set('t = ')
            widget.grid(column=0, sticky=tkinter.W)

        #add run button into last row:
        self.RunButtonText = tkinter.StringVar()
        self.Run = tkinter.Button(root, textvariable=self.RunButtonText,
                                  borderwidth = self.itemBorder, font=defaultFont)
        self.RunButtonText.set('Run')
        
        self.Run.grid(column=0, sticky=tkinterNESW)#, row=maxRow+1)
        self.Run['command'] = self.StartSimulation
        self.Run.focus_set() #does not work
        
        self.root.bind('<space>', func=self.StartSimulation) #if focus is not set to button ...
        self.root.bind("<Escape>", self.OnQuit) #an item has been selected for change
        

        root.update()
        if root.winfo_width() < 320:
            root.minsize(320,root.winfo_height())
        #root.minsize(280,50) #will create windows which are too small

        self.InitializeSolver() #solver gets ready to be called repeatedly
        self.InitializePlots()  #set up all structures for plots
        self.UpdatePlots()      #update all subplots with new sensor values

        root.deiconify()
        root.mainloop()

    #**classFunction: function called when pressing escape or closing dialog
    def OnQuit(self, event=None):
        self.simulationStopped = True
        self.RunButtonText.set('Stop')
        self.FinalizeSolver()
        self.root.quit()
        self.root.destroy()

    #**classFunction: function called on button 'Run'
    def StartSimulation(self, event=None):
        self.simulationStopped = not self.simulationStopped
        if not self.simulationStopped:
            self.RunButtonText.set('Stop')
        else:
            self.RunButtonText.set('Run')

        if self.userStartSimulation:
            self.userStartSimulation(self.simulationStopped)
        
        self.ContinuousRunFunction()

    #**classFunction: assign current values of radio buttons and sliders to mbs.variables
    def ProcessWidgetStates(self):
        for var in self.variableList:
            self.mbs.variables[var[1]] = var[0].get()
            #print(var[1],'=', var[0].get())

    #**classFunction: function which is repeatedly called when button 'Run' is pressed
    def ContinuousRunFunction(self, event=None):
        if not self.simulationStopped:
            self.ProcessWidgetStates()
            
            t = self.RunSimulationPeriod()
            if self.showTime:
                self.currentTime.set('t = '+str(round(t,6)))
            self.counter += 1
            delay = max(1, int(self.period*1000/self.realtimeFactor))
            self.Run.after(delay, self.ContinuousRunFunction)

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: initialize figure and subplots for plots structure
    def InitializePlots(self):
        plt.rcParams.update({'font.size': self.plots['fontSize']})
    
        fig = plt.figure()
        fig.dpi = 100 #in terminal, initially set to 200
        fig.tight_layout()
        if 'sizeInches' in self.plots: #otherwise use default figure size
            fig.set_size_inches(self.plots['sizeInches'][0], self.plots['sizeInches'][1], forward=True)
        nPoints = self.plots['nPoints']
        self.plots['fig'] = fig
        self.plots['currentIndex'] = 0
        nSensors = len(self.plots['sensors'])
        self.plots['data'] = [np.zeros((nPoints,2))]*nSensors
        self.plots['line'] = [0]*nSensors
        self.plots['marker'] = [0]*nSensors
        self.plots['ax'] = [0]*nSensors
    
        for j in range(nSensors):
            self.plots['ax'][j] = fig.add_subplot(self.plots['subplots'][0],self.plots['subplots'][1],j+1)
            self.plots['ax'][j].grid(True, 'major', 'both')
            self.plots['ax'][j].set_xlabel(self.plots['sensors'][j][2])
            self.plots['ax'][j].set_ylabel(self.plots['sensors'][j][3])
            
            data = np.zeros((nPoints,2))
            if self.plots['sensors'][j][0][1] == 0: #time of sensor
                data[:,0] = np.linspace(self.period,nPoints*self.period,num=nPoints)
            #data[:,1] = 0 #not needed
            self.plots['data'][j] = copy.deepcopy(data)
            
            self.plots['line'][j], = self.plots['ax'][j].plot(data[:,0],data[:,1], 'b-')
            self.plots['marker'][j], = self.plots['ax'][j].plot(0,0, 'ro') #red circle
            self.plots['ax'][j].set_xlim(min(data[:,0]), max(data[:,0]))

    #**classFunction: update all subplots with current sensor values
    def UpdatePlots(self):
        n = self.plots['nPoints']
        i = self.plots['currentIndex']
        t = self.simulationSettings.timeIntegration.startTime #current time

        for j in range(len(self.plots['sensors'])):
            data = self.plots['data'][j]
            
            sensorNum = [0,0]
            sensorCoord = [0,0]
            for k in range(2):
                sensorNum = self.plots['sensors'][j][k][0]
                sensorCoord = self.plots['sensors'][j][k][1]
                if sensorCoord == 0:
                    data[i,k] = t
                else:
                    value = self.mbs.GetSensorValues(sensorNum)
                    if type(value)==np.ndarray:
                        data[i,k] = value[sensorCoord-1]
                    elif sensorCoord == 1:
                        data[i,k] = value
                    else:
                        raise ValueError('ERROR: InteractiveDialog: plots.sensor '+str(j)+': access to invalid coordinate')
                        
            self.plots['line'][j].set_data(data[:,0], data[:,1]) 
            self.plots['marker'][j].set_data(data[i,0], data[i,1]) 

            self.plots['ax'][j].set_xlim(min(data[:,0]), max(data[:,0]))
            
            if 'limitsX' in self.plots:
                if len(self.plots['limitsX'][j]):
                    self.plots['ax'][j].set_xlim(self.plots['limitsX'][j][0],self.plots['limitsX'][j][1])
            if 'limitsY' in self.plots:
                if len(self.plots['limitsY'][j]):
                    self.plots['ax'][j].set_ylim(self.plots['limitsY'][j][0],self.plots['limitsY'][j][1])
            #autoscale, if no limits given
            self.plots['ax'][j].relim(visible_only=True)
            self.plots['ax'][j].autoscale_view(tight=True)
        
        #update figure:
        self.plots['fig'].canvas.draw()
        self.plots['fig'].canvas.flush_events()

        if i < n-1:
            self.plots['currentIndex'] += 1

        for j in range(len(self.plots['sensors'])):
            data = self.plots['data'][j]
            if i == n-1:
                data = np.roll(data, -1, axis=0)
            self.plots['data'][j] = data #data references mbs variable


    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: function to initialize solver for repeated calls
    def InitializeSolver(self):
        self.mbs.sys['solver'] = exudyn.MainSolverImplicitSecondOrder()
        self.mbs.sys['solver'].InitializeSolver(self.mbs, self.simulationSettings)

    #**classFunction: stop solver (finalize correctly)
    def FinalizeSolver(self):
        self.mbs.sys['solver'].FinalizeSolver(self.mbs, self.simulationSettings) #shut down solver correctly (finalize files, ...)


    #**classFunction: function which performs short simulation for given period        
    def RunSimulationPeriod(self):
        cnt = self.counter
        deltaT = self.period
        h = self.stepSize
        mbs = self.mbs

        #+++++++++++++++++++++++++++++++++++++++++
        #thus is the USER PART
        self.simulationFunction(self.mbs, self)
        #+++++++++++++++++++++++++++++++++++++++++
    
        if False: #slow way, always start/stop simulation:
            self.simulationSettings.timeIntegration.numberOfSteps = int(deltaT/h)
            self.simulationSettings.timeIntegration.endTime = self.simulationSettings.timeIntegration.startTime+deltaT
            exudyn.SolveDynamic(mbs, self.simulationSettings, updateInitialValues=True)
            self.simulationSettings.timeIntegration.startTime += deltaT
    
        #+++++++++++++++++++++++++++++++++++++++++
        #PLOT PART, done every time before simulation starts
        self.UpdatePlots()
    
        #+++++++++++++++++++++++++++++++++++++++++
        #TIME STEPPING PART
        if True:
        
            self.simulationSettings.timeIntegration.numberOfSteps = int(deltaT/h)
            self.simulationSettings.timeIntegration.endTime = self.simulationSettings.timeIntegration.startTime+deltaT
        
        
            mbs.sys['solver'].InitializeSolverInitialConditions(mbs, self.simulationSettings) #needed to update simulationSettings in solver
            mbs.sys['solver'].SolveSteps(mbs, self.simulationSettings)
            
            #get current values and update initial conditions for next step:
            currentState = mbs.systemData.GetSystemState() 
            mbs.systemData.SetSystemState(systemStateList=currentState, configuration = exudyn.ConfigurationType.Initial)
        
            self.simulationSettings.timeIntegration.startTime += deltaT
    
        return self.simulationSettings.timeIntegration.endTime #return current time for dialog



