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
from math import sin, pi #for animation
import time        
import tkinter
import tkinter.font as tkFont
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
# plots={'nPoints':500,              #number of stored points in subplots (higher means slower drawing)
#        'subplots':(2,1),           #(rows, columns) arrangement of subplots (for every sensor)
#        #sensors defines per subplot (sensor, coordinate), xlabel and ylabel; if coordinate=0, time is used:
#        'sensors':[[(sensPos,0),(sensPos,1),'time','mass position'], 
#                   [(sensFreq,0),(sensFreq,1),'time','excitation frequency']],
#        'limitsX':[(0,2),(-5,5)],   #x-range per subplot; if not provided, autoscale is applied
#        'limitsY':[(-5,5),(0,10),], #y-range per subplot; if not provided, autoscale is applied
#        'fontSize':16,              #custom font size for figure
#        'subplots':False,           #if not specified, subplots are created; if False, all plots go into one window 
#        'lineStyles':['r-','b-'],    #if not specified, uses default '-b', otherwise define list of line styles [string for matplotlib.pyplot.plot] per sensor
#        'sizeInches':(12,12)}       #specific x and y size of figure in inches (using 100 dpi)
class InteractiveDialog:
    #**classFunction: initialize an InteractiveDialog
    #**input: 
    #  mbs: a multibody system to be simulated
    #  simulationSettings: exudyn.SimulationSettings() according to user settings
    #  simulationFunction: a function which is called before a simulation for the short period is started (e.g, assign special values, etc.)
    #  dialogItems: a list of dictionaries, which describe the contents of the interactive items, where every dict has the structure {'type':[label, entry, button, slider, check] ... according to tkinter widgets, 'callFunction': a function to be called, if item is changed/button pressed, 'grid': (row,col) of item to be placed, 'rowSpan': number of rows to be used, 'columnSpan': number of columns to be used; for special item options see notes}
    #  plots: list of dictionaries to specify a sensor to be plotted live, see example
    #  period: a simulation time span in seconds which is simulated with the simulationFunction in every iteration
    #  realtimeFactor: if 1, the simulation is nearly performed in realtime (except for computation time); if > 1, it runs faster than realtime, if < 1, than it is slower
    #  userStartFunction: a function F(flag) which is called every time after Run/Stop is pressed. The argument flag = False if button "Run" has been pressed, flag = True, if "Stop" has been pressed
    #  title: title text for interactive dialog
    #  showTime: shows current time in dialog
    #  fontSize: adjust font size for all dialog items
    #  doTimeIntegration: performs internal time integration with given parameters
    #  runOnStart: immediately activate 'Run' button on start
    #**notes: detailed description of dialogItems and plots list/dictionary is given in commented the example below
    def __init__(self, mbs, simulationSettings, simulationFunction, 
                 dialogItems, plots = [], period = 0.04, 
                 realtimeFactor = 1, userStartSimulation=False,
                 title='',  showTime=False, fontSize = 12,
                 doTimeIntegration = True,
                 runOnStart = False):
        #store init arguments
        self.mbs = mbs
        self.simulationFunction = simulationFunction
        self.simulationSettings = simulationSettings
        self.title = title
        self.dialogItems = dialogItems
        self.period = period
        self.realtimeFactor = realtimeFactor
        self.stepSize = simulationSettings.timeIntegration.endTime/simulationSettings.timeIntegration.numberOfSteps
        self.doTimeIntegration = doTimeIntegration

        self.plots = plots
        self.userStartSimulation = userStartSimulation
        self.showTime = showTime
        self.fontSize = fontSize

        #create tkinter instance
        root = self.root = tkinter.Tk()
        exudyn.sys['tkinterRoot'] = root
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
                resolutionItem = (maxValue-minValue)/(steps-1)
                if 'resolution' in item: #resolution is needed, if we want to do bi-directional set/get of slider values (otherwise digits are lost)
                     resolutionItem = item['resolution']

                nDigits = 4
                if maxValue-minValue == steps-1:
                    nDigits = 0
                widget = tkinter.Scale(root, from_=minValue, to=maxValue,
                                       length = steps, digits=nDigits, resolution=resolutionItem,
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

        if runOnStart:          #immediately activate run function on startup
            self.StartSimulation()
        root.deiconify()
        root.mainloop()

    #**classFunction: function called when pressing escape or closing dialog
    def OnQuit(self, event=None):
        self.simulationStopped = True
        self.RunButtonText.set('Stop')
        self.FinalizeSolver()
        del exudyn.sys['tkinterRoot'] #this is not thread safe, but interuption should not happen ...
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
        #self.ProcessWidgetStates() #do this finally, to update states, which may have changed in last step (SolutionViewer!)
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
            exudyn.DoRendererIdleTasks() #for MacOS, but also to open visualization dialog, etc.
            #print(".")
            t = self.RunSimulationPeriod()
            if self.showTime:
                self.currentTime.set('t = '+str(round(t,6)))
            self.counter += 1
            delay = max(1, int(self.period*1000/self.realtimeFactor))
            self.Run.after(delay, self.ContinuousRunFunction)

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #**classFunction: initialize figure and subplots for plots structure
    def InitializePlots(self):
        if len(self.plots) != 0:
            import matplotlib.pyplot as plt
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
            lineStyle = 'b-'
            doSubplots = True
            if 'subplots' in self.plots:
                doSubplots = self.plots['subplots']
            if not doSubplots:
                axPlot = fig.add_subplot(1,1,1)
        
            for j in range(nSensors):
                if doSubplots:
                    self.plots['ax'][j] = fig.add_subplot(self.plots['subplots'][0],self.plots['subplots'][1],j+1)
                else:
                    self.plots['ax'][j] = axPlot
                self.plots['ax'][j].grid(True, 'major', 'both')
                self.plots['ax'][j].set_xlabel(self.plots['sensors'][j][2])
                self.plots['ax'][j].set_ylabel(self.plots['sensors'][j][3])
                
                data = np.zeros((nPoints,2))
                if self.plots['sensors'][j][0][1] == 0: #time of sensor
                    data[:,0] = np.linspace(self.period,nPoints*self.period,num=nPoints)
                #data[:,1] = 0 #not needed
                self.plots['data'][j] = copy.deepcopy(data)
                
                if 'lineStyles' in self.plots:
                    lineStyle = self.plots['lineStyles'][j]
                self.plots['line'][j], = self.plots['ax'][j].plot(data[:,0],data[:,1], lineStyle)
                self.plots['marker'][j], = self.plots['ax'][j].plot(0,0, 'ro') #red circle
                self.plots['ax'][j].set_xlim(min(data[:,0]), max(data[:,0]))

    #**classFunction: update all subplots with current sensor values
    def UpdatePlots(self):
        if len(self.plots) != 0:
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
        if self.doTimeIntegration:
            self.mbs.sys['solver'] = exudyn.MainSolverImplicitSecondOrder()
            self.mbs.sys['solver'].InitializeSolver(self.mbs, self.simulationSettings)

    #**classFunction: stop solver (finalize correctly)
    def FinalizeSolver(self):
        if self.doTimeIntegration:
            self.mbs.sys['solver'].FinalizeSolver(self.mbs, self.simulationSettings) #shut down solver correctly (finalize files, ...)


    #**classFunction: function which performs short simulation for given period        
    def RunSimulationPeriod(self):
        cnt = self.counter
        deltaT = self.period
        h = self.stepSize
        mbs = self.mbs

        #+++++++++++++++++++++++++++++++++++++++++
        #this is the USER PART
        self.simulationFunction(self.mbs, self)
        #+++++++++++++++++++++++++++++++++++++++++
    
        if self.doTimeIntegration and False: #slow way, always start/stop simulation:
            self.simulationSettings.timeIntegration.numberOfSteps = max(int(deltaT/h),1)
            self.simulationSettings.timeIntegration.endTime = self.simulationSettings.timeIntegration.startTime+deltaT
            exudyn.SolveDynamic(mbs, self.simulationSettings, updateInitialValues=True)
            self.simulationSettings.timeIntegration.startTime += deltaT
    
        #+++++++++++++++++++++++++++++++++++++++++
        #PLOT PART, done every time before simulation starts
        self.UpdatePlots()
    
        #+++++++++++++++++++++++++++++++++++++++++
        #TIME STEPPING PART
        if self.doTimeIntegration and True:
        
            self.simulationSettings.timeIntegration.numberOfSteps = max(int(deltaT/h),1)
            self.simulationSettings.timeIntegration.endTime = self.simulationSettings.timeIntegration.startTime+deltaT
        
        
            mbs.sys['solver'].InitializeSolverInitialConditions(mbs, self.simulationSettings) #needed to update simulationSettings in solver
            mbs.sys['solver'].SolveSteps(mbs, self.simulationSettings)
            
            #get current values and update initial conditions for next step:
            currentState = mbs.systemData.GetSystemState() 
            mbs.systemData.SetSystemState(systemStateList=currentState, configuration = exudyn.ConfigurationType.Initial)
        
            self.simulationSettings.timeIntegration.startTime += deltaT
    
        return self.simulationSettings.timeIntegration.endTime #return current time for dialog


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#**function: animate modes of ObjectFFRFreducedOrder and other objects (changes periodically one nodal coordinate); for creating snapshots, press 'Static' and 'Record animation' and press 'Run' to save one figure in the image subfolder; for creating animations for one mode, use the same procedure but use 'One Cycle'
#**input:
#    systemContainer: system container (usually SC) of your model, containing visualization settings
#    mainSystem: system (usually mbs) containing your model
#    nodeNumber: node number of which the coordinates shall be animated. In case of ObjectFFRFreducedOrder, this is the generic node, e.g., 'nGenericODE2' in the dictionary returned by the function AddObjectFFRFreducedOrderWithUserFunctions(...)
#    period: delay for animation of every frame; the default of 0.04 results in approximately 25 frames per second
#    stepsPerPeriod: number of steps into which the animation of one cycle of the mode is split into
#    showTime: show a virtual time running from 0 to 2*pi during one mode cycle
#    renderWindowText: additional text written into renderwindow before 'Mode X' (use $\backslash$n to add line breaks)
#    runOnStart: immediately go into 'Run' mode
#    runMode: 0=continuous run, 1=one cycle, 2=static (use slider/mouse to vary time steps)
#    scaleAmplitude: additional scaling for amplitude if necessary
#**output: opens interactive dialog with further settings
#**notes: Uses class InteractiveDialog in the background, which can be used to adjust animation creation. 
#    Press 'Run' to start animation; Chose 'Mode shape', according component for contour plot; to record one cycle for animation, choose 'One cycle', run once to get the according range in the contour plot, press 'Record animation' and press 'Run', now images can be found in subfolder 'images' (for further info on animation creation see \refSection{secGeneratingAnimations}); now deactivate 'Record animation' by pressing 'Off' and chose another mode
def AnimateModes(systemContainer, mainSystem, nodeNumber, period = 0.04, stepsPerPeriod = 30, showTime = True, 
                 renderWindowText = '', runOnStart = False, runMode=0, scaleAmplitude = 1):

    SC = systemContainer
    mbs = mainSystem
    SC.visualizationSettings.general.graphicsUpdateInterval = 0.25*min(period, 2e-3) #set according update interval!
    SC.visualizationSettings.general.showSolverTime = showTime
    #SC.visualizationSettings.general.showComputationInfo = False
    SC.visualizationSettings.general.showSolverInformation = False
    SC.visualizationSettings.general.renderWindowString = renderWindowText+'mode 0'
    
    coordIndex = mbs.GetNodeODE2Index(nodeNumber)
    nodeCoords = mbs.GetNodeOutput(nodeNumber,exudyn.OutputVariableType.Coordinates,exudyn.ConfigurationType.Reference)
    numberOfModes = len(nodeCoords)
    #numberOfModes = mbs.GetNode(nODE2)['numberOfODE2Coordinates']

    if (runMode != 0 and runMode != 1 and runMode != 2):
        print('ERROR in AnimateModes: illegal run mode:', runMode)
        return
    
    #use interactive dialog:
    dialogItems = [
                   {'type':'label', 'text':'Mode shape:', 'grid':(1,0)},
                   {'type':'slider', 'range':(0, numberOfModes-1), 'value':0, 'steps':numberOfModes, 'variable':'modeShapeModeNumber', 'grid':(1,1)},
                   {'type':'label', 'text':'Contour plot:', 'grid':(2,0)},
                   {'type':'radio', 'textValueList':[('None',int(exudyn.OutputVariableType._None)),
                                                     ('DisplacementLocal',int(exudyn.OutputVariableType.DisplacementLocal)),
                                                      ('Displacement',int(exudyn.OutputVariableType.Displacement)),
                                                      ('StressLocal',int(exudyn.OutputVariableType.StressLocal)),
                                                      ('StrainLocal',int(exudyn.OutputVariableType.StrainLocal))], 
                    'value':int(exudyn.OutputVariableType.DisplacementLocal), 'variable':'modeShapeOutputVariable', 'grid': [(3,0),(3,1),(3,2),(3,3),(3,4)]},
                   {'type':'label', 'text':'Contour Component (use -1 for norm):', 'grid':(4,0)},
                   {'type':'slider', 'range':(-1, 5), 'value':0, 'steps':7, 'variable':'modeShapeComponent', 'grid':(4,1)},
                   {'type':'label', 'text':'Amplitude:', 'grid':(5,0)},
                   {'type':'slider', 'range':(0, 1), 'value':0.05, 'steps':501, 'variable':'modeShapeAmplitude', 'grid':(5,1)},
                   {'type':'label', 'text':'update period:', 'grid':(6,0)},
                   {'type':'slider', 'range':(0.01, 2), 'value':0.04, 'steps':200, 'variable':'modeShapePeriod', 'grid':(6,1)},
                   {'type':'radio', 'textValueList':[('Continuous run',0), ('One cycle',1), ('Static',2)],'value':runMode, 'variable':'modeShapeRunModus', 'grid': [(7,0),(7,1),(7,2)]},
                   {'type':'radio', 'textValueList':[('Mesh+Faces',3), ('Faces only',1), ('Mesh only',2)],'value':3, 'variable':'modeShapeMesh', 'grid': [(8,0),(8,1),(8,2)]},
                   {'type':'radio', 'textValueList':[('Record animation',0), ('No recording',1)],'value':1, 'variable':'modeShapeSaveImages', 'grid': [(9,0),(9,1)]},
                   ]

    mbs.variables['modeShapePeriod'] = period
    mbs.variables['modeShapeStepsPerPeriod'] = stepsPerPeriod
    mbs.variables['modeShapeTimeIndex'] = 0
    mbs.variables['modeShapeLastSetting'] = [-1,0,0,0]
    mbs.variables['modeShapeNodeCoordIndex'] = coordIndex
    mbs.variables['modeShapeScaleAmplitude'] = scaleAmplitude

    def UFshowModes(mbs, dialog):
        i = mbs.variables['modeShapeTimeIndex']
        mbs.variables['modeShapeTimeIndex'] += 1
        stepsPerPeriod = mbs.variables['modeShapeStepsPerPeriod']
        amplitude = mbs.variables['modeShapeAmplitude']
        if amplitude == 0:
            SC.visualizationSettings.bodies.deformationScaleFactor = 0
            amplitude = 1
        else:
            SC.visualizationSettings.bodies.deformationScaleFactor = 1

        if mbs.variables['modeShapeRunModus'] == 2: #no sin(t) in static case
            stepsPerPeriod = 1
            t = 0
        else:
            t = i/stepsPerPeriod * 2 * pi
            amplitude *= sin(t)
        mbs.systemData.SetTime(t, exudyn.ConfigurationType.Visualization)

        ode2Coords = mbs.systemData.GetODE2Coordinates()
        selectedMode = int(mbs.variables['modeShapeModeNumber'])
        outputVariable = exudyn.OutputVariableType(int(mbs.variables['modeShapeOutputVariable']))
       
        ode2Coords[mbs.variables['modeShapeNodeCoordIndex']+selectedMode] = amplitude * mbs.variables['modeShapeScaleAmplitude']
        
        mbs.systemData.SetODE2Coordinates(ode2Coords, exudyn.ConfigurationType.Visualization)

        SC.visualizationSettings.contour.reduceRange = False
        #check, if automatic range of contour colors shall be recomputed:
        if (mbs.variables['modeShapeLastSetting'][0] != int(mbs.variables['modeShapeModeNumber']) or 
           mbs.variables['modeShapeLastSetting'][1] != int(mbs.variables['modeShapeOutputVariable']) or
           mbs.variables['modeShapeLastSetting'][2] != mbs.variables['modeShapeAmplitude'] or
           mbs.variables['modeShapeLastSetting'][3] != int(mbs.variables['modeShapeComponent'])):
            #print("set=",mbs.variables['modeShapeLastSetting'])
            SC.visualizationSettings.contour.reduceRange = True
            SC.visualizationSettings.general.renderWindowString = renderWindowText+'mode '+str(int(mbs.variables['modeShapeModeNumber']))
        
        mbs.variables['modeShapeLastSetting'] = [int(mbs.variables['modeShapeModeNumber']),
                                                 int(mbs.variables['modeShapeOutputVariable']),
                                                 mbs.variables['modeShapeAmplitude'],
                                                 int(mbs.variables['modeShapeComponent'])]


        SC.visualizationSettings.contour.outputVariable = outputVariable
        SC.visualizationSettings.contour.outputVariableComponent = int(mbs.variables['modeShapeComponent']) #component
        SC.visualizationSettings.openGL.showFaces = (mbs.variables['modeShapeMesh'] & 1) == 1
        SC.visualizationSettings.openGL.showFaceEdges = (mbs.variables['modeShapeMesh'] & 2) == 2
        

        mbs.SendRedrawSignal()
        if not SC.visualizationSettings.general.useMultiThreadedRendering:
            exudyn.DoRendererIdleTasks()
        if mbs.variables['modeShapeSaveImages'] == 0:
            SC.RedrawAndSaveImage() #create images for animation
        else:
            SC.visualizationSettings.exportImages.saveImageFileCounter = 0 #for next mode ...

        dialog.period = mbs.variables['modeShapePeriod']

        if mbs.variables['modeShapeTimeIndex']>=stepsPerPeriod:
           mbs.variables['modeShapeTimeIndex'] = 0
           if mbs.variables['modeShapeRunModus'] > 0: #one cylce or static
               dialog.StartSimulation()
        

    exudyn.StartRenderer()
    if 'renderState' in exudyn.sys: SC.SetRenderState(exudyn.sys['renderState']) #load last model view

    simulationSettings = exudyn.SimulationSettings() #not used, but needed in dialog
     #   self.mbs.sys['solver'].InitializeSolver(self.mbs, self.simulationSettings)
    simulationSettings.solutionSettings.solutionInformation = 'Mode X'

    if not SC.visualizationSettings.general.useMultiThreadedRendering:
        exudyn.DoRendererIdleTasks() #do an update once

    dialog = InteractiveDialog(mbs, simulationSettings=simulationSettings, 
                      simulationFunction=UFshowModes, 
                      dialogItems=dialogItems,
                      title='Animate mode shapes',
                      doTimeIntegration=False, period=period,
                      showTime=False,#done in UFshowModes
                      runOnStart=runOnStart
                      )

    
    #SC.WaitForRenderEngineStopFlag() #not needed, Render window closes when dialog is quit
    exudyn.StopRenderer() #safely close rendering window!




#++++++++++++++++++++++++++++++++++++++++++++
#**function: open interactive dialog and visulation (animate) solution loaded with LoadSolutionFile(...); Change slider 'Increment' to change the automatic increment of time frames; Change mode between continuous run, one cycle (fits perfect for animation recording) or 'Static' (to change Solution steps manually with the mouse); update period also lets you change the speed of animation; Press Run / Stop button to start/stop interactive mode (updating of grpahics)
#**input: 
#  mainSystem: the system used for animation
#  solution: solution dictionary previously loaded with exudyn.utilities.LoadSolutionFile(...); will be played from first to last row; if solution=='', it tries to load the file coordinatesSolutionFileName as stored in mbs.sys['simulationSettings'], which are the simulationSettings of the previous simulation
#  rowIncrement: can be set larger than 1 in order to skip solution frames: e.g. rowIncrement=10 visualizes every 10th row (frame)
#  timeout: in seconds is used between frames in order to limit the speed of animation; e.g. use timeout=0.04 to achieve approximately 25 frames per second
#  runOnStart: immediately go into 'Run' mode
#  runMode: 0=continuous run, 1=one cycle, 2=static (use slider/mouse to vary time steps)
#**output: updates current visualization state, renders the scene continuously (after pressing button 'Run')
#**example:
##HERE, mbs must contain same model as solution stored in coordinatesSolution.txt
#
##adjust autoFitScence, otherwise it may lead to unwanted fit to scene
#SC.visualizationSettings.general.autoFitScene = False
#
#from exudyn.interactive import SolutionViewer #import function
#sol = LoadSolutionFile('coordinatesSolution.txt') #load solution: adjust to your file name
#SolutionViewer(mbs, sol)

def SolutionViewer(mainSystem, solution=[], rowIncrement = 1, timeout=0.04, runOnStart = True, runMode=2):
    from exudyn.utilities import SetSolutionState, LoadSolutionFile
    
    mbs = mainSystem
    SC = mbs.GetSystemContainer()

    if solution==[]:
        if not 'simulationSettings' in mbs.sys:
            raise ValueError('SolutionViewer: no solution file found (already simulated?)!')
        sims = mbs.sys['simulationSettings']
        if not sims.solutionSettings.writeSolutionToFile:
            raise ValueError('SolutionViewer: previous simulation has writeSolutionToFile==False; no solution file available!')
        solution = LoadSolutionFile(sims.solutionSettings.coordinatesSolutionFileName) #load solution file of previous simulation

    nRows = solution['nRows']
    if nRows == 0:
        print('ERROR in SolutionViewer: solution file is empty')
        return
    if (runMode != 0 and runMode != 1 and runMode != 2):
        print('ERROR in SolutionViewer: illegal run mode:', runMode)
        return
    if (rowIncrement < 1) or (rowIncrement > nRows):
        print('ERROR in SolutionViewer: rowIncrement must be at least 1 and must not be larger than the number of rows in the solution file')
    oldUpdateInterval = SC.visualizationSettings.general.graphicsUpdateInterval
    SC.visualizationSettings.general.graphicsUpdateInterval = 0.5*min(timeout, 2e-3) #avoid too small values to run multithreading properly
    mbs.SetRenderEngineStopFlag(False) #not to stop right at the beginning

    # runLoop = False
    # while runLoop and not mainSystem.GetRenderEngineStopFlag():
    #     for i in range(0,nRows,rowIncrement):
    #         if not(mainSystem.GetRenderEngineStopFlag()):
    #             SetSolutionState(mainSystem, solution, i, exudyn.ConfigurationType.Visualization)
    #             exudyn.DoRendererIdleTasks(timeout)

    SetSolutionState(mainSystem, solution, 0, exudyn.ConfigurationType.Visualization)
    exudyn.DoRendererIdleTasks(timeout)

    nSteps = int(nRows)              #only make these steps available in slider!
    maxNSteps = max(500,min(nSteps,1200))     #do not allow more steps, because dialog may be too large ...
    resolution = min(1.,maxNSteps/nSteps) #do not use values smaller than 1
    
    dialogItems = [
                   {'type':'label', 'text':'Solution steps:', 'grid':(1,0)},
                   {'type':'slider', 'range':(0, nSteps-1), 'value':0, 'steps':maxNSteps, 'variable':'solutionViewerStep','resolution': resolution, 'grid':(1,1)},
                    {'type':'label', 'text':'Increment:', 'grid':(2,0)},
                    {'type':'slider', 'range':(1, 200), 'value':rowIncrement, 'steps':200, 'variable':'solutionViewerRowIncrement', 'grid':(2,1)},
                   {'type':'label', 'text':'update period:', 'grid':(3,0)},
                   {'type':'slider', 'range':(0.005, 1), 'value':timeout, 'steps':200, 'variable':'solutionViewerPeriod', 'grid':(3,1)},
                   {'type':'radio', 'textValueList':[('Continuous run',0), ('One cycle',1), ('Static',2)],'value':runMode, 'variable':'solutionViewerRunModus', 'grid': [(4,0),(4,1),(4,2)]},
                   #{'type':'radio', 'textValueList':[('Mesh+Faces',3), ('Faces only',1), ('Mesh only',2)],'value':3, 'variable':'modeShapeMesh', 'grid': [(8,0),(8,1),(8,2)]},
                   {'type':'radio', 'textValueList':[('Record animation',0), ('No recording',1)],'value':1, 'variable':'solutionViewerSaveImages', 'grid': [(5,0),(5,1)]},
                   ]


    mbs.variables['solutionViewerRowIncrement'] = float(rowIncrement)
    mbs.variables['solutionViewerNSteps'] = nSteps
    mbs.variables['solutionViewerSolution'] = solution
    # mbs.variables['solutionViewerStep'] = 0
    # mbs.variables['solutionViewerPeriod'] = timeout

    def UFviewer(mbs, dialog):
        i = int(mbs.variables['solutionViewerStep'])

        # mbs.systemData.SetTime(t, exudyn.ConfigurationType.Visualization)
        SetSolutionState(mainSystem, mbs.variables['solutionViewerSolution'], i, exudyn.ConfigurationType.Visualization)
        #exudyn.DoRendererIdleTasks(timeout)
        # SC.visualizationSettings.contour.reduceRange = False
        
        mbs.SendRedrawSignal()
        exudyn.DoRendererIdleTasks() #as there is no simulation, we must do this for graphicsDataUserFunctions
        # if mbs.variables['modeShapeSaveImages'] == 0:
        #     SC.RedrawAndSaveImage() #create images for animation
        # else:
        #     SC.visualizationSettings.exportImages.saveImageFileCounter = 0 #for next mode ...

        dialog.period = mbs.variables['solutionViewerPeriod']

        if mbs.variables['solutionViewerRunModus'] < 2:
            mbs.variables['solutionViewerStep'] += mbs.variables['solutionViewerRowIncrement']
            # print("step=", mbs.variables['solutionViewerStep'])
    
            #first variable is scale, which contains step
            dialog.variableList[0][0].set(mbs.variables['solutionViewerStep'])
            # for var in dialog.variableList:
            #     #self.mbs.variables[var[1]] = var[0].get()
            #     print(var[1],'=', var[0].get())

        if mbs.variables['solutionViewerSaveImages'] == 0:
            SC.RedrawAndSaveImage() #create images for animation
        # else: #do not reset image counter to allow creating of multi-view images, slow motion, etc.
        #     SC.visualizationSettings.exportImages.saveImageFileCounter = 0 #

        if mbs.variables['solutionViewerStep']>mbs.variables['solutionViewerNSteps']-1.:
            #or (mbs.variables['solutionViewerRunModus'] and mbs.variables['solutionViewerStep']==mbs.variables['solutionViewerNSteps']-1.):
            mbs.variables['solutionViewerStep'] = 0
            dialog.variableList[0][0].set(0)

            SetSolutionState(mainSystem, mbs.variables['solutionViewerSolution'], 0, exudyn.ConfigurationType.Visualization)
            # mbs.SendRedrawSignal()
            # exudyn.DoRendererIdleTasks() #as there is no simulation, we must do this for graphicsDataUserFunctions
            if mbs.variables['solutionViewerRunModus'] == 1: #one cylce ==> stop
                dialog.StartSimulation() #start/stop simulation

        

    exudyn.StartRenderer()
    if 'renderState' in exudyn.sys: SC.SetRenderState(exudyn.sys['renderState']) #load last model view

    simulationSettings = exudyn.SimulationSettings() #not used, but needed in dialog
     #   self.mbs.sys['solver'].InitializeSolver(self.mbs, self.simulationSettings)
    simulationSettings.solutionSettings.solutionInformation = ''

    if not SC.visualizationSettings.general.useMultiThreadedRendering:
        exudyn.DoRendererIdleTasks() #do an update once

    dialog = InteractiveDialog(mbs, simulationSettings=simulationSettings, 
                      simulationFunction=UFviewer, 
                      dialogItems=dialogItems,
                      title='Solution Viewer',
                      doTimeIntegration=False, period=timeout,
                      showTime=True, runOnStart=runOnStart
                      )

    
    #SC.WaitForRenderEngineStopFlag() #not needed, Render window closes when dialog is quit
    exudyn.StopRenderer() #safely close rendering window!




    SC.visualizationSettings.general.graphicsUpdateInterval = oldUpdateInterval #set values back to original






