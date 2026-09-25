#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  the SystemContainer class.
#           The calls are recorded by PybindInterface (pybindTypes.py) and replayed by
#           tools/generators/pybindEmitter.py into pybind_manual_classes.h, the stub fragments and
#           the Python-C++ interface documentation.
#
#           DESCRIPTIONS: read definitions/README.md, section "Writing a
#           description", before writing or changing one - what the text may
#           contain, and how it is checked.
#
# Author:   Johannes Gerstmayr
# Date:     2018-05-18 (created in autoGeneratePyBindings.py), 2026-09-14 (moved to definitions/)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

from pybindTypes import *

pb = PybindInterface()

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#currently, only latex + RST binding:
pb.CreateNewRSTfile('SystemContainer')
pyClassStr = 'SystemContainer'
classStr = 'Main'+pyClassStr
pb.BeginCppWrittenByHand() #systemcontainer manually added in C++

pb.DefPyStartClass(classStr, pyClassStr, 
                    r"""The SystemContainer is the top level of structures in Exudyn. The container holds all (multibody) systems of type `MainSystem` and the link to OpenGL renderers and raytracers (every SystemContainer has an independent rendering, while all MainSystems are rendered together).Via the MainSystems it thus contains all computational data. A SystemContainer is created by `SC = exu.SystemContainer()`, understanding `exu.SystemContainer` as a state machine where MainSystems are added and renderer state machines are processed, similar to the behavior of other Python packages. Usually, only one container shall be used, while multiple containers are possible -- e.g., for reasons of significantly different behavior (drawing, etc.). The SystemContainer contains `visualizationSettings` to adjust all kinds of visualization appearance, windows and interactions."""
                    )

pb.AddDocu(r"""The `visualizationSettings`, see [](#sec-visualizationsettingsmain), can be edited when pressing the key V in the render window and it holds the renderer substructure (type: Renderer) to start and stop the renderer, and to interact with the renderer. Regarding the \mybold{(basic) module access}, functions are related to the `exudyn = exu` module, see also the introduction of this chapter and this example:""")

pb.AddDocuCodeBlock(code="""
import exudyn as exu
#create system container and store by reference in SC:
SC = exu.SystemContainer() 
#add MainSystem to SC:
mbs = SC.AddSystem()
""")

pb.DefLatexStartTable(pyClassStr)

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#GENERAL FUNCTIONS

pb.DefPyFunctionAccess(cClass=classStr, pyName='Reset', cName='Reset', 
                        description="delete all multibody systems and reset SystemContainer (including graphics); this also releases SystemContainer from the renderer, which requires SC.renderer.Attach() to be called in order to reconnect to rendering; a safer way is to delete the current SystemContainer and create a new one (SC=SystemContainer() )",
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='AddSystem', cName='AddMainSystem', 
                        description="add a new computational system", 
                        options='py::return_value_policy::reference',
                        returnType='MainSystem',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='Append', cName='AppendMainSystem', 
                        description="append an exsiting computational system to the system container; returns the number of MainSystem in system container", options='py::return_value_policy::reference',
                        argList=['mainSystem'],
                        argTypes=['MainSystem'],
                        returnType='int',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='NumberOfSystems', cName='NumberOfSystems', 
                        description="obtain number of multibody systems available in system container",
                        returnType='int',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetSystem', cName='GetMainSystem', 
                        description="obtain multibody systems with index from system container",
                        argList=['systemNumber'],
                        argTypes=['int'],
                        returnType='MainSystem',
                        )

pb.DefLatexDataAccess('visualizationSettings',r"""this structure is read/writeable and contains visualization settings, which are immediately applied to the rendering window. \tabnewline
    EXAMPLE:\tabnewline
    SC = exu.SystemContainer()\tabnewline
    SC.visualizationSettings.autoFitScene=False  """,
                       dataType = 'VisualizationSettings')

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetDictionary', cName='GetDictionary', 
                        description="[UNDER DEVELOPMENT]: return the dictionary of the system container data, e.g., to copy the system or for pickling",
                        argList=[],
                        argTypes=[],
                        returnType='dict',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetDictionary', cName='SetDictionary', 
                        description="[UNDER DEVELOPMENT]: set system container data from given dictionary; used for pickling",
                        argList=['systemDict'],
                        argTypes=['dict'],
                        returnType='None',
                        )


#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#keep for compatibility until mid 2027:

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetRenderState', cName='PyGetRenderState', 
                        description="DEPRECATED; Get dictionary with current render state (openGL zoom, modelview, etc.); will have no effect if GLFW_GRAPHICS is deactivated",
                        returnType='dict',
                        addDocu=False,
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetRenderState', cName='PySetRenderState', 
                        description="DEPRECATED; Set current render state (openGL zoom, modelview, etc.) with given dictionary; usually, this dictionary has been obtained with GetRenderState; waitForRendererFullStartup is used to wait at startup for the first frame to be drawn (and zoom all to be set), but be be set False in case of performance issues; will have no effect if GLFW_GRAPHICS is deactivated",
                        argList=['renderState','waitForRendererFullStartup'],
                        argTypes=['dict','bool'],
                        defaultArgs=['','True'],
                        returnType='None',
                        addDocu=False,
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='RedrawAndSaveImage', cName='RedrawAndSaveImage', 
                        description="DEPRECATED; Redraw openGL scene and save image (command waits until process is finished)",
                        returnType='None',
                        addDocu=False,
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='WaitForRenderEngineStopFlag', cName='WaitForRenderEngineStopFlag', 
                        description="DEPRECTED; Wait for user to stop render engine (Press 'Q' or Escape-key); this command is used to have active response of the render window, e.g., to open the visualization dialog or use the right-mouse-button; behaves similar as mbs.WaitForUserToContinue()",
                        returnType='bool',
                        addDocu=False,
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='RenderEngineZoomAll', cName='PyZoomAll', 
                        description="DEPRECATED; Send zoom all signal, which will perform zoom all at next redraw request",
                        returnType='None',
                        addDocu=False,
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='AttachToRenderEngine', cName='AttachToRenderEngine', 
                        description="DEPRECATED; Links the SystemContainer to the render engine, such that the changes in the graphics structure drawn upon updates, etc.; done automatically on creation of SystemContainer; return False, if no renderer exists (e.g., compiled without GLFW) or cannot be linked (if other SystemContainer already linked)",
                        returnType='bool',
                        addDocu=False,
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='DetachFromRenderEngine', cName='DetachFromRenderEngine', 
                        description="DEPRECATED; Releases the SystemContainer from the render engine; return True if successfully released, False if no GLFW available or detaching failed",
                        returnType='bool',
                        addDocu=False,
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SendRedrawSignal', cName='SendRedrawSignal', 
                        description="DEPRECATED; This function is used to send a signal to the renderer that all MainSystems (mbs) shall be redrawn",
                        returnType='None',
                        addDocu=False,
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetCurrentMouseCoordinates', cName='PyGetCurrentMouseCoordinates', 
                        description="DEPRECATED; Get current mouse coordinates as list [x, y]; x and y being floats, as returned by GLFW, measured from top left corner of window; use GetCurrentMouseCoordinates(useOpenGLcoordinates=True) to obtain OpenGLcoordinates of projected plane",
                        argList=['useOpenGLcoordinates'],
                        argTypes=['bool'],
                        defaultArgs=['False'],
                        returnType='[float,float]',
                        addDocu=False,
                        )
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

pb.DefLatexDataAccess('renderer','The substructure in SystemContainer responsible for rendering (except visualizationSettings)',
                        dataType='Renderer')

pb.DefLatexDataAccess('visualizationSettings','Structure representing the settings for renderer; for details of visualizationSettings see Section Structures and Settings',
                        dataType='VisualizationSettings')

pb.EndCppWrittenByHand()  #system container manually added 


pb.DefLatexFinishTable()#only finalize latex table

pb.EndStubSection()
