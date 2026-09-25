#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN definition file
#
# Details:  the Renderer class.
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
pb.CreateNewRSTfile('Renderer')
pyClassStr = 'Renderer'
classStr = 'Main'+pyClassStr

pb.DefPyStartClass(classStr,pyClassStr, 
                    "The Renderer is the substructure of SystemContainer that collects rendering and visualization interaction, in particular starting and stopping the renderer, image retrieval and materials. "+
                    "Rendering is done for a single SystemContainer, which may include several MainSystems. "+
                    "Note that visualizationSettings are directly accessible from the SystemContainer.", 
                    labelName='sec:SC:renderer',
                    forbidPythonConstructor=True)

#pb.AddDocu()

pb.AddDocuCodeBlock(code="""
import exudyn as exu
from exudyn.utilities import *
import exudyn.graphics as graphics
SC = exu.SystemContainer()
mbs = SC.AddSystem()
mbs.CreateMassPoint(physicsMass=1)

mbs.Assemble()
SC.visualizationSettings.general.drawWorldBasis = True
SC.renderer.Start()
SC.renderer.DoIdleTasks() #wait until user presses space, etc.
mbs.SolveDynamic()
SC.renderer.Stop()
""")

pb.DefLatexStartTable(classStr)

#+++++++++++++++++++++++++++++++++
#General functions:
pb.DefPyFunctionAccess(cClass=classStr, pyName='Start', cName='Start', 
                        description="Start OpenGL rendering engine (in separate thread) for visualization of rigid or flexible multibody system; use verbose=1 to output information during OpenGL window creation; verbose=2 produces more output and verbose=3 gives a debug level; some of the information will only be seen in windows command (powershell) windows or linux shell, but not inside iPython of e.g., Spyder",
                        argList=['verbose'],
                        defaultArgs=['0'],
                        returnType='bool',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='Stop', cName='Stop', 
                        description="Stop OpenGL rendering engine; uses timeout in multithreading.",
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='IsActive', cName='IsActive', 
                        description="returns True if GLFW renderer is available and running; otherwise False",
                        #argList=[''],
                        #defaultArgs=[''],
                        returnType='bool',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='Attach', cName='Attach', 
                        description="Links the SystemContainer to the render engine, such that the changes in the graphics structure drawn upon updates, etc.; done automatically on creation of SystemContainer; return False, if no renderer exists (e.g., compiled without GLFW) or cannot be linked (if other SystemContainer already linked)",
                        returnType='bool',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='Detach', cName='Detach', 
                        description="DEPRECATED; Releases the SystemContainer from the render engine; return True if successfully released, False if no GLFW available or detaching failed",
                        returnType='bool',
                        )


pb.DefPyFunctionAccess(cClass=classStr, pyName='DoIdleTasks', cName='DoIdleTasks', 
                        description="Interrupt further computation until user input (Space, 'Q', Escape-key), representing a PAUSE function; this command runs a loop in the background to have active response of the render window, e.g., to open the visualization dialog or use the right-mouse-button; replaces former SC.WaitForRenderEngineStopFlag() and mbs.WaitForUserToContinue(); call this function in order to interact with Renderer window; use waitSeconds in order to run this idle tasks while animating a model (e.g., waitSeconds=0.04), use waitSeconds=0 without waiting, or use waitSeconds=-1 (default) to wait until window is closed; NOTE: may also first initialize renderState from visualizationSettings (if renderer is inactive)",
                        example = 'SC.renderer.DoIdleTasks()',
                        argList=['waitSeconds','printPauseMessage'],
                        argTypes=['float','bool'],
                        defaultArgs=['-1.','True'],
                        returnType='bool',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='EnableView', cName='EnableView', 
                        description="Enables a specified view (1,2 or 3) additionally to the main view (0); a window is created if createWindow=True, otherwise, the view is only enabled to use it with the raytracer; NOTE: additional views cause slightly more workload for the renderer which is why they are disabled by default; failure to create the view usually results in an exception; to check whether the view exists, check the according RenderState regarding viewEnabled and windowOpen",
                        argList=['viewID','createWindow'],
                        argTypes=['int','bool'],
                        defaultArgs=['','True'],
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='DisableView', cName='DisableView', 
                        description="Disables a specified view (1,2 or 3) and (if created) closes the according window; NOTE: in case of renderer.Stop(), all views are closed; NOTE: if you still aim to use the view without the window, you have to enable the view again EnableView(..., createWindow=False)",
                        argList=['viewID'],
                        argTypes=['int'],
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='ZoomAll', cName='ZoomAll', 
                        description="Send zoom all signal, which will perform zoom all at next redraw request; if renderer is inactive (renderer.IsActive()=0), it will perform computations for renderState, thus at the next RedrawAndGetImage() having the full view as with the OpenGL renderer; NOTE: in case of OpenGL, call ZoomAll() after renderer.Start(); NOTE: may also first initialize renderState from visualizationSettings (if renderer is inactive)",
                        argList=['computeMaxScene','viewID'],
                        argTypes=['bool','int'],
                        defaultArgs=['True',mainViewID],
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetModelView', cName='SetModelView', 
                        description="Function to adjusts the current view in renderState; rotationVector and centerPoint transform the modelView while zoom equals the visible scene height; rotationVector is the axis of rotation times the angle in radiant; if zoom=0 then zoom will be computed automatically like in autoFitScene with openGL; use this function in particular for raytracing before RedrawAndGetImage() or with regular OpenGL after renderer.Start(); you can also store the renderState and write the full renderState alternatively; NOTE: may also first initialize renderState from visualizationSettings (if renderer is inactive)",
                        example = r"""SC.renderer.SetModelView(10,[0,0,pi],[2.5,0,0])\\image=SC.renderer.RedrawAndGetImage()""",
                        argList=['zoom', 'rotationVector', 'centerPoint','viewID'],
                        argTypes=['float', vector3D, vector3D, 'int'],
                        defaultArgs=['0','(std::vector<Real>)Vector3D({0,0,0})', '(std::vector<Real>)Vector3D({0,0,0})',mainViewID],
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='RedrawAndSaveImage', cName='RedrawAndSaveImage', 
                        description="Redraw openGL scene and save image (command waits until process is finished); uses the current rendering engine (OpenGL or raytracer).",
                        argList=['viewID'],
                        argTypes=['int'],
                        defaultArgs=[mainViewID],
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='RedrawAndGetImage', cName='RedrawAndGetImage', 
                        description="Redraw scene and return image in numpy-format, containing a 3-dimensional array with 3 matrices of RGB channels; the shape is according to (height,width,3) where height and width represent the window pixels defined in visualizationSettings.window.renderWindowSize (Note: in case that raytracer.imageSizeFactor>1 the retrieved image size is smaller by the imageSizeFactor!); command waits until process is finished; if useRaytracer=False, the openGL render is used and the render window needs to be opened before; if useRaytracer=True, the software raytracer is used which runs completely without GLFW and OpenGL (e.g., on a supercomputer); NOTE: in case of useRaytracer=True the displayScaling factor is only available if the OpenGL renderer has been opened once; otherwise e.g., SC.renderer.SetState({'displayScaling':1.5}) has to be used to adjust it; NOTE: may also first initialize renderState from visualizationSettings (if renderer is inactive)",
                        example = r"""import matplotlib.pyplot as plt\\image=SC.renderer.RedrawAndGetImage()\\plt.imshow(image)\\plt.axis('off')\\plt.show()""",
                        argList=['useRaytracer','viewID'],
                        argTypes=['bool','int'],
                        defaultArgs=['False',mainViewID],
                        returnType='NDArray[np.uint8]',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetState', cName='GetState', 
                        description="Get dictionary with current render state (openGL zoom, modelview, etc.)",
                        example = r"""SC = exu.SystemContainer()\\renderState = SC.renderer.GetState() \\print(renderState['zoom'])""",
                        argList=['viewID'],
                        argTypes=['int'],
                        defaultArgs=[mainViewID],
                        returnType='dict',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetState', cName='SetState', 
                        description="Set current render state (openGL zoom, modelview, etc.) with given dictionary; usually, this dictionary has been obtained with GetRenderState; waitForRendererFullStartup is used to wait at startup for the first frame to be drawn (and zoom all to be set), but be be set False in case of performance issues; NOTE: before setting available state values, this function may also first initialize renderState from visualizationSettings (if renderer is inactive)",
                        example = r"""SC = exu.SystemContainer()\\SC.renderer.SetState(renderState)""",
                        argList=['renderState','waitForRendererFullStartup','viewID'],
                        argTypes=['dict','bool','int'],
                        defaultArgs=['','True',mainViewID],
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='RestoreSavedState', cName='RestoreSavedState',
                        description="Restore the render state that exu.sys holds: renderer.Stop() saves the state of every open view there, so this brings back the view of the previous run; returns False if nothing has been saved yet, which is not an error but the ordinary case on the first run of a script; it replaces the two lines that tested for renderState in exu.sys and called SetState with it",
                        example = r"""SC = exu.SystemContainer()\\SC.renderer.Start()\\SC.renderer.RestoreSavedState()""",
                        argList=['viewID'],
                        argTypes=['int'],
                        defaultArgs=[mainViewID],
                        returnType='bool',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetMouseCoordinates', cName='GetMouseCoordinates', 
                        description="Get current mouse coordinates as list [x, y]; x and y being floats, as returned by GLFW, measured from top left corner of window; use GetCurrentMouseCoordinates(useOpenGLcoordinates=True) to obtain OpenGLcoordinates of projected plane",
                        argList=['useOpenGLcoordinates','viewID'],
                        argTypes=['bool','int'],
                        defaultArgs=['False',mainViewID],
                        returnType='[float,float]',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetItemSelection', cName='GetItemSelection', 
                        description="Get selected item in render state; option to reset selected item afterwards; item is selected in render window by clicking left mouse button; returns [mbs number, ItemType, ItemIndex, depth] where depth is the Z-depth in the current view; note that only items of the categories activated in visualizationSettings.interactive.selectionLeftMouseItemTypes are returned; NOTE: if itemType == 0, no item has been selected",
                        argList=['resetSelection','viewID'],
                        argTypes=['bool','int'],
                        defaultArgs=['True',mainViewID],
                        returnType='[int,int,int,float]',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='ResetState', cName='ResetState', 
                        description="Reset renderState in all views to default values using current visualizationSettings; usually this does not have to be called!",
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='SendRedrawSignal', cName='SendRedrawSignal', 
                        description="This function is used to send a signal to the renderer that all MainSystems (mbs) shall be redrawn",
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetRenderCount', cName='GetRedrawCount', 
                        description="Returns the number of rendered OpenGL images; can be used to determine if image has been drawn by comparing to previous counter; also shows that first image has been drawn (needed for zoom all)",
                        returnType='bool',
                        )


pb.CppCode('        .def_readwrite("materials", &MainRenderer::materials, py::return_value_policy::reference);\n')

pb.DefLatexDataAccess('materials','GraphicsMaterialList used for raytracer (possibly for OpenGL in future); list can be accessed with [] operator, reset and extended. Note that after Reset() there are at least 10 materials available, which are copied from visualizationSettings.raytracer.materials which are synced continuously',
                        dataType='GraphicsMaterialList', isTopLevel = False)

pb.DefPyFinishClass('Renderer')
#pb.DefLatexFinishTable() #only finalize latex table

pb.EndStubSection()
