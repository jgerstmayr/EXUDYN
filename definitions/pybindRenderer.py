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
from definitionTypes import Deprecated #every deprecated function: since and the year of removal (#2807)

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

pb.AddDocuNotebook('python/Notebooks/reference/systemContainer.ipynb', part='renderer')

pb.DefStartTable(classStr)

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
                        deprecated=Deprecated('1.11.0', 2029),
                        description="DEPRECATED; Releases the SystemContainer from the render engine; return True if successfully released, False if no GLFW available or detaching failed",
                        returnType='bool',
                        )


pb.DefPyFunctionAccess(cClass=classStr, pyName='StopSimulation', cName='StopSimulation', 
                        description="Stop the simulation as closing the render window does: a running simulation ends after its current step, quietly and without an error, as if the user had stopped it; with forceQuit=True (default) a simulation that starts later ends before its first step as well, until mbs.SetRenderEngineStopFlag(False) resets it - which is what pressing Escape or closing the window does. Works without an open renderer, e.g. from a user function, another thread or a test",
                        example = 'SC.renderer.StopSimulation()',
                        argList=['forceQuit'],
                        argTypes=['bool'],
                        defaultArgs=['True'],
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='DoIdleTasks', cName='DoIdleTasks', 
                        description="Interrupt further computation until user input (Space, 'Q', Escape-key), representing a PAUSE function; this command runs a loop in the background to have active response of the render window, e.g., to open the visualization dialog or use the right-mouse-button; replaces former SC.WaitForRenderEngineStopFlag() and mbs.WaitForUserToContinue(); call this function in order to interact with Renderer window; use waitSeconds in order to run this idle tasks while animating a model (e.g., waitSeconds=0.04), use waitSeconds=0 without waiting, or use waitSeconds=-1 (default) to wait until window is closed; pollEvents=False: the renderer draws and runs queued commands, but leaves the window events to the event loop that called it - a tkinter dialog calling this from its after-callback on macOS, where both share one event loop; NOTE: may also first initialize renderState from visualizationSettings (if renderer is inactive)",
                        example = 'SC.renderer.DoIdleTasks()',
                        argList=['waitSeconds','printPauseMessage','pollEvents'],
                        argTypes=['float','bool','bool'],
                        defaultArgs=['-1.','True','True'],
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

pb.DefPyFunctionAccess(cClass=classStr, pyName='ZoomToItem', cName='ZoomToItem',
                        description="Center the view on the graphics of one item and zoom such that it fills the window, as ZoomAll does for the scene (#3027); itemIndex is the NodeIndex, ObjectIndex, MarkerIndex, LoadIndex or SensorIndex the Add functions return, mbsNumber the number of its system in the SystemContainer; with an active renderer this happens at its next update and the function returns True, else now, and it returns False if the item draws nothing; in the render window, CTRL+A zooms to the item selected with the left mouse button",
                        example = r"""SC.renderer.ZoomToItem(oBody)""",
                        argList=['itemIndex', 'mbsNumber', 'viewID'],
                        argTypes=['NodeIndex | ObjectIndex | MarkerIndex | LoadIndex | SensorIndex', 'int', 'int'],
                        defaultArgs=['', '0', mainViewID],
                        returnType='bool',
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

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetGraphicsData', cName='GetGraphicsData',
                        description=r"""the drawing elements of the scene as a dictionary of numpy arrays: what the renderer would draw, as data. It is built without a window, the way `RedrawAndGetImage(True)` builds it, so it works with `EXUDYN_SUPPRESS_UI_WINDOW_OPEN` set and needs no OpenGL; zoom, model view and window size are not part of it. The keys `lines`, `spheres`, `circles`, `texts` and `triangles` each hold a dictionary whose `items` is an int array of shape (n,3) - the system number, the `exudyn.ItemType` value and the index of the item that drew the element, or `[-1, 0, code]` for a static element of the scene; then `points` and `colors` (RGBA, per vertex for lines and triangles), and per kind `radius` and `resolution` (spheres), `radius` and `numberOfSegments` (circles), `fontSize`, `offset` and the list `text` (texts), `normals` (triangles). The curved shapes are kept as they are under `triangles6` (6-node triangles: `points`, `normals` and `colors` of shape (n,6,3) / (n,6,4), nodes as in the key `triangles6` of a TriangleList; the normals given, else those of the geometry) and `lines3` (quadratic lines: `points` (n,3,3) and `colors` (n,3,4), the end points and then the mid node); `texturedTriangles` holds the triangles with a texture as `triangles` does, and `textureCoordinates` (n,3,2) and the list `texture` of the names of their textures (#3030); with`flatShapes=True` these two are empty and their shapes are among `triangles` and `lines` instead, refined once - 4 triangles on the 6 nodes, 2 lines on the 3 nodes - independently of the tiling settings, which only the renderers apply. `formatVersion` is 1. It is the oracle of the graphics tests: the number of elements per item is exact, coordinates are floats and are compared with a tolerance""",
                        example = r"""data = SC.renderer.GetGraphicsData()\\print(len(data['triangles']['items']), 'triangles')""",
                        argList=['flatShapes'],
                        defaultArgs=['False'],
                        returnType='dict',
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


pb.DefPyFunctionAccess(cClass=classStr, pyName='AddTexture', cName='AddTexture',
                        description="Add a texture by its name, or replace the image of a name, for the triangle lists that refer to it with the key 'texture' (#3029); the textures are shared by all SystemContainers and are added before the graphics data that uses them; image is a numpy array of shape (height, width) or (height, width, channels) with 1, 3 or 4 channels, of uint8 or of floats in [0, 1]; its first row is the top of the image, at the texture coordinate v=1; interpretation is a TextureInterpretation, which says how the channels are drawn; flags are TextureFlags combined with |, e.g. exu.TextureFlags.Nearest | exu.TextureFlags.Clamp, or 0 for none; names starting with 'exudyn.' are reserved",
                        example = r"""import numpy as np\\image = np.zeros((64,64,3), dtype=np.uint8)\\image[:32,:,0] = 255 #upper half red\\SC.renderer.AddTexture('redTop', image, interpretation=exu.TextureInterpretation.RGB, flags=exu.TextureFlags.Nearest)""",
                        argList=['name', 'image', 'interpretation', 'flags'],
                        argTypes=['str', 'ArrayLike', 'TextureInterpretation', 'TextureFlags | int'],
                        defaultArgs=['', '', 'exu.TextureInterpretation::RGBA', '0'],
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='RemoveTexture', cName='RemoveTexture',
                        description="Remove a texture by its name; triangles that refer to it are drawn without texture; returns False if no texture has this name",
                        argList=['name'],
                        argTypes=['str'],
                        returnType='bool',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetTextureNames', cName='GetTextureNames',
                        description="The names of the textures added with AddTexture, in the order they were added",
                        returnType='list[str]',
                        )

pb.CppCode('        .def_readwrite("materials", &MainRenderer::materials, py::return_value_policy::reference)\n')

pb.DefDataAccess('materials','GraphicsMaterialList used for raytracer (possibly for OpenGL in future); list can be accessed with [] operator, reset and extended. Note that after Reset() there are at least 10 materials available, which are copied from visualizationSettings.raytracer.materials which are synced continuously',
                        dataType='GraphicsMaterialList', isTopLevel = False)

pb.CppCode('        .def_readwrite("scopes", &MainRenderer::scopes, py::return_value_policy::reference)\n')

pb.DefDataAccess('scopes','RendererScopes: lines, texts and points drawn in the scopes of the views, in the coordinates of their graphs (#3237)',
                        dataType='RendererScopes', isTopLevel = False)

pb.DefPyFinishClass('Renderer')
#pb.DefFinishTable() #only finalize latex table

#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
pyClassStr = 'RendererScopes'
classStr = 'Main'+pyClassStr

pb.DefPyStartClass(classStr, pyClassStr,
                    "The static graphics of the scopes, `SC.renderer.scopes` (#3237): lines, texts and points in the coordinates of the graph of a scope - "+
                    "time or the values of the x-sensor, and the y-values -, such as a threshold, an analytic reference or a marked value. "+
                    "They are one list of dicts, each with the view and the scope it is drawn in; they are clipped to the plot area, do not change the "+
                    "automatic range of the axes, and are drawn by OpenGL and the raytracer and in saved images, also by the solution viewer. "+
                    "They are not settings: they are neither stored in `config.json` nor in the settings dialog, and stay until cleared or `SC.Reset()`. "+
                    "Each dict has the key `type`, `'Line'`, `'Text'` or `'Point'`, and the keys of its type; a key not listed is refused. "+
                    "All types: `view` (0), `scope` (0), `color` (RGBA, [0,0,0,1]), `front` (True: drawn over the sensor lines; False for lines and points, True for texts). "+
                    "Line: `points` (a list of at least two [x, y], a polyline), `width` (in pixels, -1: the `lineWidth` of the scope), `name` (with a name, the line has an entry in the legend, after the sensor lines). "+
                    "Text: `position` ([x, y], the bottom left corner of the first character), `text`, `size` (relative to the tick labels, 1). "+
                    "Point: `position` ([x, y], the center), `radius` (in pixels, 4), `width` (the line width of the circle, -1).",
                    subSection=True, labelName='sec:RendererScopes',
                    forbidPythonConstructor=True)

pb.DefStartTable(pyClassStr)

pb.DefPyFunctionAccess(cClass=classStr, pyName='SetGraphics', cName='PySetGraphics',
                        description="replace the static graphics of all scopes by a list of dicts; all are checked before anything is replaced",
                        example = r"""SC.renderer.scopes.SetGraphics([{'type':'Line', 'points':[[0, 0.5], [10, 0.5]], 'color':[1, 0, 0, 1], 'name':'limit'},\\    {'type':'Text', 'position':[1, 0.55], 'text':'limit', 'scope':0}])""",
                        argList=['items'],
                        argTypes=['list[dict]'],
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='AddGraphics', cName='PyAddGraphics',
                        description="append a dict or a list of dicts to the static graphics",
                        example = r"""SC.renderer.scopes.AddGraphics({'type':'Point', 'position':[2.5, 0.8], 'radius':5, 'color':[0, 0, 1, 1], 'view':0, 'scope':1})""",
                        argList=['items'],
                        argTypes=['dict | list[dict]'],
                        returnType='None',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='GetGraphics', cName='PyGetGraphics',
                        description="the static graphics as a list of dicts, with all the keys of their types",
                        returnType='list[dict]',
                        )

pb.DefPyFunctionAccess(cClass=classStr, pyName='ClearGraphics', cName='ClearGraphics',
                        description="remove the static graphics of a view and a scope; -1 for all views or all scopes, the default: all",
                        argList=['view', 'scope'],
                        argTypes=['int', 'int'],
                        defaultArgs=['-1', '-1'],
                        returnType='None',
                        )

pb.DefPyFinishClass('RendererScopes')

pb.EndStubSection()
