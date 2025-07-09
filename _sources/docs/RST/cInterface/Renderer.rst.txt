
.. _sec-sc-renderer:


********
Renderer
********




This is the substructure of SystemContainer that collects rendering and visualization interaction. Rendering is done for a SystemContainer, which may include several MainSystems. Note that visualizationSettings are directly accessible from the SystemContainer.

.. code-block:: python
   :linenos:
   
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

\ The class **MainRenderer** has the following **functions and structures**:

* | **Start**\ (\ *verbose*\  = 0): 
  | Start OpenGL rendering engine (in separate thread) for visualization of rigid or flexible multibody system; use verbose=1 to output information during OpenGL window creation; verbose=2 produces more output and verbose=3 gives a debug level; some of the information will only be seen in windows command (powershell) windows or linux shell, but not inside iPython of e.g. Spyder
* | **Stop**\ (): 
  | Stop OpenGL rendering engine; uses timeout in multithreading.
* | **IsActive**\ (): 
  | returns True if GLFW renderer is available and running; otherwise False
* | **Attach**\ (): 
  | Links the SystemContainer to the render engine, such that the changes in the graphics structure drawn upon updates, etc.; done automatically on creation of SystemContainer; return False, if no renderer exists (e.g., compiled without GLFW) or cannot be linked (if other SystemContainer already linked)
* | **Detach**\ (): 
  | DEPRECATED; Releases the SystemContainer from the render engine; return True if successfully released, False if no GLFW available or detaching failed
* | **DoIdleTasks**\ (\ *waitSeconds*\  = -1., \ *printPauseMessage*\  = True): 
  | Interrupt further computation until user input (Space, 'Q', Escape-key), representing a PAUSE function; this command runs a loop in the background to have active response of the render window, e.g., to open the visualization dialog or use the right-mouse-button; replaces former SC.WaitForRenderEngineStopFlag() and mbs.WaitForUserToContinue(); call this function in order to interact with Renderer window; use waitSeconds in order to run this idle tasks while animating a model (e.g. waitSeconds=0.04), use waitSeconds=0 without waiting, or use waitSeconds=-1 (default) to wait until window is closed
  | *Example*:

  .. code-block:: python

     SC.renderer.DoIdleTasks()

* | **ZoomAll**\ (): 
  | Send zoom all signal, which will perform zoom all at next redraw request
* | **RedrawAndSaveImage**\ (): 
  | Redraw openGL scene and save image (command waits until process is finished)
* | **SendRedrawSignal**\ (): 
  | This function is used to send a signal to the renderer that all MainSystems (mbs) shall be redrawn
* | **GetRenderCount**\ (): 
  | Returns the number of rendered OpenGL images; can be used to determine if image has been drawn by comparing to previous counter; also shows that first image has been drawn (needed for zoom all)
* | **GetState**\ (): 
  | Get dictionary with current render state (openGL zoom, modelview, etc.); will have no effect if GLFW_GRAPHICS is deactivated
  | *Example*:

  .. code-block:: python

     SC = exu.SystemContainer()
     renderState = SC.renderer.GetState() 
     print(renderState['zoom'])

* | **SetState**\ (\ *renderState*\ , \ *waitForRendererFullStartup*\  = True): 
  | Set current render state (openGL zoom, modelview, etc.) with given dictionary; usually, this dictionary has been obtained with GetRenderState; waitForRendererFullStartup is used to wait at startup for the first frame to be drawn (and zoom all to be set), but be be set False in case of performance issues; will have no effect if GLFW_GRAPHICS is deactivated
  | *Example*:

  .. code-block:: python

     SC = exu.SystemContainer()
     SC.renderer.SetState(renderState)

* | **GetMouseCoordinates**\ (\ *useOpenGLcoordinates*\  = False): 
  | Get current mouse coordinates as list [x, y]; x and y being floats, as returned by GLFW, measured from top left corner of window; use GetCurrentMouseCoordinates(useOpenGLcoordinates=True) to obtain OpenGLcoordinates of projected plane
* | **GetItemSelection**\ (\ *resetSelection*\  = True): 
  | Get selected item in render state; option to reset selected item afterwards; item is selected in render window by clicking left mouse button; returns [mbs number, ItemType, ItemIndex, depth] where depth is the Z-depth in the current view; note that only items of the categories activated in visualizationSettings.interactive.selectionLeftMouseItemTypes are returned; if itemType == 0 if no item has been selected
* | **materials**:
  | GraphicsMaterialList used for raytracer (possibly for OpenGL in future); list can be accessed with [] operator, reset and extended. Note that after Reset() there are at least 10 materials available, which are copied from visualizationSettings.raytracer.materials which are synced continuously



