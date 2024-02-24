
***************
SystemContainer
***************




The SystemContainer is the top level of structures in Exudyn. The container holds all (multibody) systems, solvers and all other data structures for computation. A SystemContainer is created by \ ``SC = exu.SystemContainer()``\ , understanding \ ``exu.SystemContainer``\  as a class like Python's internal list class, creating a list instance with \ ``x=list()``\ . Currently, only one container shall be used. In future, multiple containers might be usable at the same time. Regarding the \ **(basic) module access**\ , functions are related to the \ ``exudyn = exu``\  module, see also the introduction of this chapter and this example:

.. code-block:: python
   :linenos:
   
   import exudyn as exu
   #create system container and store by reference in SC:
   SC = exu.SystemContainer() 
   #add MainSystem to SC:
   mbs = SC.AddSystem()

\ The class **SystemContainer** has the following **functions and structures**:

* | **Reset**\ (): 
  | delete all multibody systems and reset SystemContainer (including graphics); this also releases SystemContainer from the renderer, which requires SC.AttachToRenderEngine() to be called in order to reconnect to rendering; a safer way is to delete the current SystemContainer and create a new one (SC=SystemContainer() )
* | **AddSystem**\ (): 
  | add a new computational system
* | **Append**\ (\ *mainSystem*\ ): 
  | append an exsiting computational system to the system container; returns the number of MainSystem in system container
* | **NumberOfSystems**\ (): 
  | obtain number of multibody systems available in system container
* | **GetSystem**\ (\ *systemNumber*\ ): 
  | obtain multibody systems with index from system container
* | **visualizationSettings**:
  | this structure is read/writeable and contains visualization settings, which are immediately applied to the rendering window. 
  | EXAMPLE:
  | SC = exu.SystemContainer()
  | SC.visualizationSettings.autoFitScene=False  
* | **GetDictionary**\ (): 
  | [UNDER DEVELOPMENT]: return the dictionary of the system container data, e.g., to copy the system or for pickling
* | **SetDictionary**\ (\ *systemDict*\ ): 
  | [UNDER DEVELOPMENT]: set system container data from given dictionary; used for pickling
* | **GetRenderState**\ (): 
  | Get dictionary with current render state (openGL zoom, modelview, etc.); will have no effect if GLFW_GRAPHICS is deactivated
  | *Example*:

  .. code-block:: python

     SC = exu.SystemContainer()
     renderState = SC.GetRenderState() 
     print(renderState['zoom'])

* | **SetRenderState**\ (\ *renderState*\ ): 
  | Set current render state (openGL zoom, modelview, etc.) with given dictionary; usually, this dictionary has been obtained with GetRenderState; will have no effect if GLFW_GRAPHICS is deactivated
  | *Example*:

  .. code-block:: python

     SC = exu.SystemContainer()
     SC.SetRenderState(renderState)

* | **RedrawAndSaveImage**\ (): 
  | Redraw openGL scene and save image (command waits until process is finished)
* | **WaitForRenderEngineStopFlag**\ (): 
  | Wait for user to stop render engine (Press 'Q' or Escape-key); this command is used to have active response of the render window, e.g., to open the visualization dialog or use the right-mouse-button; behaves similar as mbs.WaitForUserToContinue()
* | **RenderEngineZoomAll**\ (): 
  | Send zoom all signal, which will perform zoom all at next redraw request
* | **AttachToRenderEngine**\ (): 
  | Links the SystemContainer to the render engine, such that the changes in the graphics structure drawn upon updates, etc.; done automatically on creation of SystemContainer; return False, if no renderer exists (e.g., compiled without GLFW) or cannot be linked (if other SystemContainer already linked)
* | **DetachFromRenderEngine**\ (): 
  | Releases the SystemContainer from the render engine; return True if successfully released, False if no GLFW available or detaching failed
* | **SendRedrawSignal**\ (): 
  | This function is used to send a signal to the renderer that all MainSystems (mbs) shall be redrawn
* | **GetCurrentMouseCoordinates**\ (\ *useOpenGLcoordinates*\  = False): 
  | Get current mouse coordinates as list [x, y]; x and y being floats, as returned by GLFW, measured from top left corner of window; use GetCurrentMouseCoordinates(useOpenGLcoordinates=True) to obtain OpenGLcoordinates of projected plane


