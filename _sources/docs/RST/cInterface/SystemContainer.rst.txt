
***************
SystemContainer
***************

The SystemContainer is the top level of structures in Exudyn. The container holds all (multibody) systems of type \ ``MainSystem``\  and the link to OpenGL renderers and raytracers (every SystemContainer has an independent rendering, while all MainSystems are rendered together).Via the MainSystems it thus contains all computational data. A SystemContainer is created by \ ``SC = exu.SystemContainer()``\ , understanding \ ``exu.SystemContainer``\  as a state machine where MainSystems are added and renderer state machines are processed, similar to the behavior of other Python packages. Usually, only one container shall be used, while multiple containers are possible -- e.g., for reasons of significantly different behavior (drawing, etc.). The SystemContainer contains \ ``visualizationSettings``\  to adjust all kinds of visualization appearance, windows and interactions.

The \ ``visualizationSettings``\ , see Section :ref:`sec-visualizationsettingsmain`\ , can be edited when pressing the key V in the render window and it holds the renderer substructure (type: Renderer) to start and stop the renderer, and to interact with the renderer. Regarding the \ **(basic) module access**\ , functions are related to the \ ``exudyn = exu``\  module, see also the introduction of this chapter and this example:

.. code-block:: python
   :linenos:
   
   import exudyn as exu
   #create system container and store by reference in SC:
   SC = exu.SystemContainer() 
   #add MainSystem to SC:
   mbs = SC.AddSystem()

\ The class **SystemContainer** has the following **functions and structures**:

* | **Reset**\ (): 
  | delete all multibody systems and reset SystemContainer (including graphics); this also releases SystemContainer from the renderer, which requires SC.renderer.Attach() to be called in order to reconnect to rendering; a safer way is to delete the current SystemContainer and create a new one (SC=SystemContainer() )
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
* | **renderer**:
  | The substructure in SystemContainer responsible for rendering (except visualizationSettings)
* | **visualizationSettings**:
  | Structure representing the settings for renderer; for details of visualizationSettings see Section Structures and Settings


