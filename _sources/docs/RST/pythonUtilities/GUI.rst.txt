
.. _sec-module-gui:

Module: GUI
===========

Helper functions and classes for graphical interaction with Exudyn

- Author:    Johannes Gerstmayr 
- Date:      2020-01-25 
- Notes:     This is an internal library, which is only used inside Exudyn for modifying settings. 


.. _sec-gui-gettkrootandnewwindow:

Function: GetTkRootAndNewWindow
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`GetTkRootAndNewWindow <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/GUI.py\#L53>`__\ ()

- | \ *function description*\ :
  | get new or current root and new window app; return list of [tkRoot, tkWindow, tkRuns]



----


.. _sec-gui-tkrootexists:

Function: TkRootExists
^^^^^^^^^^^^^^^^^^^^^^
`TkRootExists <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/GUI.py\#L65>`__\ ()

- | \ *function description*\ :
  | this function returns True, if tkinter has already a root window (which is assumed to have already a mainloop running)



----


.. _sec-gui-editdictionarywithtypeinfo:

Function: EditDictionaryWithTypeInfo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`EditDictionaryWithTypeInfo <https://github.com/jgerstmayr/EXUDYN/blob/master/main/pythonDev/exudyn/GUI.py\#L642>`__\ (\ ``settingsStructure``\ , \ ``exu = None``\ , \ ``dictionaryName = 'edit'``\ )

- | \ *function description*\ :
  | edit dictionaryData and return modified (new) dictionary
- | \ *input*\ :
  | \ ``settingsStructure``\ : hierarchical settings structure, e.g., SC.visualizationSettings
  | \ ``exu``\ : exudyn module
  | \ ``dictionaryName``\ : name displayed in dialog
- | \ *output*\ :
  | returns modified dictionary, which can be used, e.g., for SC.visualizationSettings.SetDictionary(...)

