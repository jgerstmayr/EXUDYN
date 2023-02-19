.. _sec-renderstate:


Render state
============

The system container function \ ``SC.GetRenderState()``\  returns a dictionary with current information on the renderer.
This information is updated whenever the renderer performs redrawing or when according changes in the renderer are performed.

When starting with a empty \ ``mbs``\  and calling \ ``exu.StartRenderer()``\ , the \ ``SC.GetRenderState()``\  will return:

.. code-block:: python

  {'centerPoint': [0.0, 0.0, 0.0],
  'maxSceneSize': 1.0,
  'zoom': 0.4000000059604645,
  'currentWindowSize': [1024, 768],
  'displayScaling': 1.0,
  'modelRotation': [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
  'mouseCoordinates': [0.0, 0.0],
  'openGLcoordinates': [0.0, 0.0],
  'joystickPosition': [0.0, 0.0, 0.0],
  'joystickRotation': [0.0, 0.0, 0.0],
  'joystickAvailable': -1}



There is a set of variables, which can be actively changed by calling  \ ``SC.SetRenderState(renderState)``\  with renderState
containing a modified dictionary:

+  \ ``centerPoint``\ : this is a 3D vector (list/numpy-array) containing the center point for the current view; modifying this vector allows to track objects in you simulation
+  \ ``maxSceneSize``\ : this value is used in the 3D view, clipping objects nearer or farer than this size; also used for perspective view; computed automatically based on the model
+  \ ``zoom``\ : this factor changes the zoom for the renderer, in fact for the size of the view; this leads to smaller objects with larger zoom values
+  \ ``modelRotation``\ : this is the rotation matrix used for model rotation; changing this matrix allows to rotate the model in the view; overwriting modelRotation, centerPoint and zoom with stored values allows to reset to a certain (default) view

Note that other items in renderState are ignored. The read only variables in \ ``SC.GetRenderState()``\  are:

+  \ ``currentWindowSize``\ : contains current window size, which is different from default values in visualizationSettings, if window is scaled by user
+  \ ``displayScaling``\ : contains display scaling (monitor scaling; content scaling) as returned by GLFW and Windows (always 1 on Linux); used internally in renderer to scale fonts
+  \ ``mouseCoordinates``\ : returns 2D vector of current mouse coordinates on screen
+  \ ``openGLcoordinates``\ : returns 3D vector of current mouse coordinates
+  \ ``joystickAvailable``\ : set True, if a special 6D mouse is available (only works for special hardware, e.g., 3Dconnexion space mouse)
+  \ ``joystickPosition``\ : contains current joystick position vector information 
+  \ ``joystickRotation``\ : contains current joystick rotation vector information (linearized rotation angles)


 %to resolve problems with line breaks in GraphicsData
