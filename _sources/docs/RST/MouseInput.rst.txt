.. _sec-gui-sec-mouseinput:


Mouse input
===========

The following table includes the mouse functions:

.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | \ **Button**\ 
     - | action
     - | \ **remarks**\ 
   * - | \ **left mouse button**\ 
     - | move model 
     - | keep left mouse button pressed to move the model in the current x/y plane
   * - | \ **left mouse button**\ 
     - | select item 
     - | mouse click on any node, object, etc. to see its basic information in status line; selection is deactivated if mouse coordinates are shown (see button F3)
   * - | \ **right mouse button**\ 
     - | rotate model
     - | keep right mouse button pressed to rotate model around current current \ :math:`X_1`\ /\ :math:`X_2`\  axes
   * - | \ **right mouse button**\ 
     - |  show item dictionary
     - | (short) press and release on item
   * - | \ **mouse wheel**\ 
     - | zoom
     - | use mouse wheel to zoom (on touch screens 'pinch-to-zoom' might work as well) 

Current mouse coordinates can be obtained via \ ``SystemContainer.renderer.GetMouseCoordinates()``\ .

6D mouse
--------

Graphics engines, especially in CAD and finite elements allow input of special 3D or 6D mouse devices.
There is a basic interface for so-called 3D mouse / 6D mouse or space mouse, allowing to map the 6D joystick to translation and rotation,
see \ ``visualizationSettings.interactive.useJoystickInput``\  and similar options.
The interface only works, if the device maps 6 coordinates to the joystick input of GLFW (tested with 3DCONNEXION mouse).
