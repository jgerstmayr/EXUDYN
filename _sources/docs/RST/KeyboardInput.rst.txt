.. _sec-gui-sec-keyboardinput:


Keyboard input
==============

The following table includes the keyboard shortcuts available in the window. 


.. list-table:: \ 
   :widths: auto
   :header-rows: 1

   * - | \ **Key(s)**\ 
     - | action
     - | \ **remarks**\ 
   * - | \ **1,2,3,4 or 5**\ 
     - | visualization update speed
     - | the entered digit controls the visualization update, ranging within 0.02, 0.1 (default), 0.5, 2, and 100 seconds
   * - | \ **CTRL+1 or SHIFT+CTRL+1**\ 
     - | change view
     - | set view in 1/2-plane (+SHIFT: view from opposite side)
   * - | \ **CTRL+2 or SHIFT+CTRL+2**\ 
     - | change view
     - | set view in 1/3-plane (+SHIFT: view from opposite side)
   * - | \ **CTRL+3 or SHIFT+CTRL+3**\ 
     - | change view
     - | set view in 2/3-plane (+SHIFT: view from opposite side)
   * - | \ **CTRL+4 or SHIFT+CTRL+4**\ 
     - | change view
     - | set view in 2/1-plane (+SHIFT: view from opposite side)
   * - | \ **CTRL+5 or SHIFT+CTRL+5**\ 
     - | change view
     - | set view in 3/1-plane (+SHIFT: view from opposite side)
   * - | \ **CTRL+6 or SHIFT+CTRL+6**\ 
     - | change view
     - | set view in 3/2-plane (+SHIFT: view from opposite side)
   * - | \ **A**\ 
     - | zoom all
     - | set zoom such that the whole scene is visible
   * - | \ **CURSOR UP, DOWN, ...**\ 
     - | move scene
     - | use coursor keys to move the scene up, down, left, and right (use CTRL for small movements)
   * - | \ **N**\ 
     - | show/hide nodes
     - | pressing this key switches the visibility of nodes
   * - | \ **CTRL N**\ 
     - | show/hide nodes numbers
     - | pressing this key switches the visibility of nodes numbers
   * - | \ **C**\ 
     - | show/hide connectors
     - | pressing this key switches the visibility of connectors
   * - | \ **CTRL C**\ 
     - | show/hide connector numbers
     - | pressing this key switches the visibility of connector numbers
   * - | \ **B**\ 
     - | show/hide bodies
     - | pressing this key switches the visibility of bodies
   * - | \ **CTRL B**\ 
     - | show/hide bodies numbers
     - | pressing this key switches the visibility of bodies numbers
   * - | \ **M**\ 
     - | show/hide markers
     - | pressing this key switches the visibility of markers
   * - | \ **CTRL M**\ 
     - | show/hide markers numbers
     - | pressing this key switches the visibility of markers numbers
   * - | \ **L**\ 
     - | show/hide loads
     - | pressing this key switches the visibility of loads
   * - | \ **CTRL L**\ 
     - | show/hide loads numbers
     - | pressing this key switches the visibility of loads numbers
   * - | \ **S**\ 
     - | show/hide sensors
     - | pressing this key switches the visibility of sensors
   * - | \ **CTRL S**\ 
     - | show/hide sensors numbers
     - | pressing this key switches the visibility of sensors numbers
   * - | \ **T**\ 
     - | faces / edges mode
     - | switch between faces transparent/ faces transparent + edges / only face edges / full faces with edges / only faces visible
   * - | \ **O**\ 
     - | change center
     - | change center of rotation to current center of the window (affects only current plane coordinates; rotate model to ajust other coordinates)
   * - | \ **Q**\ 
     - | stop solver
     - | current solver is stopped (proceeds to next simulation or end of file); after \ ``visualizationSettings.window.reallyQuitTimeLimit``\  seconds a dialog opens for safety
   * - | \ **SPACE**\ 
     - | pause/continue simulation
     - | pause simulation, e.g. for model inspection; if simulation is paused, it can be continued by pressing space; use SHIFT+SPACE to continuously activate 'continue simulation'
   * - | \ **ESCAPE**\ 
     - | close renderer
     - | stops the simulation (and further simulations) and closes the render window (same as close window); after \ ``visualizationSettings.window.reallyQuitTimeLimit``\  seconds a dialog opens for safety
   * - | \ **X**\ 
     - | execute command
     - | open dialog to enter a python command (in global python scope), see Section :ref:`sec-overview-basics-commandandhelp`\ ; dialog may appear behind the visualization window!
   * - | \ **V**\ 
     - | visualization settings
     - | open dialog to modify visualization settings, see Section :ref:`sec-overview-basics-visualizationsettings`\ ; dialog may appear behind the visualization window!
   * - | \ **F2**\ 
     - | ignore keys
     - |  switch key input on / off; can be used in combination with keyPressUserFunction to make simulators
   * - | \ **F3**\ 
     - | show mouse coordinates
     - | shown in status line
   * - | \ **'.' or KEYPAD +**\ 
     - | zoom in
     - | zoom one step into scene (additionally press CTRL to perform small zoom step)
   * - | \ **',' or KEYPAD -**\ 
     - | zoom out
     - | zoom one step out of scene (additionally press CTRL to perform small zoom step)
   * - | \ **KEYPAD 2/8,4/6,1/9**\ 
     - | rotate scene
     - | about 1, 2 and 3-axis (use CTRL for small rotations)


