#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# File:     The keyboard and mouse bindings of the render window, in one place
#
# Details:  What the render window does when a key is pressed was written down three times:
#           src/Graphics/GlfwClient.cpp implements it, docs/manual/GUI.md tabulated it, and the
#           help dialog printed its own text. Two of the three were prose that nothing kept in
#           step with the first, and they had drifted - keys that exist were in neither, and the
#           keypad rotation keys were named wrongly in both (revision2026b step RG6.2.6, #2591).
#
#           This table is the one source of the two prose copies: the help dialog builds its text
#           from it (RendererHelpText), and tools/generators/keyBindingsEmitter.py writes the
#           tables of the documentation from it. That emitter also compares the table with
#           GlfwClient.cpp and reports what one of them has and the other does not, which is the
#           only way the third copy can be kept honest without moving it.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-23
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# Notes:    This is an internal library, which is only used inside Exudyn for the render window.
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import textwrap
from collections import namedtuple

__all__ = [
    'KeyBinding', 'mouseBindings', 'keyBindings', 'RendererHelpText',
    ]

#keys:    what a user presses, in the spelling both the dialog and the documentation use
#action:  what it does, in three or four words - this is the second column of the table
#remarks: the whole story, for the documentation
#short:   the part of the story the help dialog shows; '' means the action alone is enough
#glfw:    the GLFW key names this binding is implemented with, and the modifier - the emitter
#         compares this with GlfwClient.cpp, and '' means "not a key of the render window"
KeyBinding = namedtuple('KeyBinding', ['keys', 'action', 'remarks', 'short', 'glfw'],
                        defaults=['', ''])

mouseBindings = [
    KeyBinding('left mouse button', 'move model',
               'keep left mouse button pressed to move the model in the current x/y plane',
               'hold and drag'),
    KeyBinding('left mouse button', 'select item',
               'mouse click on any node, object, etc. to see its basic information in the status '
               'line; selection is deactivated if mouse coordinates are shown (see F3)',
               'click; deactivated if mouse coordinates are shown'),
    KeyBinding('right mouse button', 'rotate model',
               'keep right mouse button pressed to rotate the model around the current '
               '$X_1$/$X_2$ axes', 'hold and drag'),
    KeyBinding('right mouse button', 'show item dictionary',
               '(short) press and release on an item; opens the edit dialog if it is activated '
               'in visualizationSettings', 'click: open edit dialog, if activated'),
    KeyBinding('mouse wheel', 'zoom',
               "use the mouse wheel to zoom (on touch screens 'pinch-to-zoom' might work as "
               'well)'),
    ]

keyBindings = [
    KeyBinding('1,2,3,4 or 5', 'visualization update speed',
               'the entered digit controls the visualization update, ranging within 0.02, 0.1 '
               '(default), 0.5, 2, and 100 seconds',
               '0.02, 0.1=default, 0.5, 2, 100 seconds', 'GLFW_KEY_1..5'),
    KeyBinding('CTRL+1 or SHIFT+CTRL+1', 'change view',
               'set view in the 1/2-plane (+SHIFT: viewed from the opposite side)',
               'set view to 1/2-plane (SHIFT: from behind)', 'GLFW_KEY_1+CONTROL'),
    KeyBinding('CTRL+2 or SHIFT+CTRL+2', 'change view',
               'set view in the 1/3-plane (+SHIFT: viewed from the opposite side)',
               'set view to 1/3-plane (SHIFT: from behind)', 'GLFW_KEY_2+CONTROL'),
    KeyBinding('CTRL+3 or SHIFT+CTRL+3', 'change view',
               'set view in the 2/3-plane (+SHIFT: viewed from the opposite side)',
               '', 'GLFW_KEY_3+CONTROL'),
    KeyBinding('CTRL+4 or SHIFT+CTRL+4', 'change view',
               'set view in the 2/1-plane (+SHIFT: viewed from the opposite side)',
               '', 'GLFW_KEY_4+CONTROL'),
    KeyBinding('CTRL+5 or SHIFT+CTRL+5', 'change view',
               'set view in the 3/1-plane (+SHIFT: viewed from the opposite side)',
               '', 'GLFW_KEY_5+CONTROL'),
    KeyBinding('CTRL+6 or SHIFT+CTRL+6', 'change view',
               'set view in the 3/2-plane (+SHIFT: viewed from the opposite side)',
               'CTRL+3,4,5,6 are further planes (with optional SHIFT)', 'GLFW_KEY_6+CONTROL'),
    KeyBinding('CTRL+7', '3D view',
               'set a 3D view of the model, rotated out of the coordinate planes',
               'a 3D view of the model', 'GLFW_KEY_7+CONTROL'),
    KeyBinding('A', 'zoom all', 'set the zoom such that the whole scene is visible',
               '', 'GLFW_KEY_A'),
    KeyBinding("'.' or KEYPAD +", 'zoom in',
               'zoom one step into the scene (additionally press CTRL for a small zoom step)',
               'with optional CTRL key for a small zoom',
               'GLFW_KEY_PERIOD,GLFW_KEY_KP_ADD,GLFW_KEY_PERIOD+CONTROL,'
               'GLFW_KEY_KP_ADD+CONTROL'),
    KeyBinding("',' or KEYPAD -", 'zoom out',
               'zoom one step out of the scene (additionally press CTRL for a small zoom step)',
               'with optional CTRL key for a small zoom',
               'GLFW_KEY_COMMA,GLFW_KEY_KP_SUBTRACT,GLFW_KEY_COMMA+CONTROL,'
               'GLFW_KEY_KP_SUBTRACT+CONTROL'),
    KeyBinding('CURSOR UP, DOWN, ...', 'move scene',
               'use the cursor keys to move the scene up, down, left and right (use CTRL for '
               'small movements, SHIFT to rotate instead, ALT for the z-axis)',
               'use CTRL for small movements, SHIFT for rotations (ALT for z-axis)',
               'GLFW_KEY_UP,GLFW_KEY_DOWN,GLFW_KEY_LEFT,GLFW_KEY_RIGHT'),
    KeyBinding('KEYPAD 2/8, 4/6, 7/9', 'rotate scene',
               'rotate about the 1, 2 and 3-axis (use CTRL for small rotations)',
               'about 1,2 or 3-axis (use CTRL for small rotations)',
               'GLFW_KEY_KP_2,GLFW_KEY_KP_8,GLFW_KEY_KP_4,GLFW_KEY_KP_6,GLFW_KEY_KP_7,'
               'GLFW_KEY_KP_9'),
    KeyBinding('N', 'show/hide nodes', 'switches the visibility of nodes', '', 'GLFW_KEY_N'),
    KeyBinding('CTRL+N', 'show/hide node numbers', 'switches the visibility of node numbers',
               '', 'GLFW_KEY_N+CONTROL'),
    KeyBinding('B', 'show/hide bodies', 'switches the visibility of bodies', '', 'GLFW_KEY_B'),
    KeyBinding('CTRL+B', 'show/hide body numbers', 'switches the visibility of body numbers',
               '', 'GLFW_KEY_B+CONTROL'),
    KeyBinding('C', 'show/hide connectors', 'switches the visibility of connectors',
               '', 'GLFW_KEY_C'),
    KeyBinding('CTRL+C', 'show/hide connector numbers',
               'switches the visibility of connector numbers', '', 'GLFW_KEY_C+CONTROL'),
    KeyBinding('M', 'show/hide markers', 'switches the visibility of markers', '', 'GLFW_KEY_M'),
    KeyBinding('CTRL+M', 'show/hide marker numbers', 'switches the visibility of marker numbers',
               '', 'GLFW_KEY_M+CONTROL'),
    KeyBinding('L', 'show/hide loads', 'switches the visibility of loads', '', 'GLFW_KEY_L'),
    KeyBinding('CTRL+L', 'show/hide load numbers', 'switches the visibility of load numbers',
               '', 'GLFW_KEY_L+CONTROL'),
    KeyBinding('S', 'show/hide sensors', 'switches the visibility of sensors', '', 'GLFW_KEY_S'),
    KeyBinding('CTRL+S', 'show/hide sensor numbers', 'switches the visibility of sensor numbers',
               '', 'GLFW_KEY_S+CONTROL'),
    KeyBinding('T', 'faces / edges mode',
               'switch between faces transparent / faces transparent + edges / only face edges / '
               'full faces with edges / only faces visible',
               'faces transparent / transparent + edges / only face edges / full faces with '
               'edges / only faces', 'GLFW_KEY_T'),
    KeyBinding('O', 'change center of rotation',
               'change the center of rotation to the current center of the window (affects only '
               'the current plane coordinates; rotate the model to adjust the other coordinates)',
               'to the current center of the window (affects only current plane coordinates)',
               'GLFW_KEY_O'),
    KeyBinding('R', 'auto-rotate the view',
               'switch the automatic rotation of the model view on and off, see '
               '`visualizationSettings.interactive.autoRotateModelView`',
               'switch automatic rotation of the view on/off', 'GLFW_KEY_R'),
    KeyBinding('CTRL+R', 'raytracing on/off',
               'switch raytracing on and off for the current view, see '
               '`visualizationSettings.view0.camera.useRaytracer`',
               'switch raytracing on/off for this view', 'GLFW_KEY_R+CONTROL'),
    KeyBinding('Q', 'stop solver',
               'the current solver is stopped (and proceeds to the next simulation or to the end '
               'of the file); after `visualizationSettings.general.reallyQuitTimeLimit` seconds a '
               'dialog opens for safety',
               'stop current solver and proceed to the next simulation (or end of file); after '
               'general.reallyQuitTimeLimit seconds a safety dialog opens', 'GLFW_KEY_Q'),
    KeyBinding('SPACE', 'pause/continue simulation',
               'pause the simulation, e.g. for model inspection; a paused simulation is continued '
               "by pressing space again; use SHIFT+SPACE to continuously activate 'continue "
               "simulation'", 'continue simulation', 'GLFW_KEY_SPACE'),
    KeyBinding('ESCAPE', 'close renderer',
               'stops the simulation (and further simulations) and closes the render window (same '
               'as closing the window); after '
               '`visualizationSettings.general.reallyQuitTimeLimit` seconds a dialog opens for '
               'safety',
               'close the render window and stop all simulations (same as the close button); '
               'after general.reallyQuitTimeLimit seconds a dialog opens for safety',
               'GLFW_KEY_ESCAPE'),
    KeyBinding('X', 'execute command',
               'open a dialog to enter a Python command (in the global Python scope), see '
               '{ref}`sec-overview-basics-commandandhelp`; the dialog may appear behind the '
               'visualization window',
               'the dialog may appear in the background, and a command may crash the simulation',
               'GLFW_KEY_X'),
    KeyBinding('V', 'visualization settings',
               'open a dialog to modify the visualization settings, see '
               '{ref}`sec-overview-basics-visualizationsettings`; the dialog may appear behind '
               'the visualization window',
               'the dialog may appear behind the visualization window', 'GLFW_KEY_V'),
    KeyBinding('CTRL+V', 'open a further view',
               'create the window of the next view that is configured but has no window yet',
               'open the window of the next configured view', 'GLFW_KEY_V+CONTROL'),
    KeyBinding('H', 'show help',
               'open the window that shows this list of mouse and keyboard commands',
               'show this help', 'GLFW_KEY_H'),
    KeyBinding('F2', 'ignore keys',
               'switch the key input on and off; can be used together with a keyPressUserFunction '
               'to build simulators',
               'ignore all keyboard input, except for the KeyPress user function, F2 and escape',
               'GLFW_KEY_F2'),
    KeyBinding('F3', 'show mouse coordinates', 'shown in the status line', '', 'GLFW_KEY_F3'),
    KeyBinding('CTRL+F3', 'show model view parameters',
               'shown in the status line: zoom, rotationVector (rot) and centerPoint (pos), as '
               'they are needed by `renderer.SetModelView(zoom, rot, pos)` to restore a view '
               'after the renderer started',
               'zoom, rotationVector and centerPoint, as SetModelView takes them',
               'GLFW_KEY_F3+CONTROL'),
    ]


def RendererHelpText(width=78, keyColumn=22):
    """The text of the help dialog, built from the one table of bindings.

    Args:
        width: the line width the text is wrapped to
        keyColumn: the column the description starts in

    Returns:
        the text that the help dialog of the render window shows
    """
    def Block(bindings, title):
        text = title + '\n'
        for binding in bindings:
            description = binding.action
            if binding.short != '':
                description += ': ' + binding.short
            lines = textwrap.wrap(description, width=max(20, width - keyColumn - 4)) or ['']
            #a key column too narrow for one entry must not swallow the space before the dots
            text += binding.keys.ljust(max(keyColumn, len(binding.keys) + 1)) + '... ' + lines[0] + '\n'
            for line in lines[1:]:
                text += ' '*(keyColumn + 4) + line + '\n'
        return text

    return (Block(mouseBindings, 'Mouse action:')
            + '======================\n'
            + Block(keyBindings, 'Key(s) action:'))
