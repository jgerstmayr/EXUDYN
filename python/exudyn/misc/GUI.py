#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# File:     Exudyn GUI helper files (Tkinter)
#
# Details:  Helper functions and classes for graphical interaction with Exudyn
#
# Author:   Johannes Gerstmayr
# Date:     2020-01-25
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
# Notes:    This is an internal library, which is only used inside Exudyn for modifying settings.
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import tkinter as tk
import tkinter.messagebox
import tkinter.ttk as ttk
import tkinter.font as tkFont
import numpy as np #for array checks
import sys
import re
import exudyn
from exudyn.misc.keyBindings import RendererHelpText

#the settings layer moved to its own module (#2590): it works on
#dictionaries and needs no window, and this module imports tkinter at module scope, so a model
#script could not have used it from here. The names stay importable from HERE, because they were
#the public API of this module; checkAll keeps an imported name out of __all__, so each of them
#is documented once, on the page of the module that defines it.
from exudyn.misc.settingsUtilities import (CheckType, ConvertString2Value,  # noqa: F401
                                           IsArrayInt, IsFloat, IsVector,
                                           CompiledSettingsClass,
                                           ConvertValue2String, DefaultSettingsDictionary,
                                           EnumDisplayName, EnumFullName,
                                           FindMatches, GetComboBoxListsDict,
                                           SettingsCodeLines, SettingsLeafList,
                                           SettingsPrefix, SettingsValueStrings,
                                           ValueLiteral, containerInitialisedSettings)

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'useRenderWindowDisplayScaling', 'treeviewDefaultFontSize', 'rowHeightFactor',
    'boolDoubleClickDelay', 'textHeightFactor', 'defaultColumnWidths', 'treeEditDefaultWidth',
    'treeEditDefaultHeight', 'treeEditMaxInitialHeight', 'dialogDefaultWidth',
    'dialogDefaultHeight', 'treeEditOpenItems', 'treeEditLastOpenItems', 'codeLineBackground',
    'changedValueColor', 'IsApple', 'GetRendererSystemContainer', 'MakeProcessDpiAware',
    'TkFontSystemHint', 'CheckTkFontSystem', 'ScaledFontSize', 'ApplyDialogFontScaling', 'GetTkRootAndNewWindow', 'TkRootExists', 'ColumnWidthFractions', 'DialogFontSize',
    'DialogRowMetrics', 'TkTextHeight', 'GetExudynDisplayScaling', 'GetGUIContentScaling',
    'DialogScaling', 'SplitStoredFromChanged', 'RenderStateCodeLines', 'Tooltip',
    'TkinterEditDictionaryWithTypeInfo', 'EditDictionaryWithTypeInfo', 'TkinterEditDictionary',
    'EditDictionary', 'dialogScreenMargin', 'StoreDialogPositions', 'RestoreWindowGeometry',
    'RememberWindowGeometry', 'StoreGeometryString', 'StoreWindowGeometry',
    'ApplyDialogWindowSettings', 'rendererHelpText', 'ShowHelpDialog', 'pythonCommandExamples',
    'ModelScope', 'ShowPythonCommandDialog', 'ShowVisualizationSettingsDialog',
    'ShowRightMouseSelectionDialog', 'AskQuitDialog',
    ]

useRenderWindowDisplayScaling = True #using this, scaling will change with render window

treeviewDefaultFontSize = 9 #this is then scaled; but it could be changed to make fonts smaller
rowHeightFactor = 1.15  #the factor between the MEASURED linespace of the font and the row height
                        #; 1.15 reproduces the 18 pixels the dialog
                        #had at the default font, and follows the font at any other scaling
boolDoubleClickDelay = 220 #ms a bool row waits before it opens its editor, so that a double
                        #click can cancel it and toggle instead
textHeightFactor = 1.45 #this is the factor between font size and text height; larger values leading to more space between lines

#the share of the dialog width the three fixed columns take; the description gets the rest. At the
#default width of 1024 they are 325, 188 and 113 pixels (#2667)
defaultColumnWidths = [0.31, 0.18, 0.11]

treeEditDefaultWidth = 1024     #unscaled width of e.g. visualizationSettings
treeEditDefaultHeight = 800     #unscaled height of e.g. visualizationSettings
treeEditMaxInitialHeight = 1440 #larger height, if screen resolution admits
dialogDefaultWidth = 800        #unscaled width of e.g. right mouse edit
dialogDefaultHeight = 600       #unscaled height of e.g. right mouse edit
#the folders that are open when a settings dialog is opened; a user sets this, and nothing
#else writes it (#2591)
treeEditOpenItems = ['bodies','connectors','nodes','general']
#which folders were open when a dialog was last used in this process; None until one was, and
#then it is what the next dialog opens with. This is the remembering the clicks used to do, with
#a name that says it is session state and not configuration.
treeEditLastOpenItems = None
codeLineBackground = '#eef1f6'  #the box holding the line that sets a setting (#2605)
changedValueColor = '#1a3fb0'   #a setting that differs from its default (#2606)

def IsApple():
    if sys.platform == 'darwin':
        return True
    else:
        return False

#said once per process, because this function is called by every dialog and by the scaling
_rendererContainerComplaints = []


def _ComplainOnce(reason):
    """say once per process that the renderer's SystemContainer could not be used, and why"""
    if reason not in _rendererContainerComplaints:
        _rendererContainerComplaints.append(reason)
        exudyn.Print('WARNING: the SystemContainer attached to the renderer could not be used, so'
                     ' the dialogs that need it do nothing - ' + reason)


def GetRendererSystemContainer():
    """The SystemContainer that is attached to the render engine, or None.

    Returns:
        the container, or None when no renderer is running, when the entry names a container whose
        C++ object is gone, or when it names something else entirely

    Note:
        None is the normal answer without a renderer and is not reported. Anything ELSE that goes
        wrong here says so once per process: a dialog that finds no container does nothing, and
        nothing being printed is how "the V key opens no dialog" became a reproduction rather than a
        message (#2691, maintainer 2026-09-26).
    """
    #ASKED OF THE C++ SIDE, which holds the pointer (#2692). It used to
    #be the dictionary entry exu.sys['currentRendererSystemContainer'], and a dictionary entry can
    #hold anything: #2691 was exactly that - a Python subclass under the module's own name made the
    #isinstance() that guarded it False, and every dialog that asks here found nothing, in silence.
    #A typed member cannot be wrong about its own type, so there is nothing left to check.
    try:
        guiSC = exudyn.special.currentRendererSystemContainer
        if guiSC is None:                       #no renderer: the normal answer, and not a complaint
            return None
        #the pointer is cleared when a container detaches or is destroyed, which the dictionary entry
        #was NOT (#2623, #2676 guarded the reader against that): this read is what is left of the
        #guard, and it costs nothing
        probe = guiSC.visualizationSettings.dialogs.alwaysTopmost
        if isinstance(probe, bool):
            return guiSC
        _ComplainOnce('the container answered ' + repr(probe) + ' for dialogs.alwaysTopmost')
    #RuntimeError is the access violation of a container that was destroyed while the entry
    #still names it (#2623); a dialog must not die of that
    except (KeyError, AttributeError, RuntimeError) as error:
        _ComplainOnce(type(error).__name__ + ': ' + str(error))
    return None

def MakeProcessDpiAware():
    """tell Windows that this process draws at the real resolution of the display

    Without it, Windows renders the window at 96 dpi and stretches the bitmap, which is why
    a dialog opened from a shell looked soft while the same dialog opened from the render
    window was sharp: GLFW makes the process DPI aware when it creates the render window,
    and a dialog that comes after it inherits that. From "python -m exudyn dialogs" there is
    no GLFW (#2634).

    It must be called BEFORE the first window is created; afterwards Windows refuses, which
    is not an error here - something else has already set it, which is what we wanted.

    Returns:
        True if this process is DPI aware afterwards
    """
    if sys.platform != 'win32':
        return True                 #X11 and macOS scale by themselves
    import ctypes                                                       # noqa: PLC0415
    try:
        ctypes.windll.shcore.SetProcessDpiAwareness(1)   #1 = system aware, as GLFW sets it
        return True
    except Exception:               #already set, or an older Windows without shcore
        try:
            return bool(ctypes.windll.user32.SetProcessDPIAware())
        except Exception:
            return False


def TkFontSystemHint(platform, fontSystem):
    """the hint for dialogs drawn without anti-aliasing, or '' if there is nothing to say

    Args:
        platform: sys.platform
        fontSystem: what Tk reports as its font system, 'xft' or 'x11' on Linux

    Returns:
        the text to print, '' on Windows and macOS and for a Tk with Xft
    """
    if not platform.startswith('linux') or fontSystem != 'x11':
        return ''
    return ('NOTE: the Tk of this Python draws the dialogs without anti-aliasing (built without Xft, as the tk '
            'package of the Anaconda channel on Linux); for smooth fonts install the Xft build: '
            'conda install -c conda-forge "tk=*=xft_*"')


def CheckTkFontSystem(root):
    """say once, on Linux, that the Tk of this Python has no Xft, which makes the dialog fonts
    jagged and coarse (#2890)

    Args:
        root: the tkinter root that was just created
    """
    try:
        fontSystem = str(root.tk.call('::tk::pkgconfig', 'get', 'fontsystem'))
    except tk.TclError:             #a Tk without the key: nothing known, nothing said
        fontSystem = ''
    hint = TkFontSystemHint(sys.platform, fontSystem)
    if hint:
        exudyn.Print(hint)


#the sizes the named Tk fonts had before dialogs.fontScaling changed them, per font name
_namedFontSizes = {}


def ScaledFontSize(size, fontScaling):
    """a Tk font size in points, times dialogs.fontScaling

    A negative size is pixels - the named fonts of Tk on Linux are, e.g. -12 - and pixels do not follow the scaling
    of Tk, which made the buttons and edit fields stay small while the text of the tree grew (#2898). It is converted
    to points at 96 dpi first, so that every named font follows the display scaling once, through the scaling of Tk.

    Args:
        size: the size as Tk gives it
        fontScaling: dialogs.fontScaling; 0 or less is 1

    Returns:
        the size in points, at least 6; 0 (the default of Tk) stays 0
    """
    if size == 0:
        return 0
    points = size if size > 0 else -size * 72. / 96.
    factor = fontScaling if fontScaling > 0 else 1.
    return max(6, int(round(points * factor)))


def ApplyDialogFontScaling(root):
    """set the named fonts of Tk (TkDefaultFont, TkTextFont, TkFixedFont, ...) in points, times dialogs.fontScaling,
    so that every widget of a dialog - text, buttons, edit fields, menus - takes the same font and follows the
    display scaling through the scaling of Tk (#2897, #2898)

    It is called before the widgets of a dialog are created, so nothing is rescaled while a window opens.

    Args:
        root: the tkinter root, which owns the named fonts
    """
    guiSC = GetRendererSystemContainer()
    fontScaling = guiSC.visualizationSettings.dialogs.fontScaling if guiSC is not None else 0.
    try:
        for name in tkFont.names(root):
            if not name.startswith('Tk'):
                continue
            font = tkFont.nametofont(name, root=root)
            originalSize = _namedFontSizes.setdefault(name, font.cget('size'))
            font.configure(size=ScaledFontSize(originalSize, fontScaling))
    except tk.TclError:             #a root that is being destroyed: its fonts do not matter any more
        pass


def GetTkRootAndNewWindow():
    """get new or current root and new window app; return list of [tkRoot, tkWindow, tkRuns]
    """
    if tk._default_root is None:
        MakeProcessDpiAware()       #before the first window, or Windows ignores it (#2634)
        root = tk.Tk()
        root.exudynInitialTkScaling = float(root.tk.call('tk', 'scaling')) #before anything here changes it (#2898)
        CheckTkFontSystem(root)
        tkWindow = root
        tkRuns = False
    else:
        root = tk._default_root
        tkWindow = tk.Toplevel(root)
        tkRuns = True
    if not IsApple():
        GetGUIContentScaling(root)  #the scaling of Tk from the display scaling, for EVERY dialog - help and command too (#2898)
    ApplyDialogFontScaling(root)    #before the widgets of the dialog are created (#2897)
    return [root, tkWindow, tkRuns]

def TkRootExists():
    """this function returns True, if tkinter has already a root window (which is assumed to have already a mainloop running)
    """
    return (tk._default_root is not None)



def ColumnWidthFractions(widths):
    """the three column fractions, usable whatever was configured

    Args:
        widths: `[name, value, type]`, each a fraction of the dialog width

    Returns:
        the same three, each at least 0.05, and scaled down together if they would leave the
        description column less than 10% of the dialog

    Note:
        A settings dialog whose description column is a few pixels wide cannot be read, and the
        settings that produce it are three independent numbers a user may set in any order. So the
        rule is applied here rather than refused at the setting: what a user asks for, as far as it
        still leaves a dialog.
    """
    fractions = [max(0.05, float(width)) for width in (list(widths) + defaultColumnWidths)[:3]]
    total = sum(fractions)
    if total > 0.9:
        fractions = [fraction * 0.9 / total for fraction in fractions]
    return tuple(fractions)


def DialogFontSize(fontFactor):
    """the point size the tree, its headings and its tags use

    Args:
        fontFactor: the multiplier DialogScaling returned

    Returns:
        the size in points, never below 6
    """
    return max(6, int(round(treeviewDefaultFontSize*fontFactor)))


def DialogRowMetrics(root, fontFactor):
    """how tall a row must be and how much wider the columns must be, MEASURED

    Both used to be computed from systemScaling, which is not what decides how large a
    glyph comes out: the point-to-pixel conversion follows the tk scaling of the
    display, so at dialogs.fontScaling=1 the rows were 13 pixels tall for a font with a
    linespace of 16 to 18 and the text was clipped, while the column factor
    max(1,int(round(systemScaling))) stayed at 1 for every value below 1.5 (#2631). The
    font is asked instead, which is right at any scaling and on any platform.

    Args:
        root: the tkinter root, which the fonts are measured against
        fontFactor: the multiplier DialogScaling returned

    Returns:
        [rowHeight in pixels, columnScale relative to the unscaled font]
    """
    try:
        font = tkFont.Font(root=root, size=DialogFontSize(fontFactor))
        reference = tkFont.Font(root=root, size=treeviewDefaultFontSize)
        #the digits are a fair sample of the width of a column of values
        columnScale = (font.measure('0123456789')
                       / max(1, reference.measure('0123456789')))
        return [int(round(font.metrics('linespace')*rowHeightFactor)),
                max(1., columnScale)]
    except tk.TclError:
        return [TkTextHeight(fontFactor), 1.]


#unique text height for tk with given scaling; the fallback of DialogRowMetrics
def TkTextHeight(systemScaling):
    #OLD, without     style.configure("Treeview", font=(None, treeviewDefaultFontSize ) ):
    #return int(13*systemScaling) #must be int; 13 is good; 16 is too big on surface
    
    return int((treeviewDefaultFontSize*textHeightFactor)*systemScaling) #must be int; 13 is good with treeviewDefaultFontSize = 9; 12 leads to some cuts of 'g'

#safely request scaling factor from exudyn
def GetExudynDisplayScaling(root=None):
    """the display scaling the dialogs size themselves by

    The renderer knows it and reports it in its state. Without a renderer this used to
    return 1, so a dialog opened from the command line came out at a different size than the
    same dialog opened with V in the render window (#2634); tkinter is asked instead, which
    knows it once MakeProcessDpiAware has been called.

    Args:
        root: a tkinter root to ask when there is no renderer; without one the answer is 1

    Returns:
        the scaling, 1 if nothing knows better
    """
    try:
        #GetRendererSystemContainer is the ONE place that knows how the link is found and that a
        #container whose C++ object is gone must not be used
        guiSC = GetRendererSystemContainer()
        if guiSC is not None: #None would mean that the renderer is detached
            rs = guiSC.renderer.GetState()
            #the renderer's scaling includes general.displayScaleFactor, which is for the render window only (#2898)
            factor = guiSC.visualizationSettings.general.displayScaleFactor
            return rs['displayScaling'] / (factor if factor > 0 else 1.)
        
        if root is not None:        #96 dpi is the unscaled display, 144 is 150%
            #the scaling Tk chose when the root was created: what Tk reports now is what this module set, and reading
            #it back made every dialog that opened 5% larger than the one before (#2898)
            initialScaling = getattr(root, 'exudynInitialTkScaling', None)
            if initialScaling is not None:
                return max(1., initialScaling * 72. / 96.)
            return max(1., root.winfo_fpixels('1i') / 96.)

        return 1

    except (KeyError, AttributeError, tk.TclError): 
        return 1

#return either exudyn or tkinter scaling, unified approach
def GetGUIContentScaling(root):
    try:
        if useRenderWindowDisplayScaling: #would also work under linux
            s = 1.4*GetExudynDisplayScaling(root) #gives similar size as other programs; factor 1.4 is empirical
            root.tk.call('tk', 'scaling', s) #needed to update font size internally ...
            return s
        else:
            return root.tk.call('tk', 'scaling') #obtains current scaling?
    except tk.TclError:
        return 1
    
def DialogScaling(root):
    """How tall a row of a dialog is and how large its font is, as [systemScaling, fontFactor].

    dialogs.fontScaling is 0 by default, and 0 means what every platform did before the setting
    existed: a fixed factor on MacOS, the system display scaling on Windows and Linux. A value
    > 0 sets the font on EVERY platform, which is what makes the dialogs readable on a Linux
    desktop - off MacOS the font factor used to be forced to 1 and nothing could change it
    (#2602).

    Args:
        root: the tkinter root window, which knows the display scaling

    Returns:
        [systemScaling, fontFactor]
    """
    fontScaling = 0.
    guiSC = GetRendererSystemContainer()
    if guiSC is not None:
        #the deprecated fontScalingMacOS forwards to this one in the C++, so it is read once
        fontScaling = guiSC.visualizationSettings.dialogs.fontScaling

    systemScaling = 1.35                #ideal for MacOS
    fontFactor = systemScaling
    if not IsApple():
        #the font size is not changed here; the content scaling does it internally
        fontFactor = 1
        systemScaling = GetGUIContentScaling(root)

    if fontScaling > 0:                 #explicit, and it wins on every platform; a factor on the fonts only - the
        fontFactor = fontScaling        #system scaling stays, or the two would count twice (#2898)

    return [systemScaling, fontFactor]

def SplitStoredFromChanged(changes, overriddenPaths, fileName):
    """The changed settings, with the ones the override file stores named separately.

    Args:
        changes: [(path, line)] as SettingsCodeLines returns them - every setting that differs from
            the DEFAULT, which is what the difference is to
        overriddenPaths: the paths that `~/.exudyn/config.json` stores
        fileName: the name of that file, for the comment line

    Returns:
        (lines, storedCount): the same pairs, the stored ones last and behind a comment line, and
        how many of them there are

    Note:
        The maintainer decided this on 2026-09-26: the difference is to
        the REAL default, and what the file already covers is named separately with a comment
        between them, so that a user can see what they would be copying and decide. Comparing
        against default-plus-override instead would hide exactly the settings that file is about.
    """
    stored = [(path, line) for (path, line) in changes if path in overriddenPaths]
    if not stored:
        return (list(changes), 0)
    lines = [(path, line) for (path, line) in changes if path not in overriddenPaths]
    lines += [('', '#the following are already stored in ' + str(fileName)
               + ' and are applied to every new structure:')] + stored
    return (lines, len(stored))

def RenderStateCodeLines(renderState, systemContainerName='SC'):
    """The lines that give a script the current model view of the render window, as CTRL+F3 prints them (#2862).

    Args:
        renderState: the dictionary of SC.renderer.GetState()
        systemContainerName: the name of the SystemContainer in the script

    Returns:
        [(key, line)]: SC.renderer.Start() and SC.renderer.SetModelView with zoom, rotation vector and center point
    """
    from exudyn.rigidBodyUtilities import RotationMatrix2RotationVector                      # noqa: PLC0415
    def Numbers(values):
        return '[' + ','.join(f'{float(value):.7g}' for value in values) + ']'
    rotation = np.array([list(row)[:3] for row in renderState['modelRotation']][:3], dtype=float)
    #the model rotation is the transposed of the rotation of the rotation vector, as SetModelView sets it
    rotationVector = RotationMatrix2RotationVector(rotation.T)
    centerPoint = [float(renderState['centerPoint'][0]), float(renderState['centerPoint'][1]), 0.] #without z, as CTRL+F3
    sc = systemContainerName
    indent = ' '*len(sc + '.renderer.SetModelView(')
    return [('', sc + '.renderer.Start()'),
            ('', sc + '.renderer.SetModelView(zoom=' + f"{float(renderState['zoom']):.7g}" + ',\n'
             + indent + 'rotationVector=' + Numbers(rotationVector) + ',\n'
             + indent + 'centerPoint=' + Numbers(centerPoint) + ')')]


class Tooltip:
    """The small yellow window that shows the description of the row under the mouse.

    tkinter has none, and the description used to be behind the key 'h' and a modal message box -
    which is not where a reader looks for it (#2601). It is a
    borderless Toplevel that is created when it is first needed and hidden afterwards, so a
    dialog that is never hovered never builds one.

    It is **topmost**, and that is not decoration: the dialog itself is topmost - it has to
    be, it blocks the render window - and on Windows a topmost window is always above one
    that is not, so a tooltip without the flag opens BEHIND the dialog and looks like a
    tooltip that never comes (#2639). The same mechanism hid the window of the changes in
    #2621. Turning `dialogs.alwaysTopmost` off made the tooltips work, which is what named
    the cause.
    """

    def __init__(self, widget, wrapLength=520, delay=500):
        self.widget = widget
        self.wrapLength = wrapLength
        self.delay = delay          #ms before it appears; a tooltip that is instant is in the way
        self.window = None
        self.label = None
        self.pending = None
        #a tooltip still scheduled when the dialog closes ran on a deleted command: "invalid command name" (#2897)
        widget.bind('<Destroy>', lambda event: self.Cancel(), add='+')

    def Show(self, text, x, y):
        """show the tooltip after the delay, at the screen position (x, y)

        The delay is the maintainer's (#2614): a description that appears the moment the pointer
        crosses a row is annoying, and with the find (#2607) nobody has to sweep the tree to find a
        setting any more.
        """
        self.Cancel()
        if self.delay > 0:
            self.pending = self.widget.after(self.delay, lambda: self.Place(text, x, y))
        else:
            self.Place(text, x, y)

    def Cancel(self):
        """forget a tooltip that was scheduled and has not appeared yet"""
        if self.pending is not None:
            try:
                self.widget.after_cancel(self.pending)
            except (tk.TclError, ValueError):
                pass
            self.pending = None

    def Bind(self, widget, text):
        """let a widget - a button, say - show this tooltip while the pointer rests on it"""
        widget.bind('<Enter>', lambda event: self.Show(text, event.x_root, event.y_root))
        widget.bind('<Leave>', lambda event: self.Hide())

    def Place(self, text, x, y):
        """place the tooltip at the screen position (x, y), a little below the pointer"""
        self.pending = None
        if self.window is None:
            self.window = tk.Toplevel(self.widget)
            self.window.wm_overrideredirect(True)   #no title bar, no border
            self.window.attributes('-topmost', True)  #or it opens behind the dialog (#2639)
            self.label = tk.Label(self.window, justify=tk.LEFT, background='#ffffe0',
                                  relief=tk.SOLID, borderwidth=1, wraplength=self.wrapLength)
            self.label.pack(ipadx=3, ipady=2)
        self.label.configure(text=text)
        self.window.wm_geometry('+' + str(int(x) + 16) + '+' + str(int(y) + 18))
        self.window.deiconify()
        try:                        #-topmost is the stacking BAND; lift orders within it
            self.window.lift()
        except tk.TclError:
            pass

    def Hide(self):
        self.Cancel()
        if self.window is not None:
            self.window.withdraw()


#this class gets a dictionary with type information structure in, but a plain dictionary with types (int, float, string, list, ...) out
#settingsStructure: contains hierarchical settings structure with function GetDictionaryWithTypeInfo() to obtain dictionary for editing
#dictionaryTypes: contains a dictionary with the available types, e.g. bool, etc.
#updateOnChange: every change is directly applied to the settingsStructure and redraw is signaled in stored renderer
class TkinterEditDictionaryWithTypeInfo(tk.Frame):
    def __init__(self, parent, settingsStructure, dictionaryTypesT, updateOnChange=False, treeOpen=False,
                 textHeight = 15, systemScaling = 1, fontFactor = 1, columnScale = 1., columnWidths=None):
        tk.Frame.__init__(self, parent)
        
        self.parentFrame = parent #parent frame stored for member functions
        self.settingsStructure = settingsStructure
        self.dictionaryTypesT = dictionaryTypesT #as type
        self.updateOnChange = updateOnChange
        #the fractions of the dialog width the first three columns take; the description column
        #takes what they leave (#2667)
        self.columnWidths = list(columnWidths) if columnWidths is not None else defaultColumnWidths
        self.treeOpen = treeOpen
        self.textHeight = textHeight
        self.systemScaling = systemScaling
        self.fontFactor = fontFactor
        #how much wider the columns have to be than at the unscaled font; MEASURED by
        #DialogRowMetrics, because an integer factor was 1 for every fontScaling below 1.5 (#2631)
        self.columnScale = columnScale
        #the code line is read, not edited: one size below the cells, which is what the
        #maintainer asked for after using it (#2605)
        self.codeFontSize = max(6, DialogFontSize(fontFactor) - 1)

        self.dictionaryData = settingsStructure.GetDictionaryWithTypeInfo()
        #the folders this dialog opens with: the configuration, or what was open when a dialog
        #was last used in this process (#2591)
        self.openItems = list(treeEditOpenItems if treeEditLastOpenItems is None
                              else treeEditLastOpenItems)

        #additional storage for type, size, etc.
        self.typeStorage = dict() #dictionary is stored as {'ID1': 'type1', 'ID2': 'type2', ...}
        self.sizeStorage = dict()
        self.descriptionStorage = dict()

        #create treeview:
        self.tree = ttk.Treeview(self, columns=("value","type","description"), selectmode='browse',
                                 height=self.textHeight)
        self.vertivalScrollbar = ttk.Scrollbar(self, orient="vertical", command=self.tree.yview)
        self.tree.configure(yscrollcommand=self.vertivalScrollbar.set)

        #THE FIND BAR (#2607) stands above the tree, which is
        #where CTRL-F puts one. It searches the NAMES first and the descriptions second, and the
        #drop-down holds the hits, so that one can be picked instead of stepped to.
        self.findFrame = tk.Frame(self)
        #columnspan 3, not 4: column 3 is the vertical scroll bar of the tree, and a button
        #that reaches under it looks misplaced (#2621)
        self.findFrame.grid(row=0, column=0, columnspan=3, sticky=tk.E+tk.W)
        self.findFrame.grid_columnconfigure(2, weight=1)
        tk.Label(self.findFrame, text='find:').grid(row=0, column=0, padx=(4, 2))
        self.findVar = tk.StringVar()
        self.findEntry = tk.Entry(self.findFrame, textvariable=self.findVar, width=24)
        self.findEntry.grid(row=0, column=1, pady=2)
        self.findEntry.bind('<Return>', self.OnFindNext)
        self.findEntry.bind('<Escape>', lambda event: self.tree.focus_set())
        self.findVar.trace_add('write', lambda *arguments: self.UpdateFindHits())
        #no find button: the search runs while the text is typed, which is enough (#2613);
        #and the drop-down is disabled until there is something to pick, so that it does not
        #look like a control that does nothing
        self.findCombo = ttk.Combobox(self.findFrame, state='disabled', values=())
        self.findCombo.grid(row=0, column=2, sticky=tk.E+tk.W, padx=4)
        self.findCombo.bind('<<ComboboxSelected>>', self.OnFindPick)
        self.findHits = []
        self.findIndex = -1

        self.tree.grid(row=1, column=0, columnspan=3, sticky="nsew")
        self.vertivalScrollbar.grid(row=1, column=3, sticky='nse')
        self.vertivalScrollbar.configure(command=self.tree.yview)


        self.grid_rowconfigure(1, weight=1)
        self.grid_columnconfigure(0,weight=1)
        self.grid_columnconfigure(1,weight=2)
        self.grid_columnconfigure(2,weight=2)
        self.grid_columnconfigure(3,weight=0)
        
        self.tree.heading("#0",text="Name",anchor=tk.W)
        self.tree.heading("value",text="Value",anchor=tk.W)
        self.tree.heading("type",text="Type",anchor=tk.W)
        self.tree.heading("description",text="Description",anchor=tk.W)

        #the columns had no width at all (#2601): tree.column()
        #was never called, so every one of them kept tkinter's 200 px default - the name was cut,
        #the description was unreadable, and dragging one moved the others. Only the description
        #stretches with the window; the rest keep what they are given.
        self.tree.column("#0", minwidth=120, stretch=False)
        self.tree.column("value", minwidth=60, stretch=False)
        self.tree.column("type", minwidth=50, stretch=False, anchor=tk.W)
        self.tree.column("description", minwidth=120, stretch=True)
        self.ApplyColumnWidths()


        #a row that differs from the default is written in colour and bold (#2606); the size
        #has to be given here, because a tag font does not follow the style of the tree
        self.tree.tag_configure('changed', foreground=changedValueColor,
                                font=(None, DialogFontSize(fontFactor), 'bold'))

        #Ctrl and the wheel change the font size, on every platform's spelling of the event
        #(#2668)
        for widget in [self, self.tree]:
            widget.bind('<Control-MouseWheel>',
                        lambda event: self.ChangeFontSize(1.1 if event.delta > 0 else 1/1.1))
            widget.bind('<Control-Button-4>', lambda event: self.ChangeFontSize(1.1))
            widget.bind('<Control-Button-5>', lambda event: self.ChangeFontSize(1/1.1))

        self.AddNodeFromDictionaryWithTypeInfo(value=self.dictionaryData, parentNode="")
        self.tree.bind('<<TreeviewSelect>>', self.TreeviewSelect) #selection changed
        self.tree.bind("<Double-1>", self.OnTreeDoubleClick) #an item has been selected for change
        self.tree.bind("<Return>", self.OnTreeEdit) #an item has been selected for change
        self.tree.bind("<ButtonRelease-1>", self.OnTreeEdit) #an item has been selected for change #1357
        self.tree.bind("h", self.OnTreeHelp) #show description for selected item; the tooltip
                                             #below is the ordinary way since #2601
        self.tooltip = Tooltip(self.tree)
        self.tooltipRow = ''
        self.tree.bind("<Motion>", self.OnTreeHover)
        self.tree.bind("<Leave>", lambda event: self.tooltip.Hide())
        self.tree.bind("<Escape>", self.OnQuit) 
        self.tree.bind("q", self.OnQuit) 
        
        #+++++++++++++++++++++++++++++++++++++++++
        #THE EDITOR SITS IN THE CELL (#2604). It used to be an
        #Entry and a Combobox at the bottom of the window that swapped places by z-order, so a
        #value was typed far away from the row it belonged to. These two are children of the
        #tree and are placed over the value cell while an edit is running.
        self.selectedItem = ''          #the item being edited, '' when nothing is
        self.editItemVar = tk.StringVar()
        self.cellEditor = tk.Entry(self.tree, textvariable=self.editItemVar)
        self.cellEditor.bind('<Return>', self.OnCellCommit)
        self.cellEditor.bind('<FocusOut>', self.OnCellCommit)
        self.cellEditor.bind('<Escape>', self.OnCellCancel)

        self.cellCombo = ttk.Combobox(self.tree, values=['True','False'], state='readonly')
        self.cellCombo.bind('<<ComboboxSelected>>', self.OnCellCommit)
        self.cellCombo.bind('<Escape>', self.OnCellCancel)

        #+++++++++++++++++++++++++++++++++++++++++
        #the bottom row is not an editor any more: it says what the selected item IS, as the line
        #that sets it - which is what the maintainer asked for, a line to copy into a script.
        #It sits in a BOX of its own (#2605): the plain entry on
        #the window background did not look like something one copies. The label that stood under
        #it went with the same step - it repeated the description that the tooltip already shows,
        #and the size it also carried is in the tooltip now.
        self.bottomFrame = tk.Frame(self)
        self.bottomFrame.grid(row=2, column=0, columnspan=3, sticky=tk.E+tk.W)
        self.bottomFrame.grid_columnconfigure(0, weight=1)

        codeBox = tk.Frame(self.bottomFrame, relief=tk.SOLID, borderwidth=1,
                           background=codeLineBackground)
        codeBox.grid(row=0, column=0, sticky=tk.E+tk.W, padx=4, pady=4)
        self.codeLineVar = tk.StringVar()
        self.codeLine = tk.Entry(codeBox, textvariable=self.codeLineVar, state='readonly',
                                 readonlybackground=codeLineBackground, borderwidth=0,
                                 highlightthickness=0, font=(None, self.codeFontSize))
        self.codeLine.pack(fill=tk.X, expand=True, padx=4, pady=2)

        self.copyButton = tk.Button(self.bottomFrame, text='copy line',
                                    command=self.OnCopyCodeLine)
        self.copyButton.grid(row=0, column=1, padx=(2, 4))

        #THE BUTTON ROW (#2614): the two windows of #2606 on the
        #left, and what a user does with a dialog - take it back, or leave it - on the right.
        #Every button says what it does in a tooltip; none of them fits in two words.
        self.buttonFrame = tk.Frame(self)
        self.buttonFrame.grid(row=3, column=0, columnspan=3, sticky=tk.E+tk.W)
        self.buttonFrame.grid_columnconfigure(5, weight=1)      #the gap between the two groups
        self.buttonTooltip = Tooltip(self, wrapLength=420)

        self.diffButton = tk.Button(self.buttonFrame, text='diff to default',
                                    command=self.OnShowDiffToDefault)
        self.diffButton.grid(row=0, column=0, padx=(4, 2), pady=(0, 4))
        self.sessionButton = tk.Button(self.buttonFrame, text='changes since start',
                                       command=self.OnShowSessionChanges)
        self.sessionButton.grid(row=0, column=1, padx=2, pady=(0, 4))
        self.storeButton = tk.Button(self.buttonFrame, text='store settings',
                                     command=self.OnStoreSettings)
        self.storeButton.grid(row=0, column=2, padx=2, pady=(0, 4))
        self.storePositionsButton = tk.Button(self.buttonFrame, text='store positions',
                                              command=self.OnStorePositions)
        self.storePositionsButton.grid(row=0, column=3, padx=2, pady=(0, 4))
        #the model view as code (#2862): only the visualization settings belong to a render window
        self.viewButton = None
        if CompiledSettingsClass(self.settingsStructure).__name__ == 'VisualizationSettings':
            self.viewButton = tk.Button(self.buttonFrame, text='store model view', command=self.OnShowView)
            self.viewButton.grid(row=0, column=4, padx=2, pady=(0, 4))

        self.resetButton = tk.Button(self.buttonFrame, text='reset', command=self.OnReset)
        self.resetButton.grid(row=0, column=6, padx=2, pady=(0, 4))
        self.revertButton = tk.Button(self.buttonFrame, text='revert', command=self.OnRevert)
        self.revertButton.grid(row=0, column=7, padx=2, pady=(0, 4))
        self.undoButton = tk.Button(self.buttonFrame, text='undo', command=self.OnUndo,
                                    state=tk.DISABLED)
        self.undoButton.grid(row=0, column=8, padx=2, pady=(0, 4))
        self.closeButton = tk.Button(self.buttonFrame, text='close',
                                     command=lambda: self.parentFrame.destroy())
        self.closeButton.grid(row=0, column=9, padx=(2, 4), pady=(0, 4))

        for (button, description) in [
                (self.copyButton, 'copy the line above, which sets the selected setting'),
                (self.diffButton, 'show diffs to default'),
                (self.sessionButton, 'show changes since dialog opened'),
                (self.storeButton, 'store the settings that differ from the defaults in the'
                                   ' settings file, so that every run starts with them - including'
                                   ' the size and position of the RENDER window, which are ordinary'
                                   ' settings; nothing else is stored, not exudyn.config and not the'
                                   ' simulation settings, and you are shown what will be written'
                                   ' before anything is'),
                (self.storePositionsButton, 'store the size and position of every open window - this dialog,'
                                            ' the other dialogs, the plot windows and the render window - so'
                                            ' that each opens there next time; nothing about the settings, and'
                                            ' you are shown what will be written before anything is'),
                (self.viewButton, 'show the code that sets the current model view of the render window -'
                                  ' SC.renderer.SetModelView with zoom, rotation vector and center point, as'
                                  ' CTRL+F3 prints it - to paste into a script'),
                (self.resetButton, 'reset to default'),
                (self.revertButton, 'revert to state when dialog opened'),
                (self.undoButton, 'undo the last change, a reset or a revert'),
                (self.closeButton, 'close the dialog (same as ESCAPE)')]:
            if button is not None:
                self.buttonTooltip.Bind(button, description)

        #one WHOLE state back per entry, so that undo takes a reset and a revert back too
        self.undoStack = []
        #the after() job of a bool row, which a double click cancels in order to toggle (#2630)
        self.pendingCellEdit = None
        self.tree.bind('<Destroy>', lambda event: self.CancelPendingCellEdit(), add='+') #not run on a closed dialog (#2897)

        #+++++++++++++++++++++++++++++++++++++++++
        #pre-select item
        first = self.tree.get_children('')[0]
        self.tree.focus_set()
        self.tree.focus(first)
        self.tree.selection_set((first))
        
        self.modifiedDictionary = self.GetDictionary('')

        #WHAT DIFFERS FROM THE DEFAULTS IS MARKED (#2606). The
        #defaults are one constructor call away, and a settings structure Python builds on its own
        #is safe to read as long as no DEPRECATED member is touched, which none of this does
        #(#2603). If it ever fails, the marking stays off rather than the dialog.
        self.defaultValueStrings = {}
        try:
            self.defaultValueStrings = SettingsValueStrings(
                DefaultSettingsDictionary(self.settingsStructure))
        except Exception as exception:                                       # noqa: BLE001
            exudyn.Print('WARNING: the settings defaults are not available, so nothing is'
                         ' marked as changed: ' + str(exception))
        #and what THIS dialog started with, which is the other thing a user calls "changed"
        self.openingValueStrings = {path: valueStr
                                    for (path, _, valueStr, _, _, _) in self.TreeLeaves()}
        self.MarkChangedValues()

        #what the find searches and where it jumps to (#2607); the
        #names and descriptions do not change while the dialog is open, so this is built once
        self.findLeaves = self.TreeLeaves()
        self.itemByPath = {self.ItemPath(item): item for item in self.LeafItems()}
        for widget in [self.parentFrame, self.tree]:
            widget.bind('<Control-f>', self.OnFindFocus)
            widget.bind('<F3>', self.OnFindNext)

    #create treeview from dictionary with type info
    def AddNodeFromDictionaryWithTypeInfo(self, value, parentNode="", key=None, level=0):
        if key is None:
            id = ""
        else:
            id = self.tree.insert(parentNode, "end", text=key)
        isOpen = self.treeOpen and (level<=1)

        if isinstance(value, dict):
            itemIsOpen = isOpen
            if ('itemIdentifier' not in value) and (key in self.openItems): 
                itemIsOpen = True
                
            self.tree.item(id, open=itemIsOpen)
            if 'itemIdentifier' in value: #is a value with types:
                strValue = ConvertValue2String(value['value'], value['type'], value['size'])
                
                #(value, type, description): the type is what tells a reader whether to
                #type 3, 3.0, True or [1,2,3], and it was read and never shown (#2601)
                self.tree.item(id, values=(strValue, value['type'], value['description']))
                #store additional data in dictionaries (could also be tuples ...)
                self.typeStorage[id] = value['type']
                self.sizeStorage[id] = value['size']
                self.descriptionStorage[id] = value['description']
            else: #must be another dictionary:
                #what the FOLDER is: the class description of the settings structure, which
                #reaches the dictionary (#2615) and is what the
                #tooltip shows over a folder - it had nothing to show there at all
                self.descriptionStorage[id] = str(value.get('structureDescription', ''))
                for (key, value) in value.items():
                    if key != 'structureDescription':    #a description, not a settings value
                        self.AddNodeFromDictionaryWithTypeInfo(value, id, key, level=level+1)
        else:
            exudyn.Print('ERROR: AddNodeFromDictionaryWithTypeInfo: item ' + str(value)
                         + ' (parent=' + str(parentNode) + ', key=' + str(key)
                         + ') is no settings value and no sub-structure')

    #create treeview from plain dictionary
    def AddNodeFromDictionary(self, value, parentNode="", key=None):
        if key is None:
            id = ""
        else:
            id = self.tree.insert(parentNode, "end", text=key)

        if isinstance(value, dict):
            self.tree.item(id, open=True)
            for (key, value) in value.items():
                self.AddNodeFromDictionary(value, id, key)
        else:
            self.tree.item(id, values=(value))

    def GetDictionary(self, item):
        d=dict()
        kids = self.tree.get_children(item)
        for i in kids:
            nchilds = len(self.tree.get_children(i))
            if nchilds == 0:
                if len(self.tree.item(i,'values')) != 0:
                    valStr = self.tree.item(i,'values')[0]
                    [val, errorMsg] = ConvertString2Value(valStr, str(self.typeStorage[i]), self.sizeStorage[i], self.dictionaryTypesT)
                    if errorMsg == '':
                        d.update({self.tree.item(i,'text'): val})
                    else:
                        exudyn.Print('ERROR: item ' + str(self.tree.item(i,'text'))
                                     + ' has illegal value "' + valStr + '": ' + errorMsg)
                else:
                    d.update({self.tree.item(i,'text'): ''})
            else:
                d.update({self.tree.item(i,'text'): self.GetDictionary(i)})
        return d

    def OnTreeHover(self,event):
        """the description of the row under the mouse, as a tooltip: it was behind the key 'h'
        and a modal message box, which is not where anybody looks for it (#2601)"""
        row = self.tree.identify_row(event.y)
        if row == self.tooltipRow:
            return                          #same row: leave the tooltip where it is
        self.tooltipRow = row
        self.tooltip.Hide()
        description = self.descriptionStorage.get(row, '')
        if description.strip() != '':
            name = self.tree.item(row,'text')
            vType = self.typeStorage.get(row, '')
            #the size stands here (#2605): it was the only fact
            #of the label under the tree that nothing else said
            size = self.sizeStorage.get(row, '')
            if str(size) not in ['[1]', '1', '']:
                vType += ', size ' + str(size)
            self.tooltip.Show(name + ('  [' + vType + ']' if vType else '') + '\n' + description,
                              event.x_root, event.y_root)

    def OnTreeHelp(self,event):
        item = self.tree.selection()[0]
        if item in self.descriptionStorage:
            d = self.descriptionStorage[item]
            s = self.tree.item(item,'text')
            tk.messagebox.showinfo(s, d)

    def OnTreeDoubleClick(self,event):
        self.CancelPendingCellEdit()   #the first click of this double click must not edit
        self.OnTreeEditOrDoubleClick(event, True)
        
    def CancelPendingCellEdit(self):
        """drop a scheduled cell edit, so that the second click of a double click wins"""
        if self.pendingCellEdit is not None:
            try:
                self.tree.after_cancel(self.pendingCellEdit)
            except (tk.TclError, ValueError):
                pass
            self.pendingCellEdit = None

    def OnTreeEdit(self,event):
        """a single click opens the editor - except on a bool, where it waits

        A bool is edited with a Combobox placed OVER the value cell, so opening it at
        once put it under the second click of a double click and the toggle of #1354
        became unreachable (#2630). On a bool row the edit is scheduled and a double
        click cancels the job; every other type keeps the immediate editor.
        """
        self.CancelPendingCellEdit()
        selection = self.tree.selection()
        if selection != () and self.typeStorage.get(selection[0], '') == 'bool':
            self.pendingCellEdit = self.tree.after(
                boolDoubleClickDelay, lambda: self.OnTreeEditOrDoubleClick(event))
            return
        self.OnTreeEditOrDoubleClick(event)
        
    def OnTreeEditOrDoubleClick(self,event,doubleClick=False):
        if self.tree.selection() == ():    #the row may be gone when a scheduled edit fires
            return
        item = self.tree.selection()[0]
        nchilds = len(self.tree.get_children(item))

        #only edit items which have no subitems (no folders!)
        if nchilds == 0:
            
            selectedType=self.typeStorage[item]
            #+++++++++++++++++++                
            if selectedType=='bool' and doubleClick: #just toggle value #1354
                value=self.tree.item(item,'values')[0]
                if value=='True': value='False'
                else: value='True'

                self.PushUndoState()
                self.tree.item(item, values=(value, self.typeStorage[item],
                                             self.descriptionStorage[item]))
                self.MarkChangedValue(item)
                self.modifiedDictionary = self.GetDictionary('') #update stored dictionary
                self.UpdateSettingsStructure() #only if according flag set in visualizationSettings
                return

            #the path of the item is built by ItemPath() now, which ShowInfo and the code line
            #both need
            self.ShowInfo(item)
            self.StartCellEdit(item)
        else: #folders (may be opened/closed)
            openState = self.tree.item(item, 'open')
            s = self.tree.item(item,'text')
            global treeEditLastOpenItems                                     # noqa: PLW0603
            if openState and (s not in self.openItems):
                self.openItems.append(s)
            elif not openState and (s in self.openItems):
                self.openItems.remove(s)
            #remembered for the next dialog of this process, without touching the configuration
            treeEditLastOpenItems = list(self.openItems)
        

    def OnQuit(self,event): #new selection --> nothing to edit for now
        self.parentFrame.destroy()
        
    def ApplyColumnWidths(self):
        """Give the three fixed columns their share of the dialog, and the description the rest.

        Note:
            The widths are fractions of the width of the dialog
            (`visualizationSettings.dialogs.columnWidthName` and its two neighbours), so a long
            name is a setting and not a rebuild. They are scaled by the font, which is why this is
            called again when the font size changes.
        """
        (name, value, valueType) = ColumnWidthFractions(self.columnWidths)
        width = int(treeEditDefaultWidth * self.columnScale)
        self.tree.column("#0", width=int(name * width))
        self.tree.column("value", width=int(value * width))
        self.tree.column("type", width=int(valueType * width))
        self.tree.column("description", width=int((1. - name - value - valueType) * width))

    def ChangeFontSize(self, factor):
        """Make the font of the dialog larger or smaller, and everything that follows it.

        Args:
            factor: 1.1 makes it about 10% larger, 1/1.1 about 10% smaller

        Returns:
            'break', so that the tree does not also scroll on the same event

        Note:
            The row height, the column widths and the font of a changed row all follow the font
            size, so all three are recomputed here. The range is limited: a dialog whose font is
            two pixels tall cannot be read back to a usable size.
        """
        self.fontFactor = max(0.4, min(4., self.fontFactor * factor))
        [self.textHeight, self.columnScale] = DialogRowMetrics(self, self.fontFactor)
        fontSize = DialogFontSize(self.fontFactor)

        style = ttk.Style(self)
        style.configure('Treeview', rowheight=self.textHeight, font=(None, fontSize))
        style.configure('Treeview.Heading', font=(None, fontSize))
        self.tree.tag_configure('changed', foreground=changedValueColor,
                                font=(None, fontSize, 'bold'))
        self.ApplyColumnWidths()
        return 'break'

    def TreeviewSelect(self,event):
        """a new row is selected: the bottom row follows it, whether it is edited or not"""
        self.CancelCellEdit()
        selection = self.tree.selection()
        if selection:
            self.ShowInfo(selection[0])
        
    #if visualizationSettings are accordingly, the renderer will obtain an update signal
    def UpdateSettingsStructure(self):
        #++++++++++++++++++++++++++++++++++++++++++++++++
        #update changes
        self.settingsStructure.SetDictionary(self.modifiedDictionary)  #this may also change dialogs.multiThreadedDialogs itself
        guiSC = GetRendererSystemContainer()
        if guiSC is not None:
            if guiSC.visualizationSettings.dialogs.multiThreadedDialogs:
                guiSC.renderer.SendRedrawSignal()
                guiSC.renderer.DoIdleTasks(0, pollEvents=sys.platform != 'darwin') #do not wait; in a tkinter callback on macOS the events are tkinter's (#2878)
        #++++++++++++++++++++++++++++++++++++++++++++++++

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++        
    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #THE CELL EDITOR (#2604)

    def ItemPath(self, item):
        """the dotted path of a row, as a script writes it: openGL.lineWidth"""
        path = self.tree.item(item,'text')
        parent = self.tree.parent(item)
        depth = 0
        while depth < 10 and parent != '':      #limits to 10 levels ...
            depth += 1
            path = self.tree.item(parent,'text') + '.' + path
            parent = self.tree.parent(parent)
        return path

    def CodeLine(self, item):
        """the selected setting as the LINE THAT SETS IT, ready to paste into a script:

            SC.visualizationSettings.openGL.lineWidth = 2.0

        which is what makes a session in this dialog reusable (maintainer, 2026-09-23). The
        prefix follows the structure being edited, the value is written as a Python literal."""
        literal = ValueLiteral(self.tree.item(item,'values')[0],
                               self.typeStorage.get(item, ''), self.dictionaryTypesT)
        return SettingsPrefix(self.settingsStructure) + '.' + self.ItemPath(item) + ' = ' + literal

    def ShowInfo(self, item):
        """the bottom row: the line that sets the selected item"""
        if item == '' or item not in self.typeStorage:
            self.codeLineVar.set('')
            return
        self.codeLineVar.set(self.CodeLine(item))

    def OnCopyCodeLine(self):
        """the line into the clipboard; tkinter keeps it only while it runs, so it is flushed"""
        line = self.codeLineVar.get()
        if line == '':
            return
        self.clipboard_clear()
        self.clipboard_append(line)
        self.update()                         #without this the clipboard is empty after closing

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #WHAT DIFFERS, AND FROM WHAT (#2606)

    def LeafItems(self, item=''):
        """the rows that hold a value, in tree order - folders are not settings"""
        items = []
        for child in self.tree.get_children(item):
            if self.tree.get_children(child):
                items += self.LeafItems(child)
            elif child in self.typeStorage:
                items.append(child)
        return items

    def TreeLeaves(self):
        """the rows of the tree in the shape of SettingsLeafList, so that one comparison serves
        the marking, the two windows and the tests. The TREE is asked, not the settings structure:
        what a user sees is what is copied."""
        return [(self.ItemPath(item), None, self.tree.item(item,'values')[0],
                 self.typeStorage[item], self.sizeStorage[item], self.descriptionStorage[item])
                for item in self.LeafItems()]

    def MarkChangedValues(self, item=''):
        """colour every row that differs from the default - and every FOLDER that holds such a
        row somewhere below it, because a folded folder hides the mark otherwise (#2612)

        Args:
            item: the row to start at; '' is the whole tree, and the recursion passes the
                children of a folder

        Returns:
            True if anything below the given item differs from the defaults
        """
        if self.defaultValueStrings == {}:
            return False
        anyChanged = False
        for child in self.tree.get_children(item):
            if self.tree.get_children(child):
                isChanged = self.MarkChangedValues(child)
            else:
                isChanged = self.IsChangedValue(child)
            self.SetChangedTag(child, isChanged)
            anyChanged = anyChanged or isChanged
        return anyChanged

    def IsChangedValue(self, item):
        """does this row differ from the value a user starts with"""
        path = self.ItemPath(item)
        return (path in self.defaultValueStrings
                and self.defaultValueStrings[path] != self.tree.item(item,'values')[0])

    def SetChangedTag(self, item, isChanged):
        self.tree.item(item, tags=('changed',) if isChanged else ())

    def MarkChangedValue(self, item):
        """one row after it was edited, and the folders it sits in"""
        if self.defaultValueStrings == {}:
            return
        self.SetChangedTag(item, self.IsChangedValue(item))
        parent = self.tree.parent(item)
        while parent != '':
            childIsChanged = any('changed' in self.tree.item(child, 'tags')
                                 for child in self.tree.get_children(parent))
            self.SetChangedTag(parent, childIsChanged)
            parent = self.tree.parent(parent)

    def ChangedCodeLines(self, referenceValueStrings):
        return SettingsCodeLines(self.TreeLeaves(), referenceValueStrings,
                                 SettingsPrefix(self.settingsStructure), self.dictionaryTypesT)

    def OverriddenPaths(self):
        """the paths of THIS structure that ~/.exudyn/config.json already stores

        Only visualizationSettings are stored, so a simulationSettings dialog gets an empty set.
        """
        if CompiledSettingsClass(self.settingsStructure).__name__ != 'VisualizationSettings':
            return set()
        try:
            from exudyn.misc import overrideSettings
            return set(overrideSettings.Settings().get('visualizationSettings') or {})
        except Exception:                                                    # noqa: BLE001
            return set()      #a dialog that cannot open is worse than one that groups nothing

    def OnShowDiffToDefault(self):
        #THE DIFFERENCE IS TO THE REAL DEFAULT: a setting the override file stores IS a difference
        #to the default and is listed
        #as one - comparing against default-plus-override would hide exactly the settings that file
        #is about. What the file already covers is named separately, so that a user can see what
        #they would be copying and decide.
        description = 'every setting that differs from the Exudyn defaults'
        if self.defaultValueStrings == {}:
            description = 'the defaults are not available in this session'
        (lines, stored) = SplitStoredFromChanged(self.ChangedCodeLines(self.defaultValueStrings),
                                                 self.OverriddenPaths(), self.SettingsFileName())
        if stored != 0:
            description += '; ' + str(stored) + ' of them come from the settings file'
        self.ShowCodeLines('settings differing from the defaults', lines, description)

    def OnShowView(self):
        """the current model view of the render window as code (#2862)"""
        guiSC = GetRendererSystemContainer()
        if guiSC is None:
            self.ShowCodeLines('model view', [], 'no render window is open')
            return
        self.ShowCodeLines('model view', RenderStateCodeLines(guiSC.renderer.GetState()),
                           'the current model view, as CTRL+F3 prints it; paste the lines into the script'
                           ' instead of its own renderer.Start()')

    def SettingsFileName(self):
        """the override settings file, or a readable stand-in if it cannot be asked for"""
        try:
            from exudyn.misc import overrideSettings
            return overrideSettings.FileName()
        except Exception:                                                    # noqa: BLE001
            return '~/.exudyn/config.json'

    def OnStoreSettings(self):
        """write the settings that differ from the defaults to the override settings file - after
        showing exactly which ones

        TWO BUTTONS, BECAUSE THEY ARE TWO DECISIONS (#2693):
        *I like this look* and *I like this window here*. This one is the look, and the
        geometry of the RENDER window rides along in it - `view*.window.renderWindowSize` and
        `renderWindowPosition` are ordinary visualizationSettings, and the maintainer chose that on
        purpose: *"it is the straightforward way and becomes now natural, because it is only stored if
        it differs from default"*. The geometry of THIS dialog is the other button.

        Nothing else is touched: `exudyn.config`, the results monitor and the dialogs keep what they
        have.
        """
        fileName = self.SettingsFileName()
        changes = self.ChangedCodeLines(self.defaultValueStrings)
        lines = [('', '#' + str(len(changes)) + ' setting(s) of '
                  + SettingsPrefix(self.settingsStructure) + ' will be stored in ' + fileName)] \
            + changes

        def Store():
            try:
                from exudyn.misc import overrideSettings                     # noqa: PLC0415

                values = {}
                for (path, _) in changes:
                    structure = self.settingsStructure
                    parts = path.split('.')
                    for part in parts[:-1]:
                        structure = getattr(structure, part)
                    #an enum as the name of its value, and what a file cannot carry named (#2666)
                    (storable, reason) = overrideSettings.StorableValue(path, getattr(structure, parts[-1]))
                    if storable is None:
                        exudyn.Print('NOTE: not stored: visualizationSettings.' + path + ' - ' + reason)
                    else:
                        values[path] = storable
                overrideSettings.StoreSection('visualizationSettings', values)
                exudyn.Print('stored ' + str(len(values)) + ' setting(s) in ' + fileName)
            except Exception as exception:                                   # noqa: BLE001
                exudyn.Print('WARNING: could not store the settings: ' + str(exception))

        self.ShowCodeLines('store settings in ' + fileName, lines,
                           'the settings that differ from the defaults, including the size and'
                           ' position of the RENDER window if they differ; the position of THIS'
                           ' dialog is the "store positions" button',
                           confirm=('store', Store))

    def OnStorePositions(self):
        """write the size and the position of every open window to the override settings file

        The other half of #2693: where the window is, and nothing about what is in it. It stores
        whatever `dialogs.storeDialogPositions` says, because that flag is about a dialog remembering
        itself when it closes and this is a user asking.
        """
        fileName = self.SettingsFileName()

        #EVERY WINDOW THAT IS OPEN, not only this dialog (#2719): the other interactive dialogs -
        #the SolutionViewer among them - the PlotSensor windows, and the render window
        windows = [(self.parentFrame.title(), self.parentFrame.geometry())]
        try:
            if 'exudyn.interactive' in sys.modules:
                for dialog in list(sys.modules['exudyn.interactive'].openDialogs):
                    windows.append((dialog.dialogName, dialog.tkWindow.geometry()))
            if 'exudyn.plot' in sys.modules:
                windows += sys.modules['exudyn.plot'].PlotWindowGeometries()
        except Exception as error:                                           # noqa: BLE001
            exudyn.Print('WARNING: the positions of the other windows could not be read: ' + str(error))

        #THE RENDER WINDOW, where it IS - the render state - and not what the settings say. It is
        #stored as settings of the view, view0.window.renderWindowSize and renderWindowPosition, and
        #written into this dialog as well, so that the settings it shows and a later "store settings"
        #agree with the file
        renderValues = {}
        try:
            SC = GetRendererSystemContainer()
            if SC is not None and SC.renderer.IsActive() and \
                    CompiledSettingsClass(self.settingsStructure).__name__ == 'VisualizationSettings':
                state = SC.renderer.GetState()
                size = [int(v) for v in state['currentWindowSize']]
                position = [int(v) for v in state['currentWindowPosition']]
                if size[0] > 0 and size[1] > 0:
                    renderValues = {'view0.window.renderWindowSize': size,
                                    'view0.window.renderWindowPosition': position}
        except Exception as error:                                           # noqa: BLE001
            exudyn.Print('WARNING: the render window position could not be read: ' + str(error))

        lines = [('', '#the size and the position of these windows will be stored in ' + fileName)]
        lines += [('', '#under "dialogs": ' + str(name) + ': ' + str(geometry)) for (name, geometry) in windows]
        for (path, value) in renderValues.items():
            lines.append(('', '#under "visualizationSettings": ' + path + ' = ' + str(value)))

        def Store():
            stored = sum(1 for (name, geometry) in windows if StoreGeometryString(geometry, name))
            if renderValues:
                from exudyn.misc import overrideSettings                     # noqa: PLC0415
                section = dict(overrideSettings.Load().get('visualizationSettings') or {})
                section.update(renderValues)
                overrideSettings.StoreSection('visualizationSettings', section)
                self.ApplyValues({path: str(value) for (path, value) in renderValues.items()})
                stored += 1
            exudyn.Print('stored the positions of ' + str(stored) + ' window(s) in ' + fileName)

        self.ShowCodeLines('store positions in ' + fileName, lines,
                           'where the open windows are - this dialog, the other dialogs, the plot'
                           ' windows and the render window - so that they open there next time;'
                           ' the settings this dialog shows are the "store settings" button',
                           confirm=('store', Store))

    def OnShowSessionChanges(self):
        self.ShowCodeLines('settings changed in this dialog',
                           self.ChangedCodeLines(self.openingValueStrings),
                           'what was changed since this dialog was opened')

    def CurrentValueStrings(self):
        """what every row of the tree shows now, as {path: valueString}"""
        return {self.ItemPath(item): self.tree.item(item, 'values')[0]
                for item in self.LeafItems()}

    def PushUndoState(self):
        """remember the WHOLE state before a change, which is what makes undo always work

        A single edited value, a reset and a revert all go through this, so undo takes any of
        them back, and a chain of them one by one (#2627). The state
        is ~470 short strings, which costs nothing next to the redraw it triggers.
        """
        self.undoStack.append(self.CurrentValueStrings())
        self.undoButton.configure(state=tk.NORMAL)

    def WriteValue(self, item, valueStr):
        """put a value into a row and let everything that follows from it happen"""
        self.tree.item(item, values=(valueStr, self.typeStorage[item],
                                     self.descriptionStorage[item]))
        self.MarkChangedValue(item)

    def ApplyValues(self, valueStrings, pushUndo=True):
        """write a whole set of {path: valueString} into the tree and into the settings

        Args:
            valueStrings: what to write; a path the tree does not hold is ignored
            pushUndo: remember the state before, so that undo takes the whole set back;
                False only for the undo itself, which must not push what it is popping

        Returns:
            the number of rows that changed
        """
        changed = 0
        if pushUndo:
            self.PushUndoState()
        for item in self.LeafItems():
            path = self.ItemPath(item)
            if path in valueStrings and valueStrings[path] != self.tree.item(item,'values')[0]:
                self.WriteValue(item, valueStrings[path])
                changed += 1
        if changed != 0:
            self.modifiedDictionary = self.GetDictionary('')
            self.UpdateSettingsStructure()
            self.ShowInfo(self.tree.focus())
        elif pushUndo:
            self.undoStack.pop()                    #nothing changed, so there is nothing to undo
            self.undoButton.configure(state=tk.NORMAL if self.undoStack != [] else tk.DISABLED)
        return changed

    def OnReset(self):
        """every setting back to what a user starts with

        It does NOT ask (maintainer, 2026-09-23): a wrong click is taken back by undo, or by
        revert, and a question that is always answered with yes is only in the way.
        """
        if self.defaultValueStrings == {}:
            tk.messagebox.showinfo('reset', 'the defaults are not available in this session')
            return
        self.ApplyValues(self.defaultValueStrings)

    def OnRevert(self):
        """back to the state the dialog opened with; it does not ask either"""
        self.ApplyValues(self.openingValueStrings)

    def OnUndo(self):
        """one whole state back, whether that was one value, a reset or a revert"""
        if self.undoStack == []:
            return
        previousState = self.undoStack.pop()
        self.undoButton.configure(state=tk.NORMAL if self.undoStack != [] else tk.DISABLED)
        self.ApplyValues(previousState, pushUndo=False)

    def ShowCodeLines(self, title, lines, description, confirm=None):
        """the changes as the code that makes them, in a window that shows AND copies: a dialog
        session that can be pasted into a script (maintainer, 2026-09-23)

        confirm: (buttonText, function) adds a button that closes the window and calls the
        function, which is how the store button shows what it will write before it writes it"""
        window = tk.Toplevel(self)
        window.title(title)
        #THE WINDOW HAS TO BE SEEN (#2621). The settings dialog is topmost - it has to be, it
        #blocks the render window - and a Toplevel of it opens at the same place BEHIND it, which
        #looks exactly like a button that does nothing: the content was right all along. Four
        #things together make it visible, and none of them alone was enough: transient ties it to
        #the dialog, topmost puts it in front of a topmost parent, an offset means it cannot be
        #hidden by the dialog even if the stacking fails, and grab_set makes it modal, which is
        #what a window manager never puts behind.
        window.transient(self.parentFrame)
        ApplyDialogWindowSettings(window, alwaysTopmost=True)
        #THE DIALOG STEPS ASIDE (maintainer, 2026-09-23): it keeps itself -topmost, which is why
        #this window came up behind it whatever was done to the window. The flag is taken off the
        #dialog - and off its root, where EditDictionaryWithTypeInfo sets it - while the window is
        #open, and put back when it closes.
        topmostWindows = []
        for candidate in [self.parentFrame, self.parentFrame.master]:
            try:
                if candidate is not None and candidate.attributes('-topmost'):
                    candidate.attributes('-topmost', False)
                    topmostWindows.append(candidate)
            except (tk.TclError, AttributeError):
                pass

        def RestoreTopmost():
            for candidate in topmostWindows:
                try:
                    candidate.attributes('-topmost', True)
                except tk.TclError:
                    pass

        window.bind('<Destroy>', lambda event: RestoreTopmost() if event.widget is window
                    else None)
        self.parentFrame.update_idletasks()
        window.geometry('+' + str(self.parentFrame.winfo_rootx() + 60) + '+'
                        + str(self.parentFrame.winfo_rooty() + 60))
        window.lift()
        window.focus_force()
        window.grab_set()               #released when the window is destroyed
        window.grid_columnconfigure(0, weight=1)
        window.grid_rowconfigure(1, weight=1)

        tk.Label(window, text=description, justify=tk.LEFT, anchor=tk.W).grid(
            row=0, column=0, columnspan=2, sticky=tk.E+tk.W, padx=8, pady=(8, 2))

        textArea = tk.Text(window, wrap=tk.NONE, width=80, height=min(25, max(4, len(lines) + 1)),
                           background=codeLineBackground, font=(None, self.codeFontSize))
        textArea.grid(row=1, column=0, columnspan=2, sticky=tk.NSEW, padx=(8, 0))
        #a structure can differ from the defaults in more lines than fit on a screen
        scrollbar = ttk.Scrollbar(window, orient='vertical', command=textArea.yview)
        scrollbar.grid(row=1, column=2, sticky=tk.N+tk.S, padx=(0, 8))
        textArea.configure(yscrollcommand=scrollbar.set)
        code = '\n'.join([line for (_, line) in lines])
        textArea.insert(tk.END, code if lines else '#nothing')
        textArea.configure(state='disabled')

        def CopyAll():
            if lines:
                self.clipboard_clear()
                self.clipboard_append(code)
                self.update()       #without this the clipboard is empty once the window closes

        tk.Button(window, text='copy all', command=CopyAll).grid(row=2, column=0, pady=6)
        tk.Button(window, text='cancel' if confirm is not None else 'close',
                  command=window.destroy).grid(row=2, column=1, pady=6)
        if confirm is not None:
            #the window is what asks: everything that writes outside this session shows what it
            #will write first (#2685)
            def Confirm():
                window.destroy()
                confirm[1]()

            tk.Button(window, text=confirm[0], command=Confirm).grid(row=2, column=2, pady=6)
        window.bind('<Escape>', lambda event: window.destroy())

    #++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    #FIND A SETTING (#2607)

    def OnFindFocus(self, event=None):
        """CTRL-F: the find entry takes the focus and its text is selected, ready to be replaced"""
        self.findEntry.focus_set()
        self.findEntry.select_range(0, tk.END)
        return 'break'

    def UpdateFindHits(self):
        """as the text is typed: the hits, in the drop-down, names before descriptions"""
        self.findHits = FindMatches(self.findLeaves, self.findVar.get())
        self.findIndex = -1
        self.findCombo['values'] = tuple(label for (_, label) in self.findHits)
        self.findCombo.configure(state='readonly' if self.findHits else 'disabled')
        self.findCombo.set(str(len(self.findHits)) + ' found' if self.findHits
                           else ('nothing found' if self.findVar.get().strip() != '' else ''))

    def OnFindPick(self, event=None):
        """a hit is picked from the drop-down: jump to it"""
        index = self.findCombo.current()
        if 0 <= index < len(self.findHits):
            self.findIndex = index
            self.JumpToPath(self.findHits[index][0])

    def OnFindNext(self, event=None):
        """RETURN, F3 or the find button: the next hit, and around at the end"""
        if self.findHits == []:
            return 'break'
        self.findIndex = (self.findIndex + 1) % len(self.findHits)
        self.findCombo.current(self.findIndex)
        self.JumpToPath(self.findHits[self.findIndex][0])
        return 'break'

    def JumpToPath(self, path):
        """show the row a path names: see() opens the folders it sits in and scrolls it into view.
        The focus stays in the find entry, so that RETURN steps on."""
        item = self.itemByPath.get(path, '')
        if item == '':
            return
        self.CancelCellEdit()
        self.tree.see(item)
        self.tree.selection_set(item)
        self.tree.focus(item)
        self.ShowInfo(item)

    def CellBox(self, item):
        """where the value cell of a row is, in the coordinates of the tree"""
        self.tree.see(item)                   #a row that is scrolled away has no box
        box = self.tree.bbox(item, 'value')
        return box if box != '' else None

    def StartCellEdit(self, item):
        """an Entry, or a Combobox for a type with a fixed set of values, over the value cell"""
        self.CancelCellEdit()
        box = self.CellBox(item)
        if box is None:
            return
        (x, y, width, height) = box
        self.selectedItem = item
        vType = self.typeStorage[item]
        value = self.tree.item(item,'values')[0]

        if vType in self.dictionaryTypesT:    #bool and the enums: pick, do not type
            #without its type in front of it, or the entries are all prefix (#2635)
            values = tuple(EnumDisplayName(str(entry), vType)
                           for entry in self.dictionaryTypesT[vType])
            shown = EnumDisplayName(value, vType)
            self.cellCombo['values'] = values
            self.cellCombo.set(shown if shown in values else (values[0] if values else ''))
            self.cellCombo.place(x=x, y=y, width=width, height=height)
            self.cellCombo.focus_set()
        else:
            self.editItemVar.set(value)
            self.cellEditor.place(x=x, y=y, width=width, height=height)
            self.cellEditor.focus_set()
            self.cellEditor.select_range(0, tk.END)

    def CancelCellEdit(self, event=None):
        """take the editor away without writing anything"""
        self.selectedItem = ''
        self.cellEditor.place_forget()
        self.cellCombo.place_forget()

    def OnCellCancel(self, event):
        self.CancelCellEdit()
        self.tree.focus_set()

    def OnCellCommit(self, event=None):
        """write the edited value back, if it is one the setting can take"""
        item = self.selectedItem
        if item == '':
            return
        vType = self.typeStorage[item]
        vSize = self.sizeStorage[item]
        if vType in self.dictionaryTypesT:
            valueStr = self.cellCombo.get()   #short, like every other value (#2640)
        else:
            valueStr = self.editItemVar.get()

        [isValid, message] = CheckType(valueStr, vType, vSize, self.dictionaryTypesT)
        if isValid:
            #CheckType says the SHAPE is right; the range lives in the type name and only
            #ConvertString2Value knows it (#2597)
            try:
                [_, rangeMessage] = ConvertString2Value(valueStr, vType, vSize,
                                                        self.dictionaryTypesT)
            except (ValueError, SyntaxError) as exception:
                rangeMessage = str(exception)
            if rangeMessage != '':
                (isValid, message) = (False, rangeMessage)

        if not isValid:
            self.cellEditor.unbind('<FocusOut>')    #otherwise OnCellCommit is called twice
            tk.messagebox.showerror("Error", self.ItemPath(item) + ' expects ' + vType + ':\n'
                                    + message + '\npress ESCAPE to keep the original value')
            self.cellEditor.bind('<FocusOut>', self.OnCellCommit)
            return

        self.CancelCellEdit()
        self.PushUndoState()
        self.tree.item(item, values=(valueStr, vType, self.descriptionStorage[item]))
        self.MarkChangedValue(item)
        self.tree.focus_set()
        self.tree.focus(item)
        self.ShowInfo(item)
        self.modifiedDictionary = self.GetDictionary('')
        self.UpdateSettingsStructure()      #only if according flag set in visualizationSettings


def EditDictionaryWithTypeInfo(settingsStructure, exu=None, dictionaryName='edit'):
    """edit dictionaryData and return modified (new) dictionary

    Args:
        settingsStructure: hierarchical settings structure, e.g., SC.visualizationSettings
        exu: exudyn module
        dictionaryName: name displayed in dialog

    Returns:
        returns modified dictionary, which can be used, e.g., for SC.visualizationSettings.SetDictionary(...)
    """

    [root, tkWindow, tkinterAlreadyRunning] = GetTkRootAndNewWindow()
    
    windowHeight = treeEditDefaultHeight
    if treeEditMaxInitialHeight > treeEditDefaultHeight:
        try:
            screen_height = root.winfo_screenheight()
            if screen_height > 1.2*treeEditDefaultHeight:
                windowHeight = int(min(treeEditMaxInitialHeight, 0.85*screen_height))
        except tk.TclError:
            exudyn.Print('WARNING: EditDictionaryWithTypeInfo could not determine the screen'
                         ' size; please report this with your Python version and platform as a'
                         ' github issue')

    #the size and the position of the last time, if the user asked for them to be remembered
    #(#2608)
    RestoreWindowGeometry(tkWindow, dictionaryName, treeEditDefaultWidth, windowHeight)
    recordedGeometry = RememberWindowGeometry(tkWindow, dictionaryName, settingsStructure)

    guiSC = GetRendererSystemContainer()
    updateOnChange = False
    topmost = True
    alphaTransparency = 1 #<1 means transparency
    treeOpen = True
    columnWidths = defaultColumnWidths   #a dialog of 'python -m exudyn dialogs' has no container
    if guiSC is not None:
        updateOnChange = guiSC.visualizationSettings.dialogs.multiThreadedDialogs
        topmost = guiSC.visualizationSettings.dialogs.alwaysTopmost
        if guiSC.visualizationSettings.dialogs.alphaTransparency <= 1:
            alphaTransparency = guiSC.visualizationSettings.dialogs.alphaTransparency
        treeOpen = guiSC.visualizationSettings.dialogs.openTreeView
        columnWidths = [guiSC.visualizationSettings.dialogs.columnWidthName,
                        guiSC.visualizationSettings.dialogs.columnWidthValue,
                        guiSC.visualizationSettings.dialogs.columnWidthType]

    [systemScaling, fontFactor] = DialogScaling(root)

    tkWindow.lift() #brings it to front of other; not always "strong" enough
    if topmost:
        root.attributes("-topmost", True) #puts window topmost (permanent)
    if alphaTransparency <= 1:
        tkWindow.attributes("-alpha", alphaTransparency) 
        
    tkWindow.title(dictionaryName)
    tkWindow.focus_force() #window has focus

    [textHeight, columnScale] = DialogRowMetrics(root, fontFactor)
    fontSize = DialogFontSize(fontFactor)

        
    style = ttk.Style(tkWindow)
    style.configure('Treeview', rowheight=textHeight) 
    
    style.configure("Treeview.Heading", font=(None, fontSize))
    style.configure("Treeview", font=(None, fontSize))
    
    
    comboListsT = GetComboBoxListsDict(exu)
    ex=TkinterEditDictionaryWithTypeInfo(parent=tkWindow, settingsStructure=settingsStructure, dictionaryTypesT=comboListsT, 
                                         updateOnChange=updateOnChange, treeOpen=treeOpen, textHeight = textHeight,
                                         systemScaling = systemScaling, fontFactor = fontFactor,
                                         columnScale = columnScale, columnWidths = columnWidths)
    ex.pack(fill="both", expand=True)

    if not tkinterAlreadyRunning:
        tk.mainloop()
    else:
        root.wait_window(tkWindow)
    
    StoreWindowGeometry(recordedGeometry, dictionaryName)
    settingsStructure.SetDictionary(ex.modifiedDictionary)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#hierarchical lists, without type info:
class TkinterEditDictionary(tk.Frame):
    def __init__(self, parent, dictionaryData, dictionaryIsEditable=True, textHeight = 15):
        tk.Frame.__init__(self, parent)
        
        self.parent = parent
        self.dictionaryIsEditable = dictionaryIsEditable
        self.longestColumn = 20 #min value
        self.textHeight = textHeight

        #create treeview:
        self.tree = ttk.Treeview(self, columns=("value"), height=textHeight)
        self.AddNodeFromDictionary(value=dictionaryData, parentNode="")

        self.vsb = ttk.Scrollbar(self, orient="vertical", command=self.tree.yview, )
        self.hsb = ttk.Scrollbar(self, orient="horizontal", command=self.tree.xview)
        self.tree.configure(yscrollcommand=self.vsb.set, xscrollcommand=self.hsb.set)

        self.vsb.pack(side="right", fill="y")
        self.tree.pack(side="top", fill="both", expand=True)

        self.hsb.pack(side="bottom",anchor='w', fill="x",padx=10,pady=5)

        self.tree.bind('<<TreeviewSelect>>', self.TreeviewSelect) #selection changed
        self.tree.bind("<Double-1>", self.OnTreeDoubleClick) #an item has been selected for change
        self.tree.bind("<Return>", self.OnTreeDoubleClick) #an item has been selected for change
        self.tree.bind("<Escape>", self.OnQuit) 
        self.tree.bind("q", self.OnQuit) 
        
        #create the entry field for editing the treeview value

        if True:
            self.selectedItem = '' #will change to valid item in order to change value
            self.editItemName = tk.StringVar()
            self.editItemVar = tk.StringVar()
    
            self.editName = tk.Label(self, textvariable=self.editItemName)#, width=10)
            self.editName.pack(side="left", fill="both", expand=True)
    
            self.editItem = tk.Entry(self, textvariable=self.editItemVar)#, width=40)
    #        self.editItem.grid(row=1, column=1)
            self.editItem.pack(side="right", fill="both", expand=True)
            self.editItem.bind('<Return>', self.OnEditEntryItem)
            self.editItem.bind('<FocusOut>', self.OnEditEntryItem)
            self.editItem.bind('<Escape>', self.OnEditEscapeItem)

        first = self.tree.get_children('')[0]
        self.tree.focus_set()
        self.tree.focus(first)
        self.tree.selection_set((first))
        self.tree.heading('#0', text='variable', anchor='w')
        self.tree.column("#0",minwidth=300, stretch=True)        
        self.tree.heading('#1', text='value', anchor='w')
        self.tree.column("#1",anchor='w', #width=200,
                         minwidth=self.longestColumn*12,
                         stretch=True)
        self.hsb.config(command = self.tree.xview)

        self.modifiedDictionary = self.GetDictionary('')

    def IsItemIndex(self, var):
        return (isinstance(var, exudyn.NodeIndex) or
                isinstance(var, exudyn.ObjectIndex) or
                isinstance(var, exudyn.MarkerIndex) or
                isinstance(var, exudyn.LoadIndex) or
                isinstance(var, exudyn.SensorIndex))
    #create dictionary
    #this is used e.g. for mouse right-button dialog
    def AddNodeFromDictionary(self, value, parentNode="", key=None):
        if key is None:
            id = ""
        else:
            if key != 'TODO':
                id = self.tree.insert(parentNode, "end", text=key)
            else:
                id = self.tree.insert(parentNode, "end", text='<unavailable>')

        if isinstance(value, dict):
            self.tree.item(id, open=True)
            for (key, value) in value.items():
                self.AddNodeFromDictionary(value, id, key)
        else:
            if isinstance(value, bool) : #bool first, bool is also int
                self.tree.item(id, values=(str(value)))
            elif isinstance(value, int) or isinstance(value, float):
                self.tree.item(id, values=(value))
            elif isinstance(value, np.ndarray):
                if (value.size < 5000):
                    valueList = value.tolist()
                    valueListStr = str(valueList)
                    self.longestColumn = max(self.longestColumn,len(valueListStr))
                    valueStr = valueListStr.replace(' ','\\ ').replace(',','\\,')
                    self.tree.item(id, values=(valueStr))
                else:
                    self.tree.item(id, values=('<unavailable>'))
            elif isinstance(value, list):
                if np.array(value).size < 5000:
                    valueListStr = str(value)
                    self.longestColumn = max(self.longestColumn,len(valueListStr))
                    valueStr = valueListStr.replace(' ','\\ ').replace(',','\\,')
                    self.tree.item(id, values=(valueStr))
                else:
                    self.tree.item(id, values=('<unavailable>'))
            elif isinstance(value, str) and value!='Get graphics data to be implemented':
                self.longestColumn = max(self.longestColumn,len(value))
                self.tree.item(id, values=(value.replace(' ','\\ ')))
            elif self.IsItemIndex(value):
                self.tree.item(id, values=(int(value)))
            else:
                self.tree.item(id, values=('<unavailable>'))

    def GetDictionary(self, item):
        d=dict()
        kids = self.tree.get_children(item)
        for i in kids:
            nchilds = len(self.tree.get_children(i))
            if nchilds == 0:
                d.update({self.tree.item(i,'text'): self.tree.item(i,'values')[0]})
            else:
                d.update({self.tree.item(i,'text'): self.GetDictionary(i)})
        return d

    def OnTreeDoubleClick(self,event):
        item = self.tree.selection()[0]
        nchilds = len(self.tree.get_children(item))

        if nchilds == 0:
            self.editItemVar.set(self.tree.item(item,'values')[0])
            
            s = self.tree.item(item,'text')
            i=0
            pItem=self.tree.parent(item)
            while i < 3 and pItem != '':
                i+=1
                s = self.tree.item(pItem,'text') + '.' + s
                pItem=self.tree.parent(pItem)
                
                
            self.editItemName.set(s)
            self.selectedItem = item #now item can be modified
            self.editItem.focus_set()
        else: #move to next item
            next = self.tree.get_children(item)[0]
            if next != '':
                self.tree.selection_set((next))
                self.tree.focus(next)
                self.tree.see(next)
            

    def OnQuit(self,event): #new selection --> nothing to edit for now
        self.parent.destroy()
        
    def TreeviewSelect(self,event): #new selection --> nothing to edit for now
        self.selectedItem = '' #now item can be modified
        self.editItemVar.set('')
        self.editItemName.set('')
        
        
    def OnEditEntryItem(self,event):
        if self.selectedItem != '':
            if self.dictionaryIsEditable:
                valueStr = self.editItemVar.get()
                valueStr = str(valueStr).replace(' ','\\ ')
                self.tree.item(self.selectedItem, values=(valueStr))
                currentItem = self.selectedItem
                self.selectedItem = '' #now item can be modified
                self.editItemVar.set('')
                self.editItemName.set('')
                self.tree.focus_set()
    
                #as return is pressed, move to next item
                next = self.tree.next(currentItem)
                if next != '':
                    self.tree.selection_set((next))
                    self.tree.focus(next)
                    self.tree.see(next)
                else:
                    par = self.tree.parent(currentItem)
                    if par != '' and self.tree.next(par) != '':
                        next = self.tree.next(par)
                        self.tree.selection_set(next)
                        self.tree.focus(next)
                        self.tree.see(next)
                self.modifiedDictionary = self.GetDictionary('') #update stored dictionary

    def OnEditEscapeItem(self,event):
        if self.selectedItem != '':
            valueStr = self.tree.item(self.selectedItem,'values')[0]
            valueStr = str(valueStr).replace(' ','\\ ')
            self.editItemVar.set(valueStr)
            self.selectedItem = '' #now item can be modified
#            self.editItemVar.set('')
#            self.editItemName.set('')
            self.tree.focus_set()
    

#edit dictionaryData and return modified (new) dictionary
def EditDictionary(dictionaryData, dictionaryIsEditable=True, dialogName=''):
    [root, tkWindow, tkinterAlreadyRunning] = GetTkRootAndNewWindow()

    tkWindow.geometry(str(dialogDefaultWidth)+'x'+str(dialogDefaultHeight))

    guiSC = GetRendererSystemContainer()
    topmost = True
    alphaTransparency = 1 #<1 means transparency
    if guiSC is not None:
        topmost = guiSC.visualizationSettings.dialogs.alwaysTopmost
        if guiSC.visualizationSettings.dialogs.alphaTransparency <= 1:
            alphaTransparency = guiSC.visualizationSettings.dialogs.alphaTransparency

    [systemScaling, fontFactor] = DialogScaling(root)

    tkWindow.lift() #brings it to front of other; not always "strong" enough
    if topmost:
        tkWindow.attributes("-topmost", True) #puts window topmost (permanent)
    if alphaTransparency:
        tkWindow.attributes("-alpha", alphaTransparency) 
    

    [textHeight, _] = DialogRowMetrics(root, fontFactor)

    tkWindow.title(dialogName)
    tkWindow.focus_force() #window has focus

    style = ttk.Style(tkWindow)
    style.configure('Treeview', rowheight=textHeight) 
    #it seems that the font size should not be changed (what is done due to scaling internally ...)

    style.configure("Treeview.Heading", font=(None, DialogFontSize(fontFactor)))
    style.configure("Treeview", font=(None, DialogFontSize(fontFactor)))

    #this has no effect style.configure("Vertical.TScrollbar", width=4)

    ex=TkinterEditDictionary(tkWindow, dictionaryData, dictionaryIsEditable, textHeight)
    ex.pack(fill="both", expand=True)

    if not tkinterAlreadyRunning:
        tk.mainloop() #run second main loop? will crash
    else:
        root.wait_window(tkWindow)
    
    if dictionaryIsEditable:
        return ex.modifiedDictionary
    else:
        return {}


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#THE DIALOGS THE RENDERER OPENS (#2595)
#
#They were written as Python inside src/Main/rendererPythonInterface.cpp - 220 of its 775 lines were
#raw string literals holding tkinter code, where no syntax check, no ruff and no import test ever
#saw them. Each one is a function here now, and the C++ calls it. The window setup that the C++
#assembled by string concatenation from visualizationSettings.dialogs is ApplyDialogWindowSettings.

#how much of the screen a restored dialog leaves free, in pixels: a window manager has a task
#bar and a title bar, and a dialog that fills the screen exactly has its buttons under them
dialogScreenMargin = 40


def StoreDialogPositions(settingsStructure=None):
    """True if a dialog should store where it was left, when it closes.

    Args:
        settingsStructure: the structure the dialog is editing, if it is a `visualizationSettings`;
            it is asked FIRST, because it is the one the user is looking at

    Returns:
        the value of `visualizationSettings.dialogs.storeDialogPositions`, or False when there is
        nothing to ask

    Note:
        This decides whether a dialog stores ITSELF on closing, and nothing else: a geometry that
        is already stored - by this flag, by the store button or by a script - is used whenever a
        dialog opens, see `RestoreWindowGeometry` (#2686).

        The structure is asked before the renderer's SystemContainer because
        `python -m exudyn dialogs` has no container at all, and a script that has not started the
        renderer has none that can be found - so the flag used to be False there however it was
        set, and such a dialog could never store itself.
    """
    for structure in [settingsStructure,
                      None if GetRendererSystemContainer() is None
                      else GetRendererSystemContainer().visualizationSettings]:
        if structure is None:
            continue
        try:
            return bool(structure.dialogs.storeDialogPositions)
        except AttributeError: #not a visualizationSettings, or an older core without the setting
            continue
    return False


def RestoreWindowGeometry(tkWindow, name, width=None, height=None):
    """Give a dialog the size and position it was left at, as far as that is safe.

    Args:
        tkWindow: the window
        name: the title of the dialog, which is what it is stored under
        width: the width it would otherwise get, in pixels; None leaves the size to the layout when
            nothing is stored, which is what a dialog whose size is computed from its widgets needs
        height: the height it would otherwise get, or None

    Returns:
        None

    Note:
        The **size** is restored always and the **position** only when the window would still be
        reachable on the current screen. A
        monitor that is gone, a resolution that changed, a laptop that was undocked: each of them
        would otherwise put the dialog where nobody can close it.

        A GEOMETRY THAT IS STORED IS USED, whatever `dialogs.storeDialogPositions` says
        (#2686). That flag decides whether a dialog stores ITSELF when
        it closes; this function only reads what is there, and it is there because the flag was on,
        or because the store button of the dialog wrote it, or because a script did. Asking the flag
        here is what made the store button (#2685) write something that nothing read back.
    """
    from exudyn.misc import overrideSettings as userSettings

    (size, position) = userSettings.DialogGeometry(name)
    if size is None and (width is None or height is None):
        return                     #nothing stored and no size asked for: the layout decides

    try: #the virtual desktop where tkinter knows it, the screen otherwise
        screen = [tkWindow.winfo_vrootx(), tkWindow.winfo_vrooty(),
                  max(tkWindow.winfo_vrootwidth(), tkWindow.winfo_screenwidth()),
                  max(tkWindow.winfo_vrootheight(), tkWindow.winfo_screenheight())]
    except tk.TclError:
        screen = None

    if size is not None:
        #A STORED SIZE IS CUT DOWN TO THIS SCREEN, for the reason the position is checked at all: a
        #dialog is taller than it is wide, its buttons are in the bottom row, and a size stored on a
        #larger or a rotated monitor would put that row - the close button with it - off the screen
        (width, height) = (size[0], size[1])
        if screen is not None:
            width = min(width, max(320, screen[2] - 2 * dialogScreenMargin))
            height = min(height, max(240, screen[3] - 2 * dialogScreenMargin))
    geometry = str(width) + 'x' + str(height)

    if position is not None and screen is not None \
            and userSettings.PositionIsReachable(position, screen):
        geometry += '+' + str(position[0]) + '+' + str(position[1])
    tkWindow.geometry(geometry)


def RememberWindowGeometry(tkWindow, name, settingsStructure=None):
    """Record where a dialog is while it lives, so that it can be stored when it closes.

    Args:
        tkWindow: the window
        name: the title of the dialog, which is what it is stored under
        settingsStructure: the structure being edited, which is asked for
            `dialogs.storeDialogPositions` before the renderer's SystemContainer

    Returns:
        a dictionary that `StoreWindowGeometry` reads; empty when nothing is to be stored

    Note:
        The geometry cannot be read after the window is destroyed, and a dialog is left in three
        ways - the close button, Escape, and the window manager - so it is recorded on every
        `<Configure>` and the last value is the one that is stored.
    """
    recorded = {}
    if not StoreDialogPositions(settingsStructure):
        return recorded

    def Record(event=None):
        try:
            recorded['geometry'] = tkWindow.geometry()
        except Exception: #a window that is going away has no geometry any more
            pass

    tkWindow.bind('<Configure>', Record, add='+')
    Record()
    return recorded


def StoreGeometryString(geometry, name):
    """Store one 'WIDTHxHEIGHT+X+Y' under the name of a dialog.

    Args:
        geometry: what tkWindow.geometry() reported
        name: the title of the dialog

    Returns:
        True if it was stored

    Note:
        This does NOT ask whether storeDialogPositions is on: the flag decides whether a dialog
        remembers itself on closing, and the store button of the settings dialog stores on request
        (#2685). RememberWindowGeometry is where the flag is read.
    """
    from exudyn.misc import overrideSettings as userSettings

    #'WIDTHxHEIGHT+X+Y', where a coordinate left of or above the primary screen is reported as
    #'+-1500' by some window managers and as '-1500' by others
    match = re.match(r'^(\d+)x(\d+)\+?(-?\d+)\+?(-?\d+)$', str(geometry).strip())
    if match is None:
        if str(geometry).strip() != '':
            exudyn.Print('WARNING: could not store the position of the dialog "' + str(name)
                         + '": the window reported the geometry "' + str(geometry) + '"')
        return False
    userSettings.StoreDialogGeometry(name, [int(match.group(1)), int(match.group(2))],
                                     [int(match.group(3)), int(match.group(4))])
    return True


def StoreWindowGeometry(recorded, name):
    """Store what `RememberWindowGeometry` recorded, after the dialog has closed.

    Args:
        recorded: what `RememberWindowGeometry` returned
        name: the title of the dialog
    """
    StoreGeometryString(recorded.get('geometry', ''), name)


def ApplyDialogWindowSettings(tkWindow, alwaysTopmost=None, alphaTransparency=None):
    """Apply what visualizationSettings.dialogs says about a dialog window.

    Args:
        tkWindow: the window to configure
        alwaysTopmost: None takes dialogs.alwaysTopmost; True or False overrides it
        alphaTransparency: None takes dialogs.alphaTransparency; a float overrides it

    Returns:
        None
    """
    guiSC = GetRendererSystemContainer()
    if guiSC is not None:
        if alwaysTopmost is None:
            alwaysTopmost = guiSC.visualizationSettings.dialogs.alwaysTopmost
        if alphaTransparency is None:
            alphaTransparency = guiSC.visualizationSettings.dialogs.alphaTransparency

    if alwaysTopmost:
        tkWindow.attributes('-topmost', True)   #permanent, otherwise it hides behind the renderer
    if alphaTransparency is not None and alphaTransparency < 1:
        tkWindow.attributes('-alpha', alphaTransparency)


#the keyboard and mouse commands of the renderer, from the ONE table that also feeds the
#tables of docs/manual/GUI.md through tools/generators/keyBindingsEmitter.py. This text
#was a third copy of them, and it had drifted (#2591).
rendererHelpText = RendererHelpText()


def ShowHelpDialog():
    """The keyboard and mouse commands of the renderer, in a read-only window; opened with H in
    the render window.

    Returns:
        None
    """
    [root, tkWindow, tkRuns] = GetTkRootAndNewWindow()
    ApplyDialogWindowSettings(tkWindow)

    tkWindow.title("Help on keyboard commands and mouse")
    tkWindow.lift()                          #window has focus
    tkWindow.bind("<Escape>", lambda event: tkWindow.destroy())
    tkWindow.focus_force()

    scrollW = tk.Scrollbar(tkWindow)
    #resize grid columns/rows if window is resized:
    tkWindow.grid_columnconfigure(0, weight=1)
    tkWindow.grid_rowconfigure(0, weight=1)

    textW = tk.Text(tkWindow, height=30, width=90, background='gray98')
    textW.focus_set()
    textW.grid(row=0, column=0, padx=10, pady=10, sticky=tk.NSEW)
    scrollW.grid(row=0, column=1, pady=10, sticky=tk.NSEW)
    scrollW.config(command=textW.yview)
    textW.config(yscrollcommand=scrollW.set)

    textW.insert(tk.END, rendererHelpText)
    textW.configure(state='disabled')        #unable to edit

    if tkRuns:
        root.wait_window(tkWindow)
    else:
        tk.mainloop()


#the examples shown under the command input; they are what people ask for most often while a
#simulation runs
pythonCommandExamples = ('helpful examples:\n'
                         'show overall info of mbs:\n'
                         'print(mbs)\n'
                         '#change current dynamic solver end time:\n'
                         "mbs.sys['dynamicSolver'].it.endTime=10 \n"
                         '#change verbose mode of dynamic solver:\n'
                         "mbs.sys['dynamicSolver'].output.verboseMode=1\n"
                         '#stop file writing:\n'
                         "mbs.sys['dynamicSolver'].output.writeToSolutionFile=False\n"
                         '#print values of sensor 0:\n'
                         'print(mbs.GetSensorValues(0))\n'
                         '#pause after each step:\n'
                         'simulationSettings.pauseAfterEachStep=True\n'
                         '\n#==>BUT changing simulationSettings is dangerous!')


def ModelScope():
    """the namespace a command of the command window runs in: the one the MODEL lives in

    `__main__` is the script the user started, or the console they are typing in, so this is
    where `mbs`, `SC` and everything else the model defined are. The command window used to
    be a Python string that the C++ executed in exactly this namespace; as a function of this
    module it would otherwise see the module's own globals, where there is no `mbs` (#2654).

    Returns:
        the dictionary of `__main__`, which is written to as well as read: an assignment in
        the command window has to survive the command
    """
    import __main__                                                          # noqa: PLC0415
    return vars(__main__)


def ShowPythonCommandDialog():
    """A window that executes a Python command in the global scope of the running model; opened
    with X in the render window. CTRL+RETURN runs what is in the text area.

    Returns:
        None
    """
    import traceback                                                            # noqa: PLC0415
    from tkinter import scrolledtext                                            # noqa: PLC0415

    [root, tkWindow, tkRuns] = GetTkRootAndNewWindow()
    tkWindow.title("Exudyn command window")
    ApplyDialogWindowSettings(tkWindow)

    #resize grid columns/rows if window is resized:
    tkWindow.grid_columnconfigure(0, weight=1)
    tkWindow.grid_rowconfigure(1, weight=1)

    description = ('Enter Python command which operates in global scope of you Python model;\n'
                   'Evaluate or CHANGE your current model (parameters) during simulation;\n'
                   'Press CRTL+RETURN to execute, escape to close:')

    label = tk.Label(tkWindow, text=description, justify=tk.LEFT,
                     relief=tk.SUNKEN, background='gray94')
    label.grid(row=0, column=0, padx=15, pady=(15, 0), sticky='W')

    textArea = scrolledtext.ScrolledText(tkWindow, wrap=tk.WORD, width=60, height=8)
    #configure tab size:
    font = tk.font.Font(font=textArea['font'])
    textArea.config(tabs=font.measure(' '*4))   #in pixels

    def OnRunCode(event):
        commandString = textArea.get('1.0', tk.END)
        print('command window execute:\n', commandString.strip(), sep='')  #printout the command
        print('output:')

        if commandString.strip() == '':      #empty command causes exception
            return None
        commandString = commandString.replace('\t', ' '*4)  #tabs may cause problems

        try:
            #ONE dictionary, not globals() and locals(): with two, an assignment lands in the
            #locals of this handler and is gone when it returns - and the label of this window
            #promises that a model can be CHANGED from here (#2654)
            exec(commandString, ModelScope())               # noqa: S102 - this IS the feature
        except Exception:                    #whatever the user typed; it must not kill the dialog
            print("Execution of command failed; error:")
            for line in traceback.format_exc().split('\n'):
                line = line.replace('  File "<string>", ', '')
                if ('File "' not in line) and ('exec(commandString' not in line):
                    print(line)

        if event is not None:
            return "break"                   #prevent from passing Return key to text ...
        return None

    def OnClose(event):
        tkWindow.destroy()

    textArea.grid(row=1, column=0, pady=15, padx=10, sticky=tk.NSEW)
    textArea.bind('<Control-Return>', OnRunCode)
    textArea.bind('<Escape>', OnClose)
    tkWindow.bind('<Escape>', OnClose)

    frame = tk.Frame(tkWindow)
    runButton = tk.Button(frame, text="    Run code    ", command=lambda: OnRunCode(None))
    closeButton = tk.Button(frame, text="    Close    ", command=lambda: OnClose(None))

    frame.grid(row=2, column=0, padx=15, pady=(0, 15), sticky='', columnspan=3)
    runButton.grid(row=0, column=0, padx=80, sticky='')
    closeButton.grid(row=0, column=1, padx=80, sticky='')

    textExample = scrolledtext.ScrolledText(tkWindow, wrap=tk.WORD, width=60,
                                            height=pythonCommandExamples.count('\n')+1,
                                            background='gray94')
    textExample.grid(row=3, column=0, padx=15, pady=(0, 15), sticky=tk.NSEW)
    textExample.insert(tk.END, pythonCommandExamples)
    textExample.configure(state='disabled')  #unable to edit

    textArea.focus_set()                     #placing cursor in text area
    tkWindow.focus_force()

    if tkRuns:
        root.wait_window(tkWindow)
    else:
        tk.mainloop()


def ShowVisualizationSettingsDialog():
    """The settings tree of the renderer; opened with V in the render window.

    Returns:
        None
    """
    guiSC = GetRendererSystemContainer()
    if guiSC is None:
        exudyn.Print('ERROR: ShowVisualizationSettingsDialog: problems with the'
                     ' SystemContainer, probably not attached to the renderer yet')
        return

    EditDictionaryWithTypeInfo(guiSC.visualizationSettings, exudyn, 'Visualization Settings')


def ShowRightMouseSelectionDialog():
    """The properties of the item the right mouse button selected, read-only; the renderer has
    put them into exudyn.sys['currentRendererSelectionDict'] before calling this.

    Returns:
        None
    """
    try:
        d = exudyn.sys['currentRendererSelectionDict']
        EditDictionary(d, False, dialogName='properties of <' + d['name'] + '>')
    except Exception:                        #a dict without 'name', or no dict at all
        exudyn.Print('ERROR: ShowRightMouseSelectionDialog: showing the dictionary failed')


def AskQuitDialog():
    """Ask whether a long running simulation really shall be stopped; the answer goes back to the
    renderer in exudyn.sys['quitResponse'], as 2 (do not quit) or 3 (quit).

    Returns:
        None
    """
    response = False                         #if the user just shuts the window

    [root, tkWindow, tkRuns] = GetTkRootAndNewWindow()
    #topmost unconditionally: this question is the reason the renderer is waiting
    ApplyDialogWindowSettings(tkWindow, alwaysTopmost=True, alphaTransparency=1)
    tkWindow.bind("<Escape>", lambda event: tkWindow.destroy())
    tkWindow.title("WARNING - long running simulation!")

    def QuitResponse(clickResponse):
        nonlocal response
        response = clickResponse
        tkWindow.destroy()

    label = tk.Label(tkWindow, text="Do you really want to stop simulation and close renderer?",
                     justify=tk.LEFT)
    yesButton = tk.Button(tkWindow, text="        Yes        ",
                          command=lambda: QuitResponse(True))
    noButton = tk.Button(tkWindow, text="        No        ",
                         command=lambda: QuitResponse(False))

    label.grid(row=0, column=0, pady=(20, 0), padx=50, columnspan=5)
    yesButton.grid(row=1, column=1, pady=20)
    noButton.grid(row=1, column=3, pady=20)

    tkWindow.focus_force()

    if tkRuns:
        root.wait_window(tkWindow)
    else:
        tk.mainloop()

    exudyn.sys['quitResponse'] = response + 2   #2=do not quit, 3=quit
