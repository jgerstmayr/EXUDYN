#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Reads and writes the override settings: one file, `~/.exudyn/config.json`, holding
#           settings that persist between runs - for `exudyn.config`, for `visualizationSettings`,
#           for the dialogs and for the results monitor (#2666, #2679).
#
#           WHERE THE VALUES LIVE: in `exudyn.special.overrideSettings`, a dictionary that
#           `import exudyn` fills once from the file and that both Python and the C++ core read.
#           This module is the only thing that reads or writes the file; it is internal, and a
#           user reaches the values through `exudyn.special.overrideSettings`.
#
#           WHY ONE FILE: the results monitor and the dialogs both want to remember something.
#           One file per feature is how a directory becomes unreadable, so everything that
#           persists goes in here, in a section of its own.
#
#           WHAT MAY BE OVERRIDDEN: plain values only - a number, a flag, a string, or a list of
#           numbers. A setting that holds graphics data, a user function or a container is refused
#           with a message rather than guessed at.
#
#           REPRODUCIBILITY: a stored setting makes a run behave differently than it reads, which
#           is why `import exudyn` prints ONE note naming every setting that came from the file,
#           and why `EXUDYN_NO_USER_SETTINGS=1` ignores the file completely. The test runners set
#           that variable, so a stored setting can never move a test result.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-26 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import json
import os

__all__ = ['fileFormatVersion', 'sectionNames', 'plainTypes', 'structureDefaults',
           'FileName', 'Ignoring', 'Settings', 'Load', 'Save',
           'Clear', 'Reload', 'StoreSection', 'Applied', 'Ignored', 'Print', 'ApplyConfig',
           'ApplyVisualizationSettings', 'DialogKey', 'DialogGeometry', 'StoreDialogGeometry',
           'PositionIsReachable', 'Store']

#THE VERSION OF THE FILE FORMAT, written into every file and required to match exactly
#(revision2026b step RG12.13.2, #2690). It is a plain integer and it is bumped BY HAND, and only
#when both things are true: Exudyn has been released since the last bump, and the meaning of
#something in this file has changed - so it will not move for a long time, at the earliest in the
#release after the next one. It does not follow the Exudyn version, which moves on every resolved
#issue and would kill a stored file every few days. What it is for is the long term: knowing that a
#file somebody has kept for years is not one this Exudyn can read.
#
#A file whose version does not match is IGNORED, with one note naming both versions. Nothing is
#guessed at and nothing is repaired: the settings in it are conveniences, and a wrong guess about an
#old one is a run that behaves differently for a reason nobody can see.
fileFormatVersion = 1

#the sections of the file. 'dialogs' is read by the dialogs themselves (revision2026b step
#RG6.2.26) and is listed here so that this module does not warn about it
sectionNames = ['config', 'visualizationSettings', 'dialogs', 'resultsMonitor',
                'plotSensor']

#what may be stored: a plain value, or a list of plain values. Everything else - graphics data, a
#user function, a matrix container - is a thing that a JSON file cannot carry honestly
plainTypes = (bool, int, float, str)

_applied = []                   #[(path, value)] of what was applied in this process
_ignored = []                   #[(path, reason)] of what was not

#THE DEFAULTS OF A SETTINGS STRUCTURE, taken before its constructor was wrapped (revision2026b step
#RG12.17, #2691). Since a stored visualizationSetting is applied by the CONSTRUCTOR of the class
#itself, constructing one no longer gives the defaults - so they are taken once, at import, while it
#still does, exactly as the defaults of exudyn.config are (#2685). {className: dictionaryWithTypeInfo}
#and empty when nothing is stored, which is the normal case.
structureDefaults = {}


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def FileName():
    """Path of the override settings file, `~/.exudyn/config.json`.

    Returns:
        the absolute file name; neither the file nor the directory has to exist

    Note:
        `EXUDYN_CONFIG_FILE` names a different file, which is what a test or a second
        configuration uses; `EXUDYN_NO_USER_SETTINGS=1` ignores the file altogether.
    """
    given = os.environ.get('EXUDYN_CONFIG_FILE', '')
    if given.strip() != '':
        return os.path.abspath(given.strip())
    return os.path.join(os.path.expanduser('~'), '.exudyn', 'config.json')


def Ignoring():
    """True if the override settings file is switched off for this process (`EXUDYN_NO_USER_SETTINGS`)

    Returns:
        bool
    """
    return os.environ.get('EXUDYN_NO_USER_SETTINGS', '') not in ['', '0', 'False', 'false']


def Settings():
    """The override settings of this process: `exudyn.special.overrideSettings`.

    Returns:
        the dictionary `import exudyn` filled from the file - one key per section, see
        `sectionNames`; `{}` when nothing is stored or when the file is switched off. It is the
        dictionary itself and not a copy, so a change to it is seen by everything that reads it,
        the C++ side included; it is NOT written to the file, which only `Save` and `Store` do.
    """
    import exudyn
    return exudyn.special.overrideSettings


def Load():
    """Read the override settings file.

    Returns:
        the stored dictionary, without its `version` entry; `{}` when the file does not exist,
        cannot be read, is switched off, or is **of another format version** - see
        `fileFormatVersion`. A file that cannot be read prints the reason once and is not an error:
        a broken settings file must never stop `import exudyn`.
    """
    if Ignoring():
        return {}
    fileName = FileName()
    if not os.path.exists(fileName):
        return {}
    try:
        with open(fileName, 'r', encoding='utf-8') as file:
            settings = json.load(file)
    except Exception as e:
        print('WARNING: could not read ' + fileName + ': ' + str(e))
        return {}
    if not isinstance(settings, dict):
        print('WARNING: ' + fileName + ' does not hold a dictionary; it is ignored')
        return {}

    version = settings.pop('version', None)      #never a section, and never in the store
    if version != fileFormatVersion:
        print('NOTE: ' + fileName + ' is of format version ' + str(version) + ' and this Exudyn'
              + ' reads version ' + str(fileFormatVersion) + '; it is IGNORED. Store your settings'
              + ' again to write a current file, or delete it.')
        return {}

    for name in settings:
        if name not in sectionNames:
            print('WARNING: ' + fileName + ': unknown section "' + name + '"; known sections are '
                  + ', '.join(sectionNames))
    return settings


def Save(settings):
    """Write the override settings file, creating `~/.exudyn` if it does not exist.

    Args:
        settings: the dictionary to store; its keys should be section names, see `sectionNames`

    Returns:
        the file name that was written

    Note:
        `version` is written with it and is always `fileFormatVersion`, whatever the file said
        before: what is written is what this Exudyn means.
    """
    fileName = FileName()
    directory = os.path.dirname(fileName)
    if directory != '' and not os.path.exists(directory):
        os.makedirs(directory, exist_ok=True)
    stored = dict(settings)
    stored['version'] = fileFormatVersion        #always the current one, never what was read
    with open(fileName, 'w', encoding='utf-8') as file:
        json.dump(stored, file, indent=2, sort_keys=True)
        file.write('\n')
    return fileName


def Clear():
    """Delete the override settings file, so that the next run starts from the defaults.

    Returns:
        True if a file was deleted
    """
    fileName = FileName()
    if os.path.exists(fileName):
        os.remove(fileName)
        return True
    return False


def Reload():
    """Read the file again, into `exudyn.special.overrideSettings`, and apply what can be applied.

    Returns:
        the store, `exudyn.special.overrideSettings`, as it is afterwards

    Note:
        `import exudyn` reads the file once, and importing an already imported module does nothing -
        so a file edited by hand, or written by another process, has no effect until this is called
        or the interpreter is restarted. In a console that keeps its kernel, such as Spyder, that is
        what makes a stored setting look as if it had not been stored.

        WHAT IT DOES: the store is emptied and filled from the file, the `config` section is applied,
        and `visualizationSettings` reach every structure created from now on - including the case
        where the file had none at import, which is when this installs what applies them.

        WHAT IT DOES NOT DO: undo. A setting that already reached `exudyn.config` stays at the value
        it was given, and a structure that exists keeps what it was given, so a setting REMOVED from
        the file is seen by the structures created after the reload and not by `exudyn.config` or by
        anything that exists. For that, start a new session.

        Anything a script put into `exudyn.special.overrideSettings` by hand is dropped, because this
        is a re-read of the file.

    Example:
        #after editing ~/.exudyn/config.json by hand:
        from exudyn.misc import overrideSettings
        overrideSettings.Reload()
        overrideSettings.Print()        #what came from it now
    """
    import exudyn

    del _applied[:]                #Print() describes the file as it is now, not as it was
    del _ignored[:]
    exudyn._ApplyUserSettings(reload=True)
    return Settings()


def StoreSection(name, values):
    """Write one section of the file, keeping the others, and keep the store in step.

    Args:
        name: a section of `sectionNames` - 'config', 'visualizationSettings', 'dialogs' or
            'resultsMonitor'
        values: the dictionary to store under it; it REPLACES what was there

    Returns:
        the file name that was written

    Note:
        This is the only place that writes a section, so the file and
        `exudyn.special.overrideSettings` cannot disagree within a process: everything that stores
        something - the dialogs, the results monitor, `Store` - goes through here.
    """
    if name not in sectionNames:
        raise ValueError('unknown section "' + str(name) + '"; known sections are '
                         + ', '.join(sectionNames))
    settings = Load()
    settings[name] = values
    fileName = Save(settings)
    Settings()[name] = values
    return fileName


def Applied():
    """What the override settings changed in this process.

    Returns:
        list of `(path, value)`, e.g. `[('config.outputDirectory', 'solution/')]`; empty when
        nothing was stored or when the file is switched off. This is what makes a run that behaves
        oddly explainable: `exudyn.misc.overrideSettings.Applied()` says what is not in the script.
    """
    return list(_applied)


def Ignored():
    """What the override settings asked for and did not get.

    Returns:
        list of `(path, reason)` - a setting that does not exist, or one whose type cannot be
        stored in a JSON file
    """
    return list(_ignored)


def Print():
    """Print what came from the override settings file, and what did not.

    Returns:
        None
    """
    if Ignoring():
        print('override settings: switched off by EXUDYN_NO_USER_SETTINGS')
        return
    print('override settings file: ' + FileName()
          + ('' if os.path.exists(FileName()) else '  (does not exist)'))
    for (path, value) in _applied:
        print('  applied: ' + path + ' = ' + repr(value))
    for (path, reason) in _ignored:
        print('  ignored: ' + path + ' - ' + reason)


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def _IsPlain(value):
    """a value that a JSON file can carry and a setting can hold: a number, a flag, a string, or a
    list of those"""
    if isinstance(value, plainTypes):
        return True
    if isinstance(value, (list, tuple)):
        return all(isinstance(entry, plainTypes) for entry in value)
    return False


def _Record(path, value, reason=None):
    #ONCE PER SETTING, not once per structure: the visualizationSettings are applied again for every
    #structure that is created (revision2026b step RG12.10), and Print() listing the same setting
    #five times because five structures exist says nothing about what was stored
    if reason is None:
        if (path, value) not in _applied:
            _applied.append((path, value))
    elif (path, reason) not in _ignored:
        _ignored.append((path, reason))


def ApplyConfig(config, settings=None):
    """Apply the `config` section of the override settings to `exudyn.config`.

    Args:
        config: the `exudyn.config` object
        settings: the section dictionary; `exudyn.special.overrideSettings` when None is given

    Returns:
        the number of settings applied

    Note:
        This is called once by `import exudyn`. A name that `exudyn.config` does not have, or a
        value of a type it cannot hold, is reported by `Ignored()` and changes nothing.
    """
    settings = Settings() if settings is None else settings
    applied = 0
    for (name, value) in (settings.get('config') or {}).items():
        path = 'config.' + name
        if not hasattr(config, name) or name[0].isupper():
            _Record(path, value, 'exudyn.config has no such setting')
            continue
        current = getattr(config, name)
        if not _IsPlain(current):
            _Record(path, value, 'the setting holds a ' + type(current).__name__
                    + ', which a settings file cannot carry')
            continue
        if not _IsPlain(value):
            _Record(path, value, 'the stored value is a ' + type(value).__name__)
            continue
        try:
            setattr(config, name, type(current)(value) if isinstance(current, plainTypes) else value)
        except Exception as e:
            _Record(path, value, str(e))
            continue
        _Record(path, value)
        applied += 1
    return applied


def ApplyVisualizationSettings(visualizationSettings, settings=None):
    """Apply the `visualizationSettings` section of the override settings to a settings structure.

    Args:
        visualizationSettings: `SC.visualizationSettings` of a SystemContainer
        settings: the section dictionary; `exudyn.special.overrideSettings` when None is given

    Returns:
        the number of settings applied

    Note:
        The keys are the paths the dialogs and `ChangedSettings` use - `openGL.multiSampling`,
        `nodes.defaultSize` - so what `Store(SC)` writes is what a user reads in the dialog.

    Example:
        #in ~/.exudyn/config.json:
        #{"visualizationSettings": {"openGL.multiSampling": 4, "general.drawWorldBasis": true}}
        SC = exu.SystemContainer()      #the two settings are applied here
    """
    settings = Settings() if settings is None else settings
    applied = 0
    for (path, value) in (settings.get('visualizationSettings') or {}).items():
        full = 'visualizationSettings.' + path
        structure = visualizationSettings
        parts = path.split('.')
        try:
            for part in parts[:-1]:
                structure = getattr(structure, part)
            current = getattr(structure, parts[-1])
        except AttributeError:
            _Record(full, value, 'no such setting')
            continue
        if not _IsPlain(current):
            _Record(full, value, 'the setting holds a ' + type(current).__name__
                    + ', which a settings file cannot carry')
            continue
        if not _IsPlain(value):
            _Record(full, value, 'the stored value is a ' + type(value).__name__)
            continue
        try:
            setattr(structure, parts[-1],
                    type(current)(value) if isinstance(current, plainTypes) else value)
        except Exception as e:
            _Record(full, value, str(e))
            continue
        _Record(full, value)
        applied += 1
    return applied


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the dialogs section: one entry per dialog, holding the size and the position it was left at
#(revision2026b step RG12.5.3, from RG6.2.11 / #2608)
def DialogKey(name):
    """the key a dialog is stored under: its title, without spaces and case

    Args:
        name: the title of the dialog, e.g. 'Visualization Settings'

    Returns:
        the key, e.g. 'visualizationsettings'
    """
    return ''.join(character for character in str(name).lower() if character.isalnum())


def DialogGeometry(name):
    """The stored size and position of one dialog.

    Args:
        name: the title of the dialog, see `DialogKey`

    Returns:
        `(size, position)`, each `[x, y]` or None when nothing is stored for it
    """
    def IsPair(pair):
        """two integers, and nothing else: a hand-edited file must not place a window"""
        return (isinstance(pair, list) and len(pair) == 2
                and all(isinstance(entry, int) for entry in pair))

    stored = (Settings().get('dialogs') or {}).get(DialogKey(name)) or {}
    size = stored.get('size')
    position = stored.get('position')
    return (size if IsPair(size) else None, position if IsPair(position) else None)


def StoreDialogGeometry(name, size, position):
    """Store the size and the position of one dialog.

    Args:
        name: the title of the dialog, see `DialogKey`
        size: `[width, height]` in pixels
        position: `[x, y]` of the top left corner, in pixels

    Returns:
        None

    Note:
        This is called by the dialogs themselves when
        `visualizationSettings.dialogs.storeDialogPositions` is True. Nothing else writes the
        file: see `Store`.
    """
    dialogs = dict(Load().get('dialogs') or {})
    dialogs[DialogKey(name)] = {'size': [int(size[0]), int(size[1])],
                                'position': [int(position[0]), int(position[1])]}
    StoreSection('dialogs', dialogs)


def PositionIsReachable(position, screen, margin=80):
    """Would a window at this position still be reachable on this screen?

    Args:
        position: `[x, y]` of the top left corner of the window
        screen: `[x, y, width, height]` of the screen, or of the virtual desktop when there is
            more than one
        margin: how much of the window has to remain on the screen, in pixels; the default is
            about the width of a title bar button group

    Returns:
        True if a window placed there can be reached with the mouse

    Note:
        This is the rule RG6.2.11 wrote down and did not build: **the size is restored always, the
        position only when it is still reachable**. A monitor that is unplugged, a laptop
        undocked, a resolution changed - each of them would otherwise put a dialog where nobody
        can close it, and a modal settings dialog that cannot be closed is a stuck session.

    Example:
        PositionIsReachable([100, 80], [0, 0, 1920, 1080])       #True
        PositionIsReachable([2200, 80], [0, 0, 1920, 1080])      #False: the second screen is gone
    """
    (x, y) = (position[0], position[1])
    (screenX, screenY, width, height) = (screen[0], screen[1], screen[2], screen[3])
    #the top left corner must lie on the screen, and far enough from the right and lower edge
    #that the title bar can still be grabbed. A little negative is normal on Windows, where a
    #maximised window sits at -8
    return (screenX - 16 <= x <= screenX + width - margin
            and screenY - 4 <= y <= screenY + height - margin)


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def Store(SC=None, config=None, replace=False):
    """Store the settings that differ from the defaults, so that the next run starts with them.

    Args:
        SC: a SystemContainer; every one of its `visualizationSettings` that differs from the
            defaults is stored, which is what `ChangedSettings` reports
        config: `exudyn.config`; every one of its plain settings that differs from
            `config.GetDefaults()` is stored - the defaults Exudyn started with, so a setting a
            user never touched is not stored
        replace: True replaces the sections that are written; False (the default) merges them into
            what is already in the file

    Returns:
        the dictionary that was written

    Note:
        Storing is deliberately explicit. Exudyn never writes this file by itself: a script that
        behaves differently on another machine, because something was stored there, is the failure
        mode this whole file has to be worth.

    Example:
        SC.visualizationSettings.openGL.multiSampling = 4
        overrideSettings.Store(SC)       #every run from now on starts with it
    """
    settings = {} if replace else Load()

    if SC is not None:
        from exudyn.misc.settingsUtilities import ChangedSettings
        stored = {} if replace else dict(settings.get('visualizationSettings') or {})
        for (path, line) in ChangedSettings(SC.visualizationSettings):
            structure = SC.visualizationSettings
            parts = path.split('.')
            for part in parts[:-1]:
                structure = getattr(structure, part)
            value = getattr(structure, parts[-1])
            if _IsPlain(value):
                stored[path] = list(value) if isinstance(value, (list, tuple)) else value
        settings['visualizationSettings'] = stored

    if config is not None:
        stored = {} if replace else dict(settings.get('config') or {})
        #WHAT DIFFERS FROM THE DEFAULTS, and nothing else (revision2026b step RG12.11, #2685). This
        #used to store every value that was not '', 0 or False, because exudyn.config had no
        #defaults to compare with - a guess that stored settings a user never touched.
        #config.GetDefaults() is what Exudyn started with, taken while it still was
        defaults = config.GetDefaults()
        for (name, value) in config.GetDictionary().items():
            if _IsPlain(value) and value != defaults.get(name, value):
                stored[name] = list(value) if isinstance(value, (list, tuple)) else value
        settings['config'] = stored

    Save(settings)
    Settings().clear()
    Settings().update(settings)  #the store follows the file, so the two cannot disagree
    return settings
