#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN example
#
# Details:  Main Python library file for import of C++ module
#
# Author:   Johannes Gerstmayr
# Date:     2020-08-14
# Update:   2022-12-26
#
# Notes:    see https://github.com/jgerstmayr/EXUDYN for first steps
#           see the documentation for instructions, tutorials, etc.: https://exudyn.readthedocs.io/
# Example (without visualization):
#    import exudyn as exu
#    from exudyn.itemInterface import * #conversion of data to exudyn dictionaries
#    SC = exu.SystemContainer()
#    mbs = SC.AddSystem()
#    #add a new system to work with
#    nMP = mbs.AddNode(NodePoint2D(referenceCoordinates=[0,0]))
#    mbs.AddObject(ObjectMassPoint2D(physicsMass=10, nodeNumber=nMP ))
#    mMP = mbs.AddMarker(MarkerNodePosition(nodeNumber = nMP))
#    mbs.AddLoad(Force(markerNumber = mMP, loadVector=[0.001,0,0]))
#    mbs.Assemble() #assemble system and solve
#    mbs.SolveDynamic(exu.SimulationSettings())
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#  Use the following workaround to define the 'fast' track, avoiding range checks in exudyn (speedup may be 30% and more);
#  to activate the __FAST_EXUDYN_LINALG compiled version, use the following lines (must be done befor first import of exudyn):
#import sys
#sys.exudynFast = True
#import exudyn #now exudyn loads with fast mode

import sys
__useExudynFast = hasattr(sys, 'exudynFast')
if __useExudynFast:
    __useExudynFast = sys.exudynFast #could also be False!
else:
    #EXUDYN_MODULE=fast selects the same module as sys.exudynFast, but through the environment,
    #which CHILD PROCESSES INHERIT - that is the point: the test suite runs every model in its own
    #interpreter (runTestSuite.py --parallel, pytest -n), and a sys attribute does not survive
    #that while a variable does. Release testing uses it to cover exudynCPPfast, which otherwise
    #ships untested. An explicit sys.exudynFast always wins, including
    #sys.exudynFast=False; the variable only decides when nothing was said in code.
    #Read here and not in _ApplyEnvironmentSettings() below: THAT runs after the C++ module has
    #been imported, which is too late to choose which one.
    try:
        import os
        __useExudynFast = (os.environ.get('EXUDYN_MODULE', '').strip().lower() == 'fast')
        if __useExudynFast:
            print('NOTE: EXUDYN_MODULE=fast is set; loading exudynCPPfast (no range checks)')
    except Exception:
        __useExudynFast = False #a failed environment read must never stop the import

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#(#2466) there are exactly TWO modules, with the same meaning on
#every platform: exudynCPP is built for the BASELINE instruction set and runs on any 64-bit CPU,
#and exudynCPPfast has no range checks and carries the vector extensions (AVX2). The third module
#exudynCPPnoAVX is gone, together with the sys.exudynCPUhasAVX2 switch that selected it: the
#default module IS the safe one now, so nothing has to be detected in order to import exudyn.
#
#The AVX2 check below therefore decides ONE thing only - whether a user's sys.exudynFast request
#can be honoured. A wrong answer costs speed, never a crash, which is why it may be this simple.
#It replaces a read of numpy.core._multiarray_umath.__cpu_features__, which no longer exists in
#numpy >= 2.0 (where the old code simply ASSUMED AVX2) and was skipped altogether on Linux.
def _CpuHasAVX2():
    """True if this CPU *and* the operating system support AVX2; False whenever that cannot be
    established, so that the safe module is used."""
    try:
        if sys.platform == 'win32':
            import ctypes
            #IsProcessorFeaturePresent also covers the OS XSAVE/YMM state, which CPUID alone
            #does not: a CPU may have AVX2 while the OS does not preserve the YMM registers
            PF_AVX2_INSTRUCTIONS_AVAILABLE = 40
            return bool(ctypes.windll.kernel32.IsProcessorFeaturePresent(
                PF_AVX2_INSTRUCTIONS_AVAILABLE))
        if sys.platform.startswith('linux'):
            with open('/proc/cpuinfo') as cpuInfoFile:
                for line in cpuInfoFile:
                    if line.startswith('flags'):
                        return 'avx2' in line.split()
            return False
        if sys.platform == 'darwin': #Apple silicon has no AVX at all; Intel Macs may
            import subprocess
            return subprocess.run(['sysctl', '-n', 'hw.optional.avx2_0'],
                                  capture_output=True, text=True).stdout.strip() == '1'
    except Exception:
        pass
    return False #unknown platform or a failed check: use the module that always works

#ONE FUNCTION DECIDES AND IMPORTS (#2540). It used to be a nest of four
#try/except blocks whose failure message named neither what was tried nor why, and whose decisions
#were printed unconditionally or not at all. Two properties matter here and are the reason this is
#a function and not a script:
#  - it is TESTABLE: it returns the log of what it tried, which python/testing/test_import.py reads;
#  - a total failure raises ONE ImportError that lists every candidate and the reason each was
#    skipped or failed, instead of the last error or a sentence about 32/64 bits.
#This is a hard prerequisite for phase R9: a plugin is bound to the module it was built against, so
#which one was loaded, and why, has to be answerable.
def _ImportCompiledModule(useExudynFast):
    """Import the compiled module: exudynCPPfast if it was asked for AND this CPU reports AVX2,
    otherwise exudynCPP. Each candidate is tried as a package module first and then as a top-level
    module, which is the Visual Studio layout (exudynCPP lies in Release or Debug).

    Returns (moduleName, module, attempts), where attempts is a list of (what, outcome) in the
    order they happened. Raises ImportError naming every attempt if none of them worked."""
    import importlib

    attempts = []
    candidates = []
    if useExudynFast:
        if _CpuHasAVX2():
            candidates.append('exudynCPPfast')
        else:
            attempts.append(('exudynCPPfast', 'skipped: this CPU does not report AVX2'))
    candidates.append('exudynCPP')

    for name in candidates:
        for relative in (True, False):
            try:
                module = importlib.import_module('.' + name if relative else name,
                                                 __name__ if relative else None)
                attempts.append((('.' if relative else '') + name, 'imported'))

                return (name, module, attempts)
            except ImportError as importError:
                attempts.append((('.' if relative else '') + name, str(importError)))

    raise ImportError('Exudyn could not import its compiled module. Tried, in order:\n  '
                      + '\n  '.join(what + '  ->  ' + outcome for (what, outcome) in attempts)
                      + '\nCheck that the wheel matches this Python version and platform, restart '
                        'the console (a partially imported module stays cached), or reinstall.')


(_compiledModuleName, _compiledModule, _importAttempts) = _ImportCompiledModule(__useExudynFast)

#the star import, done by hand because the module name is a variable; __all__ if the module has
#one, otherwise every public name - which is exactly what 'from X import *' would take
_exportedNames = getattr(_compiledModule, '__all__', None)
if _exportedNames is None:
    _exportedNames = [__name for __name in vars(_compiledModule) if not __name.startswith('_')]
globals().update({__name: getattr(_compiledModule, __name) for __name in _exportedNames})

#the two the rest of this file uses, written out so that a reader and a static checker can see
#where they come from. The star import above used to make them invisible to both (#2540).
config = _compiledModule.config       #exudyn.config: the run-time settings object
special = _compiledModule.special     #exudyn.special: the rarely needed corners

if _compiledModuleName == 'exudynCPPfast':
    #not behind the switch below: running without range checks is worth saying every time
    print('Imported exudyn fast version without range checks')
elif __useExudynFast:
    #the user ASKED for the fast module and did not get it; say so, and say why. Falling back
    #in silence is how someone spends an afternoon wondering where the speed went
    print('NOTE: exudyn fast version was requested but not loaded; using the regular version.'
          ' Reason: ' + _importAttempts[0][1]
          + '  (set EXUDYN_IMPORT_VERBOSE=1 for everything that was tried)')
__useExudynFast = (_compiledModuleName == 'exudynCPPfast')

#EXUDYN_IMPORT_VERBOSE=1 prints what was tried and what came of it. For a user who reports "it
#imports the wrong one" or "it does not import at all", this is the whole answer in four lines.
try:
    import os as __os
    if __os.environ.get('EXUDYN_IMPORT_VERBOSE', '').strip().lower() in ('1', 'true', 'yes'):
        print('NOTE: exudyn module selection:')
        for (__what, __outcome) in _importAttempts:
            print('  ' + __what + '  ->  ' + __outcome)
except Exception:
    pass #a failed environment read must never stop the import

#import very useful solver functionality into exudyn module (==> available as exu.SolveStatic, etc.)
try:
    from .solver import SolveStatic, SolveDynamic, SolverSuccess, ComputeLinearizedSystem, ComputeSystemDegreeOfFreedom, ComputeODE2Eigenvalues
except ImportError:
    #for run inside Visual Studio (exudynCPP lies in Release or Debug folders):
    from solver import SolveStatic, SolveDynamic, SolverSuccess, ComputeLinearizedSystem, ComputeSystemDegreeOfFreedom, ComputeODE2Eigenvalues #noqa: F401 - re-export, available as exu.SolveDynamic etc.

#use exudyn.demos.Demo1() from 1.9.137 onwards!
try:
    from . import demos #noqa: F401 - re-export, available as exudyn.demos
except ImportError:
    #for run inside Visual Studio (exudynCPP lies in Release or Debug folders):
    pass
    #import exudyn.demos as demos

try:
    from .misc.mainSystemExtensions import JointPreCheckCalcBodyMarkers #import just some function, will assign MainSystem patches
except ImportError:
    #for run inside Visual Studio (exudynCPP lies in Release or Debug folders):
    from misc.mainSystemExtensions import JointPreCheckCalcBodyMarkers #noqa: F401 - importing the module assigns the MainSystem patches


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#two environment variables, read ONCE at import (#2477). They exist for automated runs - test
#runners, CI, AI-assisted development - where a window that waits for a human stops everything and
#output written next to the model clutters the working tree. NOT recommended for users: a setting
#that lives outside the script makes a run behave differently than it reads, which is why both are
#announced on import. A failure here never stops the import.
def _ApplyEnvironmentSettings():
    import os

    outputDirectory = os.environ.get('EXUDYN_OUTPUTDIRECTORY', '')
    if outputDirectory != '':
        config.outputDirectory = outputDirectory
        print('NOTE: EXUDYN_OUTPUTDIRECTORY is set; exudyn.config.outputDirectory="'
              + outputDirectory + '"')

    if os.environ.get('EXUDYN_SUPPRESS_UI_WINDOW_OPEN', '') not in ['', '0', 'False', 'false']:
        special.userInterface.SuppressAll(True)
        print('NOTE: EXUDYN_SUPPRESS_UI_WINDOW_OPEN is set; Exudyn opens no renderer, solution '
              'viewer, plot or dialog window')
        try: #a script may call plt.show() itself, which no flag inside Exudyn can intercept; the
             #non-interactive backend is the only thing that reaches those
            import matplotlib
            matplotlib.use('Agg')
        except Exception: #matplotlib is optional, and a backend may be fixed already
            pass

try:
    _ApplyEnvironmentSettings()
except Exception as e: #an environment that cannot be read must never stop 'import exudyn'
    print('WARNING: exudyn could not apply its environment settings: ' + str(e))


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the override settings of ~/.exudyn/config.json, read ONCE here and kept in
#exudyn.special.overrideSettings, which is where both Python and the C++ core read them
#(revision2026b steps RG12.5 and RG12.9, #2666 and #2679). A stored setting makes a run behave
#differently than it reads, so every one of them is named in one note here,
#EXUDYN_NO_USER_SETTINGS=1 ignores the file, and exudyn.misc.overrideSettings.Applied() answers
#"what is not in my script" afterwards. Nothing writes the file by itself.
def _ApplyUserSettings():
    from .misc import overrideSettings as _settings

    stored = special.overrideSettings   #the one store; filled here and by nothing else
    stored.update(_settings.Load())
    if len(stored) == 0:
        return

    _settings.ApplyConfig(config, stored)

    #A STORED visualizationSetting IS APPLIED WHENEVER SUCH A STRUCTURE IS CREATED (revision2026b
    #step RG12.10, #2684), which is the two ways a user gets one: the structure a SystemContainer
    #builds in its constructor, and exu.VisualizationSettings() - which got nothing before, so a
    #script that edited one before creating a container saw the defaults.
    #
    #THE CONSTRUCTOR IS WRAPPED IN PLACE, NOT SUBCLASSED (revision2026b step RG12.17, #2691). A
    #Python subclass installed as exudyn.SystemContainer changes what that NAME is, and three things
    #broke on it in two days: DefaultSettingsDictionary constructed the subclass and reported the
    #overrides as the defaults, an isinstance() of mine in the settings dialog stopped recognising
    #SC.visualizationSettings, and - the one a user meets - GetRendererSystemContainer() does
    #"isinstance(guiSC, exudyn.SystemContainer)" on the object the C++ side stores as a POINTER,
    #which is of the COMPILED class, so it found nothing and the V key opened no dialog at all.
    #Wrapping __init__ on the class itself leaves every name and every isinstance as they were.
    if (stored.get('visualizationSettings') or {}) != {}:

        def _Apply(visualizationSettings):
            try:
                _settings.ApplyVisualizationSettings(visualizationSettings, stored)
            except Exception as error: #a stored setting must never stop a model from starting
                print('WARNING: exudyn could not apply the stored visualizationSettings: '
                      + str(error))

        def _ApplyingConstructor(theClass, GetSettings):
            """wrap theClass.__init__ so that it applies the stored settings afterwards"""
            originalInit = theClass.__init__

            def Initialize(self, *arguments, **keywordArguments):
                originalInit(self, *arguments, **keywordArguments)
                _Apply(GetSettings(self))

            theClass.__init__ = Initialize

        try:
            #WHILE CONSTRUCTING ONE STILL GIVES THE DEFAULTS: afterwards it gives the overrides,
            #and everything that shows a difference to the default needs them (the dialog's
            #marking, its "diff to default", ChangedSettings, Store(SC))
            _settings.structureDefaults['VisualizationSettings'] = \
                _compiledModule.VisualizationSettings().GetDictionaryWithTypeInfo()

            _ApplyingConstructor(_compiledModule.SystemContainer,
                                 lambda container: container.visualizationSettings)
            _ApplyingConstructor(_compiledModule.VisualizationSettings, lambda structure: structure)
        except (TypeError, AttributeError) as error:
            #a build whose classes refuse it: say so rather than fall back to a subclass, which is
            #what broke the dialogs
            print('WARNING: exudyn cannot apply the stored visualizationSettings on this build ('
                  + str(error) + '); they are in exudyn.special.overrideSettings and can be applied'
                  + ' with exudyn.misc.overrideSettings.ApplyVisualizationSettings(...)')

    #the visualizationSettings are applied when such a structure is created, so they are counted
    #here as what WILL happen rather than as what has happened
    applied = _settings.Applied()
    later = len(stored.get('visualizationSettings') or {})
    ignored = _settings.Ignored()
    if len(applied) != 0 or later != 0 or len(ignored) != 0:
        print('NOTE: ' + str(len(applied)) + ' setting(s) from ' + _settings.FileName()
              + ('' if later == 0 else ', and ' + str(later)
                 + ' visualizationSettings for every such structure that is created')
              + ' (exudyn.misc.overrideSettings.Print() for the list;'
              + ' EXUDYN_NO_USER_SETTINGS=1 to ignore them)')
        for (path, reason) in ignored:
            print('  WARNING: ' + path + ' was not applied - ' + reason)


try:
    _ApplyUserSettings()
except Exception as e: #a settings file that cannot be read must never stop 'import exudyn'
    print('WARNING: exudyn could not apply its override settings: ' + str(e))


__version__ = config.Version() #add __version__ to exudyn module ...


#add a functionality to check the current version
def RequireVersion(requiredVersionString):
    """
    Parameters
    ----------
    requiredVersionString : string
        Checks if the installed version is according to the required version.
        Major, micro and minor version must agree the required level.
    Returns
    -------
    None. But will raise RuntimeError, if required version is not met.

    Example
    ----------
    RequireVersion("1.10.0")

    """
    vExudyn = config.Version().split('.')
    vRequired = requiredVersionString.split('.')
    isOk = True
    if int(vExudyn[0]) < int(vRequired[0]):
        isOk = False
    elif int(vExudyn[0]) == int(vRequired[0]): #only for equal major versions
        if int(vExudyn[1]) < int(vRequired[1]): #check minor version
            isOk = False
        elif int(vExudyn[1]) == int(vRequired[1]): #only for equal minor versions
            if int(vExudyn[2]) < int(vRequired[2]): #check micro version
                isOk = False
    if not isOk:
        raise RuntimeError("EXUDYN version "+requiredVersionString+" required, but only " + config.Version() +
                           " available!\nYou can install the latest development version with:\npip install -U exudyn --pre\n\n")
    



