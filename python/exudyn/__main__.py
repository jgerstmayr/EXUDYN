#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  The command line of the installed Exudyn package: 'python -m exudyn <command>'.
#           It is the user-facing counterpart of tools/exudev (the maintainer driver, which
#           builds and tests the repository and never imports exudyn) - this one does nothing
#           but call functions of the installed package, so that the small things one wants
#           from a shell do not need a script file:
#
#               python -m exudyn info              what is installed, and where
#               python -m exudyn monitor --last     live view of the newest results file
#               python -m exudyn plot s.txt         static plot of a sensor or solution file
#               python -m exudyn demo               does this installation work?
#               python -m exudyn dialogs vis       browse the visualization settings
#
#           Every command is looked up in the dictionary CommandTable() and imports what it
#           needs when it is called, so an unused command costs nothing. A command's own
#           arguments are parsed by the command, which keeps 'python -m exudyn <cmd> --help'
#           working and makes the table extensible (revision2026 step R9.6, plugins).
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-19 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'dialogNames', 'CommandTable', 'Main',
    ]


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def _CommandMonitor(argumentList):
    """live view of a results file while it is written; see exudyn.misc.resultsMonitor"""
    from exudyn.misc.resultsMonitor import Main as MonitorMain
    return MonitorMain(argumentList)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def _CommandPlot(argumentList):
    """static plot of one or more sensor or solution files"""
    import argparse
    parser = argparse.ArgumentParser(
        prog='python -m exudyn plot',
        description='plot sensor or coordinates solution files; without --columns, every column '
                    'except time is plotted over time.',
        epilog='examples:\n'
               '  python -m exudyn plot solution/sensorPos.txt\n'
               '  python -m exudyn plot s0.txt s1.txt --columns 0,1 --save pos.png\n',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('file', nargs='+', help='sensor or solution file(s)')
    parser.add_argument('-c', '--columns', default='', metavar='I,J',
                        help='components to plot; 0 is the first column after time')
    parser.add_argument('--log-y', action='store_true', help='logarithmic y-axis')
    parser.add_argument('--title', default='', help='plot title')
    parser.add_argument('--save', default='', metavar='FILE',
                        help='write the figure to FILE (png, pdf, svg)')
    args = parser.parse_args(argumentList)

    import os
    from exudyn.misc.resultsMonitor import ResultsFileColumns
    import exudyn
    from exudyn.plot import PlotSensor

    sensorList = []
    componentList = []
    for fileName in args.file:
        if not os.path.exists(fileName):
            print('ERROR: file not found: ' + fileName)
            return 1
        if args.columns != '':
            components = [int(value) for value in args.columns.replace(',', ' ').split()]
        else:
            numberOfColumns = len(ResultsFileColumns(fileName))
            if numberOfColumns < 2:
                print('ERROR: ' + fileName + ' is not an Exudyn sensor or solution file')
                return 1
            components = list(range(numberOfColumns - 1))
        sensorList += [fileName] * len(components)
        componentList += components

    #PlotSensor takes file names in place of sensor numbers, but still wants a MainSystem
    mbs = exudyn.SystemContainer().AddSystem()
    PlotSensor(mbs, sensorNumbers=sensorList, components=componentList, title=args.title,
               fileName=args.save, logScaleY=args.log_y, closeAll=True)

    import matplotlib.pyplot as plt
    if args.save == '' and plt.get_backend().lower() != 'agg':
        plt.show(block=True)
    return 0


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def _CommandInfo(argumentList):
    """what is installed and where - the information a bug report needs"""
    import argparse
    parser = argparse.ArgumentParser(
        prog='python -m exudyn info',
        description='version, location and environment of this Exudyn installation.')
    parser.parse_args(argumentList)

    import os
    import platform
    import exudyn

    print('Exudyn')
    print('  version           ' + exudyn.__version__)
    print('  details           ' + exudyn.config.Version(True).replace('\n', '; '))
    print('  package           ' + os.path.dirname(os.path.abspath(exudyn.__file__)))
    print('  module            ' + os.environ.get('EXUDYN_MODULE', 'default (exudynCPP)'))
    print('  outputDirectory   ' + (exudyn.config.outputDirectory
                                    if exudyn.config.outputDirectory != '' else '(current)'))
    print('Python')
    print('  version           ' + platform.python_version() + ' (' + platform.architecture()[0] + ')')
    print('  executable        ' + sys.executable)
    print('  platform          ' + platform.platform())
    print('packages')
    for name in ['numpy', 'scipy', 'matplotlib', 'networkx', 'ngsolve', 'pytest']:
        try:
            module = __import__(name)
            print('  ' + name.ljust(18) + getattr(module, '__version__', '(no version)'))
        except ImportError:
            print('  ' + name.ljust(18) + '-')
    return 0


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def _CommandDemo(argumentList):
    """run a small built-in model, to check that the installation works"""
    import argparse
    parser = argparse.ArgumentParser(
        prog='python -m exudyn demo',
        description='run a built-in demo model; demo 1 needs no graphics, demo 2 opens the '
                    'renderer window.')
    parser.add_argument('number', nargs='?', type=int, default=1, choices=[1, 2],
                        help='which demo to run (default 1)')
    args = parser.parse_args(argumentList)

    import exudyn.demos
    if args.number == 1:
        exudyn.demos.Demo1()
    else:
        exudyn.demos.Demo2()
    return 0


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the dialogs a user may want without a render window (revision2026b step RG6.2.18, #2624). The
#settings dialog was reachable only by pressing V in the renderer, which is no help to somebody
#who wants to look up what a setting is called before writing the model
dialogNames = {
    'vis':  'visualizationSettings', 'visualizationSettings': 'visualizationSettings',
    'sim':  'simulationSettings',    'simulationSettings':    'simulationSettings',
    'help': 'help',
    }


def _CommandDialogs(argumentList):
    """browse the settings, or the key bindings, in the dialogs of the renderer"""
    import argparse
    parser = argparse.ArgumentParser(
        prog='python -m exudyn dialogs',
        description='open one of the renderer dialogs without a render window: browse the '
                    'settings, find one by name with CTRL-F, and copy the line that sets it.',
        epilog='examples:\n'
               '  python -m exudyn dialogs vis      the visualization settings\n'
               '  python -m exudyn dialogs sim      the simulation settings\n'
               '  python -m exudyn dialogs help     the keyboard and mouse commands\n',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('what', nargs='?', default='vis', choices=sorted(dialogNames),
                        metavar='WHAT',
                        help='vis | visualizationSettings, sim | simulationSettings, help '
                             '(default: vis)')
    args = parser.parse_args(argumentList)
    what = dialogNames[args.what]

    import exudyn as exu
    from exudyn.basicUtilities import UIWindowSuppressed                    # noqa: PLC0415
    if UIWindowSuppressed('Dialogs', 'python -m exudyn dialogs'):
        return 0

    try:
        import exudyn.misc.GUI as gui                                       # noqa: PLC0415
    except ImportError:
        print('this command needs tkinter, which is not installed with this Python')
        return 1

    if what == 'help':
        gui.ShowHelpDialog()
        return 0

    #a structure of its own, not a SystemContainer's: creating a container attaches it to the
    #render engine (#2625), and there is no model here anyway. Since revision2026b step RG6.2.20
    #this carries the same defaults a user really starts from
    settings = exu.VisualizationSettings() if what == 'visualizationSettings' \
        else exu.SimulationSettings()
    gui.EditDictionaryWithTypeInfo(settings, exu, what)

    #what the browsing was for: the code that reproduces it (revision2026b step RG12.3)
    from exudyn.misc.settingsUtilities import ChangedSettings, ChangedSettingsCode  # noqa: PLC0415
    changes = ChangedSettings(settings)
    if changes:
        print('')
        print(ChangedSettingsCode(settings))
    return 0


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def CommandTable():
    """The commands of `python -m exudyn`, as `{name: [function, one line description]}`. The
    function takes the remaining command line arguments and returns the process return code.

    Returns:
        a new dictionary, so that a caller may extend it without changing the table
    """
    return {
        'monitor': [_CommandMonitor, 'live view of a results file while it is written'],
        'plot':    [_CommandPlot,    'plot sensor or solution files and exit'],
        'info':    [_CommandInfo,    'version, location and environment of this installation'],
        'demo':    [_CommandDemo,    'run a built-in demo model'],
        'dialogs': [_CommandDialogs, 'browse the settings or the key bindings in a dialog'],
        }


def _Usage():
    """print the list of commands"""
    import exudyn
    print('Exudyn ' + exudyn.__version__ + ' - python -m exudyn <command> [options]')
    print('')
    print('commands:')
    for name, entry in CommandTable().items():
        print('  ' + name.ljust(10) + entry[1])
    print('')
    print('  python -m exudyn <command> --help    the options of one command')
    print('  python -m exudyn --version           the version string alone')


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
def Main(argumentList=None):
    """The entry point of `python -m exudyn`: pick the command and hand it the rest of the
    command line.

    Args:
        argumentList: the arguments without the program name; default `sys.argv[1:]`

    Returns:
        the process return code: 0 on success, 1 on a reported error, 2 on a wrong command line
    """
    if argumentList is None:
        argumentList = sys.argv[1:]
    argumentList = list(argumentList)

    if len(argumentList) == 0 or argumentList[0] in ['-h', '--help', 'help']:
        _Usage()
        return 0
    if argumentList[0] in ['-V', '--version']:
        import exudyn
        print(exudyn.__version__)
        return 0

    commands = CommandTable()
    command = argumentList[0]
    if command not in commands:
        print('ERROR: unknown command "' + command + '"')
        print('')
        _Usage()
        return 2
    return commands[command][0](argumentList[1:])


if __name__ == '__main__':
    sys.exit(Main())
