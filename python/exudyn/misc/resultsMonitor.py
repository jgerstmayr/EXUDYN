#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Live view of an Exudyn results file while it is being written: a sensor file, a
#           coordinates solution file, or the results file of ParameterVariation and
#           GeneticOptimization. The file is re-read incrementally and the curves are redrawn
#           every updatePeriod seconds, until the plot window is closed.
#
#           Three ways to use it, all doing the same thing:
#
#               python -m exudyn monitor --last              #command line, newest results file
#               python -m exudyn.misc.resultsMonitor f.txt   #command line, given file
#               from exudyn.misc.resultsMonitor import MonitorResults
#               MonitorResults('solution/genetic.txt', logY=True)   #from a script or Spyder
#
#           Calling it with no file name opens a file dialog (or, without tkinter, prints a
#           numbered list of the results files found next to the model).
#
#           This module replaces the 2021 script python/exudyn/resultsMonitor.py, which parsed
#           sys.argv and started plotting while it was being imported.
#
# Author:   Johannes Gerstmayr
# Date:     2021-01-14 (created as resultsMonitor.py)
# Date:     2026-09-19 (rewritten as a module with a library function and a CLI)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import glob
import os
import subprocess
import sys
import time

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

import exudyn
from exudyn.misc import overrideSettings
from exudyn.basicUtilities import UIWindowSuppressed
from exudyn.plot import ParseOutputFileHeader
from exudyn.advancedUtilities import PlotLineCode
from exudyn.processing import SingleIndex2SubIndices

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'knownResultsFileTypes', 'LoadSettings', 'SaveSettings',
    'ReadResultsFileHeader', 'ResultsFileColumns', 'FindResultsFiles', 'SelectResultsFile',
    'ResultsMonitor', 'MonitorResults', 'StartResultsMonitor', 'Main',
    ]

#the four header types that ParseOutputFileHeader recognizes; anything else is not a results file
knownResultsFileTypes = ['sensor', 'solution', 'geneticOptimization', 'parameterVariation']

#markers used to tell the variations apart once the 28 line codes of PlotLineCode are used up
_listMarkerStyles = ['.', '+', 'x', 'v', '^', '<', '>', '*', 'd', 'D', 's', 'X', 'P', 'o', 'p',
                     'h', 'H']

#defaults of everything that the settings file may override; the CLI overrides both
_defaultSettings = {
    'updatePeriod': 1.0,        #seconds between two updates
    'logX': False,
    'logY': False,
    'autoScale': True,          #rescale the axes to the data at every update
    'addMarker': False,         #red circle at the last point of every curve
    'sizeInches': [5.0, 5.0],   #size of ONE subplot
    'lineColor': 'b',
    'lineStyle': '-',
    'showPanel': True,          #the tkinter control panel next to the plot
    'alwaysOnTop': False,       #keep the plot window above other windows; it does NOT take the focus
    'lastDirectory': '',        #where the file dialog opens next time
    }


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#settings file
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def LoadSettings():
    """Read the stored monitor settings, filled up with the defaults.

    The settings are the `resultsMonitor` section of the override settings, `~/.exudyn/config.json`,
    which `import exudyn` has already read - one file for everything Exudyn remembers between runs
    (revision2026b step RG12.10, #2684).

    Note:
        Nothing here is an error: a section that is not there, or a key that is not known, leaves
        the default in place.

    Returns:
        a dictionary with the keys of `exudyn.misc.resultsMonitor._defaultSettings`
    """
    settings = dict(_defaultSettings)
    for (key, value) in (overrideSettings.Settings().get('resultsMonitor') or {}).items():
        if key in settings:
            settings[key] = value
    return settings


def SaveSettings(settings):
    """Store monitor settings in the `resultsMonitor` section of the override settings.

    Args:
        settings: a dictionary; only the known keys are written

    Returns:
        True if the file was written
    """
    try:
        overrideSettings.StoreSection(
            'resultsMonitor', {key: settings[key] for key in _defaultSettings if key in settings})
        return True
    except Exception as e:
        print('WARNING: could not write ' + overrideSettings.FileName() + ': ' + str(e))
        return False


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#reading results files
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def ReadResultsFileHeader(fileName, numberOfLines=12):
    """Read the header of a results file and identify its type.

    Args:
        fileName: sensor, solution, geneticOptimization or parameterVariation file
        numberOfLines: number of lines read from the file; the header is at most 10 lines

    Returns:
        the dictionary of `exudyn.plot.ParseOutputFileHeader`, with `type` one of
        `knownResultsFileTypes` or `'unknown'`; an empty dictionary if the file cannot be read
    """
    try:
        lines = []
        with open(fileName, 'r') as file:
            for _ in range(numberOfLines):
                line = file.readline()
                if line == '':
                    break
                lines.append(line)
        if len(lines) == 0:
            return {}
        return ParseOutputFileHeader(lines)
    except Exception:
        return {}


def ResultsFileColumns(fileName):
    """Names of the columns of a results file, in file order.

    Args:
        fileName: sensor, solution, geneticOptimization or parameterVariation file

    Returns:
        a list of strings; the index in the list is the column index used by `xColumns` and
        `yColumns` of `MonitorResults`

    Example:
        ResultsFileColumns('solution/sensorPos.txt')  #['time', 'Position0', 'Position1', ...]
    """
    header = ReadResultsFileHeader(fileName)
    return list(header.get('columns', []))


def FindResultsFiles(searchDirectories=None, filePatterns=('*.txt', '*.csv')):
    """Search directories for files that carry one of the known Exudyn results headers.

    Args:
        searchDirectories: list of directories; default is `exudyn.config.outputDirectory`, then
                           `'solution'`, then the current directory
        filePatterns: glob patterns tried in every directory

    Returns:
        a list of dictionaries `{'fileName', 'type', 'modified', 'sizeInBytes'}`, newest first

    Example:
        for f in FindResultsFiles(): print(f['type'], f['fileName'])
    """
    if searchDirectories is None:
        searchDirectories = [exudyn.config.outputDirectory, 'solution', '.']

    directories = []
    for directory in searchDirectories:
        if directory is None or directory == '':
            continue
        absolute = os.path.abspath(directory)
        if os.path.isdir(absolute) and absolute not in directories:
            directories.append(absolute)

    found = {}
    for directory in directories:
        for pattern in filePatterns:
            for path in glob.glob(os.path.join(directory, pattern)):
                path = os.path.abspath(path)
                if path in found:
                    continue
                header = ReadResultsFileHeader(path)
                if header.get('type', 'unknown') in knownResultsFileTypes:
                    found[path] = {'fileName': path,
                                   'type': header['type'],
                                   'modified': os.path.getmtime(path),
                                   'sizeInBytes': os.path.getsize(path)}
    return sorted(found.values(), key=lambda info: info['modified'], reverse=True)


def SelectResultsFile(searchDirectories=None, useDialog=True, initialDirectory=''):
    """Ask the user which results file to monitor: a file dialog if tkinter is available and
    dialogs are not suppressed, otherwise a numbered list of the files found.

    Args:
        searchDirectories: passed on to `FindResultsFiles` for the text mode list
        useDialog: if False, the text mode list is used even if tkinter is available
        initialDirectory: directory the file dialog opens in

    Returns:
        the chosen file name, or an empty string if nothing was chosen
    """
    if useDialog and not UIWindowSuppressed('Dialogs', 'SelectResultsFile'):
        try:
            from tkinter import filedialog
            root, window, tkRuns = _GetTkRootAndWindow()
            if not tkRuns:
                window.withdraw()
            fileName = filedialog.askopenfilename(
                parent=window, title='Exudyn results monitor: select a results file',
                initialdir=initialDirectory if initialDirectory != '' else os.getcwd(),
                filetypes=[('results files', '*.txt *.csv'), ('all files', '*.*')])
            if not tkRuns:
                window.destroy()
            return fileName if fileName else ''
        except Exception as e:
            print('NOTE: no file dialog (' + str(e) + '), using the text mode list')

    fileList = FindResultsFiles(searchDirectories)
    if len(fileList) == 0:
        print('no Exudyn results file found; give a file name')
        return ''
    print('results files found:')
    for i, info in enumerate(fileList):
        print('  ' + str(i) + ': ' + _FileInfoText(info))
    try:
        answer = input('number of the file to monitor (empty = newest, q = quit): ').strip()
    except EOFError:
        return ''
    if answer.lower() in ['q', 'quit', 'exit']:
        return ''
    if answer == '':
        return fileList[0]['fileName']
    if answer.isdigit() and int(answer) < len(fileList):
        return fileList[int(answer)]['fileName']
    print('ERROR: ' + answer + ' is not one of the numbers above')
    return ''


def _FileInfoText(info):
    """one line describing a results file, as used by the file list and the status line"""
    return (os.path.relpath(info['fileName']) + '  (' + info['type'] + ', '
            + str(round(info['sizeInBytes'] / 1024., 1)) + ' kB, '
            + time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(info['modified'])) + ')')


class _IncrementalData:
    """Reads the data rows of a results file, only the part that has been appended since the last
    call. The whole file is re-read if it got shorter, which is what happens when the next run
    overwrites it."""

    def __init__(self, fileName):
        self.fileName = fileName
        self.rows = []              #list of lists of float, in file order
        self._offset = 0            #bytes consumed so far
        self._rest = ''             #an incomplete last line, waiting for its newline

    def NumberOfRows(self):
        return len(self.rows)

    def Read(self):
        """append the new rows; returns the number of rows added, or -1 if the file vanished"""
        try:
            size = os.path.getsize(self.fileName)
        except OSError:
            return -1
        if size < self._offset:     #file was overwritten by a new run
            self.rows = []
            self._offset = 0
            self._rest = ''
        if size == self._offset:
            return 0

        with open(self.fileName, 'r') as file:
            file.seek(self._offset)
            text = file.read()
            self._offset = file.tell()

        text = self._rest + text
        if not text.endswith('\n'):         #keep the unfinished line for the next call: the
            text, _, self._rest = text.rpartition('\n')   #solver may be in the middle of writing
        else:
            self._rest = ''

        added = 0
        columns = len(self.rows[0]) if len(self.rows) else 0
        for line in text.split('\n'):
            line = line.strip()
            if line == '' or line[0] == '#':
                continue
            try:
                values = [float(value) for value in line.split(',')]
            except ValueError:
                continue                    #a partly written or corrupted row is skipped
            if columns == 0:
                columns = len(values)
            if len(values) != columns:
                continue
            self.rows.append(values)
            added += 1
        return added

    def Array(self):
        """the data read so far as a 2D numpy array (zero rows gives shape (0,0))"""
        if len(self.rows) == 0:
            return np.zeros((0, 0))
        return np.array(self.rows)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the monitor
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def _GetTkRootAndWindow():
    """[root, window, tkRuns] - a Toplevel if a Tk root already exists (which is the case with the
    TkAgg backend after the first figure was created), otherwise a new root; same behaviour as
    exudyn.GUI.GetTkRootAndNewWindow, but without importing the GUI module"""
    import tkinter as tk
    if tk._default_root is None:
        root = tk.Tk()
        return [root, root, False]
    root = tk._default_root
    return [root, tk.Toplevel(root), True]


class ResultsMonitor:
    """The live view of one results file: reads the header, creates the figure and the control
    panel, and updates the curves until the plot window is closed. Created by `MonitorResults`,
    which is the function to call."""

    def __init__(self, fileName, settings):
        """one monitored file and the settings it is drawn with

        Args:
            fileName: the results file to monitor
            settings: a dictionary as returned by `LoadSettings`, with the additional keys
                      `xColumns`, `yColumns`, `colorVariations`, `variations`, `once`,
                      `saveFigure`, `title`
        """
        self.fileName = fileName
        self.settings = settings
        self.header = {}
        self.xColumns = list(settings.get('xColumns') or [])
        self.yColumns = list(settings.get('yColumns') or [])
        self.xLabels = []
        self.yLabels = []
        self.variations = list(settings.get('variations') or [])
        self.colorVariations = bool(settings.get('colorVariations', False))
        self.lineColor = settings['lineColor']
        self.lineStyle = settings['lineStyle']   #a copy: an optimization file switches to dots,
                                                 #which must not end up in the settings file
        self.data = _IncrementalData(fileName)
        self.figure = None
        self.axisList = []
        self.lineList = []
        self.markerList = []
        self.paused = False
        self.panel = None
        self.nextFileName = ''      #set by the panel button 'open other file'

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def WaitForData(self, timeout=0.):
        """Wait until the file EXISTS, has a header and at least one data row.

        Args:
            timeout: seconds to wait; 0 waits without limit (Ctrl+C stops it)

        Returns:
            True if the file is ready

        Note:
            The file does not have to exist yet, which is the point of starting a monitor before the
            solver: `StartResultsMonitor` does exactly that, and so do the two examples. Until
            revision2026b step RG11.3.1 (#2672) the caller tested for the file and gave up before
            this function was reached, so a monitor started first said "file not found" and exited -
            while its own documentation promised it would wait.

            What is waited for is announced, naming the file, because waiting without limit for a
            file that will never appear is what a typo in the name looks like.
        """
        startTime = time.time()
        announced = False
        announcedMissing = False
        while True:
            if not os.path.exists(self.fileName):
                if timeout > 0 and time.time() - startTime > timeout:
                    print('ERROR: ' + self.fileName + ' did not appear within '
                          + str(timeout) + ' seconds')
                    return False
                if not announcedMissing:
                    print('waiting for ' + self.fileName + ' to appear ... (Ctrl+C to stop)')
                    announcedMissing = True
                time.sleep(0.25)
                continue

            header = ReadResultsFileHeader(self.fileName)
            if header.get('type', 'unknown') in knownResultsFileTypes:
                reader = _IncrementalData(self.fileName)
                if reader.Read() > 0:
                    return True
            elif self._FirstLineIsComplete():
                #the type is decided by the very first line; a complete one that does not match
                #will not start matching later, so this is not something to wait for
                print('ERROR: ' + self.fileName + ' is not an Exudyn results file (no sensor, '
                      'solution, geneticOptimization or parameterVariation header)')
                return False
            if timeout > 0 and time.time() - startTime > timeout:
                print('ERROR: no data in ' + self.fileName + ' after ' + str(timeout) + ' seconds')
                return False
            if not announced:
                print('waiting for data in ' + self.fileName + ' ... (Ctrl+C to stop)')
                announced = True
            time.sleep(0.25)

    def _FirstLineIsComplete(self):
        """True if the file already has a first line ending with a newline; while a solver is
        writing its header, the line may still be incomplete"""
        try:
            with open(self.fileName, 'r') as file:
                return file.readline().endswith('\n')
        except OSError:
            return False

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def Setup(self):
        """Read the header and work out which columns go on which axis; prints the reason and
        returns False if the file cannot be monitored."""
        self.header = ReadResultsFileHeader(self.fileName)
        fileType = self.header.get('type', 'unknown')
        if fileType not in knownResultsFileTypes:
            print('ERROR: ' + self.fileName + ' is not an Exudyn results file '
                  '(no sensor, solution, geneticOptimization or parameterVariation header)')
            return False

        columns = self.header['columns']
        if fileType in ['geneticOptimization', 'parameterVariation']:
            #the parameters go on the x-axes, the fitness resp. result value on every y-axis
            if not self.settings.get('lineStyleGiven', False):
                self.lineStyle = '.'    #a cloud of points, not a line through the generations
            valueColumn = -1
            givenColumns = len(self.xColumns) != 0
            for i, name in enumerate(columns):
                if name == 'value':
                    valueColumn = i
                elif name not in ['globalIndex', 'computationIndex'] and not givenColumns:
                    self.xColumns.append(i)
                    self.xLabels.append(name)
                    self.yLabels.append('fitness' if fileType == 'geneticOptimization' else 'result')
            if valueColumn == -1:
                print('ERROR: no "value" column in ' + self.fileName)
                return False
            if givenColumns:
                self.xLabels = [columns[i] for i in self.xColumns]
                self.yLabels = ['fitness' if fileType == 'geneticOptimization' else 'result'] * len(self.xColumns)
            self.yColumns = [valueColumn] * len(self.xColumns)
        else:                                   #sensor or solution file: everything over time
            if len(self.xColumns) == 0 and len(self.yColumns) == 0:
                self.yColumns = list(range(1, len(columns)))
                self.xColumns = [0] * len(self.yColumns)
            elif len(self.xColumns) == 0:       #only -y given: time on the x-axis
                self.xColumns = [0] * len(self.yColumns)
            elif len(self.yColumns) == 0:
                print('ERROR: --x-cols given without --y-cols')
                return False
            if len(self.xColumns) != len(self.yColumns):
                print('ERROR: ' + str(len(self.xColumns)) + ' x-columns but '
                      + str(len(self.yColumns)) + ' y-columns; they must match')
                return False
            for i in range(len(self.xColumns)):
                if max(self.xColumns[i], self.yColumns[i]) >= len(columns):
                    print('ERROR: column index ' + str(max(self.xColumns[i], self.yColumns[i]))
                          + ' does not exist; the file has ' + str(len(columns)) + ' columns '
                          '(use --list-columns)')
                    return False
                self.xLabels.append(columns[self.xColumns[i]])
                self.yLabels.append(columns[self.yColumns[i]])

        if len(self.xColumns) == 0:
            print('ERROR: nothing to plot in ' + self.fileName)
            return False

        if self.colorVariations and fileType not in ['geneticOptimization', 'parameterVariation']:
            print('NOTE: --color-variations only applies to parameter variation files; ignored')
            self.colorVariations = False
        return True

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def NumberOfCurves(self):
        """number of curves in the figure; with colorVariations this is the number of variations"""
        if not self.colorVariations:
            return len(self.xColumns)
        numberOfVariations = 1
        for oneRange in self.header['variableRanges'][1:]:
            numberOfVariations *= oneRange[2]
        if len(self.variations) == 0:
            self.variations = [0, numberOfVariations]
        return max(0, self.variations[1] - self.variations[0])

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def CreateFigure(self):
        """create the figure, the subplots and one (empty) line per curve"""
        numberOfCurves = self.NumberOfCurves()
        if numberOfCurves == 0:
            print('ERROR: no curve to plot')
            return False

        if self.colorVariations:
            numberOfRows, numberOfColumns = 1, 1     #all variations in one plot
        else:
            maximumColumns = int(np.sqrt(numberOfCurves) + 1)
            if numberOfCurves == 3:
                maximumColumns = 3
            if numberOfCurves == 4:
                maximumColumns = 2
            numberOfRows = int(np.ceil(numberOfCurves / maximumColumns))
            numberOfColumns = min(numberOfCurves, maximumColumns)

        figureName = self.settings.get('title', '') or ('results monitor: '
                                                        + os.path.basename(self.fileName))
        self.figure = plt.figure(figureName)
        self.figure.dpi = 100
        #ON TOP ONLY IF ASKED, and never focused: the monitor used to come to the front on every
        #update because plt.pause raises the window, see _Wait. A user who WANTS it above the console
        #says so with alwaysOnTop, and even then it does not take the keyboard away
        if self.settings.get('alwaysOnTop', False):
            try:
                window = getattr(self.figure.canvas.manager, 'window', None)
                if hasattr(window, 'attributes'):               #tkinter, which TkAgg uses
                    window.attributes('-topmost', True)
                else:
                    #a Qt window would need its own enum - QtCore.Qt.WindowStaysOnTopHint - and that
                    #means importing a Qt binding, which Exudyn does not depend on and will not start
                    #depending on for one line (CLAUDE.md rule 6)
                    print('NOTE: alwaysOnTop is only available with the tkinter backend (TkAgg)')
            except Exception as error:                          # noqa: BLE001
                print('WARNING: results monitor could not stay on top: ' + str(error))
        #'constrained' re-computes the margins at every draw, so the axis labels stay inside the
        #window when the user makes it smaller; tight_layout() would only do it once, at creation
        self.figure.set_layout_engine('constrained')
        sizeInches = self.settings['sizeInches']
        self.figure.set_size_inches(numberOfColumns * sizeInches[0],
                                    numberOfRows * sizeInches[1], forward=True)

        self.axisList = []
        self.lineList = []
        self.markerList = []
        axis = None
        for i in range(numberOfCurves):
            if not self.colorVariations or i == 0:
                axis = self.figure.add_subplot(numberOfRows, numberOfColumns, i + 1)
                axis.grid(True, 'major', 'both')
                if self.settings['logX']:
                    axis.set_xscale('log')
                if self.settings['logY']:
                    axis.set_yscale('log')
            self.axisList.append(axis)

            if self.colorVariations:
                lineCode = PlotLineCode(i)[0] + _listMarkerStyles[int(i / 7) % len(_listMarkerStyles)]
            else:
                lineCode = self.lineColor + self.lineStyle
            line, = axis.plot([], [], lineCode)
            self.lineList.append(line)
            if self.settings['addMarker']:
                marker, = axis.plot([], [], 'ro')
                self.markerList.append(marker)

        return True

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def CurveData(self, index, data):
        """[x, y, label] of curve 'index' out of the data array; label is '' unless variations
        are plotted"""
        if not self.colorVariations:
            return [data[:, self.xColumns[index]], data[:, self.yColumns[index]], '']

        #one curve per variation of the parameters 2..n, over the first parameter
        variableRanges = self.header['variableRanges']
        numberOfRanges = [oneRange[2] for oneRange in variableRanges]
        variationIndex = self.variations[0] + index
        restRange = int(np.array(numberOfRanges[1:]).prod())
        rowIndices = np.arange(numberOfRanges[0]) * restRange + variationIndex
        rowIndices = rowIndices[rowIndices < data.shape[0]]
        if len(rowIndices) == 0:
            return [None, None, '']

        subIndices = SingleIndex2SubIndices(variationIndex, numberOfRanges[1:])
        label = 'var' + str(variationIndex) + ':'
        for i in range(len(subIndices)):
            valueStart = variableRanges[i + 1][0]
            valueEnd = variableRanges[i + 1][1]
            value = valueStart
            if numberOfRanges[i + 1] > 1:
                value += (valueEnd - valueStart) * (subIndices[i] / (numberOfRanges[i + 1] - 1))
            label += self.header['columns'][i + 3][0:5] + str(round(value, 3)) + ' '
        return [data[rowIndices, self.xColumns[0]], data[rowIndices, self.yColumns[0]], label]

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def UpdatePlot(self, force=False):
        """read what was appended and redraw.

        Args:
            force: redraw although no row was added; needed after a setting was changed

        Returns:
            the number of rows added, or -1 if the file disappeared
        """
        added = self.data.Read()
        if self.panel is not None:
            self.panel.UpdateStatus(self.data.NumberOfRows())
        #nothing new and nothing changed: leave the canvas alone. Redrawing an unchanged figure
        #every second is what made the window flicker (the 2021 script did it too)
        if added <= 0 and not force:
            return added
        data = self.data.Array()
        if data.shape[0] == 0:
            return added

        showLegend = False
        for i in range(len(self.lineList)):
            dataX, dataY, label = self.CurveData(i, data)
            if dataX is None:
                continue
            if self.settings['logX']:
                dataX = abs(dataX)
            if self.settings['logY']:
                dataY = abs(dataY)
            self.lineList[i].set_data(dataX, dataY)
            if label != '':
                self.lineList[i].set_label(label)
                showLegend = True
            if self.settings['addMarker'] and len(dataX) > 0:
                self.markerList[i].set_data([dataX[-1]], [dataY[-1]])

            axis = self.axisList[i]
            if not self.colorVariations or i == 0:
                axis.set_xlabel(self.xLabels[0 if self.colorVariations else i])
                axis.set_ylabel(self.yLabels[0 if self.colorVariations else i])
            if self.settings['autoScale']:
                axis.relim()
                axis.autoscale_view()
        if showLegend:
            self.axisList[0].legend()

        #draw_idle lets the backend redraw once, when it is ready; plt.pause in the update
        #loop is what actually flushes it. A direct draw() here draws a second time per tick
        self.figure.canvas.draw_idle()
        return added

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def SaveFigure(self, fileName):
        """write the current figure to a file; the extension decides the format (png, pdf, svg)"""
        try:
            self.figure.savefig(fileName, bbox_inches='tight')
            print('figure written to ' + os.path.abspath(fileName))
            return True
        except Exception as e:
            print('ERROR: could not write ' + fileName + ': ' + str(e))
            return False

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def SaveData(self, fileName):
        """write the rows read so far to a comma separated file, with the column names on top"""
        try:
            data = self.data.Array()
            header = ','.join(self.header.get('columns', [])[0:data.shape[1]])
            np.savetxt(fileName, data, delimiter=',', header=header)
            print('data written to ' + os.path.abspath(fileName))
            return True
        except Exception as e:
            print('ERROR: could not write ' + fileName + ': ' + str(e))
            return False

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def SetLogScale(self, logX, logY):
        """switch the axes between linear and logarithmic while the monitor runs"""
        self.settings['logX'] = logX
        self.settings['logY'] = logY
        for axis in self.axisList:
            axis.set_xscale('log' if logX else 'linear')
            axis.set_yscale('log' if logY else 'linear')
        self.UpdatePlot(force=True)     #a log axis shows absolute values: redraw the data

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def SetCurveVisible(self, index, visible):
        """show or hide one curve (the checkboxes of the control panel)"""
        self.lineList[index].set_visible(visible)
        if self.settings['addMarker']:
            self.markerList[index].set_visible(visible)
        self.figure.canvas.draw_idle()      #the loop may be paused

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def CurveNames(self):
        """the name of every curve, as shown in the control panel"""
        if self.colorVariations:
            return ['variation ' + str(self.variations[0] + i) for i in range(len(self.lineList))]
        return [self.yLabels[i] + ' over ' + self.xLabels[i] for i in range(len(self.lineList))]

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def Run(self):
        """update the plot until the window is closed.

        Returns:
            the file name chosen with 'open other file ...' in the control panel, otherwise ''
        """
        self.UpdatePlot(force=True)
        if self.settings.get('once', False):
            return ''

        plt.ion()
        plt.show(block=False)
        if self.settings['showPanel']:
            self.panel = _ControlPanel.Create(self)

        try:
            while plt.fignum_exists(self.figure.number) and self.nextFileName == '':
                if not self.paused:
                    if self.UpdatePlot() == -1:
                        print('NOTE: ' + self.fileName + ' disappeared; stopping')
                        break
                if self.panel is not None:
                    self.panel.ProcessEvents()
                self._Wait(max(0.05, self.settings['updatePeriod']))
        except KeyboardInterrupt:
            print('\nresults monitor stopped')
        finally:
            self.Close()
        return self.nextFileName

    def _Wait(self, seconds):
        """let the backend work for a while, WITHOUT raising the window

        plt.pause() calls show(block=False) every time, and for TkAgg show() does deiconify() and
        lift() - so the window came to the front and took the focus once per update period, several
        times a second, and the control panel beside it could not be used at all. start_event_loop
        does the waiting and the event processing and nothing else.
        """
        try:
            self.figure.canvas.draw_idle()
            self.figure.canvas.start_event_loop(seconds)
        except Exception:                #a backend without an event loop: the old way, focus and all
            plt.pause(seconds)

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def Close(self):
        """close the control panel and the figure, and store the settings"""
        if self.panel is not None:
            self.panel.Close()
            self.panel = None
        if self.figure is not None and plt.fignum_exists(self.figure.number):
            plt.close(self.figure)
        if self.settings.get('useSettingsFile', True):
            self.settings['lastDirectory'] = os.path.dirname(os.path.abspath(self.fileName))
            SaveSettings(self.settings)


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the tkinter control panel; everything in here is optional - without tkinter, or with
#exudyn.special.userInterface.suppressDialogs, the monitor runs with the plot window alone
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

class _ControlPanel:
    """small tkinter window next to the plot: pause, update period, log scales, autoscale, one
    checkbox per curve, save figure / data, and open another file"""

    maximumCheckBoxes = 16      #above this, the per curve checkboxes are left out

    @staticmethod
    def Create(monitor):
        """the panel, or None if tkinter is missing or dialogs are suppressed"""
        if UIWindowSuppressed('Dialogs', 'results monitor control panel'):
            return None
        try:
            return _ControlPanel(monitor)
        except Exception as e:
            print('NOTE: no control panel (' + str(e) + '); the monitor runs without it')
            return None

    def __init__(self, monitor):
        import tkinter as tk
        import tkinter.ttk as ttk
        self.monitor = monitor
        self.root, self.window, _ = _GetTkRootAndWindow()
        self.window.title('results monitor')
        self.window.protocol('WM_DELETE_WINDOW', self.Close)

        frame = ttk.Frame(self.window, padding=8)
        frame.grid(row=0, column=0, sticky='nsew')
        row = 0

        self.pauseButton = ttk.Button(frame, text='pause', width=12, command=self.TogglePause)
        self.pauseButton.grid(row=row, column=0, sticky='w')
        ttk.Label(frame, text='update (s):').grid(row=row, column=1, sticky='e')
        self.updateValue = tk.StringVar(value=str(monitor.settings['updatePeriod']))
        entry = ttk.Entry(frame, textvariable=self.updateValue, width=6)
        entry.grid(row=row, column=2, sticky='w')
        entry.bind('<Return>', lambda event: self.ApplyUpdatePeriod())
        entry.bind('<FocusOut>', lambda event: self.ApplyUpdatePeriod())
        row += 1

        self.logXValue = tk.BooleanVar(value=monitor.settings['logX'])
        self.logYValue = tk.BooleanVar(value=monitor.settings['logY'])
        self.autoScaleValue = tk.BooleanVar(value=monitor.settings['autoScale'])
        ttk.Checkbutton(frame, text='log x', variable=self.logXValue,
                        command=self.ApplyScales).grid(row=row, column=0, sticky='w')
        ttk.Checkbutton(frame, text='log y', variable=self.logYValue,
                        command=self.ApplyScales).grid(row=row, column=1, sticky='w')
        ttk.Checkbutton(frame, text='autoscale', variable=self.autoScaleValue,
                        command=self.ApplyAutoScale).grid(row=row, column=2, sticky='w')
        row += 1

        self.curveValues = []
        names = monitor.CurveNames()
        if len(names) <= self.maximumCheckBoxes:
            curveFrame = ttk.LabelFrame(frame, text='curves', padding=4)
            curveFrame.grid(row=row, column=0, columnspan=3, sticky='we', pady=4)
            for i, name in enumerate(names):
                value = tk.BooleanVar(value=True)
                self.curveValues.append(value)
                ttk.Checkbutton(curveFrame, text=name, variable=value,
                                command=lambda index=i: self.ApplyCurveVisible(index)
                                ).grid(row=i, column=0, sticky='w')
            row += 1

        ttk.Button(frame, text='save figure ...', command=self.SaveFigure
                   ).grid(row=row, column=0, sticky='we')
        ttk.Button(frame, text='save data ...', command=self.SaveData
                   ).grid(row=row, column=1, sticky='we')
        ttk.Button(frame, text='open file ...', command=self.OpenOtherFile
                   ).grid(row=row, column=2, sticky='we')
        row += 1

        self.statusValue = tk.StringVar(value='')
        ttk.Label(frame, textvariable=self.statusValue, wraplength=320, justify='left'
                  ).grid(row=row, column=0, columnspan=3, sticky='w', pady=(6, 0))
        self.UpdateStatus(monitor.data.NumberOfRows())
        self.window.update()

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def ProcessEvents(self):
        """let tkinter work off its events; the monitor loop is driven by plt.pause, not by
        tkinter's mainloop"""
        try:
            self.window.update()
        except Exception:
            self.window = None      #the panel was destroyed

    def Close(self):
        try:
            if self.window is not None:
                self.window.destroy()
        except Exception:
            pass
        self.window = None
        self.monitor.panel = None

    def UpdateStatus(self, numberOfRows):
        if self.window is None:
            return
        self.statusValue.set(os.path.basename(self.monitor.fileName) + '\n'
                             + str(numberOfRows) + ' rows, last update '
                             + time.strftime('%H:%M:%S')
                             + (' (paused)' if self.monitor.paused else ''))

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def TogglePause(self):
        self.monitor.paused = not self.monitor.paused
        self.pauseButton.config(text='resume' if self.monitor.paused else 'pause')
        self.UpdateStatus(self.monitor.data.NumberOfRows())

    def ApplyUpdatePeriod(self):
        try:
            period = float(self.updateValue.get())
            if period <= 0:
                raise ValueError('must be positive')
            self.monitor.settings['updatePeriod'] = period
        except ValueError:
            self.updateValue.set(str(self.monitor.settings['updatePeriod']))

    def ApplyScales(self):
        self.monitor.SetLogScale(self.logXValue.get(), self.logYValue.get())

    def ApplyAutoScale(self):
        self.monitor.settings['autoScale'] = self.autoScaleValue.get()

    def ApplyCurveVisible(self, index):
        self.monitor.SetCurveVisible(index, self.curveValues[index].get())

    #%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    def SaveFigure(self):
        from tkinter import filedialog
        fileName = filedialog.asksaveasfilename(
            parent=self.window, defaultextension='.png', initialfile='resultsMonitor.png',
            filetypes=[('PNG image', '*.png'), ('PDF', '*.pdf'), ('SVG', '*.svg')])
        if fileName:
            self.monitor.SaveFigure(fileName)

    def SaveData(self):
        from tkinter import filedialog
        fileName = filedialog.asksaveasfilename(
            parent=self.window, defaultextension='.csv', initialfile='resultsMonitor.csv',
            filetypes=[('comma separated values', '*.csv'), ('text file', '*.txt')])
        if fileName:
            self.monitor.SaveData(fileName)

    def OpenOtherFile(self):
        from tkinter import filedialog
        fileName = filedialog.askopenfilename(
            parent=self.window, title='select a results file',
            initialdir=os.path.dirname(os.path.abspath(self.monitor.fileName)),
            filetypes=[('results files', '*.txt *.csv'), ('all files', '*.*')])
        if fileName:
            self.monitor.nextFileName = fileName    #the Run loop ends and the caller reopens


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the library function
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

def MonitorResults(fileName=None, xColumns=None, yColumns=None, updatePeriod=None,
                   logX=None, logY=None, autoScale=None, addMarker=None,
                   colorVariations=False, variations=None,
                   sizeInches=None, lineColor=None, lineStyle=None, title='',
                   showPanel=None, alwaysOnTop=None, once=False, saveFigure='', waitTimeout=0.,
                   searchDirectories=None, useSettingsFile=True):
    """Show the contents of an Exudyn results file while it is being written, and keep the plot
    up to date until the window is closed. This is the function behind
    `python -m exudyn monitor`; it can equally be called from a script or from the console.

    Args:
        fileName: the results file - a sensor file, a coordinates solution file, or the
                  `resultsFile` of `ParameterVariation` / `GeneticOptimization`; `None` opens a
                  file dialog, resp. offers the results files found (see `FindResultsFiles`)
        xColumns: list of column indices for the x-axes; default: time for sensor and solution
                  files, the varied parameters for optimization files (see `ResultsFileColumns`)
        yColumns: list of column indices for the y-axes, one per entry of `xColumns`; default:
                  all columns except time, resp. the fitness value
        updatePeriod: seconds between two updates; `None` takes the stored setting (1 second)
        logX: logarithmic x-axis; absolute values are plotted
        logY: logarithmic y-axis; absolute values are plotted
        autoScale: rescale the axes to the data at every update (default True)
        addMarker: mark the last point of every curve with a red circle
        colorVariations: for a parameter variation file, plot the first parameter only and use one
                         color per variation of the remaining parameters (limited to 28 colors)
        variations: `[start, end]` limiting which variations are shown with `colorVariations`
        sizeInches: `[x, y]` size of ONE subplot in inches
        lineColor: matplotlib color code of the curves, e.g. `'b'`
        lineStyle: matplotlib line style of the curves, e.g. `'-'`
        title: name of the figure window; default: 'results monitor: <file>'
        showPanel: show the control panel next to the plot (default True); needs tkinter
        once: draw the current contents once and return, instead of updating; this is what makes
              the monitor usable in a test or a script that only wants the figure
        saveFigure: if not empty, write the figure to this file (png, pdf or svg)
        waitTimeout: seconds to wait for the file to appear and for its first data row; 0 (the
            default) waits without limit, which is what a monitor started before the solver needs
        searchDirectories: where to look for results files if `fileName` is None
        alwaysOnTop: True keeps the plot window above other windows; it does NOT take the
            keyboard focus either way, see the note below
        useSettingsFile: read and write the `resultsMonitor` section of
            `~/.exudyn/config.json`; False keeps the
                         defaults and changes nothing on disk

    Returns:
        the `ResultsMonitor` of the last file shown, or None if nothing could be monitored

    Note:
        With `exudyn.special.userInterface.suppressPlots` (or the environment variable
        `EXUDYN_SUPPRESS_UI_WINDOW_OPEN`) the monitor never enters its update loop: it draws once
        and writes `saveFigure` if one was given, so that an automated run cannot hang.

    Example:
        #in one Python process, run the optimization with resultsFile='solution/genetic.txt';
        #in another one:
        from exudyn.misc.resultsMonitor import MonitorResults
        MonitorResults('solution/genetic.txt', logY=True, updatePeriod=0.5)
    """
    settings = LoadSettings() if useSettingsFile else dict(_defaultSettings)
    given = {'updatePeriod': updatePeriod, 'logX': logX, 'logY': logY, 'autoScale': autoScale,
             'addMarker': addMarker, 'sizeInches': sizeInches, 'lineColor': lineColor,
             'lineStyle': lineStyle, 'showPanel': showPanel,
             'alwaysOnTop': alwaysOnTop}
    for key, value in given.items():
        if value is not None:
            settings[key] = value
    settings['lineStyleGiven'] = lineStyle is not None
    settings['xColumns'] = xColumns
    settings['yColumns'] = yColumns
    settings['colorVariations'] = colorVariations
    settings['variations'] = variations
    settings['title'] = title
    settings['useSettingsFile'] = useSettingsFile

    #a run without windows draws once and saves; it must never wait for a human
    suppressed = (matplotlib.get_backend().lower() == 'agg'
                  or UIWindowSuppressed('Plots', 'MonitorResults'))
    settings['once'] = once or suppressed
    settings['suppressed'] = suppressed
    if suppressed:
        settings['showPanel'] = False

    if fileName is None or fileName == '':
        fileName = SelectResultsFile(searchDirectories, useDialog=not suppressed,
                                     initialDirectory=settings['lastDirectory'])
        if fileName == '':
            return None

    monitor = None
    while fileName != '':
        #ONLY --once INSISTS THAT THE FILE IS THERE (revision2026b step RG11.3.1, #2672): it plots
        #what exists now and returns, so waiting would be waiting for nothing. Every other mode waits
        #in WaitForData, which is what a monitor started BEFORE the solver needs - and what the
        #documentation has promised all along
        if settings['once'] and not os.path.exists(fileName):
            print('ERROR: file not found: ' + fileName)
            return None
        monitor = ResultsMonitor(fileName, settings)
        if not monitor.WaitForData(waitTimeout if not settings['once'] else max(waitTimeout, 1e-9)):
            return None
        if not monitor.Setup() or not monitor.CreateFigure():
            return None
        fileName = monitor.Run()
        if saveFigure != '':
            monitor.SaveFigure(saveFigure)
            saveFigure = ''
    return monitor


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the monitor beside a running simulation
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


def StartResultsMonitor(fileName, xColumns=None, yColumns=None, updatePeriod=None,
                        logX=False, logY=False, addMarker=False, title='',
                        showPanel=True, extraArguments=None):
    """Start a results monitor in a SECOND process, so that a simulation can go on while its
    results are being watched. The call returns at once.

    The monitor is `python -m exudyn monitor` in a process of its own: it reads the file that the
    simulation writes, which is the only thing the two share, so there is no question of threads,
    of the GIL, or of a plotting backend inside the solver. The file does not need to exist yet -
    the monitor waits for it to appear and for its first row, saying which file it is waiting for.

    Note:
        The process is NOT stopped when the script ends: that is what makes it useful after a
        short simulation. The returned handle is a `subprocess.Popen`, so a script that wants it
        gone calls `.terminate()` on it.

    Note:
        Nothing is started when windows are suppressed - `exudyn.special.userInterface.suppressUI`
        or `EXUDYN_SUPPRESS_UI_WINDOW_OPEN` - and None is returned. A test that calls this
        therefore neither opens a window nor leaves a process behind.

    Args:
        fileName: the results file the simulation writes: a sensor file, a coordinates solution
                  file, or the `resultsFile` of `ParameterVariation` / `GeneticOptimization`
        xColumns: list of column indices for the x-axes; None takes the default (time)
        yColumns: list of column indices for the y-axes; None takes the default (all but time)
        updatePeriod: seconds between two updates; None takes the stored setting (1 second)
        logX: logarithmic x-axis
        logY: logarithmic y-axis
        addMarker: mark the last point of every curve with a red circle
        title: name of the figure window
        showPanel: show the control panel next to the plot
        extraArguments: further command line options of `python -m exudyn monitor`, as a list of
                        strings, for what this function does not name

    Returns:
        the `subprocess.Popen` of the monitor, or None when windows are suppressed or the process
        could not be started

    Example:
        from exudyn.misc.resultsMonitor import StartResultsMonitor
        monitor = StartResultsMonitor('solution/sensorPos.txt', updatePeriod=0.5)
        mbs.SolveDynamic(simulationSettings)     #the plot follows the file while this runs
    """
    if UIWindowSuppressed('Plots', 'StartResultsMonitor'):
        return None

    arguments = [sys.executable, '-m', 'exudyn', 'monitor', str(fileName)]
    if xColumns is not None:
        arguments += ['--x-cols', ','.join([str(column) for column in xColumns])]
    if yColumns is not None:
        arguments += ['--y-cols', ','.join([str(column) for column in yColumns])]
    if updatePeriod is not None:
        arguments += ['--update', str(updatePeriod)]
    if logX:
        arguments += ['--log-x']
    if logY:
        arguments += ['--log-y']
    if addMarker:
        arguments += ['--marker']
    if title != '':
        arguments += ['--title', title]
    if not showPanel:
        arguments += ['--no-panel']
    arguments += list(extraArguments or [])

    try:
        #the child keeps its own stdout: what it prints is the user's, and capturing it would fill
        #a pipe that nobody reads and block the monitor
        return subprocess.Popen(arguments)
    except Exception as e:
        print('WARNING: could not start the results monitor: ' + str(e))
        return None


#%%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the command line
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#spellings of the 2021 script; they are translated and a note is printed, so that scripts and
#printed documentation keep working (revision2026: remove with the next major version)
_legacyArguments = {'-xcols': '--x-cols', '-ycols': '--y-cols', '-logx': '--log-x',
                    '-logy': '--log-y', '-colorVariations': '--color-variations',
                    '-variations': '--variations', '-addMarker': '--marker',
                    '-sizex': '--size-x', '-sizey': '--size-y', '-update': '--update',
                    '-color': '--color', '-style': '--style'}


def _TranslateLegacyArguments(argumentList):
    """replace the single dash spellings of the 2021 script by their current names"""
    translated = []
    used = []
    for argument in argumentList:
        if argument in _legacyArguments:
            used.append(argument)
            translated.append(_legacyArguments[argument])
        else:
            translated.append(argument)
    if len(used) > 0:
        print('NOTE: ' + ', '.join(used) + ' are the old option names; use '
              + ', '.join([_legacyArguments[name] for name in used]) + ' instead')
    return translated


def _IntegerList(text):
    """'0,1 2' -> [0, 1, 2]; used for the column and variation options"""
    values = []
    for part in text.replace(',', ' ').split():
        try:
            values.append(int(part))
        except ValueError:
            raise argparse.ArgumentTypeError('"' + part + '" is not an integer')
    return values


def _Parser():
    """the argument parser of the results monitor; also used for 'python -m exudyn monitor'"""
    parser = argparse.ArgumentParser(
        prog='python -m exudyn monitor',
        description='live view of an Exudyn results file: a sensor file, a coordinates solution '
                    'file, or the resultsFile of ParameterVariation and GeneticOptimization.',
        epilog='examples:\n'
               '  python -m exudyn monitor --last\n'
               '  python -m exudyn monitor solution/genetic.txt --log-y --update 0.2\n'
               '  python -m exudyn monitor sensorPos.txt --y-cols 1,2 --once --save pos.png\n'
               '  python -m exudyn monitor                       (file dialog)\n',
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('file', nargs='?', default=None,
                        help='the results file; without it, a file dialog opens')
    parser.add_argument('-l', '--last', action='store_true',
                        help='take the most recently written results file that is found')
    parser.add_argument('--dir', action='append', default=None, metavar='DIR',
                        help='directory to search for results files (may be given repeatedly); '
                             'default: exudyn.config.outputDirectory, solution/, .')
    parser.add_argument('--list-files', action='store_true',
                        help='list the results files that are found, and exit')
    parser.add_argument('--list-columns', action='store_true',
                        help='list the columns of the file with their indices, and exit')

    columns = parser.add_argument_group('what to plot')
    columns.add_argument('-x', '--x-cols', type=_IntegerList, default=None, metavar='I,J',
                         help='column indices for the x-axes (default: time)')
    columns.add_argument('-y', '--y-cols', type=_IntegerList, default=None, metavar='I,J',
                         help='column indices for the y-axes (default: all but time)')
    columns.add_argument('--color-variations', action='store_true',
                         help='parameter variation files: plot the first parameter only, one '
                              'color per variation of the others (limited to 28 colors)')
    columns.add_argument('--variations', type=_IntegerList, default=None, metavar='A,B',
                         help='with --color-variations: show variations A to B-1')

    appearance = parser.add_argument_group('appearance')
    appearance.add_argument('--log-x', action='store_true', help='logarithmic x-axis')
    appearance.add_argument('--log-y', action='store_true', help='logarithmic y-axis')
    appearance.add_argument('--marker', action='store_true',
                            help='red circle at the last point of every curve')
    appearance.add_argument('--color', default=None, metavar='C',
                            help='matplotlib line color, default b')
    appearance.add_argument('--style', default=None, metavar='S',
                            help='matplotlib line style, default -')
    appearance.add_argument('--size-x', type=float, default=None, metavar='F',
                            help='width of one subplot in inches, default 5')
    appearance.add_argument('--size-y', type=float, default=None, metavar='F',
                            help='height of one subplot in inches, default 5')
    appearance.add_argument('--title', default='', help='name of the figure window')
    appearance.add_argument('--no-panel', action='store_true',
                            help='do not show the control panel next to the plot')
    appearance.add_argument('--always-on-top', action='store_true',
                            help='keep the plot window above other windows; it never takes the'
                                 ' keyboard focus either way')

    behaviour = parser.add_argument_group('behaviour')
    behaviour.add_argument('-u', '--update', type=float, default=None, metavar='SECONDS',
                           help='time between two updates, default 1')
    behaviour.add_argument('--once', action='store_true',
                           help='draw the current contents once and exit (no update loop)')
    behaviour.add_argument('--save', default='', metavar='FILE',
                           help='write the figure to FILE (png, pdf, svg)')
    behaviour.add_argument('--wait', type=float, default=0., metavar='SECONDS',
                           help='how long to wait for the first data row; 0 waits without limit')
    behaviour.add_argument('--no-settings', action='store_true',
                           help='ignore and do not write the resultsMonitor section'
                                ' of ~/.exudyn/config.json')
    return parser


def Main(argumentList=None):
    """Command line of the results monitor, `python -m exudyn monitor [options]`.

    Args:
        argumentList: the arguments without the program name; default `sys.argv[1:]`

    Returns:
        the process return code: 0 on success, 1 on a reported error
    """
    if argumentList is None:
        argumentList = sys.argv[1:]
    parser = _Parser()
    args = parser.parse_args(_TranslateLegacyArguments(list(argumentList)))

    if args.list_files:
        fileList = FindResultsFiles(args.dir)
        if len(fileList) == 0:
            print('no Exudyn results file found')
            return 1
        for info in fileList:
            print('  ' + _FileInfoText(info))
        return 0

    fileName = args.file
    if fileName is None and args.last:
        fileList = FindResultsFiles(args.dir)
        if len(fileList) == 0:
            print('ERROR: --last, but no Exudyn results file was found')
            return 1
        fileName = fileList[0]['fileName']
        print('monitoring ' + _FileInfoText(fileList[0]))

    if args.list_columns:
        if fileName is None:
            print('ERROR: --list-columns needs a file name (or --last)')
            return 1
        columns = ResultsFileColumns(fileName)
        if len(columns) == 0:
            print('ERROR: ' + fileName + ' is not an Exudyn results file')
            return 1
        print(ReadResultsFileHeader(fileName)['type'] + ' file ' + fileName + ':')
        for i, name in enumerate(columns):
            print('  ' + str(i).rjust(3) + ': ' + name)
        return 0

    sizeInches = None
    if args.size_x is not None or args.size_y is not None:
        sizeInches = [args.size_x if args.size_x is not None else _defaultSettings['sizeInches'][0],
                      args.size_y if args.size_y is not None else _defaultSettings['sizeInches'][1]]

    monitor = MonitorResults(
        fileName=fileName, xColumns=args.x_cols, yColumns=args.y_cols,
        updatePeriod=args.update,
        logX=True if args.log_x else None, logY=True if args.log_y else None,
        addMarker=True if args.marker else None,
        colorVariations=args.color_variations, variations=args.variations,
        sizeInches=sizeInches, lineColor=args.color, lineStyle=args.style, title=args.title,
        showPanel=False if args.no_panel else None,
        alwaysOnTop=True if args.always_on_top else None,
        once=args.once, saveFigure=args.save, waitTimeout=args.wait,
        searchDirectories=args.dir, useSettingsFile=not args.no_settings)
    if monitor is None:
        return 1
    if args.once and not monitor.settings.get('suppressed', False):
        plt.show(block=True)    #--once from the command line: keep the window until it is closed
    return 0


if __name__ == '__main__':
    sys.exit(Main())
