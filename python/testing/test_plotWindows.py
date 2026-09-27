#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  Where the plot windows of PlotSensor go.
#
#           Plot windows have no unique title - a script makes several - so they are remembered by
#           their SEQUENCE, and PlotSensor(..., closeAll=True) starts that order over.
#           StorePlotWindowGeometry() stores where they are NOW, which is the only moment a user can
#           say "keep them like this": the automatic storing happens when a window closes, one at a
#           time, and the settings dialog is usually gone by then because the renderer has stopped.
#
#           NO WINDOW IS OPENED HERE. The test suite runs matplotlib on Agg, where a figure has no
#           window at all, so the store path is exercised with a STUB figure whose window answers
#           geometry() - which is exactly what the tkinter backend gives.
#
# Usage:    pytest python/testing/test_plotWindows.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-27
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import json

import pytest

import exudyn                                                                # noqa: F401
import exudyn.plot as plot
from exudyn.misc import overrideSettings


class StubWindow:
    """what a tkinter window answers, which is what matplotlib's TkAgg hands over"""

    def __init__(self, geometry):
        self.storedGeometry = geometry

    def geometry(self, newGeometry=None):
        if newGeometry is not None:
            self.storedGeometry = newGeometry
        return self.storedGeometry

    wm_geometry = geometry


class StubQtWindow:
    """what a Qt window answers (QtAgg): a geometry() that is a QRect, not a string (#2722)"""

    def __init__(self, width, height, x, y):
        (self._width, self._height, self._x, self._y) = (width, height, x, y)

    def geometry(self):
        return object()                                 #a QRect, which is no 'WxH+X+Y'

    def width(self):
        return self._width

    def height(self):
        return self._height

    def x(self):
        return self._x

    def y(self):
        return self._y


class StubFigure:
    """a figure whose canvas.manager has a window, or has none because it was closed"""

    def __init__(self, window):
        self.canvas = type('Canvas', (), {})()
        self.canvas.manager = type('Manager', (), {})()
        if window is not None:
            self.canvas.manager.window = window


@pytest.fixture
def settingsFile(tmp_path, monkeypatch):
    """a settings file of this test's own; the store is emptied afterwards"""
    fileName = str(tmp_path / 'config.json')
    monkeypatch.setenv('EXUDYN_CONFIG_FILE', fileName)
    monkeypatch.delenv('EXUDYN_NO_USER_SETTINGS', raising=False)
    with open(fileName, 'w', encoding='utf-8') as file:
        json.dump({'version': overrideSettings.fileFormatVersion}, file)
    overrideSettings.Settings().clear()
    yield fileName
    overrideSettings.Settings().clear()
    plot.__plotWindowFigures.clear()


def testStoringWithNoPlotWindowsIsNothingAndNotAnError(settingsFile):
    plot.__plotWindowFigures.clear()
    assert plot.StorePlotWindowGeometry() == 0


def testTheWindowsAreStoredByTheirSequence(settingsFile):
    """what a user asks for after arranging the plots: keep them like this"""
    plot.__plotWindowFigures.clear()
    plot.__plotWindowFigures.append((1, StubFigure(StubWindow('640x480+10+20'))))
    plot.__plotWindowFigures.append((2, StubFigure(StubWindow('800x600+700+30'))))

    assert plot.StorePlotWindowGeometry() == 2

    dialogs = overrideSettings.Load()['dialogs']
    assert dialogs[overrideSettings.DialogKey('PlotSensor 1')] == {'size': [640, 480],
                                                                  'position': [10, 20]}
    assert dialogs[overrideSettings.DialogKey('PlotSensor 2')] == {'size': [800, 600],
                                                                  'position': [700, 30]}


def testAWindowThatWasClosedIsSkippedAndForgotten(settingsFile):
    """the figure reference outlives the window, which is how the list knows what is still there"""
    plot.__plotWindowFigures.clear()
    plot.__plotWindowFigures.append((1, StubFigure(None)))          #closed: no window any more
    plot.__plotWindowFigures.append((2, StubFigure(StubWindow('320x240+5+6'))))

    assert plot.StorePlotWindowGeometry() == 1
    assert [number for (number, _) in plot.__plotWindowFigures] == [2]

    dialogs = overrideSettings.Load()['dialogs']
    assert overrideSettings.DialogKey('PlotSensor 1') not in dialogs
    assert overrideSettings.DialogKey('PlotSensor 2') in dialogs


def testStoringTwiceKeepsTheLatestAndNothingElse(settingsFile):
    plot.__plotWindowFigures.clear()
    window = StubWindow('640x480+10+20')
    plot.__plotWindowFigures.append((1, StubFigure(window)))
    plot.StorePlotWindowGeometry()

    window.storedGeometry = '900x700+100+200'                       #the user moved it
    plot.StorePlotWindowGeometry()

    dialogs = overrideSettings.Load()['dialogs']
    assert dialogs[overrideSettings.DialogKey('PlotSensor 1')] == {'size': [900, 700],
                                                                  'position': [100, 200]}


def testTheGateForTheAutomaticStoringIsOff():
    """storing when a window closes is a choice; asking for it explicitly always works"""
    assert plot.PlotSensorDefaults().storeWindowPositions is False


def testTheOpenPlotWindowsAreListedWithTheirGeometry(settingsFile):
    """what the store positions button of the settings dialog lists and stores (#2719)"""
    plot.__plotWindowFigures.clear()
    plot.__plotWindowFigures.append((1, StubFigure(StubWindow('640x480+10+20'))))
    plot.__plotWindowFigures.append((2, StubFigure(None)))                      #closed
    assert plot.PlotWindowGeometries() == [('PlotSensor 1', '640x480+10+20')]


def testTheInteractiveDialogsAreKnownWhileTheyAreOpen():
    """the SolutionViewer and the other interactive dialogs are listed while open (#2720)"""
    import inspect
    import exudyn.interactive as interactive
    assert interactive.openDialogs == []
    assert 'windowSize' in inspect.signature(interactive.SolutionViewer).parameters
    assert 'windowSize' in inspect.signature(interactive.InteractiveDialog.__init__).parameters


def testAQtWindowIsListedAndStoredByItsSize(settingsFile):
    """a Qt window has a geometry() as well, returning a QRect; it must not be taken for tkinter's"""
    plot.__plotWindowFigures.clear()
    plot.__plotWindowFigures.append((1, StubFigure(StubQtWindow(640, 480, 10, 20))))

    assert plot.PlotWindowGeometries() == [('PlotSensor 1', '640x480+10+20')]
    assert plot.StorePlotWindowGeometry() == 1
    dialogs = overrideSettings.Load()['dialogs']
    assert dialogs[overrideSettings.DialogKey('PlotSensor 1')] == {'size': [640, 480],
                                                                  'position': [10, 20]}
