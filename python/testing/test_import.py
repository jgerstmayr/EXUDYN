#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  Which compiled module was imported, and why (revision2026 step R6.2, #2540). The
#           selection used to be a nest of four try/except blocks: whichever one succeeded, nothing
#           recorded the decision, and a total failure raised a sentence about 32/64 bits that named
#           neither the candidates nor the reasons.
#
#           It is now one function that returns its log, which is what makes these tests possible at
#           all - and the reason the log exists is not the tests: it is the user who reports "it
#           imports the wrong one", and phase R9, where a plugin is bound to the module it was built
#           against.
#
# Usage:    pytest test_import.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-19 (created, revision2026 step R6.2)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import importlib

import pytest

import exudyn as exu


def test_theModuleThatWasImportedIsNamed():
    """which of the two binaries is loaded is a fact about the run, so it is recorded"""
    assert exu._compiledModuleName in ('exudynCPP', 'exudynCPPfast')


def test_theLogEndsWithTheImportThatWorked():
    """every attempt is in the log, in order, and the last one is the one that succeeded"""
    attempts = exu._importAttempts
    assert len(attempts) >= 1
    for attempt in attempts:
        assert len(attempt) == 2                      #(what was tried, what came of it)
    assert attempts[-1][1] == 'imported'
    assert exu._compiledModuleName in attempts[-1][0]


def test_theModuleIsActuallyTheOneTheLogNames():
    """the log is not a story told next to the import: it names the module the names came from"""
    assert exu._compiledModule.__name__.endswith(exu._compiledModuleName)
    assert exu.SystemContainer is exu._compiledModule.SystemContainer


def test_aFailedImportNamesEveryCandidateAndWhy(monkeypatch):
    """THE point of the rewrite. When nothing can be imported the user gets one error that lists
    what was tried and why each failed - not the last exception, and not a sentence about 32/64
    bits that has been wrong since the 32-bit builds were dropped."""
    def NothingImports(name, package=None):
        raise ImportError('no module named ' + repr(name))

    monkeypatch.setattr(importlib, 'import_module', NothingImports)

    with pytest.raises(ImportError) as caught:
        exu._ImportCompiledModule(useExudynFast=False)

    message = str(caught.value)
    assert '.exudynCPP' in message          #tried as a package module
    assert "'exudynCPP'" in message         #and as a top-level module, the Visual Studio layout
    assert 'no module named' in message     #with the reason, not just the name


def test_theFastModuleIsSkippedWithAReasonWhenTheCpuCannotRunIt(monkeypatch):
    """a skip is an attempt too: it belongs in the log, or the answer to "why is it slow" is
    missing from exactly the place someone looks"""
    monkeypatch.setattr(exu, '_CpuHasAVX2', lambda: False)

    (name, module, attempts) = exu._ImportCompiledModule(useExudynFast=True)

    assert name == 'exudynCPP'
    assert attempts[0][0] == 'exudynCPPfast'
    assert 'AVX2' in attempts[0][1]
