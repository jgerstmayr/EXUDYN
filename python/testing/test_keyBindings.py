#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The key bindings of the render window, which used to be written down three times:
#           GlfwClient.cpp implements them, docs/manual/GUI.md tabulated them and the help dialog
#           printed its own text. The two prose copies come from one table since #2591; these
#           tests are what keeps the THIRD copy - the implementation -
#           in step with it, because that one cannot be generated.
#
#           The strong test is the last one: every binding the table documents must be handled in
#           GlfwClient.cpp, and every key GlfwClient.cpp handles must be documented, with a list
#           of accepted exceptions that is meant to stay empty.
#
# Usage:    pytest python/testing/test_keyBindings.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-23
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import importlib.util
import io
import os
import sys

import pytest

from exudyn.misc.keyBindings import keyBindings, mouseBindings, RendererHelpText

repositoryRoot = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                               '..', '..'))
generatorFile = os.path.join(repositoryRoot, 'tools', 'generators', 'keyBindingsEmitter.py')
glfwClientFile = os.path.join(repositoryRoot, 'src', 'Graphics', 'GlfwClient.cpp')


@pytest.fixture(scope='module')
def emitter():
    """the generator of the documentation tables; it is not part of the exudyn package"""
    if not os.path.exists(generatorFile):
        pytest.skip('no repository here, only the installed package')
    specification = importlib.util.spec_from_file_location('keyBindingsEmitter', generatorFile)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the table itself

def testTheTableHasNoBindingTwice():
    """two rows for one key are two answers to one question, and the second one is invisible"""
    keys = [binding.keys for binding in keyBindings]
    assert sorted(keys) == sorted(set(keys))


def testEveryBindingReachesTheHelpDialog():
    """the dialog is a rendering of the table, so nothing may fall out of it"""
    text = RendererHelpText()
    for binding in mouseBindings + keyBindings:
        assert binding.keys in text, binding.keys + ' is in no line of the help text'
        assert binding.action in text, binding.action + ' is not shown for ' + binding.keys


def testTheHelpTextStaysInItsWidth():
    """it is shown in a plain text window that does not wrap"""
    assert all(len(line) <= 80 for line in RendererHelpText().split('\n'))


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the generated tables of the documentation

def testTheGeneratedTablesAreInStepWithTheTable(emitter):
    """the committed pages must be what the emitter produces - the failure this catches is a
    binding added to the table and never regenerated, which is a page that lies"""
    for (name, bindings, firstColumn) in [('mouseBindings.md', mouseBindings, 'Button'),
                                          ('keyBindings.md', keyBindings, 'Key(s)')]:
        path = os.path.join(repositoryRoot, 'docs', 'generated', name)
        with io.open(path, encoding='utf-8') as file:
            committed = file.read()
        assert committed == emitter.Banner() + emitter.MarkdownTable(bindings, firstColumn), (
            name + ' differs from the table; run tools/regenerate.py')


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the third copy: the implementation, which cannot be generated and is therefore compared

#keys GlfwClient.cpp handles that are deliberately not documented. The list is meant to STAY
#EMPTY: a key a user can press and finds nowhere is the defect RG6.2.6 exists for.
undocumentedOnPurpose = []


def testTheDocumentedBindingsExistInTheRenderer(emitter):
    """a documented key that nothing implements is worse than an undocumented one"""
    if not os.path.exists(glfwClientFile):
        pytest.skip('the C++ sources are not part of the installation')
    with io.open(glfwClientFile, encoding='utf-8', errors='replace') as file:
        implemented = emitter.ImplementedKeys(file.read())
    documented = emitter.DocumentedKeys(keyBindings)

    missing = sorted(key[0] + ('+CONTROL' if key[1] else '') for key in documented - implemented)
    assert missing == [], 'documented but not handled in GlfwClient.cpp: ' + ', '.join(missing)


def testTheRendererHasNoKeyThatIsDocumentedNowhere(emitter):
    if not os.path.exists(glfwClientFile):
        pytest.skip('the C++ sources are not part of the installation')
    with io.open(glfwClientFile, encoding='utf-8', errors='replace') as file:
        implemented = emitter.ImplementedKeys(file.read())
    documented = emitter.DocumentedKeys(keyBindings)

    undocumented = sorted(key[0] + ('+CONTROL' if key[1] else '')
                          for key in implemented - documented)
    unexpected = [key for key in undocumented if key not in undocumentedOnPurpose]
    assert unexpected == [], ('handled in GlfwClient.cpp and documented nowhere: '
                             + ', '.join(unexpected))


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
