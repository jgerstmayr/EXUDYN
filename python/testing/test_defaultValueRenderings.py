#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The three renderings of a default value - the C++ literal, the Python value and what a
#           documentation table shows - which definitions/definitionTypes.py computes from the
#           value and its declared type (#2682).
#
#           WHY THESE TESTS EXIST: the renderings used to be reconstructed from the C++ literal by
#           twenty str.replace() calls and three substring searches. The last of them stripped the
#           letter "f" unconditionally, to remove the suffix of 0.05f, and so
#           Transformation66List() became Transormation66List() and images/frame became
#           images/rame. Nothing in the tree said so, and an emitter carried the workaround
#           "don't do this for file names, because 'f' is erased!". The rules below are the ones
#           that replaced it, and each test names the defect it forbids.
#
#           These tests read a GENERATOR module, not the installed package: definitions/ is the
#           generators' input and is not shipped.
#
# Usage:    pytest python/testing/test_defaultValueRenderings.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-26
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import sys

import pytest

repositoryRoot = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                               '..', '..'))
definitionsDirectory = os.path.join(repositoryRoot, 'definitions')
if definitionsDirectory not in sys.path:
    sys.path.insert(0, definitionsDirectory)

import definitionTypes as dt                                                     # noqa: E402


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the f that belongs to the C++ type and the f that belongs to a word
@pytest.mark.parametrize('value, expected', [
    ('Float4({0.7f,0.5f,0.5f,0.f})', '[0.7,0.5,0.5,0.]'),   #every suffix goes
    ('Vector3D({0.,0.,0.})', '[0.,0.,0.]'),                 #none to remove
    ('Vector6D({0.,0.,0., 0.,0.,0.})', '[0.,0.,0., 0.,0.,0.]'),   #the author's spacing is kept
])
def testAFloatSuffixIsRemovedAndNothingElseIs(value, expected):
    assert dt.PythonLiteral(value) == expected
    assert dt.DocumentLiteral(value) == expected


def testTheLetterFInsideANameSurvives():
    """Transformation66List() must not become Transormation66List()

    Three members of ObjectKinematicTree had exactly this, and only the CFNoInterface flag kept the
    broken name out of itemInterface.py."""
    assert dt.DocumentLiteral('Transformation66List()') == '[]'
    assert dt.PythonLiteral('Transformation66List()') == 'None'


def testAFileNameKeepsItsF():
    """images/frame must not become images/rame

    This is what the emitter's own workaround was about: a string type was skipped entirely,
    "because 'f' is erased!"."""
    for name in ['images/frame', 'solverInformation.txt', 'undefined', 'coordinatesSolution']:
        assert dt.PythonLiteral(name, 'FileName') == name
        assert dt.DocumentLiteral(name, 'String') == name


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#the renderings a plain Python value has
@pytest.mark.parametrize('value, cpp, python', [
    (True, 'true', 'True'),
    (False, 'false', 'False'),
    (-1.0, '-1.', '-1.'),
    (0.05, '0.05', '0.05'),
    (7, '7', '7'),
])
def testAPlainValueIsItsOwnPythonValue(value, cpp, python):
    assert dt.CppLiteral(value) == cpp
    assert dt.PythonLiteral(value) == python
    assert dt.DocumentLiteral(value) == python


def testTheCppSuffixIsTheTypesAndNeverPythons():
    """0.05 is written 0.05f in a Float4 and 0.05 in Python"""
    assert dt.CppLiteral(0.05, 'Float4') == '0.05f'
    assert dt.PythonLiteral(0.05, 'Float4') == '0.05'


def testAValueWrittenAsAStringWithAStraySpaceIsTheSameValue():
    """defaultValue=' Vector()' reached the generated signature as "coefficientsHull =  []" """
    assert dt.PythonLiteral(' Vector()') == '[]'
    assert dt.PythonLiteral('0.1f ', 'float') == '0.1'


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#a CppValue carries all three renderings, and is asked rather than reconstructed
def testACppValueIsAskedForItsRenderings():
    """the three constants had ToPython() and ToDocument() from the start and nothing called them"""
    assert dt.CppLiteral(dt.DVInvalidIndex) == 'EXUstd::InvalidIndex'
    assert dt.PythonLiteral(dt.DVInvalidIndex) == 'exudyn.InvalidIndex()'
    assert dt.DocumentLiteral(dt.DVInvalidIndex) == 'invalid (-1)'
    assert dt.PythonLiteral(dt.DVZeroVector3D) == '[0.,0.,0.]'
    assert dt.DocumentLiteral(dt.DVDefaultColor) == '[-1.,-1.,-1.,-1.]'


def testANameInsideABracedInitializerIsTranslatedToo():
    """the blind "(" -> "[" put "[ invalid [-1], invalid [-1] ]" on 85 table cells"""
    value = 'ArrayIndex({ EXUstd::InvalidIndex, EXUstd::InvalidIndex })'
    assert dt.PythonLiteral(value) == '[ exudyn.InvalidIndex(), exudyn.InvalidIndex() ]'
    assert dt.DocumentLiteral(value) == '[ invalid (-1), invalid (-1) ]'


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#what happens to an expression nobody mapped
def testAnUnknownExpressionRaisesInsteadOfBeingGuessedAt():
    """the old converter guessed from a substring; this one refuses, and the generator stops

    That is the whole difference: a default value nobody thought about cannot reach a generated
    file looking plausible."""
    with pytest.raises(dt.UnknownDefaultValue) as error:
        dt.PythonLiteral('SomeNewMatrixType(3,4)', 'Matrix34D')
    assert 'SomeNewMatrixType(3,4)' in str(error.value)
    assert 'Matrix34D' in str(error.value)                 #the message names the type as well

    with pytest.raises(dt.UnknownDefaultValue) as error:
        dt.DocumentLiteral('SomeNewList()')
    assert 'emptyContainerValues' in str(error.value)       #and says which table to extend


def testAnEnumValueIsLegibleAsItIsWritten():
    for value in ['DynamicSolverType::DOPRI5', 'LinearSolverType::EXUdense', 'ItemType::_None']:
        assert dt.PythonLiteral(value) == value
        assert dt.DocumentLiteral(value) == value


#%%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#and the whole corpus: every default value of every definition renders, or the generator would stop
def testEveryDefaultValueOfEveryDefinitionRenders():
    """the gate that the generator itself is: 1376 of them, and no rule may be missing"""
    if os.path.join(repositoryRoot, 'tools', 'generators') not in sys.path:
        sys.path.insert(0, os.path.join(repositoryRoot, 'tools', 'generators'))
    import definitionLoader

    rendered = 0
    for modules in [definitionLoader.itemModules, definitionLoader.structureModules]:
        for moduleName in modules:
            for definition in __import__(moduleName).definitions:
                for member in definition.get('members', []):
                    if 'Function' in str(member.get('kind', '')):
                        continue        #a function's "default" is its C++ body
                    value = member.get('defaultValue', '')
                    if value is dt.NoDefaultValue or value is None or value == '':
                        continue
                    typeName = member.get('type', '')
                    #the assertion is that neither of these raises
                    dt.PythonLiteral(value, typeName)
                    dt.DocumentLiteral(value, typeName)
                    rendered += 1
    assert rendered > 1300, 'only ' + str(rendered) + ' default values were found'
