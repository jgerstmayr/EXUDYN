#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  What the item sources are allowed to include. The C++/Python split is an invariant of
#           the project, but nothing enforced it on the include side: until #2622 every one of
#           the 52 sources in src/ImplObjects/ pulled in pybind11,
#           most of them for no reason of their own - src/Graphics/VisualizationItemHelpers.h,
#           which every item that draws includes, reached it through
#           VisualizationSystemContainer.h.
#
#           These tests walk the include graph of the repository - the project headers only, a
#           system header has no includes worth following here - and require that the graphics
#           headers stay free of pybind11. They are a structure test, not a behaviour test: they
#           say nothing about what the code does, only about what a compiler has to read.
#
#           A source that needs Python ITSELF is not a finding: 11 of the 52 reach pybind11
#           through a user function, a PyMatrixContainer or a numpy array, and that is what
#           maximumItemSourcesWithPybind is measured against rather than a round number. It was
#           19 (#2628) removed Utilities/ExceptionsTemplates.h
#           from the fourteen sources that referred to nothing in it.
#
# Usage:    pytest python/testing/test_cppIncludes.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-23
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import io
import os
import re

import pytest

repositoryRoot = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                               '..', '..'))
sourceRoot = os.path.join(repositoryRoot, 'src')

#the sources that legitimately speak to Python themselves: a user function, a PyMatrixContainer
#or a numpy array in their own generated header. Measured 2026-09-23: 52 of 52 before RG9.1, 19
#after it, and 11 after RG9.2 removed an include fourteen sources did not use. The number is
#allowed to FALL without touching this file; it may not rise unnoticed.
maximumItemSourcesWithPybind = 11

includePattern = re.compile(r'^[ \t]*#[ \t]*include[ \t]+[<"]([^>"]+)[>"]', re.M)


def Includes(path):
    """the include lines of one file, as they are written"""
    try:
        text = io.open(path, encoding='utf-8', errors='replace').read()
    except OSError:
        return []
    return includePattern.findall(text)


def ResolveInclude(name):
    """the file an include names, or None if it is not a project header"""
    candidate = os.path.join(sourceRoot, name.replace('/', os.sep))
    return candidate if os.path.isfile(candidate) else None


def PybindChain(startFile):
    """the chain of includes from startFile to a pybind11 header, or None if there is none"""
    visited = set()
    pending = [(startFile, [os.path.relpath(startFile, repositoryRoot)])]
    while pending:
        path, chain = pending.pop()
        if path in visited:
            continue
        visited.add(path)
        for include in Includes(path):
            if include.startswith('pybind11/'):
                return chain + [include]
            following = ResolveInclude(include)
            if following is not None:
                pending.append((following, chain + [include]))
    return None


def ItemSources():
    directory = os.path.join(sourceRoot, 'ImplObjects')
    return sorted(os.path.join(directory, name) for name in os.listdir(directory)
                  if name.endswith('.cpp'))


@pytest.mark.parametrize('header', ['Graphics/VisualizationItemHelpers.h',
                                    'Graphics/VisualizationSystemContainer.h',
                                    'Graphics/VisualizationSystem.h'])
def testTheGraphicsHeadersDoNotReachPybind(header):
    """the point of RG9.1: an item that draws must not have to read pybind11 for that"""
    path = ResolveInclude(header)
    assert path is not None, header + ' does not exist any more'
    chain = PybindChain(path)
    assert chain is None, header + ' reaches pybind11 through ' + ' -> '.join(chain[1:])


def testMostItemSourcesDoNotReachPybind():
    """the sources that still do, do it for a reason of their own - see the note above"""
    withPybind = [os.path.basename(source) for source in ItemSources()
                  if PybindChain(source) is not None]
    assert len(withPybind) <= maximumItemSourcesWithPybind, (
        str(len(withPybind)) + ' of ' + str(len(ItemSources())) + ' item sources reach pybind11: '
        + ', '.join(withPybind))


def testTheItemSourcesAreThereAtAll():
    """a walker that finds nothing must not pass the test above by being silent"""
    assert len(ItemSources()) > 40
