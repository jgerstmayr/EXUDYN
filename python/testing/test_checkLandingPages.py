#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  tools/checkLandingPages.py finds the links of README.rst and the PyPI description that lead
#           nowhere (#2925), and the landing pages of the repository pass it.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-10 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or
#           modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import importlib.util
import io
import os

import pytest

pytest.importorskip('docutils')

root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
spec = importlib.util.spec_from_file_location('checkLandingPages', os.path.join(root, 'tools', 'checkLandingPages.py'))
checkLandingPages = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checkLandingPages)


def testTheLandingPagesPass():
    readme = io.open(os.path.join(root, 'README.rst'), encoding='utf-8').read()
    assert checkLandingPages.Findings(readme, 'README.rst', relativeAllowed=True) == []
    assert checkLandingPages.Findings(checkLandingPages.PyPIDescription(), 'pypi', relativeAllowed=False) == []


def testBrokenLinksAreFound():
    text = ('Text with `a file <docs/theDoc/theDoc.pdf>`_, `the license <LICENSE.txt>`_,\n'
            '`a page <https://exudyn.readthedocs.io/en/latest/docs/manual/noSuchPage.html>`_,\n'
            '`a good page <https://exudyn.readthedocs.io/en/latest/docs/manual/revisions.html>`_,\n'
            '`a source <https://github.com/jgerstmayr/EXUDYN/blob/master/docs/dev/NOSUCHFILE.md>`_ and\n'
            '`elsewhere <https://github.com/pybind/pybind11>`_.\n')
    findings = checkLandingPages.Findings(text, 'readme', relativeAllowed=True)
    assert len(findings) == 3
    assert any('docs/theDoc/theDoc.pdf' in f for f in findings)
    assert any('noSuchPage' in f for f in findings)
    assert any('NOSUCHFILE' in f for f in findings)

    #on pypi.org, any relative link is a finding, also one to an existing file
    findings = checkLandingPages.Findings(text, 'pypi', relativeAllowed=False)
    assert any('LICENSE.txt' in f and 'pypi.org' in f for f in findings)


def testAWarningOfDocutilsIsAFinding():
    findings = checkLandingPages.Findings('Title\n===\n\nan |undefined| substitution\n', 'readme', relativeAllowed=True)
    assert len(findings) >= 1
