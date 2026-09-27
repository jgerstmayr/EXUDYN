#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  tools/checkTocs.py holds index.md and pdfIndex.md together (#2697): the same pages in the
#           same order, apart from a declared difference. The first test is the drift that happened
#           - the PDF without the chapter on performance and errors (#2702) - so the check is shown to
#           find what it was written for.
#
# Usage:    pytest python/testing/test_checkTocs.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-27
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import importlib.util
import os

root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
spec = importlib.util.spec_from_file_location('checkTocs', os.path.join(root, 'tools', 'checkTocs.py'))
checkTocs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checkTocs)

html = ['README', 'docs/manual/gettingStarted', 'docs/manual/tutorial', 'docs/manual/performanceErrors',
        'docs/manual/introductionAdvanced', 'docs/generated/examples/examplesIndex']
onlyHtml = ['README', 'docs/generated/examples/examplesIndex']


def testTheDriftThatHappenedIsFound():
    pdf = ['docs/manual/gettingStarted', 'docs/manual/introductionAdvanced', 'docs/manual/tutorial',
           'docs/manual/commandLine']
    findings = checkTocs.CompareTocs(html, pdf, onlyHtml, [])
    assert 'in index.md, missing in pdfIndex.md: docs/manual/performanceErrors' in findings
    assert 'in pdfIndex.md, missing in index.md: docs/manual/commandLine' in findings
    assert any(finding.startswith('the order differs') for finding in findings)


def testTheDeclaredDifferenceIsNoFinding():
    pdf = [entry for entry in html if entry not in onlyHtml]
    assert checkTocs.CompareTocs(html, pdf, onlyHtml, []) == []


def testADeclarationThatWentStaleIsAFinding():
    pdf = [entry for entry in html if entry not in onlyHtml]
    findings = checkTocs.CompareTocs(html, pdf, onlyHtml + ['docs/gone'], [])
    assert findings == ['declared as HTML only, but not in index.md: docs/gone']


def testTheRepositoryAgrees():
    assert checkTocs.CompareTocs(checkTocs.TocEntries(os.path.join(root, 'index.md')),
                                 checkTocs.TocEntries(os.path.join(root, 'pdfIndex.md'))) == []
