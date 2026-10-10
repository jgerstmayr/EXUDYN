#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  tools/checkMathFunctions.py refuses a call of the library's math functions in the C++ code, which would
#           escape the switch EXUDYN_PORTABLE_MATH, and a 'using namespace EM;' in a header (#2953); the sources pass.
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

root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
spec = importlib.util.spec_from_file_location('checkMathFunctions', os.path.join(root, 'tools', 'checkMathFunctions.py'))
checkMathFunctions = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checkMathFunctions)


def testTheSourcesPass():
    for relative in checkMathFunctions.SourceFiles():
        raw = io.open(os.path.join(root, relative), 'rb').read()
        text = raw.decode('utf-8', errors='replace')
        assert checkMathFunctions.Findings(text, relative) == [], relative


def testLibraryCallsAreFound():
    code = ('Real a = std::sin(x) + cos(y);\n'
            'Real b = ::atan2(y, x);\n'
            'return pow(x, 2);\n')
    findings = checkMathFunctions.Findings(code, 'src/x.cpp')
    assert len(findings) == 4
    assert any('std::sin' in f and 'EM::Sin' in f for f in findings)
    assert any('::atan2' in f for f in findings) and any('pow' in f for f in findings)


def testWhatIsNoCallPasses():
    code = ('Real a = EM::Sin(x) + Cos(y) + SReal::pow(x, 2.) + v.exp(1);\n'
            '//std::sin(x) in a comment\n'
            'STDstring s = "sin(" + text + ")";\n'
            'static SReal sin(const SReal& x);\n'
            'AutoDiff<n, T> exp(AutoDiff<n, T> x);\n'
            'Real expValue = Sinc(x);\n')
    assert checkMathFunctions.Findings(code, 'src/x.cpp') == []


def testUsingNamespaceInAHeaderIsFound():
    assert len(checkMathFunctions.Findings('using namespace EM;\n', 'src/x.h')) == 1
    assert checkMathFunctions.Findings('using namespace EM;\n', 'src/x.cpp') == []
