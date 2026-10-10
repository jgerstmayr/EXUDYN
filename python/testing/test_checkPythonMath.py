#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test infrastructure file
#
# Details:  tools/checkPythonMath.py refuses the math functions of math and numpy and powers with an exponent that is
#           not a whole number in the Python library and the test models, which take exudyn.cMath instead (#2964);
#           the sources pass.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-11 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or
#           modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import importlib.util
import io
import os

root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
spec = importlib.util.spec_from_file_location('checkPythonMath', os.path.join(root, 'tools', 'checkPythonMath.py'))
checkPythonMath = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checkPythonMath)


def testTheSourcesPass():
    for relative in checkPythonMath.SourceFiles():
        text = io.open(os.path.join(root, relative), 'r', encoding='utf-8', errors='replace').read()
        assert checkPythonMath.Findings(text, relative)[0] == [], relative


def testLibraryFunctionsAndPowersAreFound():
    code = ('from math import sin, pi, sqrt\n'
            'a = math.cos(x) + np.arctan2(y, x) + numpy.exp(z)\n'
            'b = x**0.5 + (1+y)**(1/3) + z**-1.5\n')
    (findings, wholeExponents) = checkPythonMath.Findings(code, 'x.py')
    assert len(findings) == 7 and wholeExponents == 0
    assert any('exudyn.cMath.atan2' in f for f in findings) and any('**(1/3)' in f for f in findings)


def testWhatIsAllowedPasses():
    code = ('from math import pi, sqrt, fsum\n'
            'from exudyn.cMath import sin, cos\n'
            'a = sin(x) + np.sqrt(y) + math.sqrt(z) + x**2 + y**-1 + v.sin(1)\n'
            '#np.sin(x) in a comment\n'
            's = "np.sin(x) and x**0.5"\n'
            'f = (esym.abs(d))**1.2 #checkPythonMath: allowed - a symbolic.Real\n')
    assert checkPythonMath.Findings(code, 'x.py') == ([], 2)
