#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  tools/buildFigures.py, the TikZ flow charts of the documentation (#2812): every chart is
#           compiled from its current source, and a chart changed but not compiled is found - without
#           LaTeX, which the test runners do not have.
#
# Usage:    pytest python/testing/test_buildFigures.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-03
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import importlib.util
import os
import shutil

root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
spec = importlib.util.spec_from_file_location('buildFigures', os.path.join(root, 'tools', 'buildFigures.py'))
builder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(builder)


def testEveryChartIsCompiledFromItsSource():
    charts = builder.Charts()
    assert len(charts) >= 12
    assert builder.Stale(charts, builder.ReadManifest()) == []


def testAChangedChartIsStale(tmp_path):
    charts = builder.Charts()
    (name, fileName) = next(iter(charts.items()))
    changed = tmp_path / (name + '.tex')
    shutil.copy(fileName, changed)
    with open(changed, 'a', encoding='utf-8') as file:
        file.write('% a change\n')
    assert builder.Stale({name: str(changed)}, builder.ReadManifest()) == [name]
