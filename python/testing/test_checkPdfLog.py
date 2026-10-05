#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  tools/checkPdfLog.py - the check of the documentation PDF through its LaTeX log (#2856).
#           A test is a log and a .tex as strings: what the tool finds, how it knows a finding by its
#           text rather than its line, and that only what the baseline lacks fails.
#
# Usage:    pytest python/testing/test_checkPdfLog.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-05
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import importlib.util
import os

root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
spec = importlib.util.spec_from_file_location('checkPdfLog', os.path.join(root, 'tools', 'checkPdfLog.py'))
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)

tex = ['\\begin{document}',                                          #1
       'A paragraph with a word that is far too long for its line',    #2
       '&\\begin{varwidth}[t]{\\sphinxcolwidth{1}{5}}',                  #3
       '\\sphinxAtStartPar',                                            #4
       'the text of a table cell that does not fit',                    #5
       '\\sphinxbeforeendvarwidth',                                     #6
       '\\end{varwidth}',                                               #7
       ]
log = '\n'.join([
    'Overfull \\hbox (3.5pt too wide) in paragraph at lines 2--2',       #below the limit: no finding
    'Overfull \\hbox (45.0pt too wide) in paragraph at lines 2--2',
    '[1] [2',
    ']',
    'Overfull \\hbox (30.0pt too wide) detected at line 3',             #the start of a cell: the cell's text
    'Overfull \\hbox (30.0pt too wide) detected at line 6',             #the end of a cell: the same text
    "LaTeX Warning: Reference `docs/manual/GUI:missing' on page 3 undefined on input line 5.",
    'Missing character: There is no − in font lmroman10-regular!',
    'Overfull \\vbox (2.0pt too high) detected at line 7',
    ])


def testTheFindingsAreKnownByTheirText():
    found = checker.Findings(log, tex, 20.)
    assert [(kind, key, page) for (kind, key, page, detail) in found] == [
        ('line too wide', 'A paragraph with a word that is far too long for its line', 1),
        ('line too wide', 'the text of a table cell that does not fit', 3),
        ('line too wide', 'the text of a table cell that does not fit', 3),
        ('undefined reference', 'docs/manual/GUI:missing', 3),
        ('missing character', '− in lmroman10-regular', 3),
        ('box too high', 'the text of a table cell that does not fit', 3)]


def testOnlyWhatTheBaselineLacksFails(tmp_path, monkeypatch, capsys):
    (tmp_path / 'doc.log').write_text(log, encoding='utf-8')
    monkeypatch.setattr(checker, 'baselineFile', str(tmp_path / 'baseline.json'))
    monkeypatch.setattr(checker, 'TexLines', lambda: tex)
    assert checker.Main(['--log', str(tmp_path / 'doc.log')]) == 1             #no baseline: all is new
    assert checker.Main(['--log', str(tmp_path / 'doc.log'), '--write-baseline']) == 0
    assert checker.Main(['--log', str(tmp_path / 'doc.log')]) == 0
    #one more of a known finding, and an error of the engine, which never goes into the baseline
    (tmp_path / 'doc.log').write_text(log + '\nOverfull \\hbox (25.0pt too wide) in paragraph at lines 2--2\n'
                                      '! Undefined control sequence.', encoding='utf-8')
    capsys.readouterr()
    assert checker.Main(['--log', str(tmp_path / 'doc.log')]) == 1
    output = capsys.readouterr().out
    assert 'line too wide (25.0pt): A paragraph' in output and 'error: ! Undefined control sequence.' in output
    assert '2 new' in output
