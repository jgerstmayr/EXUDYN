#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN tool
#
# Details:  The PDF of the documentation, checked through the log of its LaTeX run (#2856). The html
#           build is strict; the PDF is not, and what goes wrong there - a table that runs into the
#           footer, a column one word wide, a figure larger than the page - is said only in the LaTeX
#           log, among thousands of lines the PDF has always had. This tool reads the log and
#           compares what matters with a baseline in the repository:
#           - an error of the engine (a line starting with '!') and a missing file always fail;
#           - an undefined reference or citation, a character no font has, a float larger than its
#             page, a box too high for its page, and a line wider than the text by more than
#             --overfull points (default 20) fail if the baseline does not have them.
#           A finding is known by what it is about, not by its line in the .tex, which moves with
#           every page: the label, the character and font, or the text of the .tex line the log names.
#           The report gives the page, as the log counts it.
#
# Usage:    python tools/checkPdfLog.py                     check _buildpdf/latex/exudynDocumentation.log
#           python tools/checkPdfLog.py --write-baseline    accept what the log has now
#           exudev docs --pdf --check                       build the PDF and check it
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-05
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import collections
import io
import json
import os
import re
import sys

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
latexDirectory = os.path.join(root, '_buildpdf', 'latex')
latexJob = 'exudynDocumentation'
baselineFile = os.path.join(root, 'tools', 'pdfLogBaseline.json')

#the log, line by line: what starts a finding
_overfull = re.compile(r'^Overfull \\([hv])box \(([0-9.]+)pt too (?:wide|high)\) (?:in paragraph at lines|detected at line|in alignment at lines) (\d+)')
_floatTooLarge = re.compile(r'^LaTeX Warning: Float too large for page by ([0-9.]+)pt on input line (\d+)')
_undefined = re.compile(r"^LaTeX Warning: (Reference|Citation) [`']([^']*)' on page (\d+) undefined")
_missingCharacter = re.compile(r'^Missing character: There is no (.+?) in font (.+?)!')
_missingFile = re.compile(r"(?:LaTeX Warning|Package \w+ Warning|! LaTeX Error): File [`']([^']*)' not found")
_page = re.compile(r'\[(\d+)(?=[\s\]<{]|$)')


def TexLines():
    """the lines of the .tex, to know a finding by its text"""
    texFile = os.path.join(latexDirectory, latexJob + '.tex')
    if not os.path.isfile(texFile):
        return []
    return io.open(texFile, encoding='utf-8', errors='replace').read().split('\n')


def TexText(texLines, lineNumber):
    """the text of a line of the .tex, shortened: what a finding is about; for a line that only structures - the
    start of a table cell, the end of an environment, a short line - the next line that says more after a start, the
    line before it after an end"""
    if not 1 <= lineNumber <= len(texLines):
        return 'line ' + str(lineNumber)
    index = lineNumber - 1
    line = texLines[index].strip()
    step = 1 if line.startswith(_opening) else -1
    for i in range(20): #not further than a table cell
        line = texLines[index].strip()
        if not (line.startswith(_opening) or line.startswith(_closing) or len(line) < 20):
            break
        if not 0 <= index + step < len(texLines):
            break
        index += step
    return ' '.join(texLines[index].replace('\\sphinxAtStartPar', '').split())[:100]

_opening = ('&', '\\sphinxhline', '\\begin{', '\\sphinxstartmulticolumn', '\\sphinxmultirow')
_closing = ('\\end{', '\\sphinxbeforeendvarwidth', '\\sphinxstopmulticolumn', '\\\\')


def Findings(logText, texLines, overfullLimit):
    """[(kind, key, page, detail)] of the log; kind 'error' and 'file' always fail"""
    found = []
    page = 1
    for line in logText.split('\n'):
        if line.startswith('!'):
            found.append(('error', line.strip(), page, ''))
        match = _overfull.match(line)
        if match:
            (box, amount, lineNumber) = (match.group(1), float(match.group(2)), int(match.group(3)))
            if box == 'v':
                found.append(('box too high', TexText(texLines, lineNumber), page, str(amount) + 'pt'))
            elif amount > overfullLimit:
                found.append(('line too wide', TexText(texLines, lineNumber), page, str(amount) + 'pt'))
        match = _floatTooLarge.match(line)
        if match:
            found.append(('float too large', TexText(texLines, int(match.group(2))), page, match.group(1) + 'pt'))
        match = _undefined.match(line)
        if match:
            found.append(('undefined ' + match.group(1).lower(), match.group(2), int(match.group(3)), ''))
        match = _missingCharacter.match(line)
        if match:
            found.append(('missing character', match.group(1) + ' in ' + match.group(2), page, ''))
        match = _missingFile.search(line)
        if match:
            found.append(('file', match.group(1), page, 'not found'))
        for match in _page.finditer(line): #a page is shipped out: "[123" in the log
            number = int(match.group(1))
            if page <= number <= page + 2:
                page = number + 1
    return found


def Counts(findings):
    """how often each (kind, key) occurs: the baseline"""
    return collections.Counter((kind, key) for (kind, key, page, detail) in findings)


def ReadBaseline():
    if not os.path.isfile(baselineFile):
        return collections.Counter()
    data = json.load(io.open(baselineFile, encoding='utf-8'))
    return collections.Counter({(entry['kind'], entry['key']): entry['count'] for entry in data['findings']})


def WriteBaseline(counts, overfullLimit):
    data = {'generatedBy': 'tools/checkPdfLog.py --write-baseline - do not edit by hand (#2856)',
            'overfullLimit': overfullLimit,
            'findings': [{'kind': kind, 'key': key, 'count': count}
                         for ((kind, key), count) in sorted(counts.items())]}
    io.open(baselineFile, 'w', encoding='utf-8', newline='\n').write(json.dumps(data, indent=1, ensure_ascii=False) + '\n')


def Main(argv=None):
    parser = argparse.ArgumentParser(description='check the LaTeX log of the documentation PDF against a baseline (#2856)')
    parser.add_argument('--log', default=os.path.join(latexDirectory, latexJob + '.log'), help='the log to check')
    parser.add_argument('--overfull', type=float, default=None,
                        help='a line wider than the text by more than this many points is a finding (default: the '
                             "baseline's, else 20)")
    parser.add_argument('--write-baseline', action='store_true', dest='writeBaseline',
                        help='accept what the log has now as the baseline')
    args = parser.parse_args(argv)

    if not os.path.isfile(args.log):
        print('checkPdfLog: no ' + args.log + ' - build the pdf first: exudev docs --pdf')
        return 1
    limit = args.overfull
    if limit is None:
        limit = json.load(io.open(baselineFile, encoding='utf-8')).get('overfullLimit', 20.) \
            if os.path.isfile(baselineFile) else 20.
    findings = Findings(io.open(args.log, encoding='utf-8', errors='replace').read(), TexLines(), limit)
    counts = Counts(findings)

    if args.writeBaseline:
        WriteBaseline(Counts([f for f in findings if f[0] not in ('error', 'file')]), limit)
        print('checkPdfLog: baseline written, ' + str(sum(counts.values())) + ' finding(s): ' + baselineFile)
        return 0

    baseline = ReadBaseline()
    failing = []
    seen = collections.Counter()
    for (kind, key, page, detail) in findings:
        seen[(kind, key)] += 1
        if kind in ('error', 'file') or seen[(kind, key)] > baseline[(kind, key)]:
            failing.append((kind, key, page, detail))
    gone = sum(max(0, count - counts[item]) for (item, count) in baseline.items())

    for (kind, key, page, detail) in failing:
        print('page ' + str(page) + ': ' + kind + (' (' + detail + ')' if detail else '') + ': ' + key)
    print('checkPdfLog: ' + str(len(findings)) + ' finding(s) in the log, ' + str(len(failing)) + ' new'
          + (', ' + str(gone) + ' of the baseline gone - accept with --write-baseline' if gone else '')
          + ' (lines wider by more than ' + str(limit) + 'pt)')
    return 1 if failing else 0


if __name__ == '__main__':
    sys.exit(Main())
