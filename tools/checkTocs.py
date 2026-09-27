#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# checkTocs - the table of contents of the PDF is the one of the HTML, apart from a declared difference
#
# Why this check exists (#2697): index.md is the table of contents of the HTML documentation and
# pdfIndex.md the one of the PDF. Both are hand-written, and when the user manual was restructured in
# index.md the PDF went on without a whole chapter for two days; nothing failed, because every page was
# still in some toctree (#2702). The two files differ on purpose in their front page and in the pages
# the PDF leaves out; everything else must be the same entries in the same order.
#
# What is compared: the entries of all {toctree} blocks of each file, in order. What may differ is
# declared below, and a declared entry that is no longer there is a finding too, so the declaration
# cannot go stale.
#
# Usage:
#   python tools/checkTocs.py            #report
#   python tools/checkTocs.py --check    #exit 1 on a finding (the gate)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import argparse
import io
import os
import re
import sys

#the pages only the HTML has: the landing page (README.rst holds web images LaTeX cannot include,
#pdfIndex.md is the PDF's own front page) and the source listings of the examples and test models,
#which a reader of the PDF finds in the repository
ONLY_HTML = ['README',
             'docs/generated/examples/examplesIndex',
             'docs/generated/testModels/testModelsIndex']

#the pages only the PDF has
ONLY_PDF = []


def TocEntries(path):
    """the entries of every {toctree} block of a file, in order; options such as :caption: are not
    entries"""
    text = io.open(path, encoding='utf-8').read()
    entries = []
    for block in re.findall(r'```\{toctree\}(.*?)```', text, flags=re.S):
        for line in block.split('\n'):
            line = line.strip()
            if line != '' and not line.startswith(':'):
                entries.append(line)
    return entries


def CompareTocs(htmlEntries, pdfEntries, onlyHtml=ONLY_HTML, onlyPdf=ONLY_PDF):
    """the findings, as strings; empty if the two agree up to the declared difference"""
    findings = []
    for entry in onlyHtml:
        if entry not in htmlEntries:
            findings.append('declared as HTML only, but not in index.md: ' + entry)
        if entry in pdfEntries:
            findings.append('declared as HTML only, but in pdfIndex.md: ' + entry)
    for entry in onlyPdf:
        if entry not in pdfEntries:
            findings.append('declared as PDF only, but not in pdfIndex.md: ' + entry)
        if entry in htmlEntries:
            findings.append('declared as PDF only, but in index.md: ' + entry)

    html = [entry for entry in htmlEntries if entry not in onlyHtml]
    pdf = [entry for entry in pdfEntries if entry not in onlyPdf]
    for entry in html:
        if entry not in pdf:
            findings.append('in index.md, missing in pdfIndex.md: ' + entry)
    for entry in pdf:
        if entry not in html:
            findings.append('in pdfIndex.md, missing in index.md: ' + entry)

    shared = [entry for entry in html if entry in pdf]
    sharedPdf = [entry for entry in pdf if entry in html]
    for (index, (a, b)) in enumerate(zip(shared, sharedPdf)):
        if a != b:
            findings.append('the order differs at entry ' + str(index + 1) + ': index.md has ' + a
                            + ', pdfIndex.md has ' + b)
            break
    return findings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='exit 1 on a finding')
    parser.add_argument('--quiet', action='store_true', help='print only findings')
    args = parser.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    htmlEntries = TocEntries(os.path.join(root, 'index.md'))
    pdfEntries = TocEntries(os.path.join(root, 'pdfIndex.md'))
    findings = CompareTocs(htmlEntries, pdfEntries)

    if len(findings) == 0:
        if not args.quiet:
            print('OK: index.md and pdfIndex.md list the same ' + str(len(pdfEntries) - len(ONLY_PDF))
                  + ' pages in the same order;\n    only the HTML has ' + ', '.join(ONLY_HTML) + '.')
        return 0

    print('FINDINGS in the tables of contents - index.md and pdfIndex.md must agree, see tools/checkTocs.py:')
    for finding in findings:
        print('   ' + finding)
    return 1 if args.check else 0


if __name__ == '__main__':
    sys.exit(main())
