#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# checkHeadings - every heading of the hand-written documentation is sentence case
#
# Why this check exists (#2662): the maintainer read the table of contents and found it inconsistent
# - "Mouse input" beside "Generating Animations", "Exudyn basics" beside "Installation and Getting
# Started". A heading is a sentence: only the first word and a name are capitalised.
#
# What a capital in the middle of a heading is allowed to be:
#   - a proper noun (Newmark, Runge-Kutta, Ubuntu, Python) - the NAMES list below;
#   - a name the CODE spells with a capital (GraphicsData, NodePoint, SolveDynamic) - recognised by
#     its shape, CamelCase or ALLCAPS, rather than listed one by one, because the code has hundreds;
#   - a word inside `code` or $math$, which is not prose and is not looked at.
#
# Usage:
#   python tools/checkHeadings.py            #report
#   python tools/checkHeadings.py --check    #exit 1 on a finding (the gate)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import argparse
import glob
import io
import os
import re
import sys

#the pages a human writes; docs/generated/ is the emitters' and is checked where it is generated
PAGES = ['docs/manual/*.md', 'docs/dev/*.md', 'docs/howTo/*.md', 'index.md', 'CONTRIBUTING.md']

#proper nouns: a capital that is neither the first word nor a code name
NAMES = set("""
    Exudyn Python Windows Linux Ubuntu Debian Anaconda Spyder Jupyter Sphinx MyST MiKTeX
    NGsolve OpenGL GLFW Eigen Julia ROS Abaqus ANSYS NETGEN Blender Gym
    Newton Newmark Euler Lagrange Alembert Chasles Rodrigues Bryan Tait Kutta Runge Verlet
    Craig Bampton Hurty Grubler Kutzbach Chebychev Lie Jacobi Coulomb Stribeck Hertz
    Visual Studio Code Git GitHub GitLab Microsoft
    C Main Generalized-alpha Nodes Objects Loads Markers Sensors
    """.split())

#a word that is a name of the code: CamelCase, ALLCAPS, or a name with a digit in it
codeName = re.compile(r'^(?:[A-Z][a-z0-9]*[A-Z]|[A-Z]{2,}|[A-Z][a-z]*\d)')


def Headings(path):
    """(lineNumber, level, title) for every ATX heading outside a fenced code block"""
    inFence = False
    for (number, line) in enumerate(io.open(path, encoding='utf-8'), 1):
        if line.lstrip().startswith('```'):
            inFence = not inFence
            continue
        if inFence:
            continue
        match = re.match(r'^(#{1,6}) (.+?)\s*$', line)
        if match is not None:
            yield (number, len(match.group(1)), match.group(2))


def TitleCaseWords(title):
    """the words that make this heading Title Case, if any"""
    #`code` and $math$ are not prose
    prose = re.sub(r'`[^`]*`|\$[^$]*\$', ' ', title)
    #"GraphicsData: Line" - what follows the colon are VALUES of the code name before it, so they
    #are spelled the way the code spells them
    head = re.match(r'\s*([A-Za-z][A-Za-z0-9]*)\s*:', prose)
    if head is not None and codeName.match(head.group(1)):
        return []
    #a numbered or lettered heading: "(A) Solve for ..." - the letter is a marker, not the first word
    prose = re.sub(r'^\s*\(?\w\)\s*', '', prose)
    #2D, 3D, 6D: the letter belongs to the number, not to the prose
    prose = re.sub(r'\d+[A-Za-z]\b', ' ', prose)
    words = re.findall(r"[A-Za-z][A-Za-z'\-]*", prose)
    found = []
    for word in words[1:]:
        if not word[0].isupper() or word in NAMES:
            continue
        #a hyphenated or possessive name is checked part by part - Runge-Kutta,
        #Lagrange-d'Alembert, Chasles's - and only its CAPITALISED parts have to be names
        parts = [part for part in re.split(r"[\-']", word) if part[:1].isupper()]
        if parts and all(part in NAMES or codeName.match(part) for part in parts):
            continue
        found += [word]
    return found


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='exit 1 on a finding')
    args = parser.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    paths = []
    for pattern in PAGES:
        paths += sorted(glob.glob(os.path.join(root, pattern.replace('/', os.sep))))

    findings = []
    total = 0
    for path in paths:
        for (number, _, title) in Headings(path):
            total += 1
            words = TitleCaseWords(title)
            if words:
                findings.append((os.path.relpath(path, root).replace(os.sep, '/'), number, title,
                                 words))

    if len(findings) == 0:
        print('OK: all ' + str(total) + ' headings of the hand-written documentation are '
              'sentence case.')
        return 0

    print('HEADINGS that are not sentence case - only the first word and a name are capitalised:')
    for (path, number, title, words) in findings:
        print('   ' + path + ':' + str(number) + '  ' + title
              + '      <- ' + ', '.join(words))
    print('\nA name that belongs in the list is added to NAMES in ' + __file__.replace(os.sep, '/')
          + '.')
    return 1 if args.check else 0


if __name__ == '__main__':
    sys.exit(main())
