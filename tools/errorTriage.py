#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Every place where the C++ side reports an error, sorted by WHO the message is for
#. The five helpers - CHECKandTHROW, CHECKandTHROWstring,
#           PyError, SysError, PyWarning - all end up as RuntimeError in Python today, and the
#           helper a check is written with says nothing about what the check means: the same macro
#           states a user's index mistake in one place and an Exudyn invariant in the next.
#
#           So the unit of the mapping is the CALL SITE, and this tool is what makes 2060 of them
#           reviewable. It answers two questions per site:
#
#             audience  USER      the user did something; the message must say what to change
#                       INTERNAL  an Exudyn invariant broke; the message is for a developer
#                                 reading a log from a run they cannot reproduce
#                       UNPLACED  the rules do not decide it; it is listed, not guessed
#             kind      index / size / arithmetic / notImplemented / solver / deprecation /
#                       illegal / other - what Python type the site should end up with (R6.3.6)
#
#           The rules are text patterns over the message and the path. They are not a judgement
#           about what the code does and they are not always right; what they are is REPEATABLE, so
#           a number in the plan can be re-derived, and progress through R6.3.6 can be measured
#           rather than claimed.
#
# Usage:    python tools/errorTriage.py                summary tables
#           python tools/errorTriage.py --sites KIND   every site of one kind, with file and line
#           python tools/errorTriage.py --csv FILE     one row per site
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-18 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import collections
import io
import os
import re
import sys

repositoryRoot = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sourceRoot = os.path.join(repositoryRoot, 'src')

helperNames = ['CHECKandTHROWstring', 'CHECKandTHROW', 'CHECKandTHROWcond',
               'PyError', 'SysError', 'PyWarning']
#the four files that DEFINE the helpers are not call sites of them
definingFiles = ['ReleaseAssert.h', 'Stdoutput.h', 'Stdoutput.cpp', 'ExceptionsTemplates.h']

callPattern = re.compile(r'\b(' + '|'.join(helperNames) + r')\s*\(')


#%%******************************************************************************************************
def CallSites():
    """Every live call of the five helpers, with as much of the message as the call spans. A call
    inside a commented-out line is not a call site - 188 of them exist and counting them was how
    the first inventory reached 2248 instead of 2060."""
    sites = []
    for (directory, _, files) in os.walk(sourceRoot):
        for name in sorted(files):
            if not name.endswith(('.h', '.cpp')) or name in definingFiles:
                continue
            path = os.path.join(directory, name)
            relativePath = os.path.relpath(path, repositoryRoot).replace(os.sep, '/')
            lines = io.open(path, encoding='utf-8', errors='replace').read().split('\n')

            for (index, line) in enumerate(lines):
                if line.strip().startswith('//'):
                    continue
                for match in callPattern.finditer(line):
                    #the call may continue over the next lines; take up to four more
                    text = line[match.start():]
                    for following in lines[index + 1:index + 5]:
                        if text.count('(') <= text.count(')'):
                            break
                        text += ' ' + following.strip()
                    sites.append({'path': relativePath, 'line': index + 1,
                                  'helper': match.group(1), 'text': ' '.join(text.split())})

    return sites


#%%******************************************************************************************************
#the kind of error, decided by what the message says. First match wins, so the order is the
#priority: a message about a deprecated setting is a deprecation even if it also says "invalid".
kindRules = [
    ('deprecation',    [r'deprecat', r'is *renamed', r'use +instead', r'will be removed']),
    ('notImplemented', [r'not +implemented', r'not +available', r'not +supported', r'only +implemented',
                        r'no +implementation', r'not +yet', r'unsupported', r'not +possible for']),
    ('solver',         [r'diverg', r'singular', r'not +converge', r'no +convergence', r'step *size',
                        r'newton', r'jacobian.*invert', r'linear *solver', r'factoriz']),
    ('arithmetic',     [r'division +by +zero', r'divide +by +zero', r'zero +denominator', r'sqrt',
                        r'norm +is +zero', r'length +(is +)?zero', r'must +not +be +zero',
                        r'determinant', r'singular +matrix']),
    ('index',          [r'index', r'out +of +range', r'invalid +.*number', r'exceeds', r'\bitem +number']),
    ('size',           [r'size', r'dimension', r'number +of +rows', r'number +of +columns',
                        r'length', r'incompatible', r'must +have', r'rows +and +columns', r'shape']),
    ('file',           [r'could +not +open', r'cannot +open', r'file +.*not', r'directory', r'\.txt',
                        r'write +.*file']),
    ('illegal',        [r'invalid', r'illegal', r'must +be', r'cannot +be', r'not +allowed',
                        r'wrong', r'unknown', r'inconsistent', r'failed', r'requires']),
]

#a site is INTERNAL when the message says so, whatever the path
internalPhrases = [r'internal', r'invalid +call', r'untested', r'should +not +happen',
                   r'not +expected', r'unexpected', r'contact +the +developer', r'report +this',
                   r'this +is +a +bug', r'illegal +call', r'inconsistent +state']
#...and when it is in a layer a user never reaches directly
internalAreas = ['src/Linalg', 'src/Utilities', 'src/Tests']
#...while these layers exist to talk to the user
userAreas = ['src/Pymodules', 'src/Main', 'src/Autogenerated']

internalPattern = re.compile('|'.join(internalPhrases), re.I)


#%%******************************************************************************************************
def Kind(text):
    for (kind, patterns) in kindRules:
        for pattern in patterns:
            if re.search(pattern, text, re.I):
                return kind

    return 'other'


#%%******************************************************************************************************
def Audience(site, kind):
    """USER, INTERNAL or UNPLACED. The rule that beats every other one is the message itself: a
    check that says 'internal' is internal wherever it stands."""
    if internalPattern.search(site['text']):
        return 'INTERNAL'

    if kind == 'deprecation':
        return 'USER'                        #a deprecation is always addressed to the user

    for area in internalAreas:
        if site['path'].startswith(area):
            return 'INTERNAL'

    #the generated visualization settings and pybind modules are the user's own parameters
    for area in userAreas:
        if site['path'].startswith(area):
            return 'USER'

    #CHECKandTHROWcond has no message at all: "unexpected EXUDYN internal error" is all it can say
    if site['helper'] == 'CHECKandTHROWcond':
        return 'INTERNAL'

    #src/ImplObjects, src/System, src/Solver, src/Graphics: a check on a MODEL the user built is for
    #the user; a check on a data structure Exudyn filled in is not. The helper is the best signal
    #that is available here - PyError and PyWarning were written to talk to somebody
    if site['helper'] in ['PyError', 'PyWarning']:
        return 'USER'
    if site['helper'] == 'SysError':
        return 'INTERNAL'

    #a feature that does not exist is the user's answer to a question they asked: "this item has no
    #angular velocity". Outside the internal layers that is never a broken invariant
    if kind == 'notImplemented':
        return 'USER'

    #a check ON THE MODEL: the parameters of the item, or what a marker the user attached provides.
    #The item cannot fix these; the user can, by building the model differently
    if re.search(r'parameters\.|markerData|GetMarkerData|nodeNumber|markerNumber', site['text']):
        return 'USER'

    return 'UNPLACED'


#%%******************************************************************************************************
def Classify():
    sites = CallSites()
    for site in sites:
        site['kind'] = Kind(site['text'])
        site['audience'] = Audience(site, site['kind'])

    return sites


#%%******************************************************************************************************
def Area(path):
    parts = path.split('/')

    return '/'.join(parts[:2]) if len(parts) > 2 else path


#%%******************************************************************************************************
def PrintTable(title, rows, columns):
    print('')
    print(title)
    print('    ' + 'row'.ljust(24) + ''.join(c.rjust(14) for c in columns) + 'total'.rjust(10))
    for (name, counts) in rows:
        total = sum(counts.get(c, 0) for c in columns)
        print('    ' + name.ljust(24)
              + ''.join(str(counts.get(c, 0)).rjust(14) for c in columns)
              + str(total).rjust(10))


#%%******************************************************************************************************
def Main():
    parser = argparse.ArgumentParser(description='where the C++ side reports errors, and to whom')
    parser.add_argument('--sites', metavar='KIND', help='list every site of one kind, or of one audience')
    parser.add_argument('--csv', metavar='FILE', help='write one row per site')
    args = parser.parse_args()

    sites = Classify()

    if args.csv:
        with io.open(args.csv, 'w', encoding='utf-8', newline='\n') as file:
            file.write('file;line;helper;audience;kind;text\n')
            for site in sites:
                file.write(';'.join([site['path'], str(site['line']), site['helper'],
                                     site['audience'], site['kind'],
                                     site['text'].replace(';', ',')]) + '\n')
        print('written: ' + args.csv + ' (' + str(len(sites)) + ' sites)')
        return 0

    if args.sites:
        wanted = args.sites
        selected = [s for s in sites if wanted in (s['kind'], s['audience'])]
        print(str(len(selected)) + ' sites for "' + wanted + '"')
        for site in selected:
            print('  %-58s %-20s %-14s %s' % (site['path'] + ':' + str(site['line']),
                                              site['helper'], site['kind'], site['text'][:90]))
        return 0

    print('call sites of the five error helpers in src/: ' + str(len(sites)))

    audiences = ['USER', 'INTERNAL', 'UNPLACED']
    kinds = [k for (k, _) in kindRules] + ['other']

    byArea = collections.defaultdict(collections.Counter)
    for site in sites:
        byArea[Area(site['path'])][site['audience']] += 1
    PrintTable('by area:', sorted(byArea.items(), key=lambda kv: -sum(kv[1].values())), audiences)

    byHelper = collections.defaultdict(collections.Counter)
    for site in sites:
        byHelper[site['helper']][site['audience']] += 1
    PrintTable('by helper - note that it does NOT predict the audience:',
               sorted(byHelper.items(), key=lambda kv: -sum(kv[1].values())), audiences)

    byKind = collections.defaultdict(collections.Counter)
    for site in sites:
        byKind[site['kind']][site['audience']] += 1
    PrintTable('by kind:', [(k, byKind[k]) for k in kinds if k in byKind], audiences)

    print('')
    print('"UNPLACED" is not a failure of the tool: it is the list of sites that have to be read.')
    print('    python tools/errorTriage.py --sites UNPLACED')

    return 0


#%%******************************************************************************************************
if __name__ == '__main__':
    sys.exit(Main())
