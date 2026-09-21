#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# checkIssues - the issue store is consistent, and the version follows from it
#
# Why this check exists (revision2026 step R8.5): the issues are 2,568 JSON files now, and the
# micro version of Exudyn is the COUNT OF CLOSED ISSUES among them. A file that is missing, an
# issue that lies in open/ although it is resolved, a number that exists twice, or an archive
# whose stated closedCount does not match its content would all move version.txt without anyone
# noticing. The flat file could not have those faults; a directory can, so it is checked.
#
# Usage:
#   python tools/checkIssues.py            #report
#   python tools/checkIssues.py --check    #exit 1 when something is wrong (the gate)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import argparse
import io
import os
import sys

trackerDirectory = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'issueTracker')
if trackerDirectory not in sys.path:
    sys.path.insert(0, trackerDirectory)

import issueStore                                                             # noqa: E402
import issueTracker                                                           # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='exit 1 when something is wrong')
    parser.add_argument('--quiet', action='store_true', help='say nothing when all is well')
    args = parser.parse_args()

    messages = issueStore.CheckStore()

    #the version has to be what the files say, and version.txt has to be what the version says
    closed = issueStore.ClosedCount()
    [major, minor, micro] = issueTracker.GetMajorMinorMicroVersion()
    expected = closed - issueTracker.versionResolved[-1]
    if micro != expected:
        messages += ['the version says micro ' + str(micro) + ', the store holds ' + str(closed)
                     + ' closed issues (' + str(expected) + ')']

    versionFile = os.path.join(os.path.dirname(trackerDirectory), '..', 'version.txt')
    versionFile = os.path.normpath(versionFile)
    if os.path.isfile(versionFile):
        written = io.open(versionFile, encoding='utf-8').read().strip()
        if written != issueTracker.VersionString():
            messages += ['version.txt says ' + written + ', the store says '
                         + issueTracker.VersionString()
                         + ' - run the tracker after resolving (revision2026 fact 21)']

    if messages:
        print('THE ISSUE STORE IS NOT CONSISTENT:')
        for message in messages:
            print('   ' + message)
        return 1 if args.check else 0

    if not args.quiet:
        issues = issueStore.LoadAll()
        print('OK: ' + str(len(issues)) + ' issues, ' + str(len(issues) - closed) + ' open, '
              + str(closed) + ' closed; version ' + issueTracker.VersionString() + '.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
