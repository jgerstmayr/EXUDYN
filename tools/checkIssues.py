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


def CheckStoredVersions():
    """The version every closed issue carries, against the version recomputed from the store
    (revision2026 step R8.4(b), D14).

    This is the check the version numbering did not have. The micro version is a running count,
    so a corrected date, a status changed by hand or a lost file renumbers versions that have
    already been published - and until the number was written into the issue, both sides of any
    comparison came from the same derivation, so nothing could report it.

    Four things are wrong here and each of them means a published number moved:
      - a closed issue with no version, or one that is not major.minor.micro
      - a stored version that the recomputation does not reproduce
      - two closed issues with the same version
      - an OPEN issue that carries a version (it has closed nothing)
    """
    messages = []
    issues = issueTracker.GetIssues()
    closed = [issue for issue in issues if issue['status'] in issueStore.closedStatuses]
    closed.sort(key=lambda issue: issue['dateResolved'])

    for issue in issues:
        stored = str(issue.get('resolvedInVersion', '')).strip()
        if issue['status'] not in issueStore.closedStatuses:
            if stored:
                messages.append('issue ' + str(issue['number']) + ' is ' + issue['status']
                                + ' but carries version ' + stored)
            continue
        if stored.count('.') != 2 or not all(part.isdigit() for part in stored.split('.')):
            messages.append('issue ' + str(issue['number']) + ' is closed but its version is "'
                            + stored + '"')

    seen = {}
    for (index, issue) in enumerate(closed, start=1):
        stored = str(issue.get('resolvedInVersion', '')).strip()
        computed = issueTracker.VersionOfClosedIndex(index)
        if stored and stored != computed:
            messages.append('issue ' + str(issue['number']) + ' says it produced version '
                            + stored + ', but it is the ' + str(index) + 'th closed issue, which '
                            'is version ' + computed + ' - a PUBLISHED version number moved')
        if stored in seen:
            messages.append('version ' + stored + ' belongs to issue ' + str(seen[stored])
                            + ' and to issue ' + str(issue['number']))
        seen[stored] = issue['number']

    #and the releases themselves: baselines rise, and the current one is the last
    baselines = [release['baseline'] for release in issueTracker.Releases()]
    if baselines != sorted(baselines) or len(set(baselines)) != len(baselines):
        messages.append('releases.json: the baselines are not strictly increasing: '
                        + str(baselines))

    return messages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='exit 1 when something is wrong')
    parser.add_argument('--quiet', action='store_true', help='say nothing when all is well')
    args = parser.parse_args()

    messages = issueStore.CheckStore()
    messages += CheckStoredVersions()

    #the version has to be what the files say, and version.txt has to be what the version says
    closed = issueStore.ClosedCount()
    [major, minor, micro] = issueTracker.GetMajorMinorMicroVersion()
    expected = max(0, closed - issueTracker.CurrentRelease()['baseline'])
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
              + str(closed) + ' closed, each carrying the version it produced; version '
              + issueTracker.VersionString() + '.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
