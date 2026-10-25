#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# checkIssues - the issue store is consistent, and the version follows from it
#
# Why this check exists: the issues are 2,568 JSON files now, and the
# micro version of Exudyn is the COUNT OF CLOSED ISSUES among them. A file that is missing, an
# issue that lies in open/ although it is resolved, a number that exists twice, or an archive
# whose stated closedCount does not match its content would all move version.txt without anyone
# noticing. The flat file could not have those faults; a directory can, so it is checked.
#
# Usage:
#   python tools/checkIssues.py            #report
#   python tools/checkIssues.py --check    #exit 1 when something is wrong (the gate)
#   python tools/checkIssues.py --contribution BASE   #a contribution changes no number and no version (#3083)
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


def CheckGeneratedPages():
    """The two pages the tracker publishes are renderings of the issues, so they have to BE the
    rendering: docs/generated/trackerlog.md and CHANGELOG.md.

    They go stale in one ordinary way - an issue file is added, edited or deleted by hand, which
    the tracker is allowed to permit (a wrongly raised issue is deleted, not tracked forever) but
    which leaves the pages describing the store as it was. Then the documentation shows an issue
    that does not exist. Rendering both into memory and comparing costs a tenth of a second."""
    messages = []
    pages = [(os.path.join(os.path.dirname(trackerDirectory), '..', 'docs', 'generated',
                           'trackerlog.md'), issueTracker.MarkdownText),
             (os.path.join(os.path.dirname(trackerDirectory), '..', 'CHANGELOG.md'),
              issueTracker.ChangelogText)]

    for (path, Render) in pages:
        path = os.path.normpath(path)
        name = os.path.basename(path)
        if not os.path.isfile(path):
            messages.append(name + ' is missing; any tracker verb writes it')
            continue
        written = io.open(path, encoding='utf-8', newline='').read()
        current = Render()
        #line by line, so that a checkout with CRLF endings is not reported as a difference
        if written.splitlines() != current.splitlines():
            messages.append(name + ' is not what the issues render to - it is stale or was '
                            'edited by hand; any tracker verb rewrites it '
                            '(python tools/issueTracker/issueTracker.py is not needed: '
                            '"exudev issue remark" on any issue does it)')
    return messages


def CheckStoredVersions():
    """The version every closed issue carries, against the version recomputed from the store
    (the version of an issue is written when it closes).

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


#the files of a contribution may not change: the numbered issues, the version and what is rendered from them (#3083);
#paths relative to the repository, a directory with a trailing /
contributionForbidden = ['tools/issueTracker/issues/open/', 'tools/issueTracker/issues/closed/',
                         'tools/issueTracker/issues/archive/', 'tools/issueTracker/issues/meta.json',
                         'tools/issueTracker/releases.json', 'tools/issueTracker/versionName.txt', 'version.txt',
                         'src/Autogenerated/versionCpp.cpp', 'CHANGELOG.md', 'docs/generated/trackerlog.md']


def ForbiddenContributionChanges(changedFiles):
    """the files of a contribution that only the maintainer's branch changes; proposals are allowed"""
    return [name for name in changedFiles
            if any(name == path or (path.endswith('/') and name.startswith(path)) for path in contributionForbidden)]


def CheckContribution(base):
    """the files a branch changed against base (git diff BASE...HEAD), refused where they number or version"""
    import subprocess
    repository = os.path.normpath(os.path.join(trackerDirectory, '..', '..'))
    result = subprocess.run(['git', 'diff', '--name-only', base + '...HEAD'], cwd=repository, capture_output=True, text=True)
    if result.returncode != 0:
        return ['git diff ' + base + '...HEAD failed: ' + result.stderr.strip()]
    forbidden = ForbiddenContributionChanges([line.strip() for line in result.stdout.split('\n') if line.strip()])
    return ['the contribution changes ' + name + ', which only the maintainer changes; propose issues and resolutions '
            'with "exudev issue propose" and "propose-resolution" instead' for name in forbidden]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='exit 1 when something is wrong')
    parser.add_argument('--quiet', action='store_true', help='say nothing when all is well')
    parser.add_argument('--contribution', metavar='BASE',
                        help='check only that the branch changes, against BASE, no numbered issue and no version (#3083)')
    args = parser.parse_args()

    if args.contribution:
        messages = CheckContribution(args.contribution)
        for message in messages:
            print('   ' + message)
        if not messages and not args.quiet:
            print('OK: the contribution changes no numbered issue and no version')
        return 1 if messages else 0

    messages = issueStore.CheckStore()
    messages += CheckStoredVersions()
    messages += CheckGeneratedPages()

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
                         + ' - run the tracker after resolving']

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
