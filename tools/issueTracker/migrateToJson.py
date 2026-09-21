#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  ONE-SHOT migration of trackerlog.txt into the JSON store of revision2026 step R8.5:
#           tools/issueTracker/issues/{open,closed,archive}/. The flat file is deleted with the
#           commit that lands this - not kept in parallel, or its comma-escaping trap survives.
#
#           The archives are written DIRECTLY. The 1,652 issues that belong to a year before the
#           cutoff must never exist as single files, not even for one commit, or the repository
#           carries 1,652 objects in its history forever.
#
#           The cutoff is 1 January of the PREVIOUS year (2026 -> 2025-01-01): what closed since
#           then is still interesting and stays a file of its own.
#
#           It PROVES itself: after writing, it reads the store back and compares every issue
#           field by field with the flat file it came from, and it regenerates the flat file from
#           the store and compares that against the original line by line. Nothing is deleted by
#           this script.
#
# Usage:    python migrateToJson.py [--dry-run]     (from anywhere)
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-21 (revision2026 step R8.5)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import collections
import datetime
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import issueStore                                                             # noqa: E402

trackerDirectory = os.path.dirname(os.path.abspath(__file__))
flatFile = os.path.join(trackerDirectory, 'trackerlog.txt')
nHeaderLines = 12

#the 16 columns of the flat file, and the field each one becomes
flatItems = ['number', 'issue', 'author', 'status', 'description',
             'type', 'priority', 'date raised', 'deadline', 'date resolved',
             'resolved author', 'file', 'line', 'releaseNotes', 'workingRemarks', 'effort']

fieldOfColumn = {'issue': 'title', 'date raised': 'dateRaised', 'date resolved': 'dateResolved',
                 'resolved author': 'resolvedAuthor'}


def ReadFlatFile():
    with io.open(flatFile, encoding='utf-8', newline='') as file:
        lines = [line.rstrip('\r') for line in file.read().split('\n')]
    return (lines[:nHeaderLines], [line for line in lines[nHeaderLines:] if line.strip() != ''])


def IssueOfLine(line):
    """one line into the dictionary the store writes; '\\;' is the flat file's escape for a comma
    and goes away with it"""
    items = line.split(',')
    if len(items) != len(flatItems):
        raise ValueError('line with ' + str(len(items)) + ' columns: ' + line[:60])

    issue = {}
    for (column, text) in zip(flatItems, items):
        name = fieldOfColumn.get(column, column)
        value = text.replace('\\;', ',').strip()
        if value != '':
            issue[name] = value

    issue['number'] = int(issue['number'])
    return issue


def LineOfIssue(issue):
    """the inverse, for the proof: the store back into a line of the flat file"""
    items = []
    for column in flatItems:
        value = str(issue.get(fieldOfColumn.get(column, column), ''))
        value = value.replace(',', '\\;')

        if column == 'number':
            value = value.rjust(4, '0')
        elif column == 'issue':
            value = value.ljust(20)
        elif column == 'status':
            value = value.ljust(8)
        elif column == 'description' and value != '':
            value = ' ' + value
        items += [value]
    return ','.join(items)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='report, write nothing')
    args = parser.parse_args()

    if not os.path.isfile(flatFile):
        print('nothing to migrate: ' + flatFile + ' does not exist')
        return 0

    (header, lines) = ReadFlatFile()
    issues = [IssueOfLine(line) for line in lines]
    print(str(len(issues)) + ' issues read from trackerlog.txt')

    cutoffYear = datetime.datetime.now().year - 1
    archives = collections.defaultdict(list)
    single = []
    for issue in issues:
        year = issueStore.ArchiveYear(issue)
        if year != '' and int(year) < cutoffYear and issue['status'] in issueStore.closedStatuses:
            archives[year] += [issue]
        else:
            single += [issue]

    openIssues = [issue for issue in single if issue['status'] not in issueStore.closedStatuses]
    closedIssues = [issue for issue in single if issue['status'] in issueStore.closedStatuses]
    print('  open       ' + str(len(openIssues)).rjust(5))
    print('  closed     ' + str(len(closedIssues)).rjust(5) + '   (closed in ' + str(cutoffYear)
          + ' or later)')
    print('  archived   ' + str(sum(len(v) for v in archives.values())).rjust(5) + '   in '
          + str(len(archives)) + ' year files: '
          + ', '.join(sorted(archives)))

    if args.dry_run:
        print('--dry-run: nothing written')
        return 0

    for issue in openIssues + closedIssues:
        issueStore.Save(issue)
    for (year, yearIssues) in sorted(archives.items()):
        issueStore.WriteArchive(year, yearIssues)

    #=== the proof ================================================================================
    stored = issueStore.LoadAll()
    if len(stored) != len(issues):
        raise SystemExit('MIGRATION FAILED: ' + str(len(issues)) + ' issues went in, '
                         + str(len(stored)) + ' came out')

    for (before, after) in zip(sorted(issues, key=lambda i: i['number']), stored):
        for name in set(list(before) + list(issueStore.issueFields)):
            if str(before.get(name, '')).strip() != str(after.get(name, '')).strip():
                raise SystemExit('MIGRATION FAILED: issue ' + str(before['number']) + ', field "'
                                 + name + '": "' + str(before.get(name)) + '" became "'
                                 + str(after.get(name)) + '"')

    #column by column rather than character by character: some descriptions carry trailing
    #blanks from seven years of editing, and padding is not content
    regenerated = [LineOfIssue(issue) for issue in stored]
    for (index, (old, new)) in enumerate(zip(lines, regenerated)):
        oldColumns = [column.strip() for column in old.split(',')]
        newColumns = [column.strip() for column in new.split(',')]
        if oldColumns != newColumns:
            difference = [(a, b) for (a, b) in zip(oldColumns, newColumns) if a != b]
            raise SystemExit('MIGRATION FAILED: line ' + str(index) + ' differs\n  old: '
                             + str(difference[0][0])[:100] + '\n  new: '
                             + str(difference[0][1])[:100])

    messages = issueStore.CheckStore()
    if messages:
        raise SystemExit('MIGRATION FAILED: the store is not consistent:\n  '
                         + '\n  '.join(messages))

    print('proven: every field of every issue survived, the flat file regenerates line for line, '
          'and the store checks out')
    print('closed issues in the store: ' + str(issueStore.ClosedCount())
          + '   (the micro version is derived from this)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
