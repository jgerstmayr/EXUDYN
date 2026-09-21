#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  ONE-SHOT migration of tools/issueTracker/trackerlog.txt to the schema of revision2026
#           step R8.5.3: the column 'notes' becomes 'releaseNotes', and 'workingRemarks' and
#           'effort' are appended. 14 columns become 16, and the header says so.
#
#           Why the rename is unambiguous: measured 2026-09-21, NONE of the 270 open issues used
#           the notes column, while 614 closed ones did. Every existing note therefore belongs to
#           a closed issue and is a release note; the two new columns start empty everywhere.
#
#           The priority spellings are normalized in the same pass - nine of them over 251 of the
#           2,567 issues ('', NO, NORMAL, high, med, low, HIGH, medium, LOW) - because the file is
#           being rewritten anyway and the tracker enforces the enum from now on.
#
#           The script PROVES itself: it reads every issue before and after and compares them
#           field by field, and accepts only the two new empty columns and a normalized priority.
#           It refuses to run twice.
#
# Usage:    python migrateSchema.py            (from tools/issueTracker/)
#           python migrateSchema.py --dry-run  (write nothing, report what would change)
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-21
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import io
import os
import sys

TRACKER_FILE = 'trackerlog.txt'

#the schema before this step; the tracker module now holds the one after it
OLD_ITEMS = ['number', 'issue', 'author', 'status', 'description',
             'type', 'priority', 'date raised', 'deadline', 'date resolved',
             'resolved author', 'file', 'line', 'notes']
NEW_ITEMS = OLD_ITEMS[:-1] + ['releaseNotes', 'workingRemarks', 'effort']

#the NINE spellings found in the 2,566 issues - '' (2306), NO (99), NORMAL (63), high (31),
#med (28), low (12), HIGH (10), medium (10), LOW (8) - and what each one becomes. 'NO' becomes
#empty because empty IS no priority (maintainer 2026-09-21), and med/medium is NORMAL.
PRIORITY_MAP = {'': '', 'NO': '',
                'LOW': 'LOW', 'NORMAL': 'NORMAL', 'HIGH': 'HIGH',
                'MED': 'NORMAL', 'MEDIUM': 'NORMAL'}

N_HEADER_LINES = 10


def ReadLines(path):
    """the file is CRLF; the carriage return is stripped here and written back below, or it would
    end up INSIDE the last column as soon as a column is appended after it"""
    with io.open(path, encoding='utf-8', newline='') as file:
        return [line.rstrip('\r') for line in file.read().split('\n')]


def SplitIssue(line, names):
    """one line of the flat file into a dictionary; the ',' escaping of the tracker is kept as it
    is, because this script moves columns and must not touch their content"""
    items = line.split(',')
    if len(items) != len(names):
        raise ValueError('line has ' + str(len(items)) + ' columns instead of ' + str(len(names))
                         + ': ' + line[:60])
    return dict(zip(names, items))


def MigratedHeader(header):
    """the header documents the line format and the value lists; both change here"""
    result = []
    for line in header:
        if line.startswith('# line format:'):
            line = '# line format: ' + ', '.join(
                ['number', 'issue name', 'issue author', 'status', 'description', 'type',
                 'priority', 'date raised', 'deadline', 'date resolved', 'resolved author',
                 'file(name)', 'line(number)', 'releaseNotes', 'workingRemarks', 'effort'])
        elif line.startswith('# priority:'):
            line = ('# priority: LOW, NORMAL, HIGH or empty; effort: LOW (<2h), MEDIUM (<16h), '
                    'HIGH (<40h), HUGE (>40h) or empty')
        elif line.startswith('# end of comment'):
            #the marker states the header length and the tracker reads it as nHeaderLines
            line = '# end of comment (12 lines)'
        elif line.startswith('# status:'):
            result += [line]
            line = ('# releaseNotes: written when the issue is CLOSED and published in the '
                    'release notes; workingRemarks: what the')
            result += [line]
            line = '#   work knows meanwhile - it is cleared when the issue closes'
        result += [line]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dry-run', action='store_true', help='report, write nothing')
    args = parser.parse_args()

    if not os.path.exists(TRACKER_FILE):
        print('run this from tools/issueTracker/: ' + TRACKER_FILE + ' not found')
        return 2

    lines = ReadLines(TRACKER_FILE)
    header = lines[:N_HEADER_LINES]
    body = [line for line in lines[N_HEADER_LINES:] if line.strip() != '']

    if len(body) != 0 and len(body[0].split(',')) == len(NEW_ITEMS):
        print('nothing to do: the file already has ' + str(len(NEW_ITEMS)) + ' columns')
        return 0

    before = [SplitIssue(line, OLD_ITEMS) for line in body]

    migrated = []
    priorityChanges = 0
    for issue in before:
        new = dict(issue)
        new['releaseNotes'] = new.pop('notes')
        new['workingRemarks'] = ''
        new['effort'] = ''

        normalized = PRIORITY_MAP.get(new['priority'].strip().upper())
        if normalized is None:
            raise ValueError('unknown priority "' + new['priority'] + '" in issue '
                             + new['number'])
        if normalized != new['priority']:
            priorityChanges += 1
        new['priority'] = normalized

        migrated += [','.join(new[name] for name in NEW_ITEMS)]

    #=== the proof: read the result back and compare it with what went in ==========================
    after = [SplitIssue(line, NEW_ITEMS) for line in migrated]
    assert len(before) == len(after), 'lost or gained an issue'
    for (old, new) in zip(before, after):
        for name in OLD_ITEMS:
            if name == 'notes':
                assert new['releaseNotes'] == old['notes'], 'note changed in ' + old['number']
            elif name == 'priority':
                assert new['priority'] == PRIORITY_MAP[old['priority'].strip().upper()], \
                    'priority changed unexpectedly in ' + old['number']
            else:
                assert new[name] == old[name], name + ' changed in issue ' + old['number']
        assert new['workingRemarks'] == '' and new['effort'] == ''

    notes = len([issue for issue in before if issue['notes'].strip() != ''])
    openWithNotes = len([issue for issue in before
                         if issue['notes'].strip() != '' and issue['status'].strip() == 'RAISED'])
    print(str(len(before)) + ' issues, ' + str(notes) + ' with a note (' + str(openWithNotes)
          + ' of them open), ' + str(priorityChanges) + ' priority spellings normalized')

    if args.dry_run:
        print('--dry-run: nothing written')
        return 0

    with io.open(TRACKER_FILE, 'w', encoding='utf-8', newline='') as file:
        file.write('\r\n'.join(MigratedHeader(header) + migrated) + '\r\n')
    print('written: ' + TRACKER_FILE + ' with ' + str(len(NEW_ITEMS)) + ' columns')
    return 0


if __name__ == '__main__':
    sys.exit(main())
