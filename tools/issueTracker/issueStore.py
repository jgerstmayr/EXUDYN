#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Where the issues are kept (revision2026 step R8.5): one JSON file per open or recently
#           closed issue, and one file per year for the older ones.
#
#               tools/issueTracker/issues/open/2567.json      one file per OPEN issue
#               tools/issueTracker/issues/closed/2473.json    ... per issue closed since the cutoff
#               tools/issueTracker/issues/archive/2019.json   ... per YEAR, written once
#
#           WHY: trackerlog.txt was one 723 KB line-per-issue file in which a text field could not
#           contain a comma (it was escaped to '\;'), every change rewrote the whole file, and two
#           people editing two issues produced one merge conflict. A file per issue diffs, merges
#           and reviews like the rest of the repository.
#
#           WHY THE ARCHIVE IS SHARDED BY YEAR: 2,298 of the 2,568 issues are closed and will never
#           change again. As single files they would be 2,298 entries in every directory listing
#           and 2,298 objects in git for nothing; as ONE file they would be rewritten on every
#           archiving run. A file per year is written once and then never again.
#
#           THE ONE HARD COUPLING: the micro version of Exudyn is the COUNT OF CLOSED ISSUES, so
#           that count must not depend on which files happen to be present. Each archive file
#           therefore states its own 'closedCount', and ClosedCount() adds those numbers to the
#           closed files it can see; CheckStore() compares the total against the numbers in the
#           files and says so when they disagree.
#
# Usage:    import issueStore   (through issueTracker.py, which is the API)
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-21 (revision2026 step R8.5)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import io
import json
import os

#the format of the files; a reader that does not know this number must not guess
schemaVersion = 1

#the fields of an issue, in the order they are written. Names are camelCase since revision2026
#step R8.5; the flat file had 'date raised', 'resolved author' and - for the title - 'issue'.
issueFields = [
    'number',           #int, the issue number and the file name
    'title',            #one line
    'author',           #who raised it
    'status',           #RAISED, RESOLVED, ABANDONED
    'type',             #see issueTypes in issueTracker.py
    'priority',         #LOW, NORMAL, HIGH or absent
    'effort',           #LOW, MEDIUM, HIGH, HUGE or absent (revision2026 step R8.5.3)
    'description',      #what it is about
    'dateRaised',
    'deadline',
    'dateResolved',
    'resolvedAuthor',
    'file',             #the file it is about
    'line',
    'releaseNotes',     #written when it closes; PUBLISHED in the release notes
    'workingRemarks',   #what the work knows meanwhile; cleared when it closes
    'planStep',         #"R5.4.5" - prose inside the notes until this step
    'component',        #solver / linalg / python / build / docs
    'duplicateOf',      #the issue this one repeats
    'resolvedInVersion',#the version the resolution produced; R7.4 renders CHANGELOG.md from it
    'resolvedCommit',   #the hash
    ]

#an issue without these is not an issue
requiredFields = ['number', 'title', 'status', 'type', 'dateRaised']

closedStatuses = ['RESOLVED', 'ABANDONED']

#where the three directories are, relative to this file; issueTracker.py may point them elsewhere
#(the tests do) by setting storeDirectory
storeDirectory = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'issues')


#%%******************************************************************************************************
def OpenDirectory():
    return os.path.join(storeDirectory, 'open')


def ClosedDirectory():
    """closed, not 'resolved': an ABANDONED issue is closed as well, and counts for the version"""
    return os.path.join(storeDirectory, 'closed')


def ArchiveDirectory():
    return os.path.join(storeDirectory, 'archive')


def IssueFileName(number):
    """padded to four digits, so that a directory listing is in issue order"""
    return str(int(number)).rjust(4, '0') + '.json'


#%%******************************************************************************************************
def WriteJson(path, data):
    """one style for every file the store writes: two spaces, real UTF-8, LF, and a newline at the
    end - so that a diff of an issue shows the field that changed and nothing else"""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with io.open(path, 'w', encoding='utf-8', newline='\n') as file:
        file.write(json.dumps(data, indent=2, ensure_ascii=False) + '\n')


def ReadJson(path):
    with io.open(path, encoding='utf-8') as file:
        return json.load(file)


def CleanIssue(issue):
    """the issue as it is written: the known fields in their order, and empty ones left out - a
    file says what it has, which is what makes a diff readable"""
    result = {}
    for name in issueFields:
        value = issue.get(name, '')
        if isinstance(value, str):
            value = value.strip()
        if value != '' and value is not None:
            result[name] = value

    unknown = [name for name in issue if name not in issueFields]
    if unknown:
        raise ValueError('issue ' + str(issue.get('number')) + ': unknown field(s) '
                         + ', '.join(sorted(unknown)))
    return result


def FullIssue(issue):
    """the issue as the code reads it: every field present, missing ones as empty strings, so that
    a caller never has to ask whether a field exists"""
    result = {name: '' for name in issueFields}
    result.update(issue)
    result['number'] = int(result['number'])
    return result


#%%******************************************************************************************************
def IssuePath(number, status=None):
    """where an issue with this number and status belongs; an existing file elsewhere wins, because
    the archive decides where a closed issue actually is"""
    name = IssueFileName(number)
    for directory in [OpenDirectory(), ClosedDirectory()]:
        if os.path.isfile(os.path.join(directory, name)):
            return os.path.join(directory, name)

    if status is not None and status in closedStatuses:
        return os.path.join(ClosedDirectory(), name)
    return os.path.join(OpenDirectory(), name)


def ArchivePath(year):
    return os.path.join(ArchiveDirectory(), str(year) + '.json')


def ArchiveYear(issue):
    """the year an issue is archived under: when it was closed, or - for the 77 issues closed
    without a date - when it was raised"""
    date = issue.get('dateResolved', '') or issue.get('dateRaised', '')
    return date[:4]


#%%******************************************************************************************************
def LoadAll():
    """every issue, in issue order: the open ones, the recently closed ones and the archives"""
    issues = {}

    for directory in [OpenDirectory(), ClosedDirectory()]:
        if not os.path.isdir(directory):
            continue
        for name in sorted(os.listdir(directory)):
            if name.endswith('.json'):
                issue = FullIssue(ReadJson(os.path.join(directory, name)))
                issues[issue['number']] = issue

    for name in sorted(os.listdir(ArchiveDirectory()) if os.path.isdir(ArchiveDirectory()) else []):
        if not name.endswith('.json'):
            continue
        archive = ReadJson(os.path.join(ArchiveDirectory(), name))
        for issue in archive['issues']:
            issue = FullIssue(issue)
            issues[issue['number']] = issue

    return [issues[number] for number in sorted(issues)]


def Load(number):
    """one issue, or None; the archives are read only when the number is not in the two directories
    (an archive read costs a whole year of issues)"""
    name = IssueFileName(number)
    for directory in [OpenDirectory(), ClosedDirectory()]:
        path = os.path.join(directory, name)
        if os.path.isfile(path):
            return FullIssue(ReadJson(path))

    for issue in LoadAll():
        if issue['number'] == int(number):
            return issue
    return None


def Save(issue):
    """write one issue; a closed issue moves from open/ to closed/ instead of lying in both"""
    issue = FullIssue(issue)
    path = IssuePath(issue['number'], issue['status'])
    wanted = (ClosedDirectory() if issue['status'] in closedStatuses else OpenDirectory())
    wanted = os.path.join(wanted, IssueFileName(issue['number']))

    if os.path.isfile(path) and os.path.normpath(path) != os.path.normpath(wanted):
        os.remove(path)
    WriteJson(wanted, CleanIssue(issue))
    return wanted


def NextNumber():
    """issue numbers are consecutive and never reused; the tracker's version arithmetic and every
    reference in the repository ('#2545') depend on that"""
    issues = LoadAll()
    return (max(issue['number'] for issue in issues) + 1) if issues else 0


#%%******************************************************************************************************
def ClosedCount():
    """THE number the micro version is derived from. The archives state their own count, so a
    missing archive file is noticed by CheckStore() instead of silently lowering the version."""
    count = 0
    if os.path.isdir(ClosedDirectory()):
        count += len([name for name in os.listdir(ClosedDirectory()) if name.endswith('.json')])

    for name in sorted(os.listdir(ArchiveDirectory()) if os.path.isdir(ArchiveDirectory()) else []):
        if name.endswith('.json'):
            count += ReadJson(os.path.join(ArchiveDirectory(), name))['closedCount']

    return count


def WriteArchive(year, issues):
    """one year, written once: the issues it holds and the number of closed ones in it"""
    issues = [CleanIssue(FullIssue(issue)) for issue in issues]
    issues.sort(key=lambda issue: issue['number'])
    closed = len([issue for issue in issues if issue['status'] in closedStatuses])
    WriteJson(ArchivePath(year), {'schemaVersion': schemaVersion,
                                  'year': int(year),
                                  'closedCount': closed,
                                  'issues': issues})
    return len(issues)


#%%******************************************************************************************************
def CheckStore():
    """everything that can be wrong with the files, as a list of messages (revision2026 step R8.5):
    a number twice, a file whose name and content disagree, a missing required field, an issue in
    the wrong directory, and the count the version depends on."""
    messages = []
    seen = {}

    def Check(issue, where, fileName=None):
        number = issue.get('number')
        if number is None:
            messages.append(where + ': an issue without a number')
            return
        if number in seen:
            messages.append('issue ' + str(number) + ' is in ' + seen[number] + ' and in ' + where)
        seen[number] = where

        for name in requiredFields:
            if str(issue.get(name, '')).strip() == '':
                messages.append('issue ' + str(number) + ' (' + where + '): "' + name
                                + '" is required')
        for name in issue:
            if name not in issueFields:
                messages.append('issue ' + str(number) + ' (' + where + '): unknown field "'
                                + name + '"')
        if fileName is not None and fileName != IssueFileName(number):
            messages.append('issue ' + str(number) + ': file is called ' + fileName)

    for (directory, expectClosed) in [(OpenDirectory(), False), (ClosedDirectory(), True)]:
        if not os.path.isdir(directory):
            messages.append(directory + ' does not exist')
            continue
        for name in sorted(os.listdir(directory)):
            if not name.endswith('.json'):
                messages.append(os.path.join(directory, name) + ' is not an issue file')
                continue
            issue = ReadJson(os.path.join(directory, name))
            Check(issue, os.path.basename(directory), fileName=name)
            isClosed = issue.get('status') in closedStatuses
            if isClosed != expectClosed:
                messages.append('issue ' + str(issue.get('number')) + ' is '
                                + str(issue.get('status')) + ' but lies in '
                                + os.path.basename(directory) + '/')

    archived = 0
    for name in sorted(os.listdir(ArchiveDirectory()) if os.path.isdir(ArchiveDirectory()) else []):
        if not name.endswith('.json'):
            continue
        archive = ReadJson(os.path.join(ArchiveDirectory(), name))
        if archive.get('schemaVersion') != schemaVersion:
            messages.append(name + ': schemaVersion ' + str(archive.get('schemaVersion'))
                            + ', this tool writes ' + str(schemaVersion))
        closed = 0
        for issue in archive.get('issues', []):
            Check(issue, 'archive/' + name)
            if issue.get('status') in closedStatuses:
                closed += 1
        if closed != archive.get('closedCount'):
            messages.append(name + ': closedCount says ' + str(archive.get('closedCount'))
                            + ', the file holds ' + str(closed) + ' closed issues')
        archived += closed

    numbers = sorted(seen)
    if numbers and numbers != list(range(numbers[0], numbers[-1] + 1)):
        missing = sorted(set(range(numbers[0], numbers[-1] + 1)) - set(numbers))
        messages.append('missing issue number(s): '
                        + ', '.join(str(number) for number in missing[:10])
                        + (' ...' if len(missing) > 10 else ''))

    return messages
