# -*- coding: utf-8 -*-
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Created on Fri May 10 08:53:30 2019
# @author: Johannes Gerstmayr

# Issue tracker: the rules. The FILES are issueStore.py - one JSON file per open or recently
# closed issue in tools/issueTracker/issues/, one per year for the older ones. trackerlog.txt, a
# 723 KB file of comma-separated lines in which no text field could
# contain a comma, is gone.
#
# - the fields of an issue: issueStore.issueFields
# - type, status, priority and effort: issueTypes, issueStatuses, issuePriorities, issueEfforts
#   below - that is the ONLY list of them
# - releaseNotes is written when the issue is CLOSED and is published in the release notes;
#   workingRemarks is what the work knows meanwhile and is cleared when the issue closes
# - THIS TOOL OWNS THE VERSION: the micro number is the count of closed issues, so ResolveIssue
#   and CloseIssue rewrite version.txt, versionCpp.cpp and the version line of README.rst
#
# The command line is 'exudev issue <verb>'; this module is its API.
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import datetime # for current date
import json
import os
import re
import io
import sys

#import os

#WHERE THE FILES ARE. Until this step every path was relative to the
#CURRENT directory, so the tracker could only be driven from tools/issueTracker/ - and a command
#line, which is started from wherever the user stands, was impossible. The two directories are
#module globals rather than constants inside the functions, so that the tests can point the
#module at a copy of the log instead of writing to the real one.
trackerDirectory = os.path.dirname(os.path.abspath(__file__))
repositoryRoot = os.path.normpath(os.path.join(trackerDirectory, '..', '..'))


def TrackerPath(name):
    """a file next to this tool: trackerlog.txt, its backup, the HTML overview, versionName.txt"""
    return os.path.join(trackerDirectory, name)


def RepositoryPath(*parts):
    """a file in the repository: version.txt, versionCpp.cpp, README.rst, the Markdown page"""
    return os.path.join(repositoryRoot, *parts)


absolute_path = os.path.dirname(__file__)
relative_path = '../generators'   #autoGenerateHelper moved to tools/generators
helperPath = os.path.join(absolute_path, relative_path)
sys.path.append(helperPath)

#the tracker writes Markdown, which needs no conversion helper

#the issues themselves live in issues/ as JSON; issueStore owns the
#files, this module owns the rules
sys.path.insert(0, trackerDirectory)
import issueStore                                                             # noqa: E402


#filename without file ending:
trackerFile = 'trackerlog'

trackerDateLine =    2 #line at which the date is given
trackerReleaseLine = 3 
trackerVersionLine = 4
#THE RELEASES ARE DATA. Until this step the baselines stood here as
#twelve module constants and the names in a dict beside them, so a minor bump was a hand edit of
#this file - and a MAJOR bump (1.11 -> 2.0) was not expressible at all: the major number was
#written as 1 in GetMajorMinorMicroVersion and the minor was the LENGTH of the list.
#
#They are releases.json now, and "exudev issue bump --minor | --major | --to 2.0" appends to it.
#What a baseline means: the count of CLOSED issues at which that release began, so that
#    micro = closedCount - baseline(current release)
#which is the definition the micro version has had since 2019.
releasesFile = 'releases.json'
releasesCache = None


def Releases(reload=False):
    """the releases, oldest first; each is {'version': '1.11', 'baseline': 2073, 'name': '...'}"""
    global releasesCache
    if releasesCache is None or reload:
        releasesCache = issueStore.ReadJson(TrackerPath(releasesFile))
    return releasesCache['releases']


def PlannedNames():
    """the names of releases that have not happened yet, so that a bump finds its own"""
    if releasesCache is None:
        Releases()
    return releasesCache.get('plannedNames', {})


def WriteReleases(data):
    """releases.json, written by the bump command and by nothing else"""
    global releasesCache
    with io.open(TrackerPath(releasesFile), 'w', encoding='utf-8', newline='\n') as file:
        file.write(json.dumps(data, indent=1, ensure_ascii=False) + '\n')
    releasesCache = None


def CurrentRelease():
    """the release the micro version counts within: the last one in the file"""
    return Releases()[-1]


def ReleaseOfClosedIndex(closedIndex):
    """which release the closedIndex-th closed issue belongs to (1-based over the whole history).
    The last release whose baseline it reaches: baseline 2073 means that the 2073rd closed issue
    is the FIRST of that release and carries micro 0."""
    found = Releases()[0]
    for release in Releases():
        if closedIndex >= release['baseline']:
            found = release
    return found


def VersionOfClosedIndex(closedIndex):
    """the version string the closedIndex-th closed issue produced, e.g. '1.10.160'. This is what
    ResolveIssue and CloseIssue store in the issue, and what
    tools/checkIssues.py recomputes and compares."""
    release = ReleaseOfClosedIndex(closedIndex)
    return release['version'] + '.' + str(closedIndex - release['baseline'])


def ReleaseName(version):
    """the jazz legend of a release ('1.11' -> 'McLaughlin'), or '' """
    for release in Releases():
        if release['version'] == version:
            return release['name']
    return PlannedNames().get(version, '')


# versionDev = '' #release (works in pip)
versionDev = '' #(development version, get with pip install exudyn --pre)

#the release names - jazz legends, alphabetically - moved into releases.json with the baselines
#; ReleaseName() reads them.
#https://www.britannica.com/topic/list-of-jazz-musicians-2030466

#+++++++++++++++++++++++++++++++++++++++++++++
#THE ISSUE TYPES, in one place (#2519). Before this there were three lists -
#the header of this file, the header of trackerlog.txt and what people actually typed - and all
#three disagreed: 39 distinct spellings in 2519 issues, 20 of them typos or singletons, and the
#same meaning under both 'NEW FEATURE' (used until #342) and 'EXTENSION' (used ever since).
#What each type answers is "what does this mean for a USER", which is why BUG and FIX are separate
#and why IMPROVEMENT exists.
issueTypes = {
    'BUG':         'something goes really wrong, in particular WRONG RESULTS; a user may not notice',
    'FIX':         'something goes wrong and says so - an exception, a crash, a file not written',
    'CHANGE':      'behaviour or interface changes; users have to be careful about this one',
    'EXTENSION':   'a new feature, parameter or flag; good for users and changes no behaviour',
    'IMPROVEMENT': 'the code gets better without the user seeing it: readability, cleanup, speed',
    'TESTING':     'a new or extended test',
    'DOCU':        'documentation, description or tutorial',
    'EXAMPLE':     'an example model',
    'CHECK':       'something to investigate or verify; it may turn into another issue',
    'IDEA':        'not yet a feature: how something COULD look. Becomes another issue, or is abandoned',
}

#THE STATUSES. WORK and TESTING stood in the old list and were never used once in 2519 issues,
#so they are gone; CLOSED is what was missing - without it an issue that was decided against had
#to be written as RESOLVED, which is untrue.
#
#CLOSED means EVERYTHING EXCEPT RESOLVED (maintainer 2026-09-21, D13): obsolete, won't fix,
#duplicate of #n, superseded, no longer applies, not reproducible, abandoned. The kind is named
#in the mandatory reason, NOT as a status of its own: the distinction is prose, and every extra
#status is another branch in every converter. A status such as ABANDONED
#would be wrong: one reason cannot stand for all of them.
issueStatuses = {
    'RAISED':   'open',
    'RESOLVED': 'done',
    'CLOSED':   'closed WITHOUT being done - obsolete, won\'t fix, duplicate, superseded, not '
                'reproducible, abandoned; the reason is mandatory and belongs in releaseNotes',
}

#Both count for the version number. The micro version is the count of CLOSED issues, not of
#resolved ones: if closing did not count, then closing an already-resolved issue would move
#version.txt BACKWARDS and a released version number would stop being reproducible from this file.
#Measured over the released history in fact 31 of the info document: 0 of 2,054 published version
#numbers moved when 15 issues changed to this status. The difference shows in the release notes
#instead - only RESOLVED is listed there as resolved, and a CLOSED issue is listed nowhere,
#neither as resolved nor as open.
closedStatuses = ['RESOLVED', 'CLOSED']

#types that are not announced as resolved issues: an idea that became a real issue would otherwise
#be reported twice
typesNotInReleaseNotes = ['IDEA']
#+++++++++++++++++++++++++++++++++++++++++++++

#THE EFFORT, in human working hours without AI assistance. It is a CLASSIFICATION, not an
#estimate anyone is held to: what it buys is the
#question "which open FIX is LOW", which 270 open issues cannot answer otherwise. Empty means
#not classified yet.
issueEfforts = {
    'LOW':    'within 2 hours',
    'MEDIUM': 'within 16 hours',
    'HIGH':   'within 40 hours',
    'HUGE':   'above 40 hours',
}

#THE PRIORITIES. The spellings had drifted to nine over 251 of the 2,567 issues - '', NO, NORMAL,
#high, med, low, HIGH, medium, LOW - and 180 of them were normalized by migrateSchema.py; they are
#enforced from here on. Empty is legal and means none, which is what 2,405 issues say: most issues
#have no priority, and saying so honestly is better than a default nobody chose. The size of the
#work is the 'effort' field above, which is the one meant for sorting the backlog.
issuePriorities = {
    'LOW':    'nice to have',
    'NORMAL': 'should be done',
    'HIGH':   'do this first',
}

#the fields of an issue are issueStore.issueFields; what stood here
#was the column order of trackerlog.txt with the indices that went with it - 'notes' was the last
#column, the status was padded to 8 characters, and every reader had to know both.


def ResolvedIssues2Version(resolvedIssues, totalResolvedIssues):
    """[minor, micro] of an issue that is resolvedIssues places below the newest closed one.

    Kept for callers that counted backwards from the current version; VersionOfClosedIndex() is
    the direct form and the one the stored version uses."""
    version = VersionOfClosedIndex(totalResolvedIssues - resolvedIssues)
    parts = version.split('.')
    return [int(parts[1]), int(parts[2])]


#++++++++++++++++++++++++++++++
#update all output files with new version, etc.
def UpdateFiles():
    UpdateDateAndVersion()
    WritePages()


#%%******************************************************************************************************
def ToMarkdown(s):
    """Every Markdown special character in issue text becomes literal text (#2545).

    The same rule as ToLatex above, for the format that replaces it:
    an author writes text into the tracker, not markup, and the writer knows the output format -
    so the writer escapes. The characters are different (a backtick opens code, an underscore or
    a star opens emphasis, a pipe splits a table cell, a '<' opens an HTML tag), the rule is not.

    The backslash goes FIRST, or it would escape the backslashes the other rules introduce."""
    s = s.replace(chr(92), chr(92) + chr(92))        #before everything else
    #the dollar is in this list because conf.py enables the dollarmath extension: a price
    #or a shell prompt in an issue would otherwise open math (found by checkMathMacros)
    for character in ['`', '*', '_', '[', ']', '<', '>', '#', '|', '$']:
        s = s.replace(character, chr(92) + character)
    return s


#%%******************************************************************************************************
def ToLatex(s):
    """Every LaTeX special character in issue text becomes literal text (#2544).

    trackerlog.tex is LaTeX, and its content is written by whoever raises an issue. Until
    2026-09-19 only _ # & were escaped, so a caret, a percent sign or a backslash command in any
    description was executed or aborted pdflatex - and both already happened: issue 2398 says
    "O(N^2) per step", and an issue that mentioned a backslash-input command stopped the build 629
    pages in, after six minutes.

    ESCAPING IS TOTAL, by the maintainer's decision of 2026-09-19. A handful of issues from 2016-2020
    contain deliberate markup ({\bf ATTENTION} in #0139, math in #0273) and now render as the
    characters they are. That is the trade: the documentation build can never again be broken by
    what someone typed into the tracker, and nobody has to think about LaTeX to raise an issue.

    The backslash goes FIRST, or it would escape the backslashes the other rules introduce."""
    s = s.replace(chr(92), chr(92) + 'textbackslash{}')      #before everything else
    s = s.replace('{', chr(92) + '{')
    s = s.replace('}', chr(92) + '}')
    s = s.replace('_', chr(92) + '_')
    s = s.replace('#', chr(92) + '#')
    s = s.replace('&', chr(92) + '&')
    s = s.replace('$', chr(92) + '$')
    s = s.replace('%', chr(92) + '%')
    s = s.replace('^', chr(92) + 'textasciicircum{}')
    s = s.replace('~', chr(92) + 'textasciitilde{}')

    return s

#%%******************************************************************************************************
def GetDateStr():
    """
    Compute a string from current date. Adds leading zeros if necessary.

    Returns
    -------
    dateStr : STRING
    
    Examples
    -------
    >>> GetDateStr()
    '2020-08-25'
    """
    now=datetime.datetime.now()
    monthZero = '' #add leading zero for month
    dayZero = ''   #add leading zero for day
    if now.month < 10:
        monthZero = '0'
    if now.day < 10:
        dayZero = '0'
        
    dateStr = str(now.year) + '-' + monthZero + str(now.month) + '-' + dayZero + str(now.day)

    return dateStr

#%%******************************************************************************************************
def GetDateTimeStr():
    """
    Compute a string from current date and time. Adds leading zeros if necessary.

    Returns
    -------
    dateStr : STRING
    
    Examples
    -------
    >>> GetDateStr()
    '2020-08-25'
    """
    now=datetime.datetime.now()
    minuteZero = ''   #add leading zero
    hourZero = ''   #add leading zero
    if now.minute < 10:
        minuteZero = '0'
    if now.hour < 10:
        hourZero = '0'
        
    dateStr = GetDateStr() + ' ' + hourZero + str(now.hour) + ':' + minuteZero + str(now.minute)

    return dateStr


#%%******************************************************************************************************
#THE DATA LAYER. The issues live in tools/issueTracker/issues/ as one
#JSON file per open or recently closed issue and one file per year for the older ones; issueStore
#owns the files, this module owns the rules. What went with trackerlog.txt: the ',' escaping, the
#column padding, the whole-file rewrite on every change, and IssueTrackerBackup() - a copy of the
#file before every write, which git has done better for seven years.
def NumberOfIssues():
    """the number the next issue would get; issue numbers are consecutive and never reused"""
    return issueStore.NextNumber()


def GetIssue(number):
    """one issue as a dictionary, or an empty one - the callers check for that"""
    issue = issueStore.Load(number)
    if issue is None:
        print('Issue: invalid number!')
        return {}
    return issue


def GetIssues():
    """every issue, in issue order"""
    return issueStore.LoadAll()


def ModifyDictIssue(issueDict):
    """write one issue back; a closed issue moves from open/ to closed/ on the way"""
    if 'number' not in issueDict:
        print('ERROR: issue \'number\' needed in ModifyDictIssue(...)')
        return
    issueStore.Save(issueDict)


def MetaPath():
    return os.path.join(issueStore.storeDirectory, 'meta.json')


def ReadMeta():
    """what the header lines of trackerlog.txt used to say: the date of the last change and the
    version it produced. It is data about the tracker, not about an issue, so it is one small
    file beside them rather than a field in every one."""
    if os.path.isfile(MetaPath()):
        return issueStore.ReadJson(MetaPath())
    return {'lastChange': GetDateStr(), 'release': '', 'version': ''}


def WriteMeta():
    [release, version] = GetReleaseAndVersionString()
    issueStore.WriteJson(MetaPath(), {'schemaVersion': issueStore.schemaVersion,
                                      'lastChange': GetDateStr(),
                                      'release': release,
                                      'version': version})


#%%******************************************************************************************************
def ConvertToCSV():
    """all issues as a .csv, for a spreadsheet; one row per issue, the fields of the store"""
    rows = [issueStore.issueFields]
    for issue in GetIssues():
        rows += [[str(issue[name]).replace('"', '""') for name in issueStore.issueFields]]

    with io.open(TrackerPath(trackerFile+'.csv'), 'w', encoding='utf-8', newline='') as file:
        for row in rows:
            file.write(','.join('"' + value + '"' for value in row) + '\n')


def GetReleaseAndVersionString(): #convert all issues to a .html file
    [major, minor, micro] = GetMajorMinorMicroVersion()
    return [str(major)+'.'+str(minor),str(micro)]
    # fileRead=open(trackerFile+'.txt','r', encoding='utf-8') 
    # fileLines = fileRead.readlines()
    # fileRead.close()

    # numberOfResolved = 0
    # #count resolved issues
    # for line in fileLines:
        
    #     items = line.split(',')
    #     if len(items) == numberOfItems: #only count in valid lines; error will be reported lateron
    #         if (items[indexStatus].find('RESOLVED') != -1):
    #             numberOfResolved += 1
    
    # #print('numberOfResolved=',numberOfResolved)
    # release = '1.' + str((len(versionResolved)-2))
    # #print('release=', release)
    # # release = float(fileLines[trackerReleaseLine].split('=')[1])
    # version = numberOfResolved-versionResolved[-1] 
    # #print('version=', version)
    
    # return [release, version]



def GetMajorMinorMicroVersion():
    """the version, derived from the count of CLOSED issues - resolved and closed-not-resolved
    alike (#2519). The count comes from the store, which adds the closed files it can see to the
    closedCount each archive file states; an archive that is missing is reported by
    issueStore.CheckStore() rather than silently lowering the version.

    The major number comes from the current release; it was written
    as 1 here, which is why 2.0 could not be expressed."""
    release = CurrentRelease()
    [major, minor] = [int(part) for part in release['version'].split('.')]

    #max(0, ...): a release begins at the count of the issue that will carry micro 0, so between
    #the bump and the first closed issue of the new release the difference is -1. That release has
    #closed nothing yet, and X.Y.0 is what that means
    micro = max(0, issueStore.ClosedCount() - release['baseline'])

    return [major, minor, micro]


#%%******************************************************************************************************
def VersionString():
    [release, version] = GetReleaseAndVersionString()
    return str(release)+'.'+str(version)+versionDev 

#%%******************************************************************************************************
#write date and version to tracker file; also update version in src/Autogenerated/version.h
def UpdateDateAndVersion(updateVersion = True):
    #the date and the version stood in the header lines of trackerlog.txt; they are meta.json
    #beside the issues
    WriteMeta()

    #update version in Python module versionPybind.h ==> this is shown in the module with python command version()
    #no main/ level since the flatten; version.txt is at the
    #repository ROOT, and versionName.txt sits next to this tool,
    #when docs/theDoc/ was deleted with the LaTeX build (decision D8)
    #versionFile = directoryString + 'version.h' #not used anymore
    cppVersionFile = RepositoryPath('src', 'Autogenerated', 'versionCpp.cpp')
    texVersionFile = RepositoryPath('version.txt')
    #versionName.txt sits next to this tool, when
    #docs/theDoc/ was deleted with the LaTeX build (decision D8)
    texVersionNameFile = TrackerPath('versionName.txt')
    #hand-written, except for its version line
    readmeFile = RepositoryPath('README.rst')

    #pyVersionFile = '..\\..\\src\\pythonGenerator\\exudynVersion.py'
    #[release,version] = GetReleaseAndVersion()
    releaseVersionDev = VersionString()
    print('current version=',releaseVersionDev)

#    file=open(versionFile,'w')  #clear file by one write access
#    file.write('// AUTO:  ++++++++++++++++++++++\n')
#    file.write('// AUTO:  version info automatically generated by tracker; generated by Johannes Gerstmayr\n')
#    file.write('// AUTO:  last modified = '+ GetDateStr() + '\n')
#    file.write('// AUTO:  ++++++++++++++++++++++\n')
#    versionString = 'm.attr("__version__") = "' + str(release)+'.'+str(version) + '";\n\n'
#    file.write(versionString)
#    file.close()

    #++++++++++++++++
    if updateVersion:
        #update version in C++ module versionPybind.h ==> this is shown in the module with python command version()
        file=open(cppVersionFile,'w', encoding='utf-8')  #clear file by one write access
        file.write('// AUTO:  ++++++++++++++++++++++\n')
        file.write('// AUTO:  version info automatically generated by tracker; generated by Johannes Gerstmayr\n')
        file.write('// AUTO:  last modified = '+ GetDateStr() + '\n')
        file.write('// AUTO:  ++++++++++++++++++++++\n')
        #file.write('#pragma once\n\n') #not needed for .cpp file and not compatible with gcc
        
        #OLD, fails in CMD compilation: versionString = 'namespace EXUstd {\nconst char* exudynVersion = "' + str(release)+'.'+str(version) + '";\n}\n'
        versionString = 'namespace EXUstd {\n const char* exudynVersion = "' + releaseVersionDev + '";\n}\n'
    
        file.write(versionString)
        file.close()
    
        #++++++++++++++++
        #update version in .tex documentation:
        file=open(texVersionFile,'w', encoding='utf-8')  #clear file by one write access
        versionString = releaseVersionDev 
        file.write(versionString)
        file.close()

        [release, version_] = GetReleaseAndVersionString()
        versionNameString = '('+ReleaseName(str(release))+')'

        file=open(texVersionNameFile,'w', encoding='utf-8')  #clear file by one write access
        file.write(versionNameString)
        file.close()

        #++++++++++++++++
        #README.rst is the GitHub and PyPI landing page and is hand-written; it used to be
        #generated from gettingStarted.tex, and the only part of it that
        #has to follow the version is this one line, so it is stamped here rather than by a
        #second mechanism. A README without that line is left alone.
        readmeVersionLine = '+  Exudyn version = '
        readmeText = io.open(readmeFile, encoding='utf-8', newline='').read()
        readmeLines = readmeText.split('\n')
        for (i, line) in enumerate(readmeLines):
            if line.startswith(readmeVersionLine):
                ending = '\r' if line.endswith('\r') else ''
                readmeLines[i] = (readmeVersionLine + releaseVersionDev + ' ' +
                                  versionNameString + ending)
                io.open(readmeFile, 'w', encoding='utf-8', newline='').write('\n'.join(readmeLines))
                break
        else:
            print('WARNING: no version line found in ' + readmeFile)

    
        #++++++++++++++++
        #update version in .py files:
        # this file is not changed any more, but reads version from theDoc version.txt
        # file=open(pyVersionFile,'w')  #clear file by one write access
        # file.write('# version info automatically generated by tracker; generated by Johannes Gerstmayr\n')
        # file.write('# last modified = '+ GetDateStr() + '\n')
        # versionString = 'exudynVersionString = "' + releaseVersionDev + '"\n\n'
        # file.write(versionString)
        # file.close()
    
        #++++++++++++++++
        #batVersionFile
        # file=open(batVersionFile,'w')  #clear file by one write access
        # versionString = releaseVersionDev + '\n'
        # file.write(versionString)
        # file.close()

#%%******************************************************************************************************
#the escaping of the tracker's free-text fields is ToMarkdown above (#2545); the RST escaper
#that stood here went with docs/RST/trackerlog.rst


#%%******************************************************************************************************
def ConvertToHTML():
    """tools/issueTracker/trackerlog.html: the overview a maintainer scrolls through, open issues
    first and coloured by priority. Written from the store; it was a
    loop over the lines of trackerlog.txt with column indices, and the indices had to be kept in
    step with the columns by hand."""
    issues = GetIssues()
    openIssues = [issue for issue in issues if issue['status'] not in closedStatuses]
    closedIssues = [issue for issue in issues if issue['status'] in closedStatuses]
    [major, minor, micro] = GetMajorMinorMicroVersion()

    #the columns of the table, and the field behind each one; author, deadline, file and line are
    #left out on purpose - the table is for finding an issue, not for reading it
    columns = [('nr', 'number'), ('issue', 'title'), ('status', 'status'),
               ('description', 'description'), ('type', 'type'), ('pri', 'priority'),
               ('effort', 'effort'), ('raised', 'dateRaised'), ('resolved', 'dateResolved'),
               ('res. by', 'resolvedAuthor'), ('release notes', 'releaseNotes'),
               ('working remarks', 'workingRemarks')]

    colourOfPriority = {'HIGH': '#FF8080', 'NORMAL': '#EEAA99', 'LOW': '#E0E088'}

    def Escape(value):
        return (str(value).replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;'))

    def Row(issue, colour):
        line = '  <tr style = "background-color:' + colour + '">\n'
        for (heading, name) in columns:
            line += '    <td>' + Escape(issue[name]) + '</td>\n'
        return line + '  </tr>\n'

    with io.open(TrackerPath(trackerFile+'.html'), 'w', encoding='utf-8') as file:
        file.write('<!DOCTYPE html>\n<html>\n<head>\n<style>\n')
        file.write('table, th, td{\n  border: 1px solid black;\n  padding: 1px;\n}\n')
        file.write('table{\n  border - spacing: 5px;\n}\n')
        file.write('</style>\n</head>\n<body>\n\n<h2>ISSUE Tracker</h2>\n\n')

        file.write('Number of issues = ' + str(len(issues)) + ', \n')
        file.write('Number of resolved issues = ' + str(len(closedIssues))
                   + ' (' + str(micro) + ' in current minor version), \n')
        file.write('Exudyn version = ' + VersionString() + ', \n')
        file.write('last change = ' + ReadMeta()['lastChange'] + '\n\n')

        file.write('<table style = "width:100%">\n')
        file.write('<tr style = "background-color:#AAAAAA">\n')
        for (heading, name) in columns:
            file.write('<th>' + heading + '</th>\n')
        file.write('</tr>\n')

        #open issues first and newest first: that is the order they are worked in
        for issue in reversed(openIssues):
            colour = colourOfPriority.get(issue['priority'].strip().upper(), '#EEB066')
            file.write(Row(issue, colour))
        for issue in reversed(closedIssues):
            file.write(Row(issue, '#BBFFBB' if issue['status'] == 'RESOLVED' else '#DDDDDD'))

        file.write('</table>\n\n</body>\n</html>\n')


#THE ENTRY OF ONE ISSUE (#2637). The changelog and the tracker
#page are two renderings of one store and printed the same issue in two shapes: the changelog
#as '- **1.12.50** `FIX` title (#2634) - raised by X' with no dates, the tracker page as
#'- Version 1.12.49: resolved Issue 2633: title (improvement)' with no type badge. The
#maintainer asked for one format (2026-09-24): the type as the changelog writes it, the
#priority and the effort in the same style right after it, 'raised by' and 'resolved by' the
#same again, and the sub-list of the tracker for the rest. Both pages call these three.
def Colour(cssClass, text):
    """a colour of docs/_static/custom.css, as the inline HTML that MyST passes through"""
    return '<span class="' + cssClass + '">' + text + '</span>'


def IssueBadges(issue):
    """the classification of an issue in one style: the type, the priority, the effort and
    the two authors, each in backticks. An empty field is not printed, and JG is not printed
    either - the pages say once that the author is omitted when it was Johannes Gerstmayr.

    The effort carries its word - `LOW EFF` - and the priority does not, because LOW and HIGH
    are values of BOTH and two bare badges in one line cannot be told apart (#2600)."""
    badges = []
    kind = issue['type'].strip()
    if kind != '':
        badges.append(Colour('textred', '`BUG`') if kind == 'BUG' else '`' + kind + '`')
    if issue['priority'].strip() != '':
        #an OPEN issue keeps the colour of its priority,
        #which is the only thing a reader of 270 open issues sorts them by
        badge = '`' + issue['priority'].strip() + '`'
        if issue['status'] == 'RAISED':
            badge = Colour({'HIGH': 'textred', 'NORMAL': 'textorange',
                            'LOW': 'textblue'}.get(issue['priority'].strip().upper(),
                                                   'boldblue'), badge)
        badges.append(badge)
    if issue['effort'].strip() != '':
        badges.append('`' + issue['effort'].strip() + ' EFF`')
    for (field, what) in [('author', 'raised by'), ('resolvedAuthor', 'resolved by')]:
        name = str(issue.get(field, '')).strip()
        if name not in ['', 'JG']:
            badges.append('`' + what + ': ' + ToMarkdown(name) + '`')
    return ' '.join(badges)


def IssueDetails(issue):
    """what stands under the entry of an issue: its text and its dates, as the tracker page
    has always printed them. A CLOSED issue shows what its resolution says and an OPEN one
    what the work on it knows so far; the two are different fields."""
    details = '  - description: ' + ToMarkdown(issue['description']) + '\n'
    if issue['releaseNotes'].strip(' ') != '':
        details += '  - **notes:** ' + ToMarkdown(issue['releaseNotes']) + '\n'
    if issue['workingRemarks'].strip(' ') != '':
        details += '  - **remarks:** ' + ToMarkdown(issue['workingRemarks']) + '\n'

    details += '  - '
    if issue['dateResolved'].strip() != '':
        details += 'date resolved: **' + issue['dateResolved'].strip() + '**, '
    details += 'date raised: ' + issue['dateRaised'].strip() + '\n'
    return details


def IssueEntry(issue, version='', details=True):
    """one issue as both pages print it: the version it produced, the badges, the title and
    the issue number, and the sub-list below it.

    Args:
        issue: the issue dictionary
        version: the version it resolved into, printed in front; empty for an open issue
        details: False prints the headline alone

    Returns:
        the Markdown of the entry, ending in a newline
    """
    line = '- '
    if str(version).strip() != '':
        line += '**' + str(version).strip() + '** '
    badges = IssueBadges(issue)
    if badges != '':
        line += badges + ' '
    line += (ToMarkdown(issue['title'].strip(' ')) + ' (#' + str(issue['number']) + ')\n')
    return line + (IssueDetails(issue) if details else '')


def BadgeLegend():
    """the one sentence that says what the badges of an entry mean, for both pages"""
    return ('Every entry carries the **type** of the issue, then its **priority** and its '
            '**effort** as badges (' + ', '.join('`' + name + ' EFF` is ' + issueEfforts[name]
                                                 for name in issueEfforts) + '), then who '
            'raised it and who resolved it - both omitted when that was Johannes Gerstmayr '
            '(JG).\n\n')


def ConvertToMarkdown():
    """write what MarkdownText() renders to docs/generated/trackerlog.md"""
    markdownFile = RepositoryPath('docs', 'generated', trackerFile + '.md')
    os.makedirs(os.path.dirname(markdownFile), exist_ok=True)
    with io.open(markdownFile, 'w', encoding='utf-8', newline='\n') as file:
        file.write(MarkdownText())


def MarkdownText():
    """docs/generated/trackerlog.md: the resolved issues per release, the open issues and the
    known bugs. Markdown; it wrote docs/theDoc/trackerlog.tex and
    docs/RST/trackerlog.rst until then, and the colours of the open issues, which were RST roles,
    are the CSS classes of docs/_static/custom.css written as inline HTML."""
    [releaseString,versionString] = GetReleaseAndVersionString()
    [majorCurrent,microCurrent,minorCurrent] = GetMajorMinorMicroVersion()

    releaseVersionDev = VersionString()

    numberOfRaised = NumberOfIssues()
    numberOfResolved = minorCurrent
    lastChangeDate = ReadMeta()['lastChange']
    totalResolved = issueStore.ClosedCount()        #the count the current micro counts from


    text = ('<!-- GENERATED by tools/issueTracker/issueTracker.py from trackerlog.txt '
            '- do not edit -->\n'
            '(sec-issuetracker)=\n'
            '# Issue tracker\n\n')
    text += ('This section contains resolved issues per release and known bugs. Use this '
             'information to understand changes compared to previous versions. '
             'The extension `.dev1` is not added in the issues list (e.g., 1.2.2.dev1==1.2.2), '
             'as it only marks versions that will not be available in pypi with standard pip '
             'install, but only with the `--pre` option or by specifying the exact version name, '
             'see versions on <https://pypi.org/project/exudyn/>.\n\n')
    #one entry format with the changelog (#2637)
    text += BadgeLegend()
    text += ('General information on current version:\n\n'
             '- Exudyn version = ' + releaseVersionDev + '\n'
             '- last change = ' + lastChangeDate + '\n'
             '- Number of issues = ' + str(numberOfRaised) + '\n'
             '- Number of resolved issues = ' + str(totalResolved)
             + ' (' + str(numberOfResolved) + ' in current version)\n\n')

    #the current release is in CHANGELOG.md, in full and with its release notes; this page
    #carries everything before it, so that no issue is printed twice
    text += ('## Resolved issues and resolved bugs before version ' + releaseString + '\n\n'
             'The following list contains the issues which have been **RESOLVED** in the '
             'according version. The issues of the current release, ' + releaseString + ', are '
             'in the {ref}`changelog <sec-changelog>`.\n\n')

    resolved = ''
    openIssues = ''
    bugs = ''

    issueList = GetIssues()
    issueListSorted = sorted(issueList, key = lambda i: i['dateResolved'])
    resolvedCnt = 0
    previousRelease = (majorCurrent,microCurrent)
    vIssueRelease = 1 #for now

    for issue in reversed(issueListSorted):
        #THE VERSION COMES FROM THE ISSUE. It was recomputed here on every run, which is what
        #made a published number depend on a sort by dateResolved. The
        #recomputation is the fallback for an issue that carries no stamp yet, and
        #tools/checkIssues.py compares the two for every closed issue.
        stamped = str(issue.get('resolvedInVersion', '')).strip()
        if stamped.count('.') == 2:
            parts = stamped.split('.')
            [vIssueRelease, vIssueMinor, vIssueMicro] = [int(part) for part in parts]
        else:
            [vIssueMinor, vIssueMicro] = ResolvedIssues2Version(resolvedCnt, totalResolved)
        rNew = (vIssueRelease, vIssueMinor) if vIssueMinor >= 0 else (0,1)

        if (rNew[0] < previousRelease[0] or
            (rNew[0] == previousRelease[0] and rNew[1] < previousRelease[1]) ):
            resolved += '\n### Version '+str(rNew[0])+'.'+str(rNew[1])+'\n\n'
            previousRelease = rNew


        inCurrentRelease = (rNew == (majorCurrent, microCurrent))
        if (issue['status'] == 'RESOLVED' and issue['type'] not in typesNotInReleaseNotes
                and not inCurrentRelease):
            resolved += IssueEntry(issue, str(rNew[0]) + '.' + str(rNew[1]) + '.'
                                   + str(vIssueMicro))
        elif issue['status'] == 'RAISED' and issue['type'] == 'BUG':
            bugs += IssueEntry(issue)
        elif issue['status'] == 'RAISED':       #a CLOSED issue is not an open one
            openIssues += IssueEntry(issue)

        #CLOSED, not resolved: the version a past issue is listed under is derived from this
        #counter, so counting only RESOLVED would renumber every historical entry as soon as
        #one issue is closed. A closed issue keeps its place and is simply not printed
        if issue['status'] in closedStatuses:
            resolvedCnt += 1

    text += resolved
    text += '\n## Open issues\n\n' + openIssues
    text += '\n## Known bugs\n\n' + bugs

    return text

#%%******************************************************************************************************
#%%******************************************************************************************************
#%%******************************************************************************************************
#THE CHANGELOG. CHANGELOG.md in the repository root is where a user of a
#released package looks, and it is a RENDERING of the issues - not a second place where changes
#are written down. That is the point of storing 'resolvedInVersion': the version of
#every resolved issue is data now, so this file only has to group and print.
#
#WHAT IT IS NOT: docs/generated/trackerlog.md, which is the tracker itself - every resolved issue
#with its description, its dates and its author, plus the OPEN issues and the known bugs, 119,000
#words of it. A changelog that repeated all of that would double a megabyte for no new fact
#(rule 10 of CLAUDE.md). So the current release is printed here and the earlier ones there -
#in the SAME entry format (#2637), which is IssueEntry().
changelogFile = 'CHANGELOG.md'
changelogDetailedReleases = 1   #how many releases are printed with the sub-list of an entry -
                                #the description, the notes and the dates - and not the headline
                                #alone
changelogReleases = 1           #how many releases are printed AT ALL (#2599):
                                #the earlier ones are on the tracker page in full, and
                                #2370 of the 2440 lines of this file were a shorter rendering of
                                #what that page says with the author, the description and both
                                #dates. One list, in one place.


def ReleaseOfVersionString(version):
    """'1.10.160' -> '1.10'"""
    parts = str(version).split('.')
    return '.'.join(parts[:2]) if len(parts) == 3 else ''


def VersionSortKey(version):
    """so that 1.10.9 comes before 1.10.10, which a string sort gets wrong"""
    try:
        return [int(part) for part in str(version).split('.')]
    except ValueError:
        return [0, 0, 0]


def ChangelogText():
    """CHANGELOG.md: every RESOLVED issue, newest first, grouped by release.

    An issue that was CLOSED without being resolved appears nowhere - it changed nothing for a
    user - and neither does an IDEA (typesNotInReleaseNotes), which becomes a real issue or is
    closed. That is the same rule the tracker page follows."""
    issues = [issue for issue in GetIssues()
              if issue['status'] == 'RESOLVED'
              and issue['type'] not in typesNotInReleaseNotes
              and str(issue.get('resolvedInVersion', '')).count('.') == 2]
    issues.sort(key=lambda issue: VersionSortKey(issue['resolvedInVersion']), reverse=True)

    #the releases in the order they are printed, and what stands under each of them
    order = []
    perRelease = {}
    for issue in issues:
        release = ReleaseOfVersionString(issue['resolvedInVersion'])
        if release not in perRelease:
            perRelease[release] = []
            order.append(release)
        perRelease[release].append(issue)

    text = ('<!-- GENERATED by tools/issueTracker/issueTracker.py from '
            'tools/issueTracker/issues/ - do not edit -->\n'
            '(sec-changelog)=\n'
            '# Changelog\n\n')
    text += ('Every **resolved issue** of Exudyn, newest first, with the version it produced. '
             'The micro version *is* the count of closed issues, so each line below is one '
             'version number: 1.10.160 is the 160th issue closed in release 1.10. An issue '
             'that was closed without being resolved is in no list: it changed nothing - '
             'which is why a release can span more version numbers than it has lines here.'
             '\n\n')
    #one entry format with the tracker page (#2637)
    text += BadgeLegend()
    text += ('This file is generated; it is written by the tracker whenever an issue closes.'
             '\n\n'
             '**Only the current release is below.** The issues resolved before it, in the '
             'same form, are in the {ref}`issue tracker <sec-issuetracker>` of the '
             'documentation, which also lists the open issues and the known bugs; the '
             'table that follows is the whole history at a glance.\n\n')

    #the overview: one row per release
    text += '| release | name | resolved issues | highest version |\n|---|---|---|---|\n'
    for release in order:
        name = ReleaseName(release)
        highest = perRelease[release][0]['resolvedInVersion']
        text += ('| ' + release + ' | ' + (name if name else '-') + ' | '
                 + str(len(perRelease[release])) + ' | ' + highest + ' |\n')
    text += '\n'

    for (position, release) in enumerate(order[:changelogReleases]):
        name = ReleaseName(release)
        text += ('## Version ' + release + (' - ' + name if name else '')
                 + (' (current)' if position == 0 else '') + '\n\n')

        detailed = position < changelogDetailedReleases
        for issue in perRelease[release]:
            text += IssueEntry(issue, issue['resolvedInVersion'], details=detailed)
        text += '\n'

    return text


def ConvertToChangelog():
    """write CHANGELOG.md in the repository root"""
    with io.open(RepositoryPath(changelogFile), 'w', encoding='utf-8', newline='\n') as file:
        file.write(ChangelogText())


def WritePages():
    """the three pages the tracker owns: the local HTML overview, the tracker page of the
    documentation and the changelog. Every verb that changes an issue writes all three, because
    all three are renderings of the same issues."""
    ConvertToHTML()
    ConvertToMarkdown()
    ConvertToChangelog()


#%%******************************************************************************************************
def CheckedEnumValue(fieldName, value, allowed):
    """one spelling per value, or a message that lists the ones there are.
    An empty value is legal for every enum field of an issue and means "not classified";
    saying so is better than a default nobody chose."""
    value = value.strip().upper()
    if value != '' and value not in allowed:
        raise ValueError(fieldName + ': unknown value "' + value + '". Use one of:\n  '
                         + '\n  '.join(name.ljust(8) + ' ' + allowed[name] for name in allowed)
                         + '\n  (or leave it empty)')
    return value


#%%******************************************************************************************************
#use this to completely define a new issue
def RaiseIssueDict(issueDict): #raise a new issue into list (append to end of list)


    if 'number' in issueDict: print('WARNING: issue number "' + issueDict['number'] + '" ignored; issue added to end of list')
    if not('dateRaised' in issueDict): issueDict['dateRaised'] = GetDateStr()


    issueDict['status'] = 'RAISED'
    issueDict['dateRaised'] = GetDateStr()

    #the type is checked HERE, at the one place where an issue is born. Without this the list of
    #types is only a comment, which is how 39 spellings got into 2519 issues (#2519)
    issueDict['type'] = issueDict.get('type', '').strip().upper()
    if issueDict['type'] not in issueTypes:
        raise ValueError('RaiseIssue: unknown issue type "' + issueDict['type']
                         + '". Use one of:\n  '
                         + '\n  '.join(name.ljust(12) + ' ' + issueTypes[name]
                                       for name in issueTypes))

    #the effort and the priority are checked at the same place and for the same reason; both may
    #be empty, which means "not classified"
    issueDict['effort'] = CheckedEnumValue('effort', issueDict.get('effort', ''), issueEfforts)
    issueDict['priority'] = CheckedEnumValue('priority', issueDict.get('priority', ''),
                                             issuePriorities)

    #the release note is written when the issue is CLOSED, not when it is raised: it is what the
    #release notes publish. What is known while the work goes on belongs in workingRemarks.
    if issueDict.get('releaseNotes', '').strip() != '':
        raise ValueError('RaiseIssue: releaseNotes is written by ResolveIssue or CloseIssue; '
                         'put what you know now into workingRemarks')
    issueDict['releaseNotes'] = ''

    number = NumberOfIssues()
    issueDict['number'] = number
    issueStore.Save(issueDict)

    UpdateDateAndVersion(updateVersion=False) #do not change version files!
    WritePages()

    #report the number that was actually assigned, and return it: it is needed for the commit
    #message and the documentation, and reconstructing it by hand afterwards gets it wrong
    print('issue raised: #' + str(number) + ' "' + str(issueDict['title']).strip() + '"')

    return number

#%%******************************************************************************************************
#modify an existing issue
#use this to overwrite ONE field of an issue
def ChangeIssue(issueNumber, key, value, force=False): #raise a new issue into list (append to end of list)
    """Overwrite ONE field of an issue.

    Raising, resolving and closing an issue are ordinary work; changing a field of an issue that
    is already CLOSED is not (maintainer, 2026-09-21). Such an issue has been published - its
    text stands in the release notes of a released version - and two of its fields decide the
    version number itself. So a closed issue is refused here unless the caller says force=True,
    and the fields the tracker owns are refused outright."""

    if not((issueNumber >= 0) and (issueNumber < NumberOfIssues())):
        print('ChangeIssue: invalid number! Nothing done')
        return

    d = GetIssue(issueNumber)

    if key not in d:
        print('ChangeIssue: key "' + key + '" not available!')
        return

    #THE FIELDS THE TRACKER OWNS. 'status' moves an issue between open/ and closed/ and with it
    #the micro version; 'number' is the file name; the two dates are written where they happen.
    #They are set by RaiseIssue, ResolveIssue and CloseIssue on the issue itself, never here.
    if key in ['number', 'status', 'dateRaised', 'dateResolved']:
        raise ValueError('ChangeIssue: "' + key + '" is written by the tracker (RaiseIssue, '
                         'ResolveIssue, CloseIssue), not by a field edit')

    if d['status'] in closedStatuses and not force:
        raise ValueError(
            'ChangeIssue: issue ' + str(issueNumber) + ' is ' + d['status'] + ' since '
            + str(d['dateResolved']).strip() + ' - changing a field of a CLOSED issue changes '
            'what has already been published in the release notes of a released version.\n'
            '  If that is what you want, say so: force=True, or "--force" on the command line.\n'
            '  To record something new about it, raise a new issue instead.')

    #this function writes any field, which is why the enums have to be checked here as well as in
    #RaiseIssueDict - otherwise the one list of values is a comment again
    if key == 'effort':
        value = CheckedEnumValue('effort', value, issueEfforts)
    elif key == 'priority':
        value = CheckedEnumValue('priority', value, issuePriorities)
    elif key == 'type':
        value = value.strip().upper()
        if value not in issueTypes:
            raise ValueError('ChangeIssue: unknown issue type "' + value + '"')
    elif key == 'releaseNotes' and d['status'] not in closedStatuses:
        raise ValueError('ChangeIssue: releaseNotes belongs to a CLOSED issue; while it is open, '
                         'write workingRemarks (RemarkIssue)')

    #a text field is REPLACED here, and the text somebody else wrote is gone; the two verbs that
    #add instead of overwrite are worth naming at the moment it happens
    if key in ['description', 'workingRemarks', 'releaseNotes'] and d[key].strip() != '':
        print('WARNING: the ' + key + ' of #' + str(issueNumber) + ' is REPLACED, not extended'
              + ('; ExtendIssue() and RemarkIssue() append' if key != 'releaseNotes' else ''))
        print('  previous value: ' + d[key].strip()[:300])

    d[key] = value

    ModifyDictIssue(d)

    print('new issue:')
    print(d)
    
    UpdateDateAndVersion()
    WritePages()

#%%******************************************************************************************************
#%%******************************************************************************************************
#use this when the analysis of an issue turns up more than the issue says
def ExtendIssue(issueNumber, text, author='JG'):
    """Append a dated paragraph to the description of an OPEN issue.

    The first analysis of a problem regularly turns up more than the person who raised it knew,
    and that belongs with the issue: not in a second issue, and not by overwriting a description
    somebody else wrote. So this only ever appends.

    It touches nothing else - not the status, not the type, and above all not the version, which
    is derived from the count of closed issues. A closed issue is not extended: reopen it or raise
    a new one, because its text has already been published in the release notes."""

    if not((issueNumber >= 0) and (issueNumber < NumberOfIssues())):
        print('ExtendIssue: invalid number! Nothing done')
        return None

    if text.strip() == '':
        raise ValueError('ExtendIssue: nothing to add')

    d = GetIssue(issueNumber)
    if d['status'] in closedStatuses:
        raise ValueError('ExtendIssue: issue ' + str(issueNumber) + ' is ' + d['status']
                         + '; a closed issue is not extended - reopen it or raise a new one')

    #the date says which part of the description is the later analysis
    d['description'] = (d['description'].rstrip()
                        + ' [' + GetDateStr() + ', ' + author + ']: ' + text.strip())

    ModifyDictIssue(d)

    print('issue extended: #' + str(issueNumber) + ' "' + str(d['title']).strip() + '"')

    UpdateDateAndVersion()
    WritePages()
    return issueNumber


#%%******************************************************************************************************
#use this to record what is known while the work on an issue goes on
def RemarkIssue(issueNumber, text, author='JG', replace=False):
    """Write the working remarks of an OPEN issue.

    This is the scratchpad of an issue: "duplicate of #2134", "marked for deprecation", "check
    whether this still happens", "part A solved, B open". It is worth having while the issue is
    open and worthless once it is closed, which is exactly what separates it from the release
    note - so ResolveIssue and CloseIssue clear it, and it is never published.

    Appends by default, because the previous remark is usually still true; replace=True overwrites
    it. Passing an empty text with replace=True clears the field."""

    if not((issueNumber >= 0) and (issueNumber < NumberOfIssues())):
        print('RemarkIssue: invalid number! Nothing done')
        return None

    d = GetIssue(issueNumber)
    if d['status'] in closedStatuses:
        raise ValueError('RemarkIssue: issue ' + str(issueNumber) + ' is ' + d['status']
                         + '; the working remarks of a closed issue are gone by design')

    if text.strip() == '' and not replace:
        raise ValueError('RemarkIssue: nothing to add (use replace=True to clear the field)')

    if replace or d['workingRemarks'].strip() == '':
        d['workingRemarks'] = text.strip()
    else:
        d['workingRemarks'] = d['workingRemarks'].rstrip() + '; ' + text.strip()

    ModifyDictIssue(d)

    print('working remarks of #' + str(issueNumber) + ': ' + d['workingRemarks'])

    UpdateDateAndVersion()
    WritePages()
    return issueNumber


#%%******************************************************************************************************
def NextReleaseVersion(kind):
    """'minor': 1.11 -> 1.12. 'major': 1.11 -> 2.0. Nothing else is a release step."""
    [major, minor] = [int(part) for part in CurrentRelease()['version'].split('.')]
    if kind == 'major':
        return str(major + 1) + '.0'
    if kind == 'minor':
        return str(major) + '.' + str(minor + 1)
    raise ValueError('NextReleaseVersion: "minor" or "major", not "' + str(kind) + '"')


def BumpRelease(kind=None, version=None, name=None):
    """Start a new release. THE explicit maintainer action: it is never
    a side effect of resolving an issue, because it is a decision about the product.

    It appends one entry to releases.json - the version, the count of closed issues at which it
    begins, and its name - and rewrites the version files. The micro version restarts at 0.

    Both directions work since this step: a MINOR bump (1.11 -> 1.12) and a MAJOR one
    (1.11 -> 2.0). The major number used to be written as 1 in GetMajorMinorMicroVersion, so 2.0
    could not be expressed at all, which is the reason this matters now (maintainer 2026-09-21).

    The baseline is the count of closed issues PLUS ONE: the next issue that closes is the first
    of the new release and carries micro 0, which is how every release in the file was numbered -
    1.11.0 is issue #2348, the 2073rd closed issue.
    """
    data = issueStore.ReadJson(TrackerPath(releasesFile))
    current = data['releases'][-1]

    if version is None:
        version = NextReleaseVersion(kind)
    version = str(version).strip()
    if version.count('.') != 1 or not all(part.isdigit() for part in version.split('.')):
        raise ValueError('BumpRelease: a release is "major.minor", e.g. "2.0", not "' + version + '"')

    existing = [release['version'] for release in data['releases']]
    if version in existing:
        raise ValueError('BumpRelease: release ' + version + ' is already in releases.json')
    if [int(part) for part in version.split('.')] <= [int(part) for part in current['version'].split('.')]:
        raise ValueError('BumpRelease: ' + version + ' is not after the current release '
                         + current['version'])

    if name is None or str(name).strip() == '':
        name = data.get('plannedNames', {}).get(version, '')
    if str(name).strip() == '':
        raise ValueError('BumpRelease: release ' + version + ' has no name, and releases.json '
                         'plans none for it. The names are jazz legends, alphabetically (' +
                         current['version'] + ' is ' + current['name'] + '); pass --name.')

    #A release that has closed nothing must not be left behind: its baseline and the new one would
    #be the same number, two releases would claim the same issue, and tools/checkIssues.py reports
    #it as "the baselines are not strictly increasing" (found by the tests, at
    #the boundary right after the 1.12 bump).
    closed = issueStore.ClosedCount()
    if closed < current['baseline']:
        raise ValueError('BumpRelease: nothing has closed in release ' + current['version']
                         + ' yet - it begins at closed issue ' + str(current['baseline'])
                         + ' and the store holds ' + str(closed) + '. Two releases would then '
                         'share a baseline. Resolve or close something first.')

    baseline = closed + 1
    data['releases'].append({'version': version, 'baseline': baseline, 'name': str(name).strip()})
    data.get('plannedNames', {}).pop(version, None)
    WriteReleases(data)

    print('release ' + current['version'] + ' (' + current['name'] + ') -> ' + version
          + ' (' + str(name).strip() + '), starting at closed issue ' + str(baseline))
    UpdateFiles()
    print('version is now ' + VersionString())
    return version


#%%******************************************************************************************************
def StampVersion(issue):
    """Write into the issue the version its closing produces.

    The micro version is a running count of closed issues, so every version number in the release
    notes was DERIVED on each run - from a sort by dateResolved - until this step. One corrected
    date or one lost file then renumbered versions that have been published, and nothing said so.
    The number is written down here, once, and tools/checkIssues.py recomputes it later and
    compares: a number written once and recomputed is a check, the same derivation twice is not.

    An issue that is already closed keeps its version: it has had its place in the count since it
    closed, and re-resolving it does not give it a new one."""
    if issue['status'] in closedStatuses and issue.get('resolvedInVersion', '').strip() != '':
        return issue['resolvedInVersion']

    #this issue is about to become the next closed one, so it carries the count including itself
    issue['resolvedInVersion'] = VersionOfClosedIndex(issueStore.ClosedCount() + 1)
    return issue['resolvedInVersion']


#%%******************************************************************************************************
#use this to resolve an issue
def ResolveIssue(issueNumber, notes='', author='JG'): #raise a new issue into list (append to end of list)

    if not((issueNumber >= 0) and (issueNumber < NumberOfIssues())):
        print('Issue: invalid number! Nothing done')
        return None

    d = GetIssue(issueNumber)

    #what is written here is PUBLISHED - it is the release note of this issue. The remarks that
    #were useful while the work went on ("duplicate of #2134", "part A solved") are not, so they
    #are dropped here rather than carried into the release notes
    StampVersion(d)                 #before the status changes: it counts the issue itself
    d['status'] = 'RESOLVED'
    d['dateResolved'] = GetDateTimeStr()
    d['resolvedAuthor'] = author
    d['releaseNotes'] = notes
    d['workingRemarks'] = ''

    ModifyDictIssue(d)
    
    #state the number first, so it can be copied into the commit message without recomputing it
    print('issue resolved: #' + str(issueNumber) + ' "' + str(d['title']).strip() + '"')
    print(d)

    UpdateDateAndVersion()
    WritePages()

    return issueNumber

#%%******************************************************************************************************
#use this to close an issue that will NOT be done
def CloseIssue(issueNumber, reason, author='JG'):
    """Close an issue without doing it.

    CLOSED covers every way an issue ends except being resolved - obsolete, won't fix, duplicate
    of #n, superseded, no longer applies, not reproducible, abandoned - and the kind belongs in
    the reason, which is why the reason is NOT optional: a closed issue without one is worse than
    an open one, because the next person cannot tell whether it was judged or forgotten.

    The issue keeps its place in the version count (see closedStatuses) and appears in the release
    notes neither as resolved nor as open."""

    if not((issueNumber >= 0) and (issueNumber < NumberOfIssues())):
        print('Issue: invalid number! Nothing done')
        return None

    if reason.strip() == '':
        raise ValueError('CloseIssue: say WHY in "reason" - obsolete, won\'t fix, duplicate of '
                         '#n, superseded, not reproducible, abandoned; it is the only record of '
                         'the decision')

    d = GetIssue(issueNumber)

    #as in ResolveIssue: the reason is the published record of the decision, the working remarks
    #are not and are dropped
    StampVersion(d)                 #a closed issue counts for the version like a resolved one
    d['status'] = 'CLOSED'
    d['dateResolved'] = GetDateTimeStr()
    d['resolvedAuthor'] = author
    d['releaseNotes'] = reason
    d['workingRemarks'] = ''

    ModifyDictIssue(d)

    print('issue closed: #' + str(issueNumber) + ' "' + str(d['title']).strip() + '"')
    print(d)

    UpdateDateAndVersion()
    WritePages()

    return issueNumber


#%%******************************************************************************************************
#the older name of CloseIssue; scripts outside this
#repository may still call it, and it costs one line to keep them working
def AbandonIssue(issueNumber, reason, author='JG'):
    """deprecated spelling of CloseIssue"""
    return CloseIssue(issueNumber, reason, author=author)


#%%******************************************************************************************************
#use this to quickly raise a new issue
def RaiseIssue(issueName, description, issueType='EXTENSION', fileName='', lineNumber='', deadline='', author='JG', priority=''): #raise a new issue into list (append to end of list)
    
    if deadline == '': #create 120 days deadline
        date=datetime.datetime.now() + datetime.timedelta(days=180)
        monthZero = '' #add leading zero for month
        dayZero = ''   #add leading zero for day
        if date.month < 10:
            monthZero = '0'
        if date.day < 10:
            dayZero = '0'            
        deadline = str(date.year) + '-' + monthZero + str(date.month) + '-' + dayZero + str(date.day)
        
    d={'title': issueName,
       'author': author,
       'description': description,
       'type': issueType,
       'file': fileName,
       'line': lineNumber,
       'deadline': deadline,
       'priority': priority}
    return RaiseIssueDict(d) #the assigned issue number
    #the fields are issueStore.issueFields

    print(GetIssue(NumberOfIssues()-1))
    
    
    