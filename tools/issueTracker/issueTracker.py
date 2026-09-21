# -*- coding: utf-8 -*-
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Created on Fri May 10 08:53:30 2019
# @author: Johannes Gerstmayr

# Issue tracker: the rules. The FILES are issueStore.py - one JSON file per open or recently
# closed issue in tools/issueTracker/issues/, one per year for the older ones (revision2026 step
# R8.5). trackerlog.txt, a 723 KB file of comma-separated lines in which no text field could
# contain a comma, is gone.
#
# - the fields of an issue: issueStore.issueFields
# - type, status, priority and effort: issueTypes, issueStatuses, issuePriorities, issueEfforts
#   below - that is the ONLY list of them (revision2026 steps R8.7 and R8.5.3)
# - releaseNotes is written when the issue is CLOSED and is published in the release notes;
#   workingRemarks is what the work knows meanwhile and is cleared when the issue closes
# - THIS TOOL OWNS THE VERSION: the micro number is the count of closed issues, so ResolveIssue
#   and AbandonIssue rewrite version.txt, versionCpp.cpp and the version line of README.rst
#
# The command line is 'exudev issue <verb>' (revision2026 step R8.3); this module is its API.
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import datetime # for current date
import os
import re
import io
import sys

#import os

#WHERE THE FILES ARE (revision2026 step R8.3). Until this step every path was relative to the
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
relative_path = '../generators'   #autoGenerateHelper moved to tools/generators (revision2026 step R4.3 part 2g)
helperPath = os.path.join(absolute_path, relative_path)
sys.path.append(helperPath)

#autoGenerateHelper is not needed any more: the tracker wrote LaTeX and RST until
#revision2026 step R7.1.6 and writes Markdown now, which needs no conversion helper

#the issues themselves live in issues/ as JSON since revision2026 step R8.5; issueStore owns the
#files, this module owns the rules
sys.path.insert(0, trackerDirectory)
import issueStore                                                             # noqa: E402


#filename without file ending:
trackerFile = 'trackerlog'

trackerDateLine =    2 #line at which the date is given
trackerReleaseLine = 3 
trackerVersionLine = 4
#this number should be set 1 larger than what is written in trackerlog.html (when resolving last issue):
version0xResolved = 368 #1.0
version1xResolved = 664 #1.1
version2xResolved = 842 #1.2 #according to trackerlog.html 'Number of resolved issues'
version3xResolved = 989 #1.3 #according to trackerlog.html 'Number of resolved issues'
version4xResolved = 1095#1.4 #according to trackerlog.html 'Number of resolved issues'
version5xResolved = 1161#1.5 #according to trackerlog.html 'Number of resolved issues'
version6xResolved = 1280#1.6 #according to trackerlog.html 'Number of resolved issues'
version7xResolved = 1470#1.7 #according to trackerlog.html 'Number of resolved issues'
version8xResolved = 1594#1.8 #according to trackerlog.html 'Number of resolved issues'
version9xResolved = 1676#1.9 #according to trackerlog.html 'Number of resolved issues'
version10xResolved = 1912#1.10 #according to trackerlog.html 'Number of resolved issues'
version11xResolved = 2073#1.11 #according to trackerlog.html 'Number of resolved issues'

versionResolved=[0,version0xResolved, version1xResolved, version2xResolved, version3xResolved, 
                 version4xResolved, version5xResolved, version6xResolved, version7xResolved, version8xResolved,
                 version9xResolved, version10xResolved, version11xResolved] #also adapt trackerlog.txt release 

# versionDev = '' #release (works in pip)
versionDev = '.dev1' #(development version, get with pip install exudyn --pre)

#subversions use names of jazz legends ... #https://www.britannica.com/topic/list-of-jazz-musicians-2030466
# +++++++++++++++++++++++++++++++++++++++++++++
# ++++++++ the release names; doc2rst.py held the second copy and is gone (R7.1.7) ++++++++++
versionNames = {'1.0':'Abercrombie', '1.1':'Burton', '1.2':'Corea', '1.3':'Davis', '1.4':'Ellington', '1.5':'Fitzgerald', 
                '1.6':'Gillespie', '1.7':'Hall', '1.8':'Jones', #Jim Hall, Elvin Jones; leave out 'I' as there are not many => two 'M'
                '1.9':'Krall', '1.10': 'Lagrene', '1.11':'McLaughlin', '1.12':'Metheney', #Bireli Lagrene
                '1.13':'Newborn', '1.14':'Parker'} #(Phineas) Newborn, (Charlie) Parker, (Jaco) Pastorius, (Oscar) Peterson, #3xP for missing O and Q
                #(Django) Reinhardt, Scofield, Thielemans, (Steve) Vai, (Sarah) Vaughan
# +++++++++++++++++++++++++++++++++++++++++++++

#+++++++++++++++++++++++++++++++++++++++++++++
#THE ISSUE TYPES, in one place (revision2026 step R8.7, #2519). Before this there were three lists -
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

#THE STATUSES. WORK and TESTING stood in the old list and were never used once in 2519 issues, so
#they are gone; ABANDONED is what was missing - "decided against", "no longer applies", "not
#possible" - and without it such issues had to be written as RESOLVED, which is untrue.
issueStatuses = {
    'RAISED':    'open',
    'RESOLVED':  'done',
    'ABANDONED': 'closed WITHOUT being done; the reason belongs in releaseNotes',
}

#Both count for the version number. The micro version is the count of CLOSED issues, not of
#resolved ones: if abandoning did not count, then abandoning an already-resolved issue would move
#version.txt BACKWARDS and a released version number would stop being reproducible from this file.
#The difference shows in the release notes instead - only RESOLVED is listed there as resolved,
#and ABANDONED is listed nowhere, neither as resolved nor as open.
closedStatuses = ['RESOLVED', 'ABANDONED']

#types that are not announced as resolved issues: an idea that became a real issue would otherwise
#be reported twice
typesNotInReleaseNotes = ['IDEA']
#+++++++++++++++++++++++++++++++++++++++++++++

#THE EFFORT, in human working hours without AI assistance (revision2026 step R8.5.3, maintainer
#2026-09-21). It is a CLASSIFICATION, not an estimate anyone is held to: what it buys is the
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

#the fields of an issue are issueStore.issueFields since revision2026 step R8.5; what stood here
#was the column order of trackerlog.txt with the indices that went with it - 'notes' was the last
#column, the status was padded to 8 characters, and every reader had to know both.


def ResolvedIssues2Version(resolvedIssues, totalResolvedIssues):
    #return [0,totalResolvedIssues-resolvedIssues]
    cnt = len(versionResolved)-2 #this is the max. minor version number
    for vr in reversed(versionResolved):
        if totalResolvedIssues-resolvedIssues >= vr:
            return [cnt, totalResolvedIssues-vr-resolvedIssues]
        cnt -= 1
    raise ValueError('ResolvedIssues2Version: something went wrong')
    return None

#++++++++++++++++++++++++++++++
#update all output files with new version, etc.
def UpdateFiles():
    UpdateDateAndVersion()
    ConvertToHTML() #update html version of issue tracker
    ConvertToMarkdown() #update html version of issue tracker


#%%******************************************************************************************************
def ToMarkdown(s):
    """Every Markdown special character in issue text becomes literal text (#2545).

    The same rule as ToLatex above, for the format that replaces it in revision2026 step R7.1.6:
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
#THE DATA LAYER (revision2026 step R8.5). The issues live in tools/issueTracker/issues/ as one
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
    """the version, derived from the count of CLOSED issues - resolved and abandoned alike (#2519).
    The count comes from the store, which adds the closed files it can see to the closedCount each
    archive file states; an archive that is missing is reported by issueStore.CheckStore() rather
    than silently lowering the version (revision2026 step R8.5)."""
    numberOfResolved = issueStore.ClosedCount()

    major = 1
    minor = (len(versionResolved)-2)

    micro = numberOfResolved-versionResolved[-1] 
    
    return [major, minor, micro]

#%%******************************************************************************************************
def VersionString():
    [release, version] = GetReleaseAndVersionString()
    return str(release)+'.'+str(version)+versionDev 

#%%******************************************************************************************************
#write date and version to tracker file; also update version in src/Autogenerated/version.h
def UpdateDateAndVersion(updateVersion = True):
    #the date and the version stood in the header lines of trackerlog.txt; they are meta.json
    #beside the issues since revision2026 step R8.5
    WriteMeta()

    #update version in Python module versionPybind.h ==> this is shown in the module with python command version()
    #no main/ level since the flatten (revision2026 step R3.1); version.txt is at the
    #repository ROOT since R3.4, and versionName.txt sits next to this tool since R7.1.7,
    #when docs/theDoc/ was deleted with the LaTeX build (decision D8)
    #versionFile = directoryString + 'version.h' #not used anymore
    cppVersionFile = RepositoryPath('src', 'Autogenerated', 'versionCpp.cpp')
    texVersionFile = RepositoryPath('version.txt')
    #versionName.txt sits next to this tool since revision2026 step R7.1.7, when
    #docs/theDoc/ was deleted with the LaTeX build (decision D8)
    texVersionNameFile = TrackerPath('versionName.txt')
    #hand-written since revision2026 step R7.1.5, except for its version line
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
        versionNameString = '('+versionNames[str(release)]+')'

        file=open(texVersionNameFile,'w', encoding='utf-8')  #clear file by one write access
        file.write(versionNameString)
        file.close()

        #++++++++++++++++
        #README.rst is the GitHub and PyPI landing page and is hand-written (revision2026 step
        #R7.1.5); it used to be generated from gettingStarted.tex, and the only part of it that
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
#that stood here went with docs/RST/trackerlog.rst in revision2026 step R7.1.6


#%%******************************************************************************************************
def ConvertToHTML():
    """tools/issueTracker/trackerlog.html: the overview a maintainer scrolls through, open issues
    first and coloured by priority. Written from the store since revision2026 step R8.5; it was a
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


def ConvertToMarkdown():
    """docs/generated/trackerlog.md: the resolved issues per release, the open issues and the
    known bugs. Markdown since revision2026 step R7.1.6; it wrote docs/theDoc/trackerlog.tex and
    docs/RST/trackerlog.rst until then, and the colours of the open issues, which were RST roles,
    are the CSS classes of docs/_static/custom.css written as inline HTML."""
    [releaseString,versionString] = GetReleaseAndVersionString()
    [majorCurrent,microCurrent,minorCurrent] = GetMajorMinorMicroVersion()

    releaseVersionDev = VersionString()

    numberOfRaised = NumberOfIssues()
    numberOfResolved = minorCurrent
    lastChangeDate = ReadMeta()['lastChange']
    totalResolved = versionResolved[-1] + numberOfResolved

    def IssueNumberString(issue):
        #the number is an int in the store; the release notes print it as it was written
        #in seven years of issue texts, four digits with leading zeros
        return str(issue['number']).rjust(4, '0')

    def Colour(cssClass, text):
        return '<span class="' + cssClass + '">' + text + '</span>'

    text = ('<!-- GENERATED by tools/issueTracker/issueTracker.py from trackerlog.txt '
            '- do not edit -->\n'
            '(sec-issuetracker)=\n'
            '# Issue tracker\n\n')
    text += ('This section contains resolved issues per release and known bugs. Use this '
             'information to understand changes compared to previous versions. The author field '
             'is omitted if it was Johannes Gerstmayr (JG).\n'
             'The extension `.dev1` is not added in the issues list (e.g., 1.2.2.dev1==1.2.2), '
             'as it only marks versions that will not be available in pypi with standard pip '
             'install, but only with the `--pre` option or by specifying the exact version name, '
             'see versions on <https://pypi.org/project/exudyn/>.\n'
             'BUG numbers refer to the according issue numbers.\n\n')
    text += ('General information on current version:\n\n'
             '- Exudyn version = ' + releaseVersionDev + '\n'
             '- last change = ' + lastChangeDate + '\n'
             '- Number of issues = ' + str(numberOfRaised) + '\n'
             '- Number of resolved issues = ' + str(totalResolved)
             + ' (' + str(numberOfResolved) + ' in current version)\n\n')

    text += ('## Resolved issues and resolved bugs\n\n'
             'The following list contains the issues which have been **RESOLVED** in the '
             'according version:\n\n')

    resolved = '### Version ' + releaseString + '\n\n'
    openIssues = ''
    bugs = ''

    issueList = GetIssues()
    issueListSorted = sorted(issueList, key = lambda i: i['dateResolved'])
    resolvedCnt = 0
    previousRelease = (majorCurrent,microCurrent)
    vIssueRelease = 1 #for now

    for issue in reversed(issueListSorted):
        [vIssueMinor, vIssueMicro] = ResolvedIssues2Version(resolvedCnt, totalResolved)
        rNew = (vIssueRelease, vIssueMinor) if vIssueMinor >= 0 else (0,1)

        if (rNew[0] < previousRelease[0] or
            (rNew[0] == previousRelease[0] and rNew[1] < previousRelease[1]) ):
            resolved += '\n### Version '+str(rNew[0])+'.'+str(rNew[1])+'\n\n'
            previousRelease = rNew

        #the details of one issue, as the sub-list of its entry
        details = ''
        if issue['author'] != 'JG':
            details += '  - issue author: '+ToMarkdown(issue['author'])+'\n'
        details += '  - description: '+ToMarkdown(issue['description'])+'\n'
        #a CLOSED issue shows what its resolution says, an OPEN one what the work on it knows so
        #far; the two are different fields since revision2026 step R8.5.3
        if len(issue['releaseNotes'].strip(' ')) != 0:
            details += '  - **notes:** '+ToMarkdown(issue['releaseNotes'])+'\n'
        if len(issue['workingRemarks'].strip(' ')) != 0:
            details += '  - **remarks:** '+ToMarkdown(issue['workingRemarks'])+'\n'
        if len(issue['effort'].strip(' ')) != 0:
            details += ('  - effort: ' + ToMarkdown(issue['effort'].strip())
                        + ' (' + issueEfforts.get(issue['effort'].strip(), '') + ')\n')

        details += '  - '
        if len(issue['dateResolved']) != 0:
            details += 'date resolved: **'+issue['dateResolved'].strip()+'**, '
        details += 'date raised: '+issue['dateRaised'].strip()
        if issue['resolvedAuthor'] != 'JG' and len(issue['resolvedAuthor']) != 0:
            details += ', resolved by: '+ToMarkdown(issue['resolvedAuthor'])
        details += '\n'

        title = ToMarkdown(issue['title'].strip(' '))

        if issue['status'] == 'RESOLVED' and issue['type'] not in typesNotInReleaseNotes:
            entry = ('- Version '+str(rNew[0])+'.'+str(rNew[1])+'.'+str(vIssueMicro)+': ')
            if issue['type'] == 'BUG':
                entry += Colour('textred', 'resolved BUG '+IssueNumberString(issue))+': '+title
            else:
                entry += ('resolved Issue '+IssueNumberString(issue)+': '+title
                          + ' ('+issue['type'].lower()+')')
            resolved += entry + '\n' + details
        elif issue['status'] == 'RAISED' and issue['type'] == 'BUG':
            bugs += '- '+Colour('textred', 'open BUG '+IssueNumberString(issue)+':')+' '+title+'\n'
            bugs += details
        elif issue['status'] == 'RAISED':       #an ABANDONED issue is not an open one
            #one spelling per priority since revision2026 step R8.5.3; no priority is the
            #normal case and gets the neutral colour
            cssClass = {'HIGH': 'textred', 'NORMAL': 'textorange',
                        'LOW': 'textblue'}.get(issue['priority'].strip().upper(), 'boldblue')
            openIssues += ('- '+Colour(cssClass, 'open issue '+IssueNumberString(issue)+':')+' '
                           + title+'\n')
            openIssues += details

        #CLOSED, not resolved: the version a past issue is listed under is derived from this
        #counter, so counting only RESOLVED would renumber every historical entry as soon as
        #one issue is abandoned. An abandoned issue keeps its place and is simply not printed
        if issue['status'] in closedStatuses:
            resolvedCnt += 1

    text += resolved
    text += '\n## Open issues\n\n' + openIssues
    text += '\n## Known bugs\n\n' + bugs

    markdownFile = RepositoryPath('docs', 'generated', trackerFile+'.md')
    os.makedirs(os.path.dirname(markdownFile), exist_ok=True)
    with io.open(markdownFile, 'w', encoding='utf-8', newline='\n') as file:
        file.write(text)

#%%******************************************************************************************************
#%%******************************************************************************************************
#%%******************************************************************************************************
def CheckedEnumValue(fieldName, value, allowed):
    """one spelling per value, or a message that lists the ones there are (revision2026 step
    R8.5.3). An empty value is legal for every enum field of an issue and means "not classified";
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
    #be empty, which means "not classified" (revision2026 step R8.5.3)
    issueDict['effort'] = CheckedEnumValue('effort', issueDict.get('effort', ''), issueEfforts)
    issueDict['priority'] = CheckedEnumValue('priority', issueDict.get('priority', ''),
                                             issuePriorities)

    #the release note is written when the issue is CLOSED, not when it is raised: it is what the
    #release notes publish. What is known while the work goes on belongs in workingRemarks.
    if issueDict.get('releaseNotes', '').strip() != '':
        raise ValueError('RaiseIssue: releaseNotes is written by ResolveIssue or AbandonIssue; '
                         'put what you know now into workingRemarks')
    issueDict['releaseNotes'] = ''

    number = NumberOfIssues()
    issueDict['number'] = number
    issueStore.Save(issueDict)

    UpdateDateAndVersion(updateVersion=False) #do not change version files!
    ConvertToHTML() #update html version of issue tracker
    ConvertToMarkdown() #update latex issues in docu (only contains resolved issues in version and bugs)

    #report the number that was actually assigned, and return it: it is needed for the commit
    #message and the documentation, and reconstructing it by hand afterwards gets it wrong
    print('issue raised: #' + str(number) + ' "' + str(issueDict['title']).strip() + '"')

    return number

#%%******************************************************************************************************
#modify an existing issue
#use this to overwrite ONE field of an issue
def ChangeIssue(issueNumber, key, value): #raise a new issue into list (append to end of list)

    if not((issueNumber >= 0) and (issueNumber < NumberOfIssues())):
        print('ChangeIssue: invalid number! Nothing done')
        return
    
    d = GetIssue(issueNumber)

    if key not in d:
        print('ChangeIssue: key "' + key + '" not available!')
        return

    #this function writes any field, which is why the enums have to be checked here as well as in
    #RaiseIssueDict - otherwise the one list of values is a comment again (revision2026 R8.5.3)
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

    d[key] = value

    ModifyDictIssue(d)
    
    print('new issue:')
    print(d)
    
    UpdateDateAndVersion()
    ConvertToHTML() #update html version of issue tracker
    ConvertToMarkdown() #update html version of issue tracker

#%%******************************************************************************************************
#%%******************************************************************************************************
#use this when the analysis of an issue turns up more than the issue says
def ExtendIssue(issueNumber, text, author='JG'):
    """Append a dated paragraph to the description of an OPEN issue (revision2026 step R8.3.3).

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

    #the date says which part of the description is the later analysis; in the JSON format of
    #revision2026 step R8.5 this becomes one entry of an "updates" list
    d['description'] = (d['description'].rstrip()
                        + ' [' + GetDateStr() + ', ' + author + ']: ' + text.strip())

    ModifyDictIssue(d)

    print('issue extended: #' + str(issueNumber) + ' "' + str(d['title']).strip() + '"')

    UpdateDateAndVersion()
    ConvertToHTML()
    ConvertToMarkdown()
    return issueNumber


#%%******************************************************************************************************
#use this to record what is known while the work on an issue goes on
def RemarkIssue(issueNumber, text, author='JG', replace=False):
    """Write the working remarks of an OPEN issue (revision2026 step R8.5.3).

    This is the scratchpad of an issue: "duplicate of #2134", "marked for deprecation", "check
    whether this still happens", "part A solved, B open". It is worth having while the issue is
    open and worthless once it is closed, which is exactly what separates it from the release
    note - so ResolveIssue and AbandonIssue clear it, and it is never published.

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
    ConvertToHTML()
    ConvertToMarkdown()
    return issueNumber


#%%******************************************************************************************************
#use this to resolve an issue
def ResolveIssue(issueNumber, notes='', author='JG'): #raise a new issue into list (append to end of list)

    if not((issueNumber >= 0) and (issueNumber < NumberOfIssues())):
        print('Issue: invalid number! Nothing done')
        return None

    d = GetIssue(issueNumber)

    #what is written here is PUBLISHED - it is the release note of this issue. The remarks that
    #were useful while the work went on ("duplicate of #2134", "part A solved") are not, so they
    #are dropped here rather than carried into the release notes (revision2026 step R8.5.3)
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
    ConvertToHTML() #update html version of issue tracker
    ConvertToMarkdown() #update html version of issue tracker

    return issueNumber

#%%******************************************************************************************************
#use this to close an issue that will NOT be done
def AbandonIssue(issueNumber, reason, author='JG'):
    """Close an issue without doing it: decided against, no longer applies, not possible. The
    reason is not optional - an abandoned issue with no reason is worse than an open one, because
    the next person cannot tell whether it was judged or forgotten. The issue keeps its place in
    the version count (see closedStatuses) and appears in the release notes neither as resolved nor
    as open."""

    if not((issueNumber >= 0) and (issueNumber < NumberOfIssues())):
        print('Issue: invalid number! Nothing done')
        return None

    if reason.strip() == '':
        raise ValueError('AbandonIssue: say WHY in "reason"; it is the only record of the decision')

    d = GetIssue(issueNumber)

    #as in ResolveIssue: the reason is the published record of the decision, the working remarks
    #are not and are dropped (revision2026 step R8.5.3)
    d['status'] = 'ABANDONED'
    d['dateResolved'] = GetDateTimeStr()
    d['resolvedAuthor'] = author
    d['releaseNotes'] = reason
    d['workingRemarks'] = ''

    ModifyDictIssue(d)

    print('issue abandoned: #' + str(issueNumber) + ' "' + str(d['title']).strip() + '"')
    print(d)

    UpdateDateAndVersion()
    ConvertToHTML()
    ConvertToMarkdown()

    return issueNumber


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
    #the fields are issueStore.issueFields (revision2026 step R8.5)

    print(GetIssue(NumberOfIssues()-1))
    
    
    