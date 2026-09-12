# -*- coding: utf-8 -*-
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Created on Fri May 10 08:53:30 2019
# @author: Johannes Gerstmayr

# Issue tracker

# line format: number, issue name, issue author, status, description, type, priority, date raised, deadline, date resolved, resolved author, file, line, notes
# - type: BUG, FIX, NEW FEATURE, EXTENSION, CHANGE, PERFORMANCE, IDEA, CHECK, 
#         CLEANUP, DOCU, TUTORIAL, TESTING, EXAMPLE
# - status: RAISED, RESOLVED, WORK
# - priority: NO (empty: ''), LOW, NORMAL, HIGH

# NOTE: in 'trackerlog.txt', the text fields may not use ',', but '\;' is used instead!
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import datetime # for current date
import os
import io
import sys

#import os

absolute_path = os.path.dirname(__file__)
relative_path = '../../src/pythonGenerator'   #repository root is two levels up since the flatten (revision plan step 25)
helperPath = os.path.join(absolute_path, relative_path)
sys.path.append(helperPath)

from autoGenerateHelper import LatexString2RST, RSTheaderString


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
# ++++++++ also sync with doc2rst.py ++++++++++
versionNames = {'1.0':'Abercrombie', '1.1':'Burton', '1.2':'Corea', '1.3':'Davis', '1.4':'Ellington', '1.5':'Fitzgerald', 
                '1.6':'Gillespie', '1.7':'Hall', '1.8':'Jones', #Jim Hall, Elvin Jones; leave out 'I' as there are not many => two 'M'
                '1.9':'Krall', '1.10': 'Lagrene', '1.11':'McLaughlin', '1.12':'Metheney', #Bireli Lagrene
                '1.13':'Newborn', '1.14':'Parker'} #(Phineas) Newborn, (Charlie) Parker, (Jaco) Pastorius, (Oscar) Peterson, #3xP for missing O and Q
                #(Django) Reinhardt, Scofield, Thielemans, (Steve) Vai, (Sarah) Vaughan
# +++++++++++++++++++++++++++++++++++++++++++++

trackerItems = ['number', 'issue', 'author', 'status', 'description', 
                'type', 'priority', 'date raised', 'deadline', 'date resolved', 
                'resolved author', 'file', 'line', 'notes']
omitItemsHTML=[2,8,11,12] #author, pri, deadline, file, line
numberOfItems = len(trackerItems)

nHeaderLines = 10 #number of header lines in trackerlog.txt

indexStatus = trackerItems.index('status')      #3
indexType = trackerItems.index('type')          #5
indexPriority = trackerItems.index('priority')  #6
indexDeadline = trackerItems.index('deadline')  #8
indexDateRaised = trackerItems.index('date raised')


#++++++++++++++++++++++++++++++
#compute micro and minor version from number of resolved issues
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
    ConvertToLatex() #update html version of issue tracker


#%%******************************************************************************************************
def ToLatex(s): #replace _ and other symbols to fit into latex code

    s = s.replace('_','\\_')
    # s = s.replace('{','\{')
    # s = s.replace('}','\}')
    s = s.replace('#','\\#')
    s = s.replace('&','\\&')

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
def IssueTrackerBackup(): #create backup of issue log file
    fileRead=open(trackerFile+'.txt','r', encoding='utf-8') 
    fileLines = fileRead.readlines()
    fileRead.close()
    
    fileWrite=open(trackerFile+'_backup.txt','w', encoding='utf-8')  #write file
    
    for line in fileLines:
        fileWrite.write(line)
        #fileWrite.write(line.strip('\n')+',,\n')

    fileWrite.close()


#%%******************************************************************************************************
def NumberOfIssues(): #count number of existing issues
    fileRead=open(trackerFile+'.txt','r', encoding='utf-8') 
    fileLines = fileRead.readlines()
    fileRead.close()
    return len(fileLines)-nHeaderLines

#%%******************************************************************************************************
def GetIssue(number): #0-based, get dictionary of issue with number
    fileRead=open(trackerFile+'.txt','r', encoding='utf-8') 
    fileLines = fileRead.readlines()
    fileRead.close()
    
    #print(len(fileLines))
    #print(number)
    
    d={} #empty dictionary
    if (number >= 0) & (number < len(fileLines)-nHeaderLines):
        line = fileLines[number+nHeaderLines]
        line = (line.strip('\n'))
    
        items = line.split(',')
        if len(items) != numberOfItems:
            print('ERROR: issue ' + str(number) + ' has inconsistent line definition (' + str(len(items)) + ' columns instead of ' + str(numberOfItems) + ' columns) - check commas')
            return 0
      
        cnt = 0;
        for s in items: 
            txt = s.replace('\\;',',')
            d[trackerItems[cnt]] = txt
            cnt += 1
    else:
        print('Issue: invalid number!')
    
    return d

#%%******************************************************************************************************
def GetIssues(): #get list of dictionaries of all issues
    fileRead=open(trackerFile+'.txt','r', encoding='utf-8') 
    fileLines = fileRead.readlines()
    fileRead.close()
    
    #print(len(fileLines))
    #print(number)
    
    issuesList = []
    
    for number in range(len(fileLines)-nHeaderLines):
        d={} #empty dictionary
        line = fileLines[number+nHeaderLines]
        line = (line.strip('\n'))
    
        items = line.split(',')
        if len(items) != numberOfItems:
            print('ERROR: issue ' + str(number) + ' has inconsistent line definition (' + str(len(items)) + ' columns instead of ' + str(numberOfItems) + ' columns) - check commas')
            return 0
      
        cnt = 0;
        for s in items: 
            txt = s.replace('\\;',',')
            d[trackerItems[cnt]] = txt
            cnt += 1
        issuesList += [d]
    
    return issuesList

#%%******************************************************************************************************
def ConvertToCSV(): #convert all issues to a .CSV file
    fileRead=open(trackerFile+'.txt','r', encoding='utf-8') 
    fileLines = fileRead.readlines()
    fileRead.close()
    
    #print(len(fileLines))
    #print(number)

    fileWrite=open(trackerFile+'.csv','w', encoding='utf-8')  #write file
    
    nLine = 0
    for lineEOL in fileLines:
        if nLine >= nHeaderLines:
            line = (lineEOL.strip('\n'))
        
            items = line.split(',')
            if len(items) != numberOfItems:
                print('ERROR: issue ' + str(nLine-nHeaderLines) + ' has inconsistent line definition (' + str(len(items)) + ' columns instead of ' + str(numberOfItems) + ' columns) - check commas')
            else:
                cnt = 0
                for s in items: 
                    txt = s.replace('\\;',',')
                    fileWrite.write('"' + txt + '"')
                    cnt += 1
                    if cnt < numberOfItems: fileWrite.write(',')
    
            fileWrite.write('\n')
        
        nLine += 1

    fileWrite.close()

#%%******************************************************************************************************
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



def GetMajorMinorMicroVersion(): #convert all issues to a .html file
    fileRead=open(trackerFile+'.txt','r', encoding='utf-8') 
    fileLines = fileRead.readlines()
    fileRead.close()

    numberOfResolved = 0
    #count resolved issues
    for line in fileLines:
        
        items = line.split(',')
        if len(items) == numberOfItems: #only count in valid lines; error will be reported lateron
            if (items[indexStatus].find('RESOLVED') != -1):
                numberOfResolved += 1
    
    #print('numberOfResolved=',numberOfResolved)
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
    IssueTrackerBackup()
    
    fileRead=open(trackerFile+'.txt','r', encoding='utf-8') 
    fileLines = fileRead.readlines()
    fileRead.close()
    
    fileLines[trackerDateLine] = '# date = ' + GetDateStr() + '\n'
    fileLines[trackerReleaseLine ] = '# release = ' + str(GetReleaseAndVersionString()[0]) + '\n'
    fileLines[trackerVersionLine] = '# version = ' + str(GetReleaseAndVersionString()[1]) + '\n'

    fileWrite=open(trackerFile+'.txt','w', encoding='utf-8') 
    for line in fileLines:
        fileWrite.write(line)
        
    fileWrite.close()

    #update version in Python module versionPybind.h ==> this is shown in the module with python command version()
    directoryString = '..\\..\\src\\Autogenerated\\'   #no main/ level since the flatten (revision plan step 25)
    #versionFile = directoryString + 'version.h' #not used anymore
    cppVersionFile = directoryString + 'versionCpp.cpp'
    texVersionFile = '..\\..\\docs\\theDoc\\version.txt'
    texVersionNameFile = '..\\..\\docs\\theDoc\\versionName.txt'
    #batVersionFile = '..\\..\\tools\\makeWindowsBinaries\\version.txt' #pure version number for file names

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
        file=open(cppVersionFile,'w')  #clear file by one write access
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
        file=open(texVersionFile,'w')  #clear file by one write access
        versionString = releaseVersionDev 
        file.write(versionString)
        file.close()

        [release, version_] = GetReleaseAndVersionString()
        versionNameString = '('+versionNames[str(release)]+')'

        file=open(texVersionNameFile,'w')  #clear file by one write access
        file.write(versionNameString)
        file.close()

    
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
def ConvertToHTML(): #convert all issues to a .html file
    fileRead=open(trackerFile+'.txt','r', encoding='utf-8') 
    fileLines = fileRead.readlines()
    fileRead.close()

    #[release,version] = GetReleaseAndVersion()
    [major,micro,minor] = GetMajorMinorMicroVersion()

    numberOfRaised = len(fileLines)-nHeaderLines   
    numberOfResolved = minor 
    lastChangeDate = fileLines[trackerDateLine].split('=')[1] 
    
    fileWrite=io.open(trackerFile+'.html','w', encoding='utf-8')  #write file; utf-8 needed for sphinx
    # fileWrite=io.open('../../docs/'+trackerFile+'.html','w', encoding='utf-8')  #write file; utf-8 needed for sphinx

    #follows https://www.w3schools.com/html/tryit.asp?filename=tryhtml_table_cellspacing
    try:
        fileWrite.write('<!DOCTYPE html>\n')
        fileWrite.write('<html>\n')
        fileWrite.write('<head>\n')
        fileWrite.write('<style>\n')
        fileWrite.write('table, th, td{\n')
        fileWrite.write('  border: 1px solid black;\n')
        fileWrite.write('  padding: 1px;\n')
        fileWrite.write('}\n')
        fileWrite.write('table{\n')
        fileWrite.write('  border - spacing: 5px;\n')
        fileWrite.write('}\n')
        fileWrite.write('</style>\n')
        fileWrite.write('</head>\n')
        fileWrite.write('<body>\n')
        fileWrite.write('\n')
        fileWrite.write('<h2>ISSUE Tracker</h2>\n')
        #fileWrite.write('<p>Border spacing specifies the space between the cells.</p>\n')
        
        fileWrite.write('\n')
        fileWrite.write('Number of issues = ' + str(numberOfRaised) + ', \n')
        fileWrite.write('Number of resolved issues = '  + 
                        str(versionResolved[-1] + numberOfResolved) + 
                        ' ('+str(numberOfResolved) +' in current minor version), \n')
        fileWrite.write('Exudyn version = '+VersionString()+', \n')
        fileWrite.write('last change = '+lastChangeDate+'\n')
        
        fileWrite.write('\n')
        fileWrite.write('<table style = "width:100%">\n')
        fileWrite.write('<tr style = "background-color:#AAAAAA">\n')
        fileWrite.write('<th>nr</th>\n')
        fileWrite.write('<th>issue</th>\n')
        #fileWrite.write('<th>auth</th>\n')
        fileWrite.write('<th>status</th>\n')
        fileWrite.write('<th>description</th>\n')
        fileWrite.write('<th>type</th>\n')
        fileWrite.write('<th>pri</th>\n')
        fileWrite.write('<th>raised</th>\n')
        #fileWrite.write('<th>deadline</th>\n')
        fileWrite.write('<th>resolved</th>\n')
        fileWrite.write('<th>res. by</th>\n')
        # fileWrite.write('<th>f</th>\n')
        # fileWrite.write('<th>l</th>\n')
        fileWrite.write('<th>notes</th>\n')
        fileWrite.write('</tr>\n')
    
        #nLine = 0
        for mode in range(2): #mode0: raised items, mode1: resolved items
            for nLine, lineEOL in reversed(list(enumerate(fileLines))):
                if nLine > 9:
                    line = (lineEOL.strip('\n'))
                
                    nIssue = nLine-nHeaderLines
                    items = line.split(',')
                    if len(items) != numberOfItems:
                        print('ERROR: issue ' + str(nIssue) + ' has inconsistent line definition (' + str(len(items)) + ' columns instead of ' + str(numberOfItems) + ' columns) - check commas')
                    else:
                        if (items[indexStatus].find('RAISED') != -1) & (mode == 0):
                            #                cnt = 0
                            color = '#EEB066'
                            priStr = items[indexPriority].strip().lower()

                            if priStr == 'high':
                                color = '#FF8080' #red
                            elif priStr == 'med':
                                color = '#EEAA99' #dark-orange
                            elif priStr == 'low':
                                color = '#E0E088' #orange-yellow
                            elif priStr != '':
                                print('WARNING: issue '+str(nIssue) + ': priority undefined')

                            if items[indexStatus].find('RAISED') != -1:
                                deadline = int(items[indexDeadline].replace('-',''))
                                today = int(GetDateStr().replace('-',''))
                                if items[indexType].find('DISCUSSION') != -1:
                                    color = '#AAAAFF'
                                    # if deadline <= today:
                            elif items[indexStatus].find('RESOLVED') != -1:
                                color = '#BBFFBB'

                            if color != '':
                                #fileWrite.write('  <tr style = "background-color:#AAAAFF">\n')  
                                fileWrite.write('  <tr style = "background-color:'+color+'">\n')  
                            else:
                                fileWrite.write('  <tr>\n')
                                
                            for k, s in enumerate(items):
                                if k not in omitItemsHTML: 
                                    txt = s.replace('\\;',',')
                                    fileWrite.write('    <td>' + txt+'</td>\n')
                                    #fileWrite.write('"' + txt + '"')
                            fileWrite.write('  </tr>\n')
                        elif (items[indexStatus].find('RAISED') == -1) & (mode == 1):
                            #numberOfResolved += 1
                            if items[indexStatus].find('RAISED') != -1:
                                deadline = int(items[indexDeadline].replace('-',''))
                                today = int(GetDateStr().replace('-',''))
                                if items[indexType].find('DISCUSSION') != -1:
                                    fileWrite.write('  <tr style = "background-color:#AAAAFF">\n')  
                                else:
                                    if deadline <= today:
                                        fileWrite.write('  <tr style = "background-color:#FFAAAA">\n')
                                    else:
                                        fileWrite.write('  <tr style = "background-color:#FFB266">\n')
                            elif items[indexStatus].find('RESOLVED') != -1:
                                fileWrite.write('  <tr style = "background-color:#BBFFBB">\n')
                            else:
                                fileWrite.write('  <tr>\n')
                            for k, s in enumerate(items):
                                if k not in omitItemsHTML: 
                                    txt = s.replace('\\;',',')
                                    fileWrite.write('    <td>' + txt+'</td>\n')
                                    #fileWrite.write('"' + txt + '"')
                            fileWrite.write('  </tr>\n')
                            
                    #fileWrite.write('\n')
                
                #nLine += 1
    
        fileWrite.write('</table>\n')
        fileWrite.write('\n')
        fileWrite.write('</body>\n')
        fileWrite.write('</html>\n')
    finally:
        fileWrite.close()

    # import shutil
    # shutil.copyfile(trackerFile+'.html', '../../docs/'+trackerFile+'.html')



#%%******************************************************************************************************
def ConvertToLatex(): #convert resolved issues of current release to latex
    fileRead=open(trackerFile+'.txt','r', encoding='utf-8') 
    fileLines = fileRead.readlines()
    fileRead.close()
    
    [releaseString,versionString] = GetReleaseAndVersionString()
    [majorCurrent,microCurrent,minorCurrent] = GetMajorMinorMicroVersion()

    releaseVersionDev = VersionString()
    release = majorCurrent # releaseString.split('.')[0]
    #versionMinor = str(release2).split('.')[1]

    numberOfRaised = len(fileLines)-nHeaderLines   
    numberOfResolved = minorCurrent
    lastChangeDate = fileLines[trackerDateLine].split('=')[1][0:-1] #without EOL
    totalResolved = versionResolved[-1] + numberOfResolved

    bugstr = '' #string will contain bugs
    
    #makes problems? fileWrite=open('../../docs/theDoc/'+trackerFile+'.tex','w')  #write file
    trackerFileTex = '..\\..\\docs\\theDoc\\'+trackerFile+'.tex'
    absPathTexFile = os.path.abspath(trackerFileTex)
    fileWrite=open(absPathTexFile,'w')  #write file
    fileRST=io.open('../../docs/RST/'+trackerFile+'.rst','w', encoding='utf-8')  #write file; utf-8 needed for sphinx

    try:
        sInfo =  'This section contains resolved issues per release and known bugs. Use this information to understand changes compared to previous versions. The author field is omitted if it was Johannes Gerstmayr (JG).\n'
        sInfo += 'The extension \\texttt{.dev1} is not added in the issues list (e.g., 1.2.2.dev1==1.2.2), as it only marks versions that will not be available in pypi with standard pip install, but only with the \\texttt{-}\\texttt{-pre} option or by specifying the exact version name, see versions on \\exuUrl{https://pypi.org/project/exudyn/}{https://pypi.org/project/exudyn/}.\n'
        sInfo += 'BUG numbers refer to the according issue numbers.\n'
        #sInfo += 'For details, see the \\texttt{trackerlog.html} file.\n'
        versionInfo = ''
        versionInfo += '\\noindent General information on current version:\n'
        versionInfo += '\\bi \n'
        versionInfo += '  \\item Exudyn version = '+releaseVersionDev+', \n'
        versionInfo += '  \\item last change = '+lastChangeDate+', \n'
        versionInfo += '  \\item Number of issues = ' + str(numberOfRaised) + ', \n'
        versionInfo += '  \\item Number of resolved issues = '  + str(totalResolved) + ' ('+str(numberOfResolved) +' in current version), \n'
        
        
        fileWrite.write('%automatically generated by issueTracker\n')
        fileWrite.write('%\n')
        fileWrite.write(sInfo)
        fileWrite.write('\n')
        fileWrite.write('\n')
        fileWrite.write('\n')
        fileWrite.write('\n')
        fileWrite.write(versionInfo)
        fileWrite.write('\\ei\n')
        
        fileWrite.write('\n')
        fileWrite.write('\\mysubsection{Resolved issues and resolved bugs}\n')
        fileWrite.write('\\par \\noindent The following list contains the issues which have been {\\bf RESOLVED} in the according version:\n')
        
        #fileWrite.write('\\bi \\footnotesize \n')
        fileWrite.write('\\bi \\setlength\\itemsep{-4pt} \\scriptsize \n') #slightly smaller than footnotesize
    
        fileRST.write('.. role:: textred\n') #defined in docs/_static/custom.css
        fileRST.write('.. role:: textorange\n')
        fileRST.write('.. role:: textblue\n')
        fileRST.write('.. role:: textgreen\n')
        fileRST.write('.. role:: boldred\n')
        fileRST.write('.. role:: boldorange\n')
        fileRST.write('.. role:: boldblue\n')
        fileRST.write('.. role:: boldgreen\n')

        fileRST.write('\n')
        fileRST.write('.. _sec-issuetracker:\n')
        fileRST.write('\n')
        fileRST.write('=============\n')
        fileRST.write('Issue tracker\n')
        fileRST.write('=============\n')
        fileRST.write('\n')
        fileRST.write(LatexString2RST(sInfo))
        fileRST.write('\n')
        fileRST.write(LatexString2RST(versionInfo))
        fileRST.write('\n')

        sRST = ''
        bugRST = ''
        sRST += RSTheaderString('Version '+releaseString, level=1)+'\n'
        
        openRST = ''
        openRST += '\n'+RSTheaderString('Open issues', level=1)+'\n'

        issueList = GetIssues()
        issueListSorted = sorted(issueList, key = lambda i: i['date resolved'])
        resolvedCnt = 0
        issueCnt = 0
        IDS = '  ' #additional space
        rstSpace = '    - '
        previousRelease = (majorCurrent,microCurrent)
        vIssueRelease = 1 #for now
        
        for issue in reversed(issueListSorted):
            si = ''
            rst = ''
            #vIssue = numberOfResolved - resolvedCnt #version in which issue has been resolved
            [vIssueMinor, vIssueMicro] = ResolvedIssues2Version(resolvedCnt, totalResolved)
            rNew = previousRelease
            if vIssueMinor >= 0:
                rNew = (vIssueRelease, vIssueMinor)
            else:
                # rNew = 0.1 #this is the very early version
                rNew = (0,1) #this is the very early version

            if (rNew[0] < previousRelease[0] or 
                (rNew[0] == previousRelease[0] and rNew[1] < previousRelease[1]) ):
                sRST += '\n'
                sRST += RSTheaderString('Version '+str(rNew[0])+'.'+str(rNew[1]), level=1) + '\n'
                previousRelease = rNew
                
            #si += '  \\bi\n'
            #si += '  \\begin{itemize}[label=$\\bullet$]\n'
            si += '  \\begin{itemize} \\setlength\\itemsep{-1pt}\n'
            if issue['author'] != 'JG':
                si += IDS+'  \\item issue author: '+issue['author']+'\n'
                rst += rstSpace+'issue author: '+issue['author']+'\n'
    
            #+++++++++++++++++        
            #description
            si += IDS+'  \\item {description:'+ToLatex(issue['description'])+'}\n'
            rst += rstSpace+'description: '+LatexString2RST(issue['description'], replaceMarkups=True)+'\n'
            
            if len(issue['notes'].strip(' ')) != 0:
                si += IDS+'  \\item {\\bf notes: '+ToLatex(issue['notes'])+'}\n'
                rst += rstSpace+'**notes:** '+LatexString2RST(issue['notes'])+'\n'
    
            #+++++++++++++++++
            #resolved
            si += IDS+'  \\item '
            rst += rstSpace
            if len(issue['date resolved']) != 0:
                si += '  date resolved: {\\bf '+issue['date resolved']+'},\n'
                rst += 'date resolved: **'+(issue['date resolved']).strip()+'**\\ , '
                
            si += 'date raised: '+issue['date raised']+' '
            rst += 'date raised: '+issue['date raised']+' '
            if issue['resolved author'] != 'JG' and len(issue['resolved author']) != 0:
                si += '(resolved by: '+issue['resolved author']+')'
                rst += '\n'+rstSpace+'resolved by: '+issue['resolved author']

            #+++++++++++++++++
            si += '\n  \\ei\n'
            rst += '\n'
            
            # if issue['status'] == 'RESOLVED' and issue['type'] != 'DISCUSSION' and vIssue >= 0:
            if issue['status'] == 'RESOLVED' and issue['type'] != 'DISCUSSION':
                s2 = '  \\item[] {\\bf Version '+str(release)+'.'+str(vIssueMinor)+'.'+str(vIssueMicro)+'}:' #' \\vspace{-6pt} \n'
                rst2 = ' * Version '+str(rNew[0])+'.'+str(rNew[1])+'.'+str(vIssueMicro) + ': '
                # attrPre = ''
                attrPost = ''
                attrPostRST = ''
                if issue['type'] == 'BUG':
                    s2 += ' {\\bf \\color{warningRed}'
                    attrPost = '}'
                    rst2 += ':textred:`'
                    attrPostRST = '` '

                if issue['type'] == 'BUG':
                    s2 += '  resolved BUG '
                    rst2 += 'resolved BUG '
                else:
                    s2 += '  resolved Issue '
                    rst2 += 'resolved Issue '
                s2 += issue['number']+attrPost+': {\\bf '+ToLatex(issue['issue']).strip(' ')+'}\n'
                rst2 += issue['number']+attrPostRST+': '+LatexString2RST(issue['issue'], replaceMarkups=True).strip(' ')+' '
                if issue['type'] != 'BUG':
                    s2 += '('+issue['type'].lower()+')\n'
                    rst2 += '('+issue['type'].lower()+')'
                s2 += '\\vspace{-6pt} '
                rst2 += '\n'

                if vIssueMinor >= 0:
                    fileWrite.write(s2+si) #latex does not include 0.1 issues
                sRST += rst2 + rst
                issueCnt += 1
            elif issue['status'] != 'RESOLVED' and issue['type'] == 'BUG':
                bugstr += '  \\item open {\\bf BUG '+issue['number'] + '}: '
                bugstr += ' {\\bf '+ToLatex(issue['issue']).strip(' ')+'}\n'
                bugstr += si
                bugRST += ' * :textred:`open BUG '+issue['number'] +':` ' + LatexString2RST(issue['issue'], replaceMarkups=True) + '\n'
                bugRST += rst + '\n'
            elif issue['status'] != 'RESOLVED':
                preRST = '**'
                postRST = '**'
    
                if issue['priority'].lower() == 'high' :
                    preRST = ':textred:`' #bold not needed, as item heading is anyway bold ...
                    postRST = '`'
                elif issue['priority'].lower() == 'med' :
                    preRST = ':textorange:`'
                    postRST = '`'
                elif issue['priority'].lower() == 'low' :
                    preRST = ':textblue:`'
                    postRST = '`'

                openRST += ' * '+preRST+'open issue '+issue['number'] + ':' + postRST + ' ' + LatexString2RST(issue['issue'], replaceMarkups=True) + '\n'
                openRST += rst + '\n'
                
            if issue['status'] == 'RESOLVED':
                resolvedCnt += 1        
        
        if issueCnt == 0:
            fileWrite.write('\\item[]\n %dummy item in order to avoid problems if list is empty\n')
        fileWrite.write('\\ei\n')
        
        fileWrite.write('\\mysubsection{Known open bugs}\n')
        if len(bugstr) != 0:
            fileWrite.write('\\bi \\footnotesize \n')
            fileWrite.write(bugstr)
            fileWrite.write('\\ei\n')

        sRST += openRST

        sRST += RSTheaderString('Known bugs', level=1)+'\n'
        sRST += bugRST
        fileRST.write(sRST+'\n')
        
        fileWrite.write('%end of file\n')
    finally:
        fileWrite.close()
        fileRST.close()








#%%******************************************************************************************************
#%%******************************************************************************************************
def IssueDictToList(d): #convert dict to sorted issue list; replace ',' with '\;'
    listDest = []
    for i in range(numberOfItems): listDest+=['']

    #check if all keys are valid and put into right order!
    for key in d.keys():
        if (key in trackerItems):
            k = trackerItems.index(key)
            txt = d[key]
            txt = txt.replace(',','\\;')
            
            if key=='issue':
                nChar = len(txt)
                if nChar < 20: txt += ' '*(20-nChar)

            if key=='status':
                nChar = len(txt)
                if nChar < 8: txt += ' '*(8-nChar)
            
            if key=='description':
                if txt[0] != ' ': txt = ' ' + txt
                
            listDest[k] = txt
        else: 
            print('ERROR: invalid key "' + key + '"')

    return listDest
    

#%%******************************************************************************************************
#use this to completely define a new issue
def RaiseIssueDict(issueDict): #raise a new issue into list (append to end of list)
    IssueTrackerBackup()


    if 'number' in issueDict: print('WARNING: issue number "' + issueDict['number'] + '" ignored; issue added to end of list')
    if not('date raised' in issueDict): issueDict['date raised'] = GetDateStr()


    issueDict['status'] = 'RAISED'
    issueDict['date raised'] = GetDateStr()

    listDest = IssueDictToList(issueDict)
    numStr = str(NumberOfIssues())
    while len(numStr) < 4: numStr = '0' + numStr
    listDest[0] = numStr

    newIssue = ''
    cnt = 0
    for item in listDest:
        newIssue += item
        if cnt < numberOfItems-1:
            newIssue += ','
        cnt += 1
    
    newIssue += '\n'
#    print(listDest)
#    print(newIssue)
    
    fileWrite=open(trackerFile+'.txt','a', encoding='utf-8')  #append to file
    fileWrite.write(newIssue)            
    fileWrite.close()

    UpdateDateAndVersion(updateVersion=False) #do not change version files!
    ConvertToHTML() #update html version of issue tracker
    ConvertToLatex() #update latex issues in docu (only contains resolved issues in version and bugs)

    #report the number that was actually assigned, and return it: it is needed for the commit
    #message and the documentation, and reconstructing it by hand afterwards gets it wrong
    print('issue raised: #' + str(int(numStr)) + ' "' + str(issueDict['issue']).strip() + '"')

    return int(numStr)

#%%******************************************************************************************************
#modify an existing issue
def ModifyDictIssue(issueDict): #raise a new issue into list (append to end of list)
    IssueTrackerBackup()
    
    listDest = IssueDictToList(issueDict)

    issueNumber = 0
    if not('number' in issueDict): 
        print('ERROR: issue \'number\' needed in ModifyDictIssue(...)')
        return
    else: 
        issueNumber = int(issueDict['number'])

    numStr = str(issueNumber)
    while len(numStr) < 4: numStr = '0' + numStr
    listDest[0] = numStr

    newIssue = ''
    cnt = 0
    for item in listDest:
        newIssue += item
        if cnt < numberOfItems-1:
            newIssue += ','
        cnt += 1
    
    newIssue += '\n'

    fileRead=open(trackerFile+'.txt','r', encoding='utf-8') 
    fileLines = fileRead.readlines()
    fileRead.close()
    
    fileLines[issueNumber+nHeaderLines] = newIssue

    fileWrite=open(trackerFile+'.txt','w', encoding='utf-8') 
    for line in fileLines:
        fileWrite.write(line)
        
    fileWrite.close()

    ConvertToHTML() #update html version of issue tracker
    ConvertToLatex() #update html version of issue tracker

#%%******************************************************************************************************
#use this to resolve an issue
def ChangeIssue(issueNumber, key, value): #raise a new issue into list (append to end of list)
    IssueTrackerBackup()

    if not((issueNumber >= 0) and (issueNumber < NumberOfIssues())):
        print('ChangeIssue: invalid number! Nothing done')
        return
    
    d = GetIssue(issueNumber)

    if key not in d:
        print('ChangeIssue: key "' + key + '" not available!')
        return
    
    
    d[key] = value
    
    ModifyDictIssue(d)
    
    print('new issue:')
    print(d)
    
    UpdateDateAndVersion()
    ConvertToHTML() #update html version of issue tracker
    ConvertToLatex() #update html version of issue tracker

#%%******************************************************************************************************
#use this to resolve an issue
def ResolveIssue(issueNumber, notes='', author='JG'): #raise a new issue into list (append to end of list)
    IssueTrackerBackup()

    if not((issueNumber >= 0) and (issueNumber < NumberOfIssues())):
        print('Issue: invalid number! Nothing done')
        return None

    d = GetIssue(issueNumber)
    
    d['status'] = 'RESOLVED'
    d['date resolved'] = GetDateTimeStr()
    d['resolved author'] = author
    if d['notes'] != '':
        notes = d['notes'] + '; ' + notes
    d['notes'] = notes
    
    ModifyDictIssue(d)
    
    #state the number first, so it can be copied into the commit message without recomputing it
    print('issue resolved: #' + str(issueNumber) + ' "' + str(d['issue']).strip() + '"')
    print(d)

    UpdateDateAndVersion()
    ConvertToHTML() #update html version of issue tracker
    ConvertToLatex() #update html version of issue tracker

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
        
    d={'issue': issueName, 
       'author': author,
       'description': description, 
       'type': issueType, 
       'file': fileName, 
       'line': lineNumber, 
       'deadline': deadline,
       'priority': priority}
    return RaiseIssueDict(d) #the assigned issue number
    #['number', 'issue', 'author', 'description', 'type', 'status', 'priority', 'date raised', 'deadline', 'date resolved', 'resolved author', 'notes']

    print(GetIssue(NumberOfIssues()-1))
    
    
    