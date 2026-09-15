# -*- coding: utf-8 -*-
"""
Created on Fri Apr 29, 08:53:30 2022

@author: Johannes Gerstmayr

goal: scripts run only when all binaries are built, changing e.g. latex files for documenations
"""

import os, sys  #generatorPaths and the shared helpers live in tools/generators/ (revision2026 step R4.3, part 2g)
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'tools', 'generators'))
import os
import datetime

#++++++++++++++++++++++++++++++++++++++++++
#add date and time into latex and .rst file
def NumTo2digits(n):
    return '0'*(n<10)+str(n)

now=datetime.datetime.now()
buildDateString = str(now.year) + '-' + NumTo2digits(now.month) + '-' + NumTo2digits(now.day)
buildDateString += '  ' + NumTo2digits(now.hour) + ':' + NumTo2digits(now.minute)# + ':' + NumTo2digits(now.second)
buildDateString = 'build date and time='+buildDateString

import generatorPaths as paths
fileDate =open(paths.theDocDir+'buildDate.tex','w',encoding='utf8')  #clear file by one write access
fileDate.write(buildDateString)
fileDate.close()

#++++++++++++++++++++++++++++++++++++++++++

        