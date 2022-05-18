# -*- coding: utf-8 -*-
"""
Created on Fri Apr 29, 08:53:30 2022

@author: Johannes Gerstmayr

goal: scripts run only when all binaries are built, changing e.g. latex files for documenations
"""

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

#current directory is main/src/pythonGenerator
fileDate =open('../../../docs/theDoc/buildDate.tex','w')  #clear file by one write access
fileDate.write(buildDateString)
fileDate.close()

#++++++++++++++++++++++++++++++++++++++++++

        