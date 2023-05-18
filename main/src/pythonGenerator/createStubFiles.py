# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN autogeneration file
#
# Details: 	used to create a single __init__.pyi file for the exudyn module;
#           alleviates auto-completion in Spyder or VScode
#
# Author:   Johannes Gerstmayr
# Date:     2023-05-09 (created)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import copy #for deep copies
import io   #for utf-8 encoding

rstFolder = 'docs/RST/' #folder where generated .rst files are stored

sourceDir=''
destFile=  '../../../main/pythonDev/exudyn/__init__.pyi'

#main files
filesParsed=[
              'stubHeader.pyi',
              'generated/stubEnums.pyi',
              'generated/stubSystemStructures.pyi',
              'generated/stubAutoBindings.pyi',
              'generated/stubAutoBindingsExt.pyi',
            ]

mergedFile=''
for fileName in filesParsed:
    file=open(sourceDir+fileName,'r') 
    # fileLines = file.readlines()
    mergedFile += file.read()
    file.close()


if True: 
    file=io.open(destFile,'w',encoding='utf8')
    file.write(mergedFile)
    file.close()




