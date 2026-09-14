#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Assembles the SHIPPED module python/exudyn/mainSystemExtensions.py: it copies
#           src/pythonGenerator/mainSystemExtensionsHeader.py and appends one link
#           exu.<Class>.<Function> = <module>.<Function> for every utility function whose #**
#           comment carries belongsTo (revision plan step 33, part 2e). Moved out of
#           utilitiesDocuGenerator.py; plan step 35 replaces this assembly by an @extends registry.
#
# Usage:    python tools/generators/mainSystemExtensionsEmitter.py
#
# Author:   Johannes Gerstmayr
# Date:     2020-06-09 (created as utilitiesDocuGenerator.py), 2026-09-14 (emitter)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import io
import os
import sys

toolsDirectory = os.path.dirname(os.path.abspath(__file__))
if toolsDirectory not in sys.path:
    sys.path.insert(0, toolsDirectory)

from utilityDocsModel import *                                                   # noqa: E402,F403


def main():
    with open(paths.pythonGeneratorDir+'mainSystemExtensionsHeader.py','r',encoding='utf8') as f:
        pyExtensions = f.read()

    print('*** updating exudyn.mainSystemExtensions.py ***')
    #write the header first: mainSystemExtensions.py is one of the parsed files
    file=io.open(fileDir+'mainSystemExtensions.py','w',encoding='utf8')  #clear file by one write access
    file.write(pyExtensions)
    file.close()

    for fileName in filesParsed:
        [functionList,classList,header] = ParsePythonFile(fileDir+fileName)
        moduleName, moduleNameLatex, moduleNamePython = ModuleNames(fileName)
        for funcDict in functionList:
            if 'belongsTo' not in funcDict:
                continue
            belongsTo = funcDict['belongsTo'].strip()
            functionName = funcDict['functionName']
            #exu.MainSystem.PlotSensor = exu.plot.PlotSensor
            moduleAdd = ('exu.'+moduleNamePython+'.')*(moduleNamePython!='mainSystemExtensions')

            sPy = '\n#link '+belongsTo+' function to Python function:\n'
            sPy += 'exu.'+belongsTo+'.'+functionName.replace(belongsTo,'')+ '=' +  moduleAdd+functionName + '\n\n'
            pyExtensions += sPy

    file=io.open(fileDir+'mainSystemExtensions.py','w',encoding='utf8')  #clear file by one write access
    file.write(pyExtensions)
    file.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
