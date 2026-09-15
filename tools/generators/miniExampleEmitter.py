#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Emits python/TestModels/MiniExamples/<Item>.py for every item that defines a
#           miniExample, and miniExamplesFileList.py, from definitions/ (revision2026 step R4.3,
#           part 2b). Moved out of src/pythonGenerator/pythonAutoGenerateObjects.py; byte-identical.
#
# Usage:    python tools/generators/miniExampleEmitter.py [--output-dir DIR]
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-14 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import os
import sys

toolsDirectory = os.path.dirname(os.path.abspath(__file__))
if toolsDirectory not in sys.path:
    sys.path.insert(0, toolsDirectory)

import itemModel as im                                  # noqa: E402
from autoGenerateHelper import RemoveIndentation        # noqa: E402

space4 = '    '


#function which writes the mini examples for every item into a separate file
def WriteMiniExample(className, miniExample, outputDir):
    s=''
    # s+= '#+++++++++++++++++++++++++++++++++++++++++++\n'
    # s+= '# Mini example for class ' + className + '\n'
    # s+= '#+++++++++++++++++++++++++++++++++++++++++++\n\n'
    
    s+= '#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n'
    s+= '# This is an EXUDYN example\n'
    s+= '# \n'
    s+= '# Details:  Mini example for class ' + className + '\n'
    s+= '# \n'
    s+= "# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.\n"
    s+= '# \n'
    s+= '#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n'

    s+= 'import sys\n'
    s+= "sys.path.append('../TestModels')\n"
    s+= "sys.path.append('../../TestModels') #for direct run in directory\n\n"
    s+= 'import exudyn as exu\n'

    s+= 'from exudyn.utilities import *\n'
    s+= 'import exudyn.graphics as graphics\n\n'
    s+= 'from modelUnitTests import ExudynTestStructure, exudynTestGlobals\n'
    s+= 'import numpy as np\n'
    s+= '\n'
    s+= '#create an environment for mini example\n'
    s+= 'SC = exu.SystemContainer()\n'
    s+= 'mbs = SC.AddSystem()\n'
    s+= '\n'
    s+= 'oGround=mbs.AddObject(ObjectGround(referencePosition= [0,0,0]))\n'
    s+= 'nGround = mbs.AddNode(NodePointGround(referenceCoordinates=[0,0,0]))\n'
    s+= '\n'
    # s+= 'testError=1 #set default error, if failed\n'
    #s+= 'exu.Print("start mini example for class ' + className + '")\n'
    #s+= 'try: #puts example in safe environment\n'
    #s+= miniExample
    s+= RemoveIndentation( miniExample, removeAllSpaces=False)
    s+= '\n'
    #s+= 'except BaseException as e:\n'
    #s+= space4+'exu.Print("An error occured in test example for ' + className + ':", e)\n'
    #s+= 'else:\n'
    #s+= space4+'exu.Print("example for ' + className + ' completed, test result =", exudynTestGlobals.testResult)\n'
    s+= 'exu.Print("example for ' + className + ' completed, test result =", exudynTestGlobals.testResult)\n'
    s+= '\n'
    
    fileExample=open(outputDir+className+'.py','w',encoding='utf8') 
    fileExample.write(s)
    fileExample.close()



def main(argv=None):
    parser = argparse.ArgumentParser(description='Emit the item mini examples from definitions/.')
    parser.add_argument('--output-dir', default=os.path.join(im.repositoryRoot, 'python', 'TestModels', 'MiniExamples'),
                        help='directory to write into (default: python/TestModels/MiniExamples)')
    args = parser.parse_args(argv)
    outputDir = args.output_dir.replace(os.sep, '/').rstrip('/') + '/'

    miniExamplesList = []
    for definition in im.ItemDefinitions():
        miniExample = definition.get('miniExample', '') or ''
        if len(miniExample) != 0:
            WriteMiniExample(definition['className'], miniExample, outputDir)
            miniExamplesList += [definition['className']+'.py']

    fileExampleList=open(outputDir+'miniExamplesFileList.py','w',encoding='utf8') 
    s = '#this file provides a list of file names for mini examples\n'
    s+= '\n'
    s+= 'miniExamplesFileList = ['
    sepStr = ''
    for item in miniExamplesList:
        s+= sepStr + "'" + item + "'"
        sepStr=',\n'
    s+= ']\n'
    s+= '\n'
    fileExampleList.write(s)
    fileExampleList.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
