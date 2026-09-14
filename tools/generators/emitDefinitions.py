#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Writes objectDefinition.py and systemStructuresDefinition.py out in the NEW format -
#           real Python, one file per category, under definitions/ at the repository root.
#
#           It runs the two generators with EXUDYN_EMIT_DEFINITIONS set. They parse the old
#           format exactly as they always do and hand their in-memory representation to
#           definitionEmitter.py, so there is no second parser to drift. Their normal output is
#           unaffected - tools/regenerate.py --check stays a no-op.
#
#           Revision plan step 31a. The generators do not read the emitted files yet - see
#           tools/generators/README.md.
#
# Usage:    python tools/generators/emitDefinitions.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-13 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os
import subprocess
import sys

#the generators use relative paths throughout and must run from their own directory
generatorDirectory = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  '..', '..', 'src', 'pythonGenerator')

generators = [
    ('pythonAutoGenerateSystemStructures.py', 'systemStructuresDefinition.py'),
    ('pythonAutoGenerateObjects.py', 'objectDefinition.py'),
    ]


#%%******************************************************************************************************
def Main():
    environment = dict(os.environ)
    environment['EXUDYN_EMIT_DEFINITIONS'] = '1'

    for generator, source in generators:
        print('=' * 99)
        print('running ' + generator + '  (' + source + ')')
        print('=' * 99)
        result = subprocess.run([sys.executable, generator],
                                cwd=os.path.normpath(generatorDirectory), env=environment)
        if result.returncode != 0:
            print('FAILED: ' + generator + ' returned ' + str(result.returncode))
            return result.returncode

    print('')
    print('emitted to definitions/ at the repository root - nothing reads these yet.')
    print('The generators wrote their usual output too; run tools/regenerate.py --check to confirm')
    print('it is unchanged.')

    return 0


#%%******************************************************************************************************
if __name__ == '__main__':
    sys.exit(Main())
