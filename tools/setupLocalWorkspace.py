#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Creates the local, UNTRACKED working files from their committed templates. Run once
#           after cloning; running it again is harmless, because an existing file is never
#           overwritten.
#
#               exudynTemplate.sln                   ->  exudyn.sln
#               python/pytestTemplate.py             ->  python/pytest.py
#               tools/vscodeCppPropertiesTemplate.json ->  .vscode/c_cpp_properties.json
#
#           WHY: both files are scratch space. The solution accumulates per-machine state, and
#           pytest.py is the file used to try something out in Visual Studio with mixed
#           Python/C++ debugging. Previously the experimental pytest.py had to be overwritten by
#           hand with the default version before committing - easy to forget, and the mistake is
#           invisible in review. Now the working copies are in .gitignore, so an experiment
#           CANNOT be committed, and the committed templates never move.
#
#           Consequence: a fresh clone has no exudyn.sln and no python/pytest.py, so Visual Studio
#           shows pytest.py as missing until this has been run once. That is the trade for making
#           the mistake impossible.
#
# Usage:    python tools/setupLocalWorkspace.py
#           python tools/setupLocalWorkspace.py --force   overwrite existing local files
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-12 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import os
import shutil
import sys

#template -> local working copy, both relative to the repository root
localFiles = [
    ('exudynTemplate.sln', 'exudyn.sln'),
    ('python/pytestTemplate.py', 'python/pytest.py'),
    #without this VS Code cannot follow a single #include of the C++ sources (#2619)
    ('tools/vscodeCppPropertiesTemplate.json', '.vscode/c_cpp_properties.json'),
    ]


#%%******************************************************************************************************
def GetRepositoryRoot():
    """Absolute path of the repository root; this tool lives in tools/."""
    here = os.path.dirname(os.path.abspath(__file__))

    return os.path.normpath(os.path.join(here, '..'))


#%%******************************************************************************************************
def Main():
    parser = argparse.ArgumentParser(
        description='Create the local untracked working files from their committed templates.')
    parser.add_argument('--force', action='store_true',
                        help='overwrite an existing local file (DISCARDS your experiments)')
    args = parser.parse_args()

    repositoryRoot = GetRepositoryRoot()
    created = 0
    for template, local in localFiles:
        templatePath = os.path.join(repositoryRoot, template)
        localPath = os.path.join(repositoryRoot, local)

        if not os.path.isfile(templatePath):
            print('ERROR: missing template ' + template)
            return 1

        if os.path.isfile(localPath) and not args.force:
            print('kept    ' + local + '   (already exists; --force overwrites)')
            continue

        localDirectory = os.path.dirname(localPath)
        if localDirectory != '' and not os.path.isdir(localDirectory):
            os.makedirs(localDirectory)      #.vscode/ does not exist in a fresh clone
        shutil.copyfile(templatePath, localPath)
        print('created ' + local + '   from ' + template)
        created += 1

    print('')
    print(str(created) + ' file(s) created. All of them are in .gitignore, so anything you do in')
    print('them stays local - open exudyn.sln in Visual Studio, experiment in python/pytest.py,')
    print('and VS Code follows the C++ includes.')

    return 0


#%%******************************************************************************************************
if __name__ == '__main__':
    sys.exit(Main())
