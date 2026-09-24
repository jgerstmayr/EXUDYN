#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool - part of the exudev driver, see tools/exudev/README.md
#
# Details:  Reports what is installed in ONE environment, and optionally checks the exudyn version.
#           This is the only part of the driver that runs INSIDE a conda environment; everything
#           else runs in whatever interpreter started exudev and never imports exudyn.
#
#           It is a file rather than a 'python -c' string because 'conda run' refuses any argument
#           containing a newline (conda/utils.py: 'assert not any("\n" in arg ...)'), so a multi-line
#           probe cannot be passed on the command line at all.
#
# Usage:    python tools/exudev/probe.py                   report python, exudyn and numpy
#           python tools/exudev/probe.py 1.11.160.dev1     also exit non-zero on another version
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-18 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import sys

expectedVersion = sys.argv[1] if len(sys.argv) > 1 else None

print('  python  ' + sys.version.split()[0] + '   ' + sys.executable)

installedVersion = None
for name in ['exudyn', 'numpy', 'scipy', 'matplotlib']:
    try:
        module = __import__(name)
        version = getattr(module, '__version__', '?')
        print('  ' + name.ljust(11) + version)
        if name == 'exudyn':
            installedVersion = version
            print('  ' + ''.ljust(11) + module.__file__)
    except Exception as error:           #noqa: BLE001 - any import problem is reported, not raised
        print('  ' + name.ljust(11) + 'NOT INSTALLED (' + str(error) + ')')

if expectedVersion is not None:
    if installedVersion != expectedVersion:
        print('*** the installed exudyn is ' + str(installedVersion) + ', expected '
              + expectedVersion + ' - the build did not reach this environment')
        sys.exit(1)
    print('  exudyn matches version.txt')

sys.exit(0)
