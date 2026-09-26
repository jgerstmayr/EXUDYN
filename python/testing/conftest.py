#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  What every test in this directory needs before exudyn is imported. pytest loads a
#           conftest.py before the test modules beside it, which is the only place early enough.
#
#           The user settings of ~/.exudyn/config.json are ignored: a maintainer who stores a
#           setting must not thereby change what a test computes (revision2026b step RG12.5,
#           #2666). The runners do the same, and a child process inherits it.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-26
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import os

os.environ['EXUDYN_NO_USER_SETTINGS'] = '1'
