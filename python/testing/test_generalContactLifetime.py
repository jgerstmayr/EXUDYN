#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The GeneralContact that AddGeneralContact and GetGeneralContact return keeps its system alive (#1512):
#           it refers to memory of the system, so a script that keeps only the GeneralContact must not let the
#           system go. Checked by the reference count of the system, and by using the GeneralContact after the
#           script dropped its system.
#
# Usage:    pytest python/testing/test_generalContactLifetime.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-05
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import gc
import sys

import exudyn as exu


def test_generalContactKeepsItsSystemAlive():
    SC = exu.SystemContainer()
    mbs = SC.AddSystem()
    before = sys.getrefcount(mbs)
    contact = mbs.AddGeneralContact()
    assert sys.getrefcount(mbs) == before + 1
    assert mbs.GetGeneralContact(0) is contact
    del mbs
    gc.collect()
    contact.verboseMode = 2     #the system is still there
    assert contact.verboseMode == 2
