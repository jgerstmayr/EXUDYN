# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# exudyn.misc - what is not a modelling module
#
# The modules next to this one in exudyn/ are for building and simulating models. The ones in here
# are not: docmeta and extensionRegistry are machinery that the other modules use, GUI and
# resultsMonitor are tools with a user interface, overrideSettings reads and writes
# ~/.exudyn/config.json, and mainSystemExtensions is the mechanism that adds the Create...
# functions to MainSystem.
#
# Moved here (#2552); nothing but the location changed.
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#the subpackage itself exports nothing; import exudyn.misc.<module> directly
__all__ = []
