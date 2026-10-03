#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Deprecation of the functions and arguments of the Python library (#2807): a function is
#           decorated with Deprecated(since, expires, use), an argument reported with
#           DeprecatedArgument(name, since, expires, use) where the function sees it. Both report
#           through exudyn.special.deprecations - a DeprecationWarning once per session and name,
#           counted in exudyn.sys['deprecationUse']['library'] - as the deprecations of the C++ side
#           do. Every deprecation of Exudyn carries the version it was deprecated in and the year it
#           is removed; tools/checkDeprecations.py lists them all and fails for one whose year has come.
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-03
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import functools
import sys

import exudyn

__all__ = ['Deprecated', 'DeprecatedArgument']


def _Message(name, since, expires, use):
    """the text of the warning"""
    return (name + ' is deprecated since ' + str(since) + ' and removed in ' + str(expires)
            + ('; use ' + use + ' instead' if use else ''))


def _LibraryName(moduleName, functionName):
    """the name under which a deprecation of the library is reported: module and function, without 'exudyn.'"""
    if moduleName.startswith('exudyn.'):
        moduleName = moduleName[len('exudyn.'):]
    return moduleName + '.' + functionName


class Deprecated:
    """decorator of a deprecated function of the library: @Deprecated('1.11.0', 2029, use='mbs.CreateRigidBody');
    each call reports the use, the function itself is unchanged; its docstring starts with DEPRECATED, which
    tools/checkDeprecations.py checks"""
    def __init__(self, since, expires, use=''):
        self.since = since
        self.expires = expires
        self.use = use

    def __call__(self, function):
        name = _LibraryName(function.__module__, function.__qualname__)
        message = _Message(name, self.since, self.expires, self.use)

        @functools.wraps(function)
        def DeprecatedFunction(*args, **kwargs):
            #stackLevel 2: the warning names the line that called the deprecated function
            exudyn.special.deprecations.Warn('library', name, message, 2)
            return function(*args, **kwargs)
        return DeprecatedFunction


def DeprecatedArgument(argument, since, expires, use='', function=None, stackLevel=3):
    """report a deprecated argument of a library function; called by that function where it sees the argument given;
    function is the name it is reported under (default: the module and name of the calling function); stackLevel 3
    names the line that called the calling function, one more for each helper in between"""
    if function is None:
        frame = sys._getframe(1)
        function = _LibraryName(frame.f_globals.get('__name__', ''), frame.f_code.co_name)
    name = function + '.' + argument
    exudyn.special.deprecations.Warn('library', name, _Message('the argument ' + name, since, expires, use), stackLevel)
