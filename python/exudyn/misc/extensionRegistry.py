#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN python utility library
#
# Details:  Registry that binds Python functions as methods of the C++ classes (e.g. MainSystem).
#           A function is marked where it is defined, with @extends(exudyn.MainSystem); install()
#           binds all marked functions and raises if a name already exists as a C++ method.
#           install() is called at the end of mainSystemExtensions.py.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-15 (created; revision2026 step R4.5)
#
# Copyright:This file is part of Exudyn. Exudyn is free software. You can redistribute it and/or modify it under the terms of the Exudyn license. See 'LICENSE.txt' for more details.
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

#public API of this module; kept complete by tools/checkAll.py (#2444)
__all__ = [
    'extends', 'install',
    ]

_registry = [] #(class, method name, function) in registration order


def extends(cls, name=None):
    """Mark a function to become method 'name' of class cls. Without name, the method name is the
    function name with the class name removed from its front (MainSystemCreateMassPoint ->
    CreateMassPoint, PlotSensor -> PlotSensor). The function itself is returned unchanged."""
    def Register(function):
        methodName = name
        if methodName is None:
            methodName = function.__name__
            if methodName.startswith(cls.__name__):
                methodName = methodName[len(cls.__name__):]
        _registry.append((cls, methodName, function))
        return function
    return Register


def install():
    """Bind every registered function to its class. A name that already exists on the class raises
    AttributeError, unless it was bound by an earlier install() of the same function (re-import)."""
    for cls, methodName, function in _registry:
        key = (methodName, function.__qualname__)
        if hasattr(cls, methodName) and getattr(getattr(cls, methodName), '_exudynExtension', None) != key:
            raise AttributeError('extensionRegistry.install(): ' + cls.__name__ + '.' + methodName
                                 + ' already exists; the Python function ' + function.__module__ + '.'
                                 + function.__qualname__ + ' cannot be bound under this name')
        function._exudynExtension = key
        setattr(cls, methodName, function)
