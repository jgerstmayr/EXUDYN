#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  All user functions at once (#2671). Since revision2026b step RG12.4 (#2664) every user
#           function is an ordinary Python def in definitions/, and FOUR things are generated from
#           that one source:
#
#               the documentation block of the item page
#               the entry of userFunctionArgsDict in itemInterface.py
#               the Protocol class in itemInterface.py
#               the check against the C++ std::function of the parameter's type
#
#           The generators check them against each other at generation time. Nothing checked that
#           what was SHIPPED agrees - and the item test models exercise a handful of user functions,
#           never the set. These tests read only the installed package, so they fail if a generated
#           file is stale, if a Protocol was dropped from __all__, or if an argument was renamed in
#           one place and not the other.
#
#           They deliberately do NOT run a simulation: a model per user function is the job of the
#           test models, and would test the solver rather than the interface.
#
# Usage:    pytest python/testing/test_userFunctions.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-26
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import inspect

import pytest

import exudyn.itemInterface as itemInterface
from exudyn.itemInterface import userFunctionArgsDict


def ItemEntries():
    """the (item, user function) entries that belong to an item; MainSystem's own are not items"""
    return [key for key in userFunctionArgsDict if not key.startswith('MainSystem,')]


def Protocols():
    """{name: class} of the generated Protocol classes of itemInterface"""
    #typing.Protocol itself is imported into that namespace and is a protocol class too
    return {name: value for (name, value) in vars(itemInterface).items()
            if inspect.isclass(value) and getattr(value, '_is_protocol', False)
            and value.__module__ == itemInterface.__name__}


def test_everyEntryHasItsProtocol():
    """the fourth field of an entry names a Protocol that the module really has"""
    protocols = Protocols()
    missing = []
    for key in ItemEntries():
        entry = userFunctionArgsDict[key]
        assert len(entry) == 4, key + ': no Protocol name in the entry'
        name = entry[3][0]
        if name not in protocols:
            missing.append(key + ' -> ' + name)
    assert missing == [], 'Protocol classes that itemInterface does not have: ' + str(missing)


def test_protocolAndRegistryAgreeOnTheArguments():
    """the Protocol's __call__ has the argument names the registry has, in the same order

    These come from the same def, so a difference means one of the two was generated from something
    else - a stale itemInterface.py is the likely reason, and it is what a user's editor would then
    disagree with."""
    protocols = Protocols()
    for key in ItemEntries():
        entry = userFunctionArgsDict[key]
        protocol = protocols[entry[3][0]]
        signature = inspect.signature(protocol.__call__)
        names = [name for name in signature.parameters if name != 'self']
        assert names == entry[1], (key + ': the Protocol takes ' + str(names)
                                   + ', the registry ' + str(entry[1]))
        assert len(entry[0]) == len(entry[1]), (key + ': ' + str(len(entry[0])) + ' types for '
                                                + str(len(entry[1])) + ' arguments')


def test_noArgumentIsStillCalledArgN():
    """every argument of an item's user function is named, not arg0

    Before RG12.4 the registry knew the types and not the names, so it filled in arg0, arg1, ...
    Every item's user function is a def now, so nothing should be left."""
    unnamed = [key for key in ItemEntries()
               if any(name.startswith('arg') and name[3:].isdigit()
                      for name in userFunctionArgsDict[key][1])]
    assert unnamed == [], 'user functions whose arguments are still arg0, arg1, ...: ' + str(unnamed)


def test_theFirstArgumentIsAlwaysMbs():
    """a user function of an item is called with the MainSystem first; a page that says otherwise
    would be describing something that cannot happen"""
    for key in ItemEntries():
        entry = userFunctionArgsDict[key]
        assert entry[1][0] == 'mbs', key + ': the first argument is ' + entry[1][0]
        assert entry[0][0] == 'MainSystem', key + ': the first type is ' + entry[0][0]


def test_everyProtocolIsExported():
    """a Protocol a user cannot import is of no use to them"""
    notExported = [name for name in Protocols() if name not in itemInterface.__all__]
    assert notExported == [], 'Protocol classes missing from __all__: ' + str(notExported)


def test_everyProtocolHasADocstring():
    """the Protocol carries the documentation of the user function, which is what an editor shows"""
    withoutText = [name for (name, protocol) in Protocols().items()
                   if not (protocol.__doc__ or '').strip()]
    assert withoutText == [], 'Protocol classes without a docstring: ' + str(withoutText)


def test_anItemAcceptsAFunctionForItsUserFunction():
    """the item class takes a Python function where its user function is, and keeps it

    The parameter is annotated with the Protocol since RG12.4.4; an annotation must not turn into a
    check that rejects an ordinary function, which is exactly what a wrong annotation would do."""
    def UserFunction(*arguments):
        return 0

    checked = 0
    for key in ItemEntries():
        (itemName, parameterName) = key.split(',')
        itemClass = getattr(itemInterface, itemName, None)
        visualizationClass = getattr(itemInterface, 'V' + itemName, None)
        for owner in [itemClass, visualizationClass]:
            if owner is None:
                continue
            if parameterName not in inspect.signature(owner).parameters:
                continue
            item = owner(**{parameterName: UserFunction})
            assert getattr(item, parameterName) is UserFunction, (
                key + ': the item did not keep the function it was given')
            checked += 1
    assert checked >= len(ItemEntries()), (str(checked) + ' of ' + str(len(ItemEntries()))
                                           + ' user function parameters were found on their class')


@pytest.mark.parametrize('key', sorted(ItemEntries()))
def test_theTypesAreOnesTheInterfaceKnows(key):
    """a type in the registry is one of the C++ types the interface exchanges

    A new type reaching this dictionary without anyone noticing is how a user function ends up
    documented as something the core cannot pass."""
    known = set(['MainSystem', 'Real', 'Index', 'bool', 'StdVector', 'StdVector2D', 'StdVector3D',
                 'StdVector6D', 'StdMatrix3D', 'StdMatrix6D', 'NumpyMatrix', 'StdArrayIndex',
                 'ConfigurationType', 'py::object'])
    entry = userFunctionArgsDict[key]
    unknown = [name for name in entry[0] + entry[2] if name not in known]
    assert unknown == [], key + ': types the interface does not know: ' + str(unknown)
