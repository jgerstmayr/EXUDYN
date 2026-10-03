#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The functions added to MainSystem (@extends) reach editors and type checkers through the
#           stub, which assigns them to the class (#2825); so their return types are their own
#           annotations (#2826). Every registered function has one, and it is the type its docstring
#           names under Returns:.
#
# Usage:    pytest python/testing/test_extensionAnnotations.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-04
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import inspect
import re

import pytest

import exudyn                                                                # noqa: F401 - installs the extensions
from exudyn.misc.extensionRegistry import _registry

registered = [(cls.__name__ + '.' + methodName, function) for (cls, methodName, function) in _registry]


def test_extensionsAreRegistered():
    assert len(registered) >= 30


@pytest.mark.parametrize('name,function', registered, ids=[name for (name, function) in registered])
def test_everyAddedFunctionHasItsReturnAnnotation(name, function):
    annotation = inspect.signature(function).return_annotation
    assert annotation is not inspect.Signature.empty, name + ' has no return annotation'
    documented = re.search(r'Returns:\s*\n\s*:([^:]*):', inspect.getdoc(function) or '')
    assert documented is not None, name + ': no type under Returns:'
    documented = documented.group(1).strip()
    text = inspect.formatannotation(annotation)
    if documented.startswith('['):
        assert annotation is list, name + ': ' + text + ' for ' + documented
    elif documented == 'None':
        assert annotation is None, name + ': ' + text + ' for ' + documented
    else:
        assert text.replace('exudyn.exudynCPP.', '').replace('typing.', '') == documented, name + ': ' + text + ' for ' + documented
