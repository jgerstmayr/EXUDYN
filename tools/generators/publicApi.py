#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  The rule for the public names of a Python module of the exudyn package, and the text of
#           its __all__ (revision2026 step R4.22.3). Public are the top-level functions, classes and
#           assigned names (also inside top-level if/try blocks) that do not start with '_',
#           except functions and classes marked @docmeta(public=False). Imported names are never
#           public, so 'from module import *' exports what the module defines, not what it uses.
#           Used by tools/checkAll.py and by itemInterfaceEmitter.py.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-15 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import ast

allComment = '#public API of this module; kept complete by tools/checkAll.py (#2444)'


def _IsPrivateByDecorator(node):
    for decorator in node.decorator_list:
        if (isinstance(decorator, ast.Call) and isinstance(decorator.func, ast.Name)
                and decorator.func.id == 'docmeta'):
            for keyword in decorator.keywords:
                if keyword.arg == 'public' and isinstance(keyword.value, ast.Constant) \
                        and keyword.value.value is False:
                    return True
    return False


def _Statements(body):
    for node in body:
        if isinstance(node, ast.If):
            if 'name' in ast.unparse(node.test) and '__main__' in ast.unparse(node.test):
                continue    #script part: if __name__ == '__main__':
            yield from _Statements(node.body)
            yield from _Statements(node.orelse)
        elif isinstance(node, ast.Try):
            yield from _Statements(node.body)
            for handler in node.handlers:
                yield from _Statements(handler.body)
            yield from _Statements(node.orelse)
            yield from _Statements(node.finalbody)
        else:
            yield node


def PublicNames(source):
    """public names of a module source, in order of first definition"""
    names = []
    def Add(name):
        if not name.startswith('_') and name not in names:
            names.append(name)
    for node in _Statements(ast.parse(source).body):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
            if not _IsPrivateByDecorator(node):
                Add(node.name)
        elif isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            for target in targets:
                for element in ast.walk(target):
                    if isinstance(element, ast.Name):
                        Add(element.id)
    return names


def DeclaredAll(source):
    """(names, first line, last line) of a top-level literal __all__, or None"""
    for node in ast.parse(source).body:
        if (isinstance(node, ast.Assign) and len(node.targets) == 1
                and isinstance(node.targets[0], ast.Name) and node.targets[0].id == '__all__'):
            if not isinstance(node.value, (ast.List, ast.Tuple)):
                return None
            return ([ast.literal_eval(e) for e in node.value.elts], node.lineno, node.end_lineno)
    return None


def AllText(names, width=100):
    """the __all__ assignment for a list of names, wrapped"""
    lines = [allComment, '__all__ = [']
    current = '   '
    for name in names:
        item = " '" + name + "',"
        if len(current) + len(item) > width:
            lines.append(current)
            current = '   '
        current += item
    if current.strip():
        lines.append(current)
    lines.append('    ]')
    return '\n'.join(lines) + '\n'
