#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN test file
#
# Details:  The notebooks of python/Notebooks/ (#2831): the examples of the reference manual
#           (python/Notebooks/reference/) and of the user manual (python/Notebooks/snippets/) run with
#           the exudyn of this build, so an example on a page
#           cannot go stale; and every notebook, tutorials included, stores the outputs of its
#           current code (tools/runNotebooks.py --check), which the documentation shows.
#
# Usage:    pytest python/testing/test_referenceNotebooks.py
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-04
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import glob
import importlib.util
import json
import os
import subprocess
import sys

import pytest

root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
runnerPath = os.path.join(root, 'tools', 'runNotebooks.py')
spec = importlib.util.spec_from_file_location('runNotebooks', runnerPath)
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)

referenceNotebooks = sorted(glob.glob(os.path.join(root, 'python', 'Notebooks', 'reference', '*.ipynb'))
                            + glob.glob(os.path.join(root, 'python', 'Notebooks', 'snippets', '*.ipynb')))
allNotebooks = sorted(glob.glob(os.path.join(root, 'python', 'Notebooks', '**', '*.ipynb'), recursive=True))
#packages a reference example may need beyond numpy; without them the example is skipped, not failed
optionalPackages = ['scipy', 'matplotlib']


@pytest.mark.parametrize('path', referenceNotebooks, ids=lambda path: os.path.basename(path))
def testReferenceNotebookRuns(path):
    for package in optionalPackages:
        if importlib.util.find_spec(package) is None and package in open(path, encoding='utf-8').read():
            pytest.skip(package + ' is not installed')
    environment = dict(os.environ)
    environment.pop('PYTHONPATH', None)
    result = subprocess.run([sys.executable, runnerPath, '--test', os.path.splitext(os.path.basename(path))[0]],
                            env=environment, capture_output=True, text=True, timeout=300)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize('path', allNotebooks, ids=lambda path: os.path.relpath(path, os.path.join(root, 'python', 'Notebooks')))
def testNotebookStoresTheOutputsOfItsCode(path):
    notebook = json.load(open(path, encoding='utf-8'))
    assert notebook['metadata'].get('exudyn', {}).get('codeHash') == runner.CodeHash(notebook), \
        'the code changed after the outputs were stored: exudev notebooks ' + os.path.splitext(os.path.basename(path))[0]
