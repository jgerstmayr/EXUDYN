#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Runs the notebooks of python/Notebooks/ and stores their outputs in them: the printed
#           text, the value of a cell's last expression, and the matplotlib figures (PlotSensor,
#           exudyn.interactive.ShowImage) as PNG. The outputs are what the documentation shows
#           (docs/generated/notebooks/, written by tools/generators/notebookEmitter.py from them), so
#           they change only when a notebook is run again here - not with every build (#2831).
#
#           Each notebook runs in an interpreter of its own, in python/Notebooks/, with no window
#           (EXUDYN_SUPPRESS_UI_WINDOW_OPEN) and its files written to a temporary directory
#           (EXUDYN_OUTPUTDIRECTORY). No Jupyter package is needed: the cells are executed one after
#           the other in one namespace, as a kernel would.
#
# Usage:    python tools/runNotebooks.py                 #all notebooks
#           python tools/runNotebooks.py tutorialRigidBody #one, by name
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-04
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import ast
import base64
import contextlib
import glob
import io
import json
import os
import subprocess
import sys
import tempfile

repositoryRoot = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
notebookDirectory = os.path.join(repositoryRoot, 'python', 'Notebooks')

#lines of the window suppression that say only that no window opened; they are not output of the model
noticePrefixes = ('NOTE: EXUDYN_SUPPRESS_UI_WINDOW_OPEN', 'NOTE: EXUDYN_OUTPUTDIRECTORY', 'NOTE: SolutionViewer opens no window',
                  'NOTE: the renderer is suppressed', 'NOTE: plots are suppressed', 'NOTE: PlotSensor opens no window',
                  'NOTE: ShowImage')
imageDpi = 100
#a plot (PlotSensor, matplotlib) is stored at twice the size of its default 6.4 x 4.8 inch and wider, 8:4: the page shows
#it at half its pixels, so it is sharp and its fonts, which PlotSensor makes large for the screen, are half as large;
#an image of ShowImage keeps its pixels - the notebook asks for twice the size it wants to show
plotSizeInches = (12.8, 6.4)


def Source(cell):
    return ''.join(cell['source']) if isinstance(cell['source'], list) else cell['source']


def Lines(text):
    """text as the list of lines a notebook stores"""
    lines = text.split('\n')
    return [line + '\n' for line in lines[:-1]] + ([lines[-1]] if lines[-1] != '' else [])


def RunCell(code, namespace, plt):
    """execute one cell; returns its outputs in the form of nbformat 4"""
    outputs = []
    stream = io.StringIO()
    tree = ast.parse(code)
    lastExpression = None
    if len(tree.body) != 0 and isinstance(tree.body[-1], ast.Expr):
        lastExpression = ast.Expression(tree.body.pop().value)
    with contextlib.redirect_stdout(stream), contextlib.redirect_stderr(stream):
        exec(compile(tree, '<cell>', 'exec'), namespace)
        value = eval(compile(lastExpression, '<cell>', 'eval'), namespace) if lastExpression is not None else None
    text = '\n'.join(line for line in stream.getvalue().split('\n') if not line.startswith(noticePrefixes))
    if text.strip() != '':
        outputs.append({'output_type': 'stream', 'name': 'stdout', 'text': Lines(text.rstrip('\n') + '\n')})
    if value is not None and not (isinstance(value, (list, tuple)) and plt is not None
                                  and any(type(v).__module__.startswith('matplotlib') for v in value)):
        outputs.append({'output_type': 'execute_result', 'metadata': {}, 'execution_count': None,
                        'data': {'text/plain': Lines(repr(value))}})
    if plt is not None:
        for number in plt.get_fignums():
            figure = plt.figure(number)
            if figure.get_label() != 'exudyn.ShowImage':
                figure.set_size_inches(*plotSizeInches)
                figure.tight_layout()
            png = io.BytesIO()
            figure.savefig(png, format='png', dpi=imageDpi)
            outputs.append({'output_type': 'display_data', 'metadata': {},
                            'data': {'image/png': base64.b64encode(png.getvalue()).decode('ascii'),
                                     'text/plain': ['<Figure>']}})
        plt.close('all')
    return outputs


def Worker(path):
    """run one notebook in this interpreter and write its outputs into it"""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    notebook = json.load(open(path, encoding='utf-8'))
    namespace = {'__name__': '__main__'}
    count = 0
    for cell in notebook['cells']:
        if cell['cell_type'] != 'code':
            continue
        count += 1
        cell['execution_count'] = count
        try:
            cell['outputs'] = RunCell(Source(cell), namespace, plt)
        except Exception as exception:
            cell['outputs'] = [{'output_type': 'error', 'ename': type(exception).__name__,
                                'evalue': str(exception), 'traceback': []}]
            print('runNotebooks: ' + os.path.basename(path) + ', cell ' + str(count) + ': '
                  + type(exception).__name__ + ': ' + str(exception), file=sys.__stdout__)
            return 1
        for output in cell['outputs']:
            if output['output_type'] == 'execute_result':
                output['execution_count'] = count
    with open(path, 'w', encoding='utf-8', newline='\n') as file:
        json.dump(notebook, file, indent=1, ensure_ascii=False)
        file.write('\n')
    return 0


def main():
    if len(sys.argv) == 3 and sys.argv[1] == '--worker':
        os.chdir(os.path.dirname(os.path.abspath(sys.argv[2])))
        return Worker(sys.argv[2])

    names = sys.argv[1:]
    paths = sorted(glob.glob(os.path.join(notebookDirectory, '**', '*.ipynb'), recursive=True))
    if names:
        paths = [p for p in paths if os.path.splitext(os.path.basename(p))[0] in names]
    failures = 0
    with tempfile.TemporaryDirectory() as outputDirectory:
        environment = dict(os.environ, EXUDYN_SUPPRESS_UI_WINDOW_OPEN='1', EXUDYN_OUTPUTDIRECTORY=outputDirectory)
        environment.pop('PYTHONPATH', None)
        for path in paths:
            result = subprocess.run([sys.executable, os.path.abspath(__file__), '--worker', path], env=environment)
            print(('ran:    ' if result.returncode == 0 else 'FAILED: ') + os.path.relpath(path, repositoryRoot))
            failures += (result.returncode != 0)
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
