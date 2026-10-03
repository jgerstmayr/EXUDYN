#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  The flow charts of the documentation are TikZ (#2812): one standalone .tex per chart in
#           docs/figures/tikz/, with the styles of flowcharts.sty. This tool compiles a chart into
#           docs/figures/<name>.pdf - vector graphics for the PDF documentation - and
#           docs/figures/<name>.svg - for the web -, which the pages reference as
#           /docs/figures/<name>.* (Sphinx takes the SVG for html and the PDF for LaTeX).
#           Both are committed, so that the documentation builds without LaTeX; only a chart that
#           changed is compiled again, which needs pdflatex and pdftocairo (MiKTeX or TeX Live).
#           docs/figures/tikz/figures.json holds the hash of the source each output was made from;
#           --check compares it without LaTeX, and fails for a chart changed but not compiled.
#
# Usage:    python tools/buildFigures.py            compile what changed
#           python tools/buildFigures.py --all      compile every chart
#           python tools/buildFigures.py --check    exit 1 if a chart was changed and not compiled
#           exudev figures [--all]                  the same, through the driver
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-03
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import glob
import hashlib
import io
import json
import os
import shutil
import subprocess
import sys
import tempfile

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sourceDirectory = os.path.join(root, 'docs', 'figures', 'tikz')
outputDirectory = os.path.join(root, 'docs', 'figures')
manifestFile = os.path.join(sourceDirectory, 'figures.json')
styleFile = os.path.join(sourceDirectory, 'flowcharts.sty')


def Charts():
    """{name: source file} of every chart"""
    return {os.path.splitext(os.path.basename(fileName))[0]: fileName
            for fileName in sorted(glob.glob(os.path.join(sourceDirectory, '*.tex')))}


def SourceHash(fileName):
    """the hash of a chart and of the styles it uses, with the line endings of either system"""
    content = b''
    for name in [fileName, styleFile]:
        content += io.open(name, 'rb').read().replace(b'\r\n', b'\n')
    return hashlib.sha256(content).hexdigest()


def ReadManifest():
    if not os.path.exists(manifestFile):
        return {}
    return json.load(io.open(manifestFile, encoding='utf-8'))


def Stale(charts, manifest):
    """the charts whose outputs are missing or were made from another source"""
    stale = []
    for (name, fileName) in charts.items():
        outputs = [os.path.join(outputDirectory, name + extension) for extension in ['.pdf', '.svg']]
        if manifest.get(name) != SourceHash(fileName) or not all(os.path.exists(output) for output in outputs):
            stale.append(name)
    return stale


def Compile(name, fileName):
    """pdflatex in a temporary directory, the PDF and an SVG of it into docs/figures"""
    for tool in ['pdflatex', 'pdftocairo']:
        if shutil.which(tool) is None:
            raise SystemExit('buildFigures: ' + tool + ' not found; it comes with MiKTeX or TeX Live')
    environment = dict(os.environ, SOURCE_DATE_EPOCH='0', FORCE_SOURCE_DATE='1') #the same PDF for the same source
    with tempfile.TemporaryDirectory() as directory:
        shutil.copy(fileName, directory)
        shutil.copy(styleFile, directory)
        result = subprocess.run(['pdflatex', '-interaction=nonstopmode', '-halt-on-error', name + '.tex'],
                                cwd=directory, capture_output=True, text=True, env=environment)
        pdf = os.path.join(directory, name + '.pdf')
        if result.returncode != 0 or not os.path.exists(pdf):
            print(result.stdout[-3000:])
            raise SystemExit('buildFigures: ' + name + '.tex did not compile')
        shutil.copy(pdf, os.path.join(outputDirectory, name + '.pdf'))
        subprocess.run(['pdftocairo', '-svg', pdf, os.path.join(outputDirectory, name + '.svg')], check=True)


def main():
    parser = argparse.ArgumentParser(description='compile the TikZ flow charts of the documentation')
    parser.add_argument('--all', action='store_true', help='compile every chart, not only those that changed')
    parser.add_argument('--check', action='store_true', help='only check, without LaTeX; exit 1 for a stale chart')
    parser.add_argument('--quiet', action='store_true', help='print only what is stale or compiled')
    args = parser.parse_args()

    charts = Charts()
    manifest = ReadManifest()
    stale = Stale(charts, manifest)
    if args.check:
        for name in stale:
            print('STALE: docs/figures/tikz/' + name + '.tex changed, but was not compiled; run "exudev figures"')
        if not args.quiet or stale:
            print('buildFigures: ' + str(len(charts)) + ' charts, ' + str(len(stale)) + ' stale')
        return 1 if stale else 0

    for name in (list(charts) if args.all else stale):
        Compile(name, charts[name])
        manifest[name] = SourceHash(charts[name])
        print('compiled ' + name)
    manifest = {name: manifest[name] for name in sorted(manifest) if name in charts}
    io.open(manifestFile, 'w', encoding='utf-8', newline='\n').write(json.dumps(manifest, indent=2) + '\n')
    print('buildFigures: ' + str(len(charts)) + ' charts, ' + str(len(list(charts) if args.all else stale)) + ' compiled')
    return 0


if __name__ == '__main__':
    sys.exit(main())
