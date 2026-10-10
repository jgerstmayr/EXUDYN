#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# checkLandingPages - the links of README.rst and of the PyPI description lead somewhere
#
# Why this check exists (#2925): 1.13.2 could not be published, because the PyPI description - the
# text of [project.readme] in pyproject.toml, which every wheel carries - and the stale README.md linked
# the deleted docs/theDoc/theDoc.pdf (#2917). Nothing in the commit gate looked at these two texts.
#
# What is checked, without the network:
#   - both texts render with docutils without a warning;
#   - a relative link or image of README.rst names a file of the repository (GitHub shows it from there);
#   - the PyPI description has no relative link or image at all, as pypi.org cannot show them;
#   - a link into this repository on GitHub (github.com/jgerstmayr/EXUDYN/blob|tree/<branch>/<path>) names
#     an existing path, and a link into the documentation on Read the Docs (exudyn.readthedocs.io/en/<version>/
#     <page>.html) names a page of the documentation sources.
#
# Usage:
#   python tools/checkLandingPages.py            #report
#   python tools/checkLandingPages.py --check    #exit 1 on a finding (the gate)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
import argparse
import io
import os
import re
import sys

root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

githubPrefix = re.compile(r'https?://github\.com/jgerstmayr/EXUDYN/(blob|tree)/[^/]+/(.*)$', re.IGNORECASE)
readTheDocsPrefix = re.compile(r'https?://exudyn\.readthedocs\.io/en/[^/]+/(.*)$')


def PyPIDescription():
    """the text of [project.readme] in pyproject.toml"""
    import tomllib
    with open(os.path.join(root, 'pyproject.toml'), 'rb') as f:
        return tomllib.load(f)['project']['readme']['text']


def Links(text, sourceName):
    """(links, messages): the targets of the references and images of an reST text, and the docutils
    messages of level warning and above, as strings"""
    import docutils.core
    import docutils.nodes
    import docutils.utils

    warnings = io.StringIO()
    document = docutils.core.publish_doctree(text, source_path=sourceName, settings_overrides={
        'report_level': docutils.utils.Reporter.WARNING_LEVEL, 'halt_level': docutils.utils.Reporter.SEVERE_LEVEL + 1,
        'warning_stream': warnings, 'file_insertion_enabled': False, 'raw_enabled': False})
    links = []
    for node in document.findall(docutils.nodes.reference):
        if 'refuri' in node:
            links.append(node['refuri'])
    for node in document.findall(docutils.nodes.image):
        links.append(node['uri'])
    for node in document.findall(docutils.nodes.target):
        if 'refuri' in node:
            links.append(node['refuri'])
    messages = [line for line in warnings.getvalue().splitlines() if line.strip()]
    return (sorted(set(links)), messages)


def DocumentationPageExists(page):
    """page as Read the Docs serves it ('docs/manual/x.html', '', 'index.html', '?badge=latest') -> its source exists"""
    page = page.split('#')[0].split('?')[0]
    if page in ['', 'index.html']:
        return True
    if not page.endswith('.html'):
        return os.path.exists(os.path.join(root, page))
    base = os.path.join(root, page[:-len('.html')])
    return any(os.path.isfile(base + extension) for extension in ['.md', '.rst', '.ipynb'])


def Findings(text, sourceName, relativeAllowed):
    (links, messages) = Links(text, sourceName)
    findings = [sourceName + ': ' + message for message in messages]
    for link in links:
        target = link.split('#')[0]
        if re.match(r'^[a-z][a-z0-9+.-]*:', target, re.IGNORECASE):
            match = githubPrefix.match(target)
            if match and not os.path.exists(os.path.join(root, match.group(2).rstrip('/'))):
                findings.append(sourceName + ': the link into the repository names no file: ' + link)
            match = readTheDocsPrefix.match(target)
            if match and not DocumentationPageExists(match.group(1)):
                findings.append(sourceName + ': the link into the documentation names no page: ' + link)
        elif not relativeAllowed:
            findings.append(sourceName + ': a relative link, which pypi.org cannot show: ' + link)
        elif target and not os.path.exists(os.path.join(root, target)):
            findings.append(sourceName + ': the relative link names no file: ' + link)
    return findings


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='the links of README.rst and of the PyPI description (#2925)')
    parser.add_argument('--check', action='store_true', help='exit 1 on a finding')
    args = parser.parse_args()

    readme = io.open(os.path.join(root, 'README.rst'), encoding='utf-8').read()
    findings = Findings(readme, 'README.rst', relativeAllowed=True)
    findings += Findings(PyPIDescription(), 'pyproject.toml [project.readme]', relativeAllowed=False)
    for finding in findings:
        print(finding)
    print('checkLandingPages: ' + (str(len(findings)) + ' finding(s)' if findings else 'OK'))
    sys.exit(1 if (findings and args.check) else 0)
