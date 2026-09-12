#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Derives sources.json - the list of .cpp files that setup.py compiles - from the
#           ClCompile entries of msvc/cppsrc.vcxproj, so that the Visual Studio project stays
#           the single source of truth for the source list and setup.py no longer carries a
#           hand-maintained copy of it.
#
#           Only <ItemGroup> blocks are read. Every <PropertyGroup> setting - the configurations,
#           the preprocessor defines, the optimisation flags - stays hand-maintained in the
#           vcxproj and is NOT touched here.
#
#           WHY A COMMITTED JSON, and not XML parsing inside setup.py: the wheel must build from
#           an sdist that need not contain the vcxproj, and a build should not depend on the
#           layout of a Visual Studio file. The JSON is the contract; this tool keeps it honest.
#
#           THE MINIMAL SUBSET is a setup.py concept, not a Visual Studio one: --minimal compiles
#           a reduced file list AND defines EXUDYN_MINIMAL_COMPILATION, so the list and the C++
#           '#ifdef's have to agree. The vcxproj cannot express it, so 'minimal' is carried over
#           from the existing sources.json and only validated here (it must be a subset of 'all').
#
#           CASE MATTERS. Windows resolves 'src/tests/X.cpp' and 'src/Tests/X.cpp' to the same
#           file; Linux does not. Comparisons here are therefore case-exact against the real
#           on-disk spelling, which is how a wrong-case vcxproj entry is caught before it breaks
#           the manylinux build rather than after.
#
# Usage:    python tools/gen_sources.py            regenerate sources.json
#           python tools/gen_sources.py --check    do not write; exit non-zero on any disagreement
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-11 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import json
import os
import re
import sys

vcxprojPath  = 'msvc/cppsrc.vcxproj'
sourcesPath  = 'sources.json'
sourceRoot   = 'src'                #every listed file must live under here

#paths in the vcxproj are relative to msvc/, i.e. they start with '..\'
vcxprojPrefix = '../'


#%%******************************************************************************************************
def GetRepositoryRoot():
    """Absolute path of the repository root; this tool is always run from there."""
    here = os.path.dirname(os.path.abspath(__file__))

    return os.path.normpath(os.path.join(here, '..'))


#%%******************************************************************************************************
def ReadVcxprojSources(repositoryRoot):
    """
    The ClCompile entries of the vcxproj, as repository-relative paths with forward slashes.
    Only ItemGroup blocks are considered; ClCompile also appears inside ItemDefinitionGroup,
    where it carries compiler settings rather than a file, and must not be read as a source.
    """
    with open(os.path.join(repositoryRoot, vcxprojPath), 'r', encoding='utf8',
              errors='replace') as f:
        xmlText = f.read()

    sources = []
    for itemGroup in re.findall(r'<ItemGroup[^>]*>(.*?)</ItemGroup>', xmlText, re.DOTALL):
        for include in re.findall(r'<ClCompile\s+Include="([^"]+)"', itemGroup):
            path = include.replace('\\', '/')
            if path.startswith(vcxprojPrefix):
                path = path[len(vcxprojPrefix):]
            sources += [path]

    return sources


#%%******************************************************************************************************
def ListSourcesOnDisk(repositoryRoot):
    """
    Every .cpp file under src/, repository-relative, in its REAL on-disk spelling. os.walk
    reports the true case even on Windows, which os.path.isfile does not - that is the whole
    point of building this set rather than probing paths one by one.
    """
    mainDir = repositoryRoot
    found = set()
    for currentDirectory, subDirectories, fileNames in os.walk(os.path.join(repositoryRoot,
                                                                            sourceRoot)):
        for fileName in fileNames:
            if fileName.endswith('.cpp'):
                full = os.path.join(currentDirectory, fileName)
                found.add(os.path.relpath(full, mainDir).replace('\\', '/'))

    return found


#%%******************************************************************************************************
def ReadSourcesJson(repositoryRoot):
    """The committed sources.json, or None when it does not exist yet."""
    fullName = os.path.join(repositoryRoot, sourcesPath)
    if not os.path.isfile(fullName):
        return None

    with open(fullName, 'r', encoding='utf8') as f:
        return json.load(f)


#%%******************************************************************************************************
def WriteSourcesJson(repositoryRoot, data):
    """Write sources.json with LF endings, so Windows and Linux produce an identical file."""
    fullName = os.path.join(repositoryRoot, sourcesPath)
    text = json.dumps(data, indent=4) + '\n'
    with open(fullName, 'w', encoding='utf8', newline='\n') as f:
        f.write(text)


#%%******************************************************************************************************
def Report(title, entries, limit=20):
    print(title + ': ' + str(len(entries)))
    for entry in sorted(entries)[:limit]:
        print('    ' + entry)
    if len(entries) > limit:
        print('    ... and ' + str(len(entries)-limit) + ' more')


#%%******************************************************************************************************
def Main():
    parser = argparse.ArgumentParser(
        description='Derive sources.json from the ClCompile entries of cppsrc.vcxproj.')
    parser.add_argument('--check', action='store_true',
                        help='do not write; exit non-zero when anything disagrees (use in CI)')
    parser.add_argument('--quiet', action='store_true', help='less output')
    args = parser.parse_args()

    verbose = not args.quiet
    repositoryRoot = GetRepositoryRoot()

    vcxprojSources = ReadVcxprojSources(repositoryRoot)
    onDisk = ListSourcesOnDisk(repositoryRoot)
    existing = ReadSourcesJson(repositoryRoot)

    if verbose:
        print('repository       : ' + repositoryRoot)
        print('vcxproj sources  : ' + str(len(vcxprojSources)))
        print('.cpp on disk     : ' + str(len(onDisk)))

    problems = []

    #1: the vcxproj must not list the same file twice
    duplicates = set([p for p in vcxprojSources if vcxprojSources.count(p) > 1])
    if duplicates:
        Report('DUPLICATE entries in the vcxproj', duplicates)
        problems += ['duplicates']

    vcxprojSet = set(vcxprojSources)

    #2: case-exact existence. A wrong-case entry resolves on Windows and fails on Linux, so it
    #   cannot be found by simply opening the file on the machine that usually builds.
    notOnDisk = vcxprojSet - onDisk
    if notOnDisk:
        wrongCase = {}
        lowerToReal = {p.lower(): p for p in onDisk}
        for path in notOnDisk:
            if path.lower() in lowerToReal:
                wrongCase[path] = lowerToReal[path.lower()]
        if wrongCase:
            print('WRONG CASE in the vcxproj (resolves on Windows, FAILS on Linux): '
                  + str(len(wrongCase)))
            for path in sorted(wrongCase):
                print('    ' + path + '   ->   ' + wrongCase[path])
            problems += ['wrong case']
        trulyMissing = set(notOnDisk) - set(wrongCase)
        if trulyMissing:
            Report('LISTED in the vcxproj but NOT on disk', trulyMissing)
            problems += ['missing files']

    #3: a source on disk that nothing compiles is either dead code or a forgotten vcxproj entry
    unlisted = onDisk - vcxprojSet
    if unlisted:
        Report('on disk but NOT listed in the vcxproj', unlisted)
        problems += ['unlisted files']

    #the minimal subset cannot be derived from the vcxproj; carry it over and validate it
    minimalSources = existing.get('minimal', []) if existing else []
    badMinimal = set(minimalSources) - vcxprojSet
    if badMinimal:
        Report("'minimal' entries that are not in the vcxproj list", badMinimal)
        problems += ['minimal not a subset']

    data = {
        '_comment': ['GENERATED by tools/gen_sources.py from the ClCompile entries of '
                     'msvc/cppsrc.vcxproj - do not edit by hand.',
                     "'all' is the full compile list; 'minimal' is the reduced list used by "
                     "setup.py --minimal, which also defines EXUDYN_MINIMAL_COMPILATION, so it "
                     "must stay in sync with the C++ #ifdefs by hand.",
                     'Paths are repository-relative and case-exact: Linux needs them to be.'],
        'all': sorted(vcxprojSet),
        'minimal': sorted(minimalSources),
        }

    if args.check:
        if existing is None:
            print('')
            print('FAILED: ' + sourcesPath + ' does not exist; run tools/gen_sources.py')
            return 1
        if existing.get('all') != data['all'] or existing.get('minimal') != data['minimal']:
            print('')
            print('DRIFT: ' + sourcesPath + ' does not match the vcxproj.')
            Report('    in the vcxproj but not in sources.json',
                   set(data['all']) - set(existing.get('all', [])))
            Report('    in sources.json but not in the vcxproj',
                   set(existing.get('all', [])) - set(data['all']))
            problems += ['sources.json is stale']

        print('')
        if problems:
            print('FAILED: ' + ', '.join(sorted(set(problems))))
            print('Fix the vcxproj (it is the source of truth) and run tools/gen_sources.py.')
            return 1

        print('OK: vcxproj, ' + sourcesPath + ' and the files on disk agree ('
              + str(len(data['all'])) + ' sources, ' + str(len(data['minimal'])) + ' minimal).')
        return 0

    if problems:
        print('')
        print('REFUSING to write ' + sourcesPath + ': ' + ', '.join(sorted(set(problems))))
        print('The vcxproj is the source of truth - correct it first.')
        return 1

    WriteSourcesJson(repositoryRoot, data)
    print('')
    print('wrote ' + sourcesPath + ': ' + str(len(data['all'])) + ' sources, '
          + str(len(data['minimal'])) + ' minimal.')

    return 0


#%%******************************************************************************************************
if __name__ == '__main__':
    sys.exit(Main())
