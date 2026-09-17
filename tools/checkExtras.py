#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Compares the optional-dependency extras declared in pyproject.toml against the
#           third-party packages that the code actually imports, so that 'pip install exudyn[tests]'
#           and 'pip install exudyn[all]' cannot quietly go stale. The comparison is a pure AST
#           scan: nothing is imported, nothing is built, and exudyn itself need not be installed.
#
#           Four comparisons are made:
#             1 uncovered      a third-party import in exudyn/ or TestModels/ that is in no extra
#                              and on no exemption list                             -> FAILS
#             2 uncovered      the same for Examples/, checked against [all]        -> FAILS
#             3 dead entry     a distribution in an extra that nothing imports      -> reported
#             4 rotted exempt  an exemption for something no longer imported        -> reported
#           3 and 4 only report: a package can be needed at runtime without a literal import, and
#           an exemption may cover an import that is temporarily absent.
#
#           An import name that is not the name of its distribution needs an entry in
#           importToDistribution below. There is no guessing: an unmapped name is compared under
#           its own spelling and therefore shows up as 'uncovered' (1/2) rather than silently
#           passing - that is deliberate, a checker that guesses is a checker that rots.
#
# Usage:    python tools/checkExtras.py             report
#           python tools/checkExtras.py --check     the same, but exit non-zero on 1 or 2 (CI)
#           python tools/checkExtras.py --list      only list the third-party imports found
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-11 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import ast
import os
import subprocess
import sys

if sys.version_info < (3, 11):
    raise SystemExit('checkExtras.py needs Python 3.11+ for tomllib; the CI image is python:3.13')
import tomllib

#the three source trees, with the extra that has to cover each.
#  TestModels -> [tests]: exactly what is needed to run the test suite.
#  exudyn/    -> [all]  : the shipped package has optional imports BY DESIGN (CLAUDE.md invariant
#                         6 - optional dependencies behind a clear failure at the point of use),
#                         so requiring [tests] to install mpi4py or dispy would be wrong. [all] is
#                         'everything referred to internally', which is what covers them.
#  Examples   -> [all].
scanTargets = [
    ('python/TestModels', 'tests'),
    ('python/exudyn',     'all'),
    ('python/Examples',   'all'),
    ]

#import name -> PyPI distribution name, for the cases where the two differ. Only real mismatches
#belong here; anything else is compared under its own name.
importToDistribution = {
    'stl':               'numpy-stl',
    'roboticstoolbox':   'roboticstoolbox-python',
    'spatialmath':       'spatialmath-python',
    'ffmpeg':            'ffmpeg-python',
    'stable_baselines3': 'stable-baselines3',
    'mpl_toolkits':      'matplotlib',
    'netgen':            'ngsolve',
    'PIL':               'pillow',
    'cv2':               'opencv-python',
    'yaml':              'pyyaml',
    'OpenGL':            'pyopengl',
    'skimage':           'scikit-image',
    'sklearn':           'scikit-learn',
    }

#imports that can never become an extra. Each entry carries the reason; the reason is the point of
#the list - without it the next maintainer cannot tell an exemption from an oversight.
exemptImports = {
    'rospy':         'ROS is not distributed on PyPI; installed with the ROS distribution',
    'geometry_msgs': 'ROS message package, not on PyPI',
    'std_msgs':      'ROS message package, not on PyPI',
    'exudynCPP':     'the compiled extension itself, built by setup.py',
    'pyansys':       'imported inside GenerateStressModesFromPyAnsys() only, behind a flag that '
                     'is False; the distribution was renamed to ansys-mapdl-reader and pulling '
                     'it into [tests] would be a large dependency for dead code',
    'mkl':           'optional performance tweak in a try/except with a working fallback; which '
                     'MKL build is correct depends on the BLAS the environment already has',
    }

#imports of modules that exist NOWHERE - neither on PyPI nor in this repository. These are real
#broken imports in the files listed, not packaging gaps, so they are reported as a warning rather
#than treated as an uncovered dependency. Raised as an issue; see revision2026 step R2.12.
knownMissingLocalModules = {
    'RL_Spot': 'Examples/FurtherExamples/spotReinforcementLearning.py imports it, but no such '
               'file is in the repository - the model module was never committed',
    }

#the [rl] extra is deliberately NOT part of [all]: torch is multi-GB and the CPU/CUDA choice is
#made with an --index-url that cannot be expressed in wheel metadata. Imports covered by [rl] are
#therefore accepted everywhere without being required by [tests] or [all].
#NOTE exudyn/artificialIntelligence.py imports stable_baselines3 at MODULE level with no guard,
#and TestModels/allExudynModulesTest.py imports that module - so that one test cannot run in a
#plain [tests] environment and is expected to skip. See revision2026 step R2.12.
optionalExtras = ['rl']


#%%******************************************************************************************************
def GetRepositoryRoot():
    """Absolute path of the repository root, independent of the current working directory."""
    result = subprocess.run(['git', 'rev-parse', '--show-toplevel'],
                            capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError('not inside a git repository: ' + result.stderr.strip())

    return os.path.normpath(result.stdout.strip())


#%%******************************************************************************************************
def NormalizeDistribution(name):
    """PEP 503 normalised distribution name, so 'numpy_stl' and 'numpy-stl' compare equal."""
    normalized = name.strip().lower()
    for character in '_.':
        normalized = normalized.replace(character, '-')

    return normalized


#%%******************************************************************************************************
def DistributionOfRequirement(requirement):
    """Distribution name of a PEP 508 requirement string: 'scipy>=1.0 ; python_version<"3.9"'."""
    name = requirement.strip()
    for separator in ['[', '(', ';', '<', '>', '=', '!', '~', ' ']:
        name = name.split(separator)[0]

    return NormalizeDistribution(name)


#%%******************************************************************************************************
def ReadExtras(pyprojectPath):
    """
    {extraName: set of normalised distribution names}, with self-references such as
    all = ["exudyn[tests]"] resolved, so that [all] really means everything [all] installs.
    """
    with open(pyprojectPath, 'rb') as f:
        pyproject = tomllib.load(f)

    projectName = NormalizeDistribution(pyproject['project']['name'])
    rawExtras = pyproject['project'].get('optional-dependencies', {})

    def Resolve(extraName, seen):
        if extraName in seen:
            raise RuntimeError('circular extra reference: ' + ' -> '.join(list(seen)+[extraName]))
        distributions = set()
        for requirement in rawExtras[extraName]:
            distribution = DistributionOfRequirement(requirement)
            #a self-reference 'exudyn[tests]' pulls in that extra rather than naming a package
            if distribution == projectName and '[' in requirement:
                referenced = requirement.split('[')[1].split(']')[0]
                for part in referenced.split(','):
                    distributions |= Resolve(part.strip(), seen | {extraName})
            else:
                distributions.add(distribution)

        return distributions

    return {name: Resolve(name, set()) for name in rawExtras}


#%%******************************************************************************************************
def LocalModuleNames(repositoryRoot, scanDirectory):
    """
    Top-level names that resolve inside the project rather than to a third-party package: the
    shipped packages under python/, and the sibling modules of the scanned directory
    (TestModels and Examples import their own helpers by bare name).
    """
    localNames = set()
    pythonDev = os.path.join(repositoryRoot, 'python')
    for entry in os.listdir(pythonDev):
        if entry.endswith('.py'):
            localNames.add(entry[:-3])
        elif os.path.isdir(os.path.join(pythonDev, entry)):
            localNames.add(entry)

    #recursive: a helper next to an example is imported by bare name, whatever depth it sits at
    for currentDirectory, subDirectories, fileNames in os.walk(scanDirectory):
        subDirectories[:] = [d for d in subDirectories if d != '__pycache__']
        localNames |= set(subDirectories)
        localNames |= set([f[:-3] for f in fileNames if f.endswith('.py')])

    return localNames


#%%******************************************************************************************************
def TopLevelImportsOfFile(fileName):
    """Set of top-level import names in one file; relative imports are skipped as local."""
    with open(fileName, 'r', encoding='utf8', errors='replace') as f:
        source = f.read()

    tree = ast.parse(source, filename=fileName)   #SyntaxError is reported by the caller

    importNames = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                importNames.add(alias.name.split('.')[0])
        elif isinstance(node, ast.ImportFrom):
            if node.level == 0 and node.module is not None:
                importNames.add(node.module.split('.')[0])

    return importNames


#%%******************************************************************************************************
def ScanDirectory(repositoryRoot, relativeDirectory):
    """
    {importName: list of repository-relative files that import it} for third-party imports found
    under relativeDirectory. Standard library and project-local names are filtered out.
    """
    scanDirectory = os.path.join(repositoryRoot, relativeDirectory)
    localNames = LocalModuleNames(repositoryRoot, scanDirectory)
    standardLibrary = sys.stdlib_module_names

    found = {}
    unparsableFiles = []
    for currentDirectory, subDirectories, fileNames in os.walk(scanDirectory):
        #build outputs and caches are not source
        subDirectories[:] = [d for d in subDirectories
                             if d not in ['__pycache__', 'build', '.git']
                             and not d.endswith('.egg-info')]
        for fileName in sorted(fileNames):
            if not fileName.endswith('.py'):
                continue
            fullName = os.path.join(currentDirectory, fileName)
            relativeName = os.path.relpath(fullName, repositoryRoot).replace('\\', '/')
            try:
                importNames = TopLevelImportsOfFile(fullName)
            except SyntaxError as e:
                unparsableFiles += [relativeName + ': ' + str(e)]
                continue
            for importName in sorted(importNames):
                if importName in standardLibrary or importName in localNames:
                    continue
                found.setdefault(importName, []).append(relativeName)

    return found, unparsableFiles


#%%******************************************************************************************************
def Report(title, entries, limit=25):
    print(title + ': ' + str(len(entries)))
    for line in sorted(entries)[:limit]:
        print('    ' + line)
    if len(entries) > limit:
        print('    ... and ' + str(len(entries)-limit) + ' more')


#%%******************************************************************************************************
def Main():
    parser = argparse.ArgumentParser(
        description='Check the pyproject.toml extras against the imports in the code.')
    parser.add_argument('--check', action='store_true',
                        help='exit non-zero when an import is covered by no extra (use in CI)')
    parser.add_argument('--list', action='store_true',
                        help='only list the third-party imports found, with their files')
    parser.add_argument('--quiet', action='store_true', help='less output')
    args = parser.parse_args()

    verbose = not args.quiet
    repositoryRoot = GetRepositoryRoot()
    pyprojectPath = os.path.join(repositoryRoot, 'pyproject.toml')
    extras = ReadExtras(pyprojectPath)

    with open(pyprojectPath, 'rb') as f:
        mandatory = set([DistributionOfRequirement(r)
                         for r in tomllib.load(f)['project'].get('dependencies', [])])

    if verbose:
        print('repository       : ' + repositoryRoot)
        print('extras declared  : ' + (', '.join(sorted(extras)) or '(none)'))

    #always acceptable: the mandatory dependencies plus the opt-in extras
    alwaysAccepted = set(mandatory)
    for extraName in optionalExtras:
        alwaysAccepted |= extras.get(extraName, set())

    uncovered = []
    usedDistributions = {}
    usedExemptions = set()
    unparsableFiles = []
    missingModules = []

    for relativeDirectory, requiredExtra in scanTargets:
        found, unparsable = ScanDirectory(repositoryRoot, relativeDirectory)
        unparsableFiles += unparsable
        covering = extras.get(requiredExtra, set()) | alwaysAccepted

        if args.list:
            print('')
            print(relativeDirectory + ' (must be covered by [' + requiredExtra + '])')
            for importName in sorted(found):
                extraFiles = len(found[importName])-1
                print('    %-24s %s%s' % (importName, found[importName][0],
                                          ' (+%d)' % extraFiles if extraFiles else ''))
            continue

        for importName in sorted(found):
            if importName in knownMissingLocalModules:
                missingModules += ['%-22s %s' % (importName,
                                                 knownMissingLocalModules[importName])]
                continue
            if importName in exemptImports:
                usedExemptions.add(importName)
                continue
            distribution = NormalizeDistribution(importToDistribution.get(importName, importName))
            usedDistributions.setdefault(distribution, set()).add(relativeDirectory)
            if distribution not in covering:
                uncovered += ['%-22s (as %s) needed by [%s], first seen in %s'
                              % (importName, distribution, requiredExtra, found[importName][0])]

    if args.list:
        return 0

    #3: declared but never imported. Not fatal - a runtime dependency need not be imported here.
    declared = set()
    for distributions in extras.values():
        declared |= distributions
    deadEntries = ['%-22s declared in [%s] but imported nowhere'
                   % (d, ','.join(sorted([n for n in extras if d in extras[n]])))
                   for d in sorted(declared - set(usedDistributions))]

    #4: an exemption that no longer covers anything
    rottedExemptions = ['%-22s exempt (%s) but imported nowhere' % (n, exemptImports[n])
                        for n in sorted(set(exemptImports) - usedExemptions)]

    print('')
    if unparsableFiles:
        Report('FILES THAT DID NOT PARSE (not scanned)', unparsableFiles)
    if uncovered:
        Report('UNCOVERED IMPORTS (no extra installs these)', uncovered)
    if missingModules:
        Report('broken imports - module exists neither on PyPI nor here (warning)', missingModules)
    if deadEntries:
        Report('dead extra entries (warning)', deadEntries)
    if rottedExemptions:
        Report('rotted exemptions (warning)', rottedExemptions)

    if not uncovered and not unparsableFiles:
        print('OK: every third-party import is covered by an extra or an exemption ('
              + str(len(usedDistributions)) + ' distributions, '
              + str(len(usedExemptions)) + ' exemptions used).')
        return 0

    if uncovered:
        print('')
        print('Add each package to the right extra in pyproject.toml, or - if its import name')
        print('differs from its distribution name - add it to importToDistribution in this file.')
        print('If it can never be installed from PyPI, add it to exemptImports WITH a reason.')

    return 1 if args.check else 0


#%%******************************************************************************************************
if __name__ == '__main__':
    sys.exit(Main())
