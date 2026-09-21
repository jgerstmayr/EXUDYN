#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool - the exudev driver, see tools/exudev/README.md
#
# Details:  The whole command-line surface: every option, every spelling, every help text, in one
#           file. It replaces the sixteen batch files of tools/buildAndGenerate/ (issue #2503,
#           revision2026 step R5.18), which could not print a --help, could not reject a wrong
#           option and could not pass on an option they did not know.
#
#           Usage:    exudev <command> [options]           (Windows: exudev.bat in the root)
#                     python tools/exudev <command> ...    (everywhere else)
#
#           QUIET IS THE DEFAULT and --verbose turns the tools' output back on (maintainer
#           2026-09-18). -n/--dry-run prints the commands instead of running them - that is the
#           answer to "how does this actually work", and it does not need this file to be read.
#
#           THE DRIVER NEVER IMPORTS EXUDYN: it is the thing that selects the environment, so it
#           has to run in any interpreter. Standard library only, Python 3.8 syntax.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-18 (created; revision2026 step R5.18)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import commands                          #noqa: E402 - the path above has to be set first
import runner                            #noqa: E402


epilogText = """examples:
  exudev build                      build and install for Python 3.13 (no docs, no tests)
  exudev build --env venvExuP313    ... into a named environment, whatever Python it has
  exudev build --fast               ... including the fast module exudynCPPfast
  exudev build --complete           clean, regenerate, docs, all versions, all tests
  exudev test --py all              the test suite in every environment
  exudev test --fast                the test suite against the fast module
  exudev perf --fast                the performance tests against the fast module
  exudev env                        which environment has which python, exudyn and numpy
  exudev issue list --open          the open issues; "issue raise/resolve/remark" write
  exudev -n release                 print everything a release would run, and do nothing

every command takes --help, -n/--dry-run and -v/--verbose; quiet is the default.
"""


#%%******************************************************************************************************
def GlobalParser():
    """The options every subcommand shares. They are defined once and attached as a parent, so they
    appear in every subcommand's --help as well."""
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('-n', '--dry-run', action='store_true',
                        help='print the commands that would run, and exit')
    parser.add_argument('-v', '--verbose', action='store_true',
                        help='show the output of the tools (quiet is the default)')
    parser.add_argument('--no-conda', action='store_true',
                        help='run in the current environment instead of "conda run -n ..."')

    return parser


#%%******************************************************************************************************
def VersionParser():
    """--py / --env, for the commands that work per Python version."""
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--py', metavar='VERSIONS',
                        help='Python version(s): 313, P313, 3.13, "all", or a list like 310,313')
    parser.add_argument('--env', metavar='NAME',
                        help='conda environment to use (default: ' + runner.generatorEnvironment
                             + ' for generation and documentation, venvP3xx for the version matrix);'
                             ' for "build" the environment is asked which Python it has')

    return parser


#%%******************************************************************************************************
def FastParser():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--fast', action='store_true',
                        help='the fast module exudynCPPfast (no range checks, AVX2); opt-in')

    return parser


#%%******************************************************************************************************
def BuildParser(subParsers, parents):
    parser = subParsers.add_parser('build', parents=parents,
        help='build the wheel and install it (no docs and no tests unless --complete)',
        description='Build the wheel with "pip wheel . -w dist --no-deps" and install exactly that '
                    'wheel by path. By default nothing else happens: no clean, no regeneration, no '
                    'documentation and no tests - that is what --complete is for.')
    parser.add_argument('--complete', action='store_true',
                        help='the whole path: clean, regenerate, docs, all versions, all tests')
    parser.add_argument('--clean', action='store_true',
                        help='remove the build directories first (implied by --complete)')
    parser.add_argument('--no-install', action='store_true',
                        help='build the wheel but do not install it')
    parser.add_argument('--parallel', action='store_true',
                        help='compile the sources in parallel - this is ALREADY the default')
    parser.add_argument('--no-parallel', action='store_true',
                        help='compile serially (EXUDYN_COMPILE_PARALLEL=0); the worker count itself '
                             'is not configurable, setup.py uses every core')
    parser.add_argument('--unittests', action='store_true',
                        help='compile the C++ unit tests into the module')
    parser.add_argument('--minimal', action='store_true',
                        help='the minimal source subset (EXUDYN_MINIMAL_CPP_FILES)')
    parser.add_argument('--no-glfw', action='store_true',
                        help='build without the OpenGL/GLFW renderer')
    parser.add_argument('--no-clean', action='store_true', help='--complete: keep the build directories')
    parser.add_argument('--no-docs', action='store_true', help='--complete: skip the documentation')
    parser.add_argument('--no-tests', action='store_true', help='--complete: skip all tests')
    parser.set_defaults(function=commands.Build)


#%%******************************************************************************************************
#%%******************************************************************************************************
def IssueParser(subParsers, globalParser):
    """exudev issue <verb>: the issue tracker, which was driven by importing its module from its
    own directory until revision2026 step R8.3. One driver for the build, the tests, the
    documentation and the tracker (maintainer 2026-09-21).

    Every verb that WRITES is a step with an action, so "exudev -n issue resolve 42 ..." prints
    what it would do and changes nothing - which matters here, because resolving an issue bumps
    the version of the package."""
    issue = subParsers.add_parser('issue', parents=[globalParser],
        help='the issue tracker: raise, extend, remark, resolve, abandon, show, list',
        description='The issue tracker of tools/issueTracker/. It also owns the version: the micro '
                    'number is the count of closed issues, so "resolve" and "abandon" rewrite '
                    'version.txt, versionCpp.cpp, the version line of README.rst and the tracker '
                    'pages. Runs in the current interpreter; no conda environment is involved.')
    verbs = issue.add_subparsers(dest='issueVerb', metavar='<verb>')
    issue.set_defaults(function=commands.Issue)

    author = argparse.ArgumentParser(add_help=False)
    author.add_argument('--author', default='JG', metavar='NAME',
                        help='who does this (default JG); Claude passes Claude-JG')

    raiseIssue = verbs.add_parser('raise', parents=[globalParser, author],
        help='raise a new issue', description='The type is checked against the one list of types.')
    raiseIssue.add_argument('title', help='the issue in one line')
    raiseIssue.add_argument('description', help='what it is about, in prose')
    raiseIssue.add_argument('--type', required=True, metavar='TYPE',
                            help='BUG, FIX, CHANGE, EXTENSION, IMPROVEMENT, TESTING, DOCU, '
                                 'EXAMPLE, CHECK, IDEA')
    raiseIssue.add_argument('--effort', metavar='E', help='LOW, MEDIUM, HIGH, HUGE')
    raiseIssue.add_argument('--priority', metavar='P', help='LOW, NORMAL, HIGH')
    raiseIssue.add_argument('--file', metavar='PATH', help='the file it is about')
    raiseIssue.add_argument('--line', metavar='N', help='the line it is about')

    extend = verbs.add_parser('extend', parents=[globalParser, author],
        help='append to the description of an open issue',
        description='For what the first analysis turns up: it is appended with the date and the '
                    'author, never overwritten, and a closed issue is refused.')
    extend.add_argument('number', type=int)
    extend.add_argument('text')

    remark = verbs.add_parser('remark', parents=[globalParser, author],
        help='write the working remarks of an open issue',
        description='"duplicate of #2134", "part A solved, B open", "check whether this still '
                    'happens": what is worth knowing while the issue is open. It is CLEARED when '
                    'the issue closes and is never published.')
    remark.add_argument('number', type=int)
    remark.add_argument('text')
    remark.add_argument('--replace', action='store_true',
                        help='replace the remarks instead of appending to them')

    resolve = verbs.add_parser('resolve', parents=[globalParser, author],
        help='resolve an issue (BUMPS THE VERSION)',
        description='The note is the release note of this issue and is published.')
    resolve.add_argument('number', type=int)
    resolve.add_argument('notes', help='the release note: what was done')

    abandon = verbs.add_parser('abandon', parents=[globalParser, author], aliases=['close'],
        help='close an issue WITHOUT resolving it',
        description='"decided against", "no longer applies", "superseded": the reason is '
                    'mandatory. It counts for the version like a resolved issue and appears in '
                    'the release notes as neither resolved nor open.')
    abandon.add_argument('number', type=int)
    abandon.add_argument('reason')

    show = verbs.add_parser('show', parents=[globalParser], help='one issue, field by field')
    show.add_argument('number', type=int)

    listIssues = verbs.add_parser('list', parents=[globalParser],
        help='the issues, newest first, with filters',
        description='The list a triage pass works from: "--open --type FIX --effort LOW".')
    listIssues.add_argument('--open', action='store_true', help='only the open ones')
    listIssues.add_argument('--type', metavar='TYPE')
    listIssues.add_argument('--effort', metavar='E')
    listIssues.add_argument('--priority', metavar='P')
    listIssues.add_argument('--limit', type=int, default=40, metavar='N',
                            help='how many to print (default 40; 0 for all)')

    modify = verbs.add_parser('modify', parents=[globalParser],
        help='set one field of an issue',
        description='The enum fields are checked here as well as when an issue is raised.')
    modify.add_argument('number', type=int)
    modify.add_argument('field')
    modify.add_argument('value')

    verbs.add_parser('triage', parents=[globalParser],
        help='the open issues by type and effort',
        description='The table a triage pass works from: how many open issues of each type are '
                    'LOW, MEDIUM, HIGH, HUGE - and how many are not classified yet.')

    mode = verbs.add_parser('mode', parents=[globalParser],
        help='switch between release and development build mode',
        description='Fact 26: the .dev1 suffix decides the version string, which modules setup.py '
                    'builds (168.9 s against 58.1 s on Windows/cp313) and whether a plain '
                    '"pip install exudyn" would take the version. This was a hand edit of '
                    'issueTracker.py.')
    group = mode.add_mutually_exclusive_group(required=True)
    group.add_argument('--release', action='store_true', help="versionDev = '' - 1.11.223")
    group.add_argument('--dev', action='store_true', help="versionDev = '.dev1' - 1.11.223.dev1")

    return issue


#%%******************************************************************************************************
def BuildParsers():
    globalParser = GlobalParser()
    versionParser = VersionParser()
    fastParser = FastParser()

    parser = argparse.ArgumentParser(
        prog='exudev',
        description='The Exudyn maintainer driver: build, test and document the repository. '
                    'Every command has its own --help.',
        epilog=epilogText,
        formatter_class=argparse.RawDescriptionHelpFormatter)

    #the same switches on the top level, so that "exudev -n build" works as well as "exudev build -n"
    parser.add_argument('-n', '--dry-run', dest='dryRunGlobal', action='store_true',
                        help='print the commands that would run, and exit')
    parser.add_argument('-v', '--verbose', dest='verboseGlobal', action='store_true',
                        help='show the output of the tools (quiet is the default)')
    parser.add_argument('--no-conda', dest='noCondaGlobal', action='store_true',
                        help='run in the current environment instead of "conda run -n ..."')

    subParsers = parser.add_subparsers(dest='command', metavar='<command>')

    generate = subParsers.add_parser('generate', parents=[globalParser, versionParser],
        help='regenerate the generated files (and optionally run the checking tools)',
        description='Run tools/regenerate.py, which drives the code generators and reports drift '
                    'against the commit.')
    generate.add_argument('--check', action='store_true',
                          help='exit non-zero when generated files differ (use before a release)')
    generate.add_argument('--no-run', action='store_true',
                          help='do not run the generators, only check the tree for drift')
    generate.add_argument('--all-checks', action='store_true',
                          help='also run checkAll, checkExtras, checkPython, checkPython --stubs '
                               'and gen_sources, each with --check')
    generate.set_defaults(function=commands.Generate)

    BuildParser(subParsers, [globalParser, versionParser, fastParser])

    test = subParsers.add_parser('test', parents=[globalParser, versionParser, fastParser],
        help='run the test suite (runTestSuite.py)',
        description='Run python/testing/runTestSuite.py. "--exit-code" is always passed, because '
                    'without it the suite returns 0 no matter how many models failed.')
    test.add_argument('--subset', action='store_true',
                      help='the pull-request subset only (the suite calls this option --fast, which '
                           'here means the fast MODULE - hence the different name)')
    test.add_argument('--parallel', nargs='?', type=int, const=0, metavar='N',
                      help='every model in its own interpreter; N processes, default all cores')
    test.add_argument('--overwrite-log', action='store_true',
                      help='overwrite an existing log instead of diverting it to python/logs/tmp/')
    test.add_argument('--local', action='store_true',
                      help='also copy the log next to the runner (the suite\'s -local)')
    test.add_argument('extra', nargs=argparse.REMAINDER,
                      help="after '--': arguments passed to runTestSuite.py verbatim")
    test.set_defaults(function=commands.Test)

    examples = subParsers.add_parser('examples', parents=[globalParser, versionParser],
        help='run the Examples set (runTestExamples.py; slow)',
        description='Run python/testing/runTestExamples.py. The default environment is venvP312, '
                    'unchanged from the batch script this replaces, so that "a clean examples run" '
                    'keeps meaning what it always meant.')
    examples.add_argument('--serial', action='store_true', help='one interpreter, in process')
    examples.add_argument('--parallel', nargs='?', type=int, const=0, metavar='N',
                          help='N processes, default all cores')
    examples.add_argument('--timeout', type=float, metavar='SECONDS',
                          help='seconds per example (default 60)')
    examples.add_argument('--overwrite-log', action='store_true', help='overwrite an existing log')
    examples.add_argument('extra', nargs=argparse.REMAINDER,
                          help="after '--': arguments passed to runTestExamples.py verbatim")
    examples.set_defaults(function=commands.Examples)

    performance = subParsers.add_parser('perf', parents=[globalParser, versionParser, fastParser],
        help='run the performance tests (runPerformanceTests.py)')
    performance.add_argument('--machine-id', metavar='ID',
                             help='EXUDYN_MACHINE_ID: the subfolder of python/logs/performance/')
    performance.add_argument('--overwrite-log', action='store_true', help='overwrite an existing log')
    performance.add_argument('extra', nargs=argparse.REMAINDER,
                             help="after '--': arguments passed to runPerformanceTests.py verbatim")
    performance.set_defaults(function=commands.Performance)

    docs = subParsers.add_parser('docs', parents=[globalParser, versionParser],
        help='build the html documentation with sphinx',
        description='sphinx-build -b html . _build -E, from the repository root. The LaTeX document '
                    'docs/theDoc is NOT built here: it has not compiled for many commits and '
                    'revision2026 phase R7 replaces it.')
    docs.add_argument('--keep-cache', action='store_true',
                      help='incremental build (drop -E); faster, but stale pages are possible')
    docs.add_argument('--no-strict', action='store_true',
                      help='do not turn warnings into errors; the default is -W --keep-going, '
                           'which is what the GitLab docs job runs')
    docs.add_argument('--open', action='store_true', help='open _build/index.html afterwards')
    docs.set_defaults(function=commands.Docs)

    linux = subParsers.add_parser('linux', parents=[globalParser, versionParser, fastParser],
        help='build the linux wheels through WSL',
        description='The manylinux wheels are built in the docker image quay.io/pypa/'
                    'manylinux_2_28_x86_64 through WSL; this is the path release wheels come from.')
    linux.add_argument('--manylinux', action='store_true',
                       help='manylinux wheels in docker (the default)')
    linux.add_argument('--wsl-conda', action='store_true',
                       help='build in the WSL conda environments instead (not manylinux)')
    linux.set_defaults(function=commands.Linux)

    release = subParsers.add_parser('release', parents=[globalParser, versionParser],
        help='the full release path, with the guards a release needs',
        description='"build --complete" over every Python version, plus: it refuses a development '
                    'version, it uses "generate --check" so that a release cannot silently '
                    'regenerate, and it builds the linux wheels afterwards.')
    release.add_argument('--dev', action='store_true',
                         help='allow a release build from a .dev version')
    release.add_argument('--no-linux', action='store_true', help='skip the linux wheels')
    release.add_argument('--no-docs', action='store_true', help='skip the documentation')
    release.set_defaults(function=commands.Release)

    clean = subParsers.add_parser('clean', parents=[globalParser],
        help='remove the Windows build directories and eggs',
        description='The set removeBuildsAndEggs.bat removed. dist/ and the linux build directories '
                    'are kept unless asked for - removing the linux directories used to break the '
                    'linux build.')
    clean.add_argument('--dist', action='store_true', help='also delete the wheels in dist/')
    clean.add_argument('--linux', action='store_true', help='also delete build/*linux*')
    clean.add_argument('--all', action='store_true', help='--dist and --linux together')
    clean.set_defaults(function=commands.Clean)

    IssueParser(subParsers, globalParser)

    environments = subParsers.add_parser('env', parents=[globalParser, versionParser],
        help='show python, exudyn and numpy in each environment',
        description='The diagnostic for "it passes here and fails there": a stale exudyn install, '
                    'or a different numpy, changes test results (see issues #2501 and #2502).')
    environments.set_defaults(function=commands.Environments)

    return parser


#%%******************************************************************************************************
def Main():
    parser = BuildParsers()
    options = parser.parse_args()

    if not getattr(options, 'command', None):
        parser.print_help()
        return 0

    #a switch given before the command counts just as much as one given after it
    options.dryRun  = options.dry_run or options.dryRunGlobal
    options.verbose = options.verbose or options.verboseGlobal
    options.noConda = options.no_conda or options.noCondaGlobal

    steps = options.function(options)
    if not steps:
        print('exudev: nothing to do')
        return 0

    returnCode = runner.RunSteps(steps, options)

    if returnCode != 0 and options.command in ['build', 'release'] and not options.verbose:
        outputFile = os.path.join(runner.RepositoryRoot(), 'setuppy.output.txt')
        if os.path.isfile(outputFile):
            print('compiler output: ' + outputFile + '   (re-run with --verbose to see it live)')

    return returnCode


#%%******************************************************************************************************
if __name__ == '__main__':
    try:
        sys.exit(Main())
    except KeyboardInterrupt:
        print('')
        print('exudev: interrupted')
        sys.exit(130)
