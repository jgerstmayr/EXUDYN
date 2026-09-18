#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Runs the ruff linter over the shipped Python package and judges the result against a
#           baseline, so that the findings present when the check was introduced are tolerated
#           while a NEW finding fails the check. Same idea as the pydoclint baseline of
#           revision2026 step R4.8 - but ruff has no baseline of its own, so it is implemented here.
#
#           The rule set is not ruff's implicit default: it is written down in pyproject.toml as
#           select = ["E4", "E7", "E9", "F"], because the implicit default is not stable across
#           ruff versions (with ruff 0.16 the same run reports 2236 findings instead of 565).
#           Those four families are what is broken or misleading - undefined names, names defined
#           twice, unused imports, bare 'except:', '== None' - and nothing about formatting.
#
#           A finding is recorded WITHOUT its line number, as
#               <count>  <file>  <code>  <message>
#           so that editing a file elsewhere does not invalidate the baseline; the count is what
#           keeps a second occurrence of the same finding in the same file from slipping through.
#           A baseline entry that no longer occurs is reported, not tolerated silently: the
#           baseline is meant to shrink, and a stale entry hides the next regression.
#
#           The second half (--stubs) compares the generated stub files against the module that is
#           actually imported, with mypy's stubtest (revision2026 step R5.5.4). Two allowlists:
#           tools/ci/stubtestNoise.txt is curated (pybind dunders, stub-only typing helpers) and
#           tools/ci/stubtestBaseline.txt is the generated backlog, which is meant to shrink.
#           Which allowlist entries are USED depends on the wheel - a build without the fast module
#           has no exudynCPPfast to disagree about - so unused entries are not an error; the
#           backlog is compared against the current findings here instead (step R5.5.7, #2515).
#           NOTE this checks the INSTALLED package, not python/exudyn/ - install before believing it.
#
# Usage:    python tools/checkPython.py             report
#           python tools/checkPython.py --check     the same, but exit non-zero on a new finding (gate, CI)
#           python tools/checkPython.py --write     regenerate the baseline from the current findings
#           python tools/checkPython.py --all       report every finding, ignoring the baseline
#           python tools/checkPython.py --stubs             compare stubs and module (stubtest)
#           python tools/checkPython.py --stubs --check     the same, exit non-zero on a new disagreement
#           python tools/checkPython.py --stubs --write     regenerate the stubtest backlog
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-17 (created, revision2026 step R5.5)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import json
import os
import subprocess
import sys

repositoryRoot = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
baselineFile = os.path.join(repositoryRoot, 'tools', 'ci', 'ruffBaseline.txt')

#what is linted; the generated stub files are excluded in pyproject.toml, they are checked
#against the module itself instead (revision2026 step R5.5, half B)
checkedPaths = ['python/exudyn']


#run ruff and return the findings as a list of (file, code, message); raises SystemExit if ruff
#is not installed, because a gate that passes when its tool is missing is not a gate
def RunRuff(paths):
    command = [sys.executable, '-m', 'ruff', 'check', '--output-format', 'json', '--no-cache'] + paths
    try:
        process = subprocess.run(command, cwd=repositoryRoot, capture_output=True, text=True)
    except OSError as error:
        raise SystemExit('could not run ruff: ' + str(error))

    if process.stdout.strip() == '':
        raise SystemExit('ruff produced no output; is it installed?\n'
                         '  pip install --group lint\n' + process.stderr.strip())
    try:
        findings = json.loads(process.stdout)
    except json.JSONDecodeError:
        raise SystemExit('could not read the output of ruff:\n' + process.stdout[:2000]
                         + '\n' + process.stderr[:2000])

    result = []
    for finding in findings:
        fileName = os.path.relpath(finding['filename'], repositoryRoot).replace('\\', '/')
        result.append((fileName, finding['code'], finding['message']))
    return result


#collect findings into {(file, code, message): count}
def CountFindings(findings):
    counted = {}
    for finding in findings:
        counted[finding] = counted.get(finding, 0) + 1
    return counted


def ReadBaseline():
    counted = {}
    if not os.path.exists(baselineFile):
        return counted
    with open(baselineFile, 'r', encoding='utf8') as file:
        for line in file:
            line = line.rstrip('\n')
            if line.strip() == '' or line.startswith('#'):
                continue
            parts = line.split('\t')
            if len(parts) != 4:
                raise SystemExit('malformed line in ' + baselineFile + ':\n  ' + line)
            counted[(parts[1], parts[2], parts[3])] = int(parts[0])
    return counted


def WriteBaseline(counted):
    lines = ['#Baseline of tools/checkPython.py - the ruff findings that were present when the check',
             '#was introduced (revision2026 step R5.5). A NEW finding fails the check; these do not.',
             '#The list is meant to SHRINK: regenerate with "python tools/checkPython.py --write"',
             '#after fixing findings. Format: count, file, rule, message - tab separated, no line',
             '#numbers, so that an edit elsewhere in a file does not invalidate the entry.',
             '']
    for key in sorted(counted):
        lines.append(str(counted[key]) + '\t' + key[0] + '\t' + key[1] + '\t' + key[2])
    with open(baselineFile, 'w', encoding='utf8', newline='\n') as file:
        file.write('\n'.join(lines) + '\n')


def PrintFindings(title, items):
    print('')
    print(title + ':')
    for key, count in items:
        countStr = '' if count == 1 else ' (' + str(count) + 'x)'
        print('    ' + key[0] + '  ' + key[1] + '  ' + key[2] + countStr)


#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#stubtest half: compare python/exudyn/__init__.pyi and symbolic.pyi against the imported module
stubtestNoiseFile = os.path.join(repositoryRoot, 'tools', 'ci', 'stubtestNoise.txt')
stubtestBaselineFile = os.path.join(repositoryRoot, 'tools', 'ci', 'stubtestBaseline.txt')
mypyConfigFile = os.path.join(repositoryRoot, 'tools', 'ci', 'mypyStubtest.ini')


def RunStubtest(generate=False):
    command = [sys.executable, '-m', 'mypy.stubtest', 'exudyn',
               '--ignore-positional-only',                  #pybind reports every argument as positional
               '--ignore-unused-allowlist',                 #which entries are used depends on the wheel, #2515
               '--mypy-config-file', mypyConfigFile,
               '--allowlist', stubtestNoiseFile]
    if generate:
        command.append('--generate-allowlist')
    else:
        command += ['--allowlist', stubtestBaselineFile]

    environment = dict(os.environ)
    environment['EXUDYN_SUPPRESS_UI_WINDOW_OPEN'] = '1'      #stubtest imports the module, #2477
    try:
        process = subprocess.run(command, cwd=repositoryRoot, capture_output=True, text=True, env=environment)
    except OSError as error:
        raise SystemExit('could not run stubtest: ' + str(error))
    if 'No module named mypy' in process.stderr:
        raise SystemExit('mypy is not installed, so the stubs cannot be checked:\n  pip install --group lint')
    return process


def BacklogEntries():
    """the entries of the generated backlog, in file order"""
    if not os.path.exists(stubtestBaselineFile):
        return []
    with open(stubtestBaselineFile, 'r', encoding='utf8') as file:
        return [line.strip() for line in file
                if line.strip() != '' and not line.startswith('#')]


def GeneratedEntries(process):
    """the error names a --generate-allowlist run reports: everything the curated noise file does
    not already cover"""
    return [line.strip() for line in process.stdout.split('\n')
            if line.strip() != '' and not line.startswith('note:')]


def CheckStubs(args):
    if args.write:
        entries = GeneratedEntries(RunStubtest(generate=True))
        header = ['#Backlog of tools/checkPython.py --stubs (revision2026 step R5.5.4): the stub-vs-module',
                  '#disagreements that existed when the check was introduced. GENERATED - regenerate with',
                  "#'python tools/checkPython.py --stubs --write'. A disagreement that is NOT in here fails the",
                  '#check. This list is meant to SHRINK; curated noise belongs in stubtestNoise.txt instead.',
                  '#The large groups are: pybind enums reported as metaclass differences, names the stub declares',
                  '#that the module no longer has, and signatures that differ in their argument names.',
                  '']
        with open(stubtestBaselineFile, 'w', encoding='utf8', newline='\n') as file:
            file.write('\n'.join(header + entries) + '\n')
        print('stubtest backlog written: ' + str(len(entries)) + ' entries.')
        return 0

    #One --generate-allowlist run answers both questions at once: which disagreements exist now,
    #and which backlog entries no longer occur. The backlog is generated, so its entries are plain
    #error names and compare literally. Unused entries cannot be left to stubtest (#2515): the
    #curated noise file names BOTH compiled modules, and a wheel built without the fast module has
    #only one of them - which used to fail this gate with "unused allowlist entry" although nothing
    #was wrong with the stubs. The backlog is checked for stale entries here instead, which is the
    #half of that signal worth keeping.
    current = GeneratedEntries(RunStubtest(generate=True))
    backlog = BacklogEntries()
    newFindings = [entry for entry in current if entry not in set(backlog)]
    staleEntries = [entry for entry in backlog if entry not in set(current)]

    if staleEntries:
        print(str(len(staleEntries)) + (' entry of ' if len(staleEntries) == 1 else ' entries of ')
              + os.path.relpath(stubtestBaselineFile, repositoryRoot)
              + (' no longer occurs:' if len(staleEntries) == 1 else ' no longer occur:'))
        for entry in staleEntries[:20]:
            print('    ' + entry)
        if len(staleEntries) > 20:
            print('    ... and ' + str(len(staleEntries) - 20) + ' more')
        print('The backlog is meant to SHRINK. Regenerate it so that they cannot come back unnoticed:')
        print('    python tools/checkPython.py --stubs --write')
        print('')

    if not newFindings:
        print('OK: the stubs and the imported module agree, apart from the curated noise and the')
        print('    ' + str(len(backlog)) + ' entries of the backlog.')
        return 0

    #a real disagreement: run once more with the backlog, for stubtest's own explanation of it
    print(RunStubtest().stdout.strip())
    print('')
    print('A name the stub and the module disagree about is either a stub that went stale or a')
    print('binding that is not described. Note that stubtest checks the INSTALLED package: if you')
    print('changed python/exudyn/, install it before believing this output. Once a disagreement is')
    print('understood and cannot be fixed now, add it with:')
    print('    python tools/checkPython.py --stubs --write')
    return 1 if args.check else 0


def Main():
    parser = argparse.ArgumentParser(description='run ruff over the shipped package and compare to the baseline')
    parser.add_argument('--check', action='store_true', help='exit non-zero on a new finding')
    parser.add_argument('--write', action='store_true', help='regenerate the baseline')
    parser.add_argument('--all', action='store_true', help='report every finding, ignoring the baseline')
    parser.add_argument('--stubs', action='store_true', help='compare the stub files against the imported module')
    args = parser.parse_args()

    if args.stubs:
        return CheckStubs(args)

    current = CountFindings(RunRuff(checkedPaths))

    if args.write:
        WriteBaseline(current)
        print('baseline written: ' + str(len(current)) + ' distinct findings, '
              + str(sum(current.values())) + ' in total.')
        return 0

    if args.all:
        byRule = {}
        for key, count in current.items():
            byRule[key[1]] = byRule.get(key[1], 0) + count
        print('ruff findings by rule:')
        for code in sorted(byRule, key=lambda c: (-byRule[c], c)):
            print('    ' + str(byRule[code]).rjust(4) + '  ' + code)
        print('    ' + str(sum(byRule.values())).rjust(4) + '  total')
        return 0

    baseline = ReadBaseline()
    newFindings = []
    fixedFindings = []
    for key, count in current.items():
        allowed = baseline.get(key, 0)
        if count > allowed:
            newFindings.append((key, count - allowed))
    for key, count in baseline.items():
        remaining = current.get(key, 0)
        if remaining < count:
            fixedFindings.append((key, count - remaining))

    if newFindings:
        PrintFindings('NEW ruff findings, not in ' + os.path.relpath(baselineFile, repositoryRoot),
                      sorted(newFindings))
        print('')
        print('Fix them, or - if the finding is intended - silence that one line with a trailing')
        print('"# noqa: <code>" and a reason. Only if it genuinely belongs to the tolerated debt,')
        print('add it with "python tools/checkPython.py --write".')

    if fixedFindings:
        PrintFindings('findings fixed since the baseline was written (thank you)', sorted(fixedFindings))
        print('')
        print('Regenerate the baseline so that they cannot come back unnoticed:')
        print('    python tools/checkPython.py --write')

    if not newFindings:
        print('OK: ruff reports no finding outside the baseline ('
              + str(sum(current.values())) + ' tolerated, '
              + str(sum(baseline.values())) + ' in the baseline).')
        return 0

    return 1 if args.check else 0


#%%******************************************************************************************************
if __name__ == '__main__':
    sys.exit(Main())
