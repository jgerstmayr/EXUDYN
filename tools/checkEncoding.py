#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Every tracked text file must be UTF-8 (#2533). Twelve were not:
#           eleven C++ sources and one example carried cp1252 bytes - a degree sign, a micro sign,
#           an en dash, German umlauts - and docs/theDoc/trackerlog.tex was written by a plain
#           open(..., 'w'), which on Windows takes the code page.
#
#           This is not cosmetic. A tool that reads the tree assuming UTF-8 either raises
#           UnicodeDecodeError in the middle of a run, or - worse - reads with a fallback and
#           writes back UTF-8, rewriting every byte of a file it was asked to touch in one place.
#           Both happened while step R6.3.6 was mapping the error sites.
#
#           Binary files that happen to carry a text extension are exempt by name, not by guessing.
#
# Usage:    python tools/checkEncoding.py            report
#           python tools/checkEncoding.py --check    the same, exit non-zero on a finding (CI)
#           python tools/checkEncoding.py --write    convert the offenders from cp1252 to UTF-8
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-18 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import io
import os
import subprocess
import sys

repositoryRoot = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

#extensions that are text and are read by the tools
textExtensions = ['.h', '.cpp', '.py', '.pyi', '.md', '.rst', '.tex', '.txt', '.json', '.yml',
                  '.yaml', '.cfg', '.toml', '.bat', '.sh']

#files with a text extension that are not text. Listed by name: guessing whether a file is binary
#is exactly the kind of cleverness that makes a check untrustworthy
notText = [
    'python/TestModels/testData/rotorAnsys.rst',   #an ANSYS result file, not reStructuredText
]


#%%******************************************************************************************************
def TrackedFiles():
    """the files git knows about, so that an untracked scratch file cannot fail the check"""
    output = subprocess.run(['git', 'ls-files'], cwd=repositoryRoot, capture_output=True,
                            text=True, check=True).stdout

    return [line.strip() for line in output.splitlines() if line.strip()]


#%%******************************************************************************************************
def Offenders():
    """(path, decodable-as-cp1252) for every tracked text file that is not UTF-8"""
    findings = []
    for path in TrackedFiles():
        if path in notText or os.path.splitext(path)[1].lower() not in textExtensions:
            continue
        fullPath = os.path.join(repositoryRoot, path)
        if not os.path.isfile(fullPath):
            continue
        raw = io.open(fullPath, 'rb').read()
        try:
            raw.decode('utf-8')
            continue
        except UnicodeDecodeError:
            pass
        try:
            raw.decode('cp1252')
            findings.append((path, True))
        except UnicodeDecodeError:
            findings.append((path, False))

    return findings


#%%******************************************************************************************************
def main():
    parser = argparse.ArgumentParser(description='every tracked text file must be UTF-8')
    parser.add_argument('--check', action='store_true', help='exit non-zero on a finding')
    parser.add_argument('--write', action='store_true', help='convert cp1252 files to UTF-8')
    parser.add_argument('--quiet', action='store_true', help='print only what is wrong')
    options = parser.parse_args()

    findings = Offenders()
    if not findings:
        if not options.quiet:
            print('OK: every tracked text file is UTF-8.')

        return 0

    for (path, isCp1252) in findings:
        print('  ' + path + ('   (cp1252)' if isCp1252 else '   (neither UTF-8 nor cp1252)'))

    if options.write:
        converted = 0
        for (path, isCp1252) in findings:
            if not isCp1252:
                print('NOT converted, the encoding is unknown: ' + path)
                continue
            fullPath = os.path.join(repositoryRoot, path)
            raw = io.open(fullPath, 'rb').read()
            io.open(fullPath, 'wb').write(raw.decode('cp1252').encode('utf-8'))
            converted += 1
        print('converted ' + str(converted) + ' file(s) to UTF-8')

        return 0

    print(str(len(findings)) + ' tracked text file(s) are not UTF-8; '
          'run "python tools/checkEncoding.py --write" to convert them')

    return 1 if options.check else 0


if __name__ == '__main__':
    sys.exit(main())
