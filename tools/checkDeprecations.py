#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  The deprecations of Exudyn against their year of removal (#2807). Every deprecation -
#           settings, item parameters, functions of C++, functions and arguments of the library - is
#           declared with the version it was deprecated in and the year it is removed
#           (tools/generators/deprecationModel.py collects them). This check
#             FAILS  for a deprecation whose year has come: it is outdated and has to be removed;
#             FAILS  for an inconsistent one: said in a text but not declared, declared without a year,
#                    warned about in C++ but not declared;
#             WARNS  for one in its last year.
#           The list itself is docs/generated/deprecations.md.
#
# Usage:    python tools/checkDeprecations.py             report
#           python tools/checkDeprecations.py --check     exit 1 for an outdated or inconsistent one
#           python tools/checkDeprecations.py --year 2028 as if it were that year
#
# Author:   Johannes Gerstmayr
# Date:     2026-10-03
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

import argparse
import datetime
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'generators'))
import deprecationModel                                                     # noqa: E402


def Outdated(entries, year):
    """(outdated, lastYear): the entries whose year of removal has come, and those removed next year"""
    outdated = [entry for entry in entries if isinstance(entry['expires'], int) and entry['expires'] <= year]
    lastYear = [entry for entry in entries if isinstance(entry['expires'], int) and entry['expires'] == year + 1]
    return (outdated, lastYear)


def main():
    parser = argparse.ArgumentParser(description='the deprecations of Exudyn against their year of removal')
    parser.add_argument('--check', action='store_true', help='exit 1 for an outdated or inconsistent deprecation')
    parser.add_argument('--year', type=int, default=datetime.date.today().year, help='the current year (for a test)')
    parser.add_argument('--quiet', action='store_true', help='print only what fails or warns')
    args = parser.parse_args()

    entries = deprecationModel.Collect()
    problems = deprecationModel.Problems()
    (outdated, lastYear) = Outdated(entries, args.year)

    for entry in outdated:
        print('OUTDATED: ' + entry['name'] + ' (' + entry['source'] + ', ' + entry['where'] + ') was to be removed in '
              + str(entry['expires']) + '; remove it')
    for problem in problems:
        print('INCONSISTENT: ' + problem)
    for entry in lastYear:
        print('warning: ' + entry['name'] + ' (' + entry['source'] + ') is removed in ' + str(entry['expires']))
    if not args.quiet or outdated or problems:
        counts = {}
        for entry in entries:
            counts[entry['source']] = counts.get(entry['source'], 0) + 1
        print('checkDeprecations: ' + str(len(entries)) + ' deprecations ('
              + ', '.join(source + ' ' + str(count) for (source, count) in counts.items()) + '); '
              + str(len(outdated)) + ' outdated, ' + str(len(problems)) + ' inconsistent, '
              + str(len(lastYear)) + ' in their last year')
    return 1 if args.check and (outdated or problems) else 0


if __name__ == '__main__':
    sys.exit(main())
