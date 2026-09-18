#!/usr/bin/env bash
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Build Exudyn with AddressSanitizer and UndefinedBehaviorSanitizer and run the test
#           suite against it. For a C++ library that calls arbitrary user callbacks this catches
#           the class of bug users report as "it crashed with no message": a read past the end of
#           a vector, a use after free, a signed overflow, a misaligned load. A normal build
#           produces the wrong answer or a silent crash; this one prints the file and line.
#           revision2026 step R5.6.
#
#           NO CHANGE TO setup.py IS NEEDED: the flags travel through EXUDYN_EXTRA_COMPILE_ARGS
#           and EXUDYN_EXTRA_LINK_ARGS, which setup.py already appends to every extension
#           (setup.py:683-689). CFLAGS does NOT work there.
#
#           THE FAST MODULE IS NOT BUILT. It is compiled with __FAST_EXUDYN_LINALG; which removes
#           exactly the range checks a sanitizer run is looking for; and it doubles the build time.
#
#           WHY LD_PRELOAD: python itself is not instrumented, and the ASan runtime has to be the
#           first library in the process. Loading an instrumented .so into a plain interpreter
#           without preloading libasan gives "ASan runtime does not come first in initial library
#           list" and nothing runs.
#
#           WHY libstdc++ IS PRELOADED TOO: if the interpreter brings its own C++ runtime - every
#           conda python does - ASan intercepts __cxa_throw against the wrong libstdc++ and dies at
#           the first exception with
#             AddressSanitizer: CHECK failed: asan_interceptors.cpp "real___cxa_throw != 0"
#           which looks like a sanitizer finding and is nothing but a mismatch. Preloading the
#           COMPILER's libstdc++ next to libasan makes the two agree. Measured 2026-09-18: without
#           it the run dies in the first model, with it the suite runs to the end.
#
#           WHY detect_leaks=0: CPython and numpy keep allocations alive until exit by design, so
#           LeakSanitizer reports hundreds of "leaks" that are nothing of the kind. Memory ERRORS
#           are still caught; only the exit-time leak report is off.
#
# Usage:    bash tools/ci/buildSanitizers.sh                 uses python3
#           bash tools/ci/buildSanitizers.sh /usr/bin/python3.13
#
#           EXUDYN_SANITIZER_REPORT_ONLY=1   print the findings but exit 0 (for the first runs,
#                                            while the backlog is being triaged)
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-18 (created; revision2026 step R5.6)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

set -euo pipefail

basePython="${1:-python3}"

if ! command -v "$basePython" >/dev/null 2>&1; then
    echo "ERROR: no interpreter '$basePython'" >&2
    exit 2
fi

scriptDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repoRoot="$(cd "$scriptDir/../.." && pwd)"
cd "$repoRoot"

compiler="${CC:-gcc}"
asanRuntime="$("$compiler" -print-file-name=libasan.so)"
if [ ! -e "$asanRuntime" ]; then
    echo "ERROR: $compiler cannot find libasan.so - install the sanitizer runtime" >&2
    echo "       Ubuntu/Debian: apt-get install -y libasan8 libubsan1" >&2
    echo "       AlmaLinux/manylinux: dnf install -y libasan libubsan" >&2
    exit 2
fi

#the build below uses --no-build-isolation, so pip does NOT fetch [build-system] requires and they
#have to be present already. Checked here, because the alternative is a 200-line pip traceback
#whose actual message - "exudyn needs the pybind11 headers" - is line 443 of it (seen on the first
#CI run of this job, 2026-09-18)
for buildModule in setuptools wheel pybind11; do
    if ! "$basePython" -c "import $buildModule" >/dev/null 2>&1; then
        echo "ERROR: $buildModule is not installed for $basePython, and this script builds with" >&2
        echo "       --no-build-isolation, so pip will not fetch it." >&2
        echo "       pip install 'setuptools>=77' wheel 'pybind11<3.0'" >&2
        exit 2
    fi
done

echo "=== exudyn sanitizer build"
echo "    repository : $repoRoot"
echo "    interpreter: $basePython"
echo "    compiler   : $($compiler --version | head -1)"
echo "    asan       : $asanRuntime"

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#build
#
#-O1 and -fno-omit-frame-pointer: the sanitizers need frames to report a usable stack; -O0 would
#make the suite unbearably slow and -O2 inlines the frames away. -g for file and line.
#-fno-sanitize-recover=undefined is deliberately NOT set: the first run should report EVERY
#undefined operation rather than stopping at the first one.
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
sanitizerFlags="-fsanitize=address,undefined -fno-omit-frame-pointer -g -O1"

export EXUDYN_EXTRA_COMPILE_ARGS="$sanitizerFlags"
export EXUDYN_EXTRA_LINK_ARGS="$sanitizerFlags"
export EXUDYN_COMPILE_EXUDYN_FAST=0
export EXUDYN_QUIET_COMPILE="${EXUDYN_QUIET_COMPILE:-1}"

#the flag signature changed, so setup.py discards the previous object files by itself
#(WriteBuildFlagStamp/DiscardBuildOutputIfFlagsChanged); the explicit removal keeps a half
#instrumented tree from a cancelled run out of the way
rm -rf build/*linux* .eggs ./*.egg-info

wheelDirectory="dist/sanitizers"
rm -rf "$wheelDirectory"

#a sanitizer wheel must NEVER end up next to the shippable ones: it is slow, it depends on the
#ASan runtime and it is useless to a user
"$basePython" -m pip wheel . -w "$wheelDirectory" --no-deps --no-build-isolation

sanitizerWheel="$(ls "$wheelDirectory"/exudyn-*.whl | head -n 1)"
echo "=== wheel: $sanitizerWheel"

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#a throwaway environment, so that the caller's interpreter never ends up with an instrumented
#exudyn it did not ask for; --system-site-packages keeps numpy and friends without a download
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
venvDirectory="${TMPDIR:-/tmp}/exudynSanitizers"
rm -rf "$venvDirectory"
"$basePython" -m venv --system-site-packages "$venvDirectory"
venvPython="$venvDirectory/bin/python"

if ! "$venvPython" -c "import numpy" >/dev/null 2>&1; then
    "$venvPython" -m pip install --no-cache-dir numpy
fi
"$venvPython" -m pip install --no-cache-dir --force-reinstall --no-deps "$sanitizerWheel"

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#run
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
reportFile="$repoRoot/dist/sanitizers/sanitizerRun.log"

cxxRuntime="$("$compiler" -print-file-name=libstdc++.so)"
export LD_PRELOAD="$asanRuntime:$cxxRuntime"
export ASAN_OPTIONS="detect_leaks=0:abort_on_error=0:print_stacktrace=1:handle_abort=1"
export UBSAN_OPTIONS="print_stacktrace=1:report_error_type=1"

#the suite must not run its models in child processes here: each one would pay the preload cost
#again and the findings would be spread over processes that the parent never shows
cd "$repoRoot/python/testing"
set +e
"$venvPython" runTestSuite.py -quiet --exit-code > "$reportFile" 2>&1
suiteExit=$?
set -e
cd "$repoRoot"

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#report: the sanitizers write to stderr and do NOT change the exit code unless they abort, so the
#findings are counted here rather than inferred from $?
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#"ERROR: AddressSanitizer" is the report of a memory error; "CHECK failed" is the runtime itself
#giving up (the libstdc++ mismatch above is one) - both must count, or a run that died in its first
#model is reported as clean
addressFindings=$(grep -c "ERROR: AddressSanitizer\|AddressSanitizer: CHECK failed" "$reportFile" || true)
undefinedFindings=$(grep -c "runtime error:" "$reportFile" || true)

echo ""
echo "=== sanitizer summary"
echo "    test suite exit code      : $suiteExit"
echo "    AddressSanitizer errors   : $addressFindings"
echo "    UndefinedBehavior reports : $undefinedFindings"
echo "    full log                  : $reportFile"

if [ "$undefinedFindings" != "0" ]; then
    echo ""
    echo "--- distinct undefined-behaviour reports ---"
    grep "runtime error:" "$reportFile" | sed 's/^.*\/src\//src\//' | sort -u | head -40
fi

if [ "$addressFindings" != "0" ]; then
    echo ""
    echo "--- first AddressSanitizer report ---"
    grep -A 25 "ERROR: AddressSanitizer" "$reportFile" | head -30
fi

if [ "${EXUDYN_SANITIZER_REPORT_ONLY:-0}" = "1" ]; then
    echo ""
    echo "EXUDYN_SANITIZER_REPORT_ONLY=1: exiting 0 whatever was found"
    exit 0
fi

#DELIBERATELY NOT a failure: the test suite's own exit code. This build is -O1 and instrumented, on
#Linux, in a throwaway environment without the optional packages - so reference values differ, and
#models that need scipy or NGsolve are skipped or fail. Judging those here would make the job red
#for reasons that have nothing to do with memory safety, which is the one thing it exists to check.
#The count is printed; correctness is the business of wheels_linux. An exit code that is neither 0
#nor 1 DOES fail: that is a crash or a Python-level error, not a failed comparison.
if [ "$suiteExit" != "0" ] && [ "$suiteExit" != "1" ]; then
    echo "the test suite exited $suiteExit - that is not a failed comparison; failing the job"
    exit 1
fi

if [ "$addressFindings" != "0" ] || [ "$undefinedFindings" != "0" ]; then
    exit 1
fi

echo "=== sanitizers clean"
