#!/usr/bin/env bash
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# This is an EXUDYN maintainer tool
#
# Details:  Build and test one manylinux wheel for a single Python version. Must be run INSIDE a
#           manylinux container (the image provides /opt/python/<tag>-<tag>/bin), which is why it
#           works unchanged both under GitLab CI - where the image is the job image, so no
#           docker-in-docker and no privileged mode is needed - and under a local
#           'docker run ... quay.io/pypa/manylinux_2_28_x86_64'.
#
# Usage:    bash tools/ci/buildManylinux.sh cp313
#
#           EXUDYN_NOFAST=1   skip the __FAST_EXUDYN_LINALG variant, roughly halving build time.
#                             Regular test runs do not exercise the fast binary, so this is the
#                             sensible setting for routine CI; release builds must NOT set it.
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-09 (created)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

set -euo pipefail

if [ $# -lt 1 ]; then
    echo "usage: $0 <python tag, e.g. cp313>" >&2
    exit 2
fi

pyTag="$1"
pyBin="/opt/python/${pyTag}-${pyTag}/bin"

if [ ! -x "$pyBin/python" ]; then
    echo "ERROR: no interpreter at $pyBin - is this running inside a manylinux container?" >&2
    exit 2
fi

#locate the repository regardless of the current working directory
scriptDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repoRoot="$(cd "$scriptDir/../.." && pwd)"
mainDir="$repoRoot/main"

echo "=== exudyn manylinux build: $pyTag"
echo "    repository : $repoRoot"

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#system libraries: GLFW and X11 are needed because setup.py links -lglfw -lGL (see setup.py:225)
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
if ! rpm -q glfw-devel >/dev/null 2>&1; then
    dnf install -y epel-release
    dnf install -y glfw-devel libX11-devel mesa-libGL-devel mesa-libGLU-devel
fi

#manylinux sets this for its own tooling; it confuses the build
unset LD_LIBRARY_PATH

cd "$mainDir"

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#optionally disable the fast variant. setup.py reads setupPyConfig.json from the working
#directory, and 'pip wheel' gives no way to pass --nofast through, so the file is edited and
#restored. The trap guarantees restoration even if the build fails.
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
configFile="$mainDir/setupPyConfig.json"
configBackup=""
RestoreConfig() {
    if [ -n "$configBackup" ] && [ -f "$configBackup" ]; then
        mv -f "$configBackup" "$configFile"
        echo "    restored setupPyConfig.json"
    fi
}
trap RestoreConfig EXIT

if [ "${EXUDYN_NOFAST:-0}" = "1" ]; then
    echo "    fast variant: DISABLED (EXUDYN_NOFAST=1)"
    configBackup="$(mktemp)"
    cp "$configFile" "$configBackup"
    sed -i 's/"compileExudynFast"[[:space:]]*:[[:space:]]*"True"/"compileExudynFast": "False"/' "$configFile"
else
    echo "    fast variant: enabled"
fi

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#build
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
rm -rf build/*linux* .eggs ./*.egg-info

"$pyBin/pip" install -U pip setuptools
"$pyBin/pip" wheel . -v -w dist/manylinux --no-cache-dir --no-deps

rawWheel="$(ls dist/manylinux/exu*"${pyTag}"-linux*.whl | head -n 1)"
echo "=== raw wheel: $rawWheel"
auditwheel show "$rawWheel"
auditwheel repair "$rawWheel" -w dist/manylinux

repairedWheel="$(ls dist/manylinux/exu*"${pyTag}"*manylinux*.whl | head -n 1)"
echo "=== repaired wheel: $repairedWheel"

#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#test the wheel that would actually ship, not the source tree
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#scipy is PINNED: newer releases have a much slower sparse eigenvalue solver. Measured on the
#GitLab runner 2026-09-10 with an unpinned scipy, abaqusImportTest.py alone took 60.0 s of the
#106 s test suite - 56% of the whole run for one otherwise unremarkable test. See revision
#plan fact 19. Raise this pin deliberately, and re-measure when doing so.
"$pyBin/pip" install --no-cache-dir numpy "scipy==1.15.2" matplotlib
"$pyBin/pip" install --no-cache-dir --force-reinstall --no-deps "$repairedWheel"

cd "$mainDir/pythonDev/TestModels"
"$pyBin/python" runTestSuite.py -quiet -local --exit-code

echo "=== $pyTag OK"
