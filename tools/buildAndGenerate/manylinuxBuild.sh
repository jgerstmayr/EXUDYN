#!/usr/bin/env bash
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
# Build all manylinux wheels inside a manylinux container, for high compatibility.
#
# This used to be the same block copy-pasted once per Python version. The body now lives in
# tools/ci/buildManylinux.sh, which takes the Python tag as its argument, so that the GitLab CI
# matrix and this local docker path run exactly the same code.
#
# Usage:  invoked by makeUbuntuManyLinuxWheels.bat, or directly inside the container:
#           bash /work/tools/buildAndGenerate/manylinuxBuild.sh
#           bash /work/tools/buildAndGenerate/manylinuxBuild.sh cp312 cp313   # a subset
#
# Author:   Johannes Gerstmayr
# Date:     2026-09-09 (restructured)
# Copyright:This file is part of Exudyn. Exudyn is free software: see 'LICENSE.txt'
#
#+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

set -euo pipefail

scriptDir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
buildOne="$scriptDir/../ci/buildManylinux.sh"

pythonTags=("$@")
if [ ${#pythonTags[@]} -eq 0 ]; then
    pythonTags=(cp310 cp311 cp312 cp313 cp314)
fi

for pyTag in "${pythonTags[@]}"; do
    bash "$buildOne" "$pyTag"
done

echo "=== all manylinux wheels built and tested: ${pythonTags[*]}"
