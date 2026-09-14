@echo off
REM Build all manylinux wheels in the manylinux docker image, through WSL.
REM Requires Docker Desktop with WSL integration enabled for your distribution
REM (Docker -> Settings -> Resources -> WSL integration). The Python versions are selected in
REM manylinuxBuild.sh; the per-version build is tools/ci/buildManylinux.sh (same code as GitLab CI).
REM
REM Usage:   makeUbuntuManyLinuxWheels.bat [nofast]     nofast: skip the fast-linalg variant (EXUDYN_NOFAST=1)
REM
REM Author: Johannes Gerstmayr
REM Date: 2020-03-04, 2026-09-14 (portable paths)

setlocal
set "noFastEnv="
for %%x in (%*) do (
   if [%%~x] EQU [nofast] set "noFastEnv=-e EXUDYN_NOFAST=1"
)

pushd "%~dp0..\.."
REM the repository root as seen from WSL, e.g. /mnt/c/DATA/cpp/EXUDYN_git
for /f "delims=" %%p in ('wsl wslpath -a "%CD%"') do set "wslRoot=%%p"
echo repository in WSL: %wslRoot%

wsl -e bash -lc "rm -rf build/*linux* dist/manylinux/*linux*.whl"
wsl -e bash -lc "docker run --rm -e PLAT=manylinux_2_28_x86_64 %noFastEnv% -v '%wslRoot%:/work' -w /work quay.io/pypa/manylinux_2_28_x86_64 bash /work/tools/buildAndGenerate/manylinuxBuild.sh"

popd
endlocal
