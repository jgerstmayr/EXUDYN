@echo off
REM Build, repair and test linux wheels directly in WSL with its conda environments venvP310 ...
REM venvP314 (not in docker). The preferred path for release wheels is makeUbuntuManyLinuxWheels.bat.
REM
REM Usage:   makeUbuntuWheels.bat [quiet] [nofast] [P310 ... P314]     default: all versions
REM
REM Author: Johannes Gerstmayr
REM Date: 2020-03-04, 2026-09-14 (portable paths, one loop instead of one block per version)

setlocal enabledelayedexpansion
set "quiet="
set "nofast="
set "versions="
for %%x in (%*) do (
   if [%%~x] EQU [quiet] set "quiet=--quiet"
   if [%%~x] EQU [nofast] set "nofast=--nofast"
   if /i "%%~x" GEQ "P3" if /i "%%~x" LEQ "P399" set "versions=!versions! %%~x"
)
if not defined versions set "versions=P310 P311 P312 P313 P314"

for /f "delims=" %%x in (%~dp0..\..\version.txt) do set "Build=%%x"

pushd "%~dp0..\.."
for /f "delims=" %%p in ('wsl wslpath -a "%CD%"') do set "wslRoot=%%p"
wsl -e bash -lc "cd '%wslRoot%' && rm -rf build/*linux*"

for %%v in (%versions%) do (
    set "tag=%%v"
    set "tag=cp!tag:~1!"
    echo +++++ linux wheel for %%v ^(!tag!^)
    wsl -e bash -ic "cd '%wslRoot%' && conda activate venv%%v && python3 -m pip wheel . -w dist --no-deps && python3 -m pip uninstall exudyn -y"
    wsl -e bash -ic "cd '%wslRoot%' && auditwheel repair dist/exudyn-%Build%-!tag!-!tag!-linux_x86_64.whl -w ./dist"
    wsl -e bash -ic "cd '%wslRoot%' && conda activate venv%%v && python3 -m pip install --no-index --pre --find-links=dist exudyn"
    wsl -e bash -ic "cd '%wslRoot%/python/TestModels' && conda activate venv%%v && python3 runTestSuite.py -quiet && python3 runPerformanceTests.py -quiet"
)

popd
endlocal
