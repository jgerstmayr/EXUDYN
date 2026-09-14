@echo off
REM Build the wheel for the active Python and reinstall it. Called from the repository root inside
REM an activated environment, usually through execWithPythonVersion.bat / execWithAllPythonVersions.bat.
REM
REM Usage:   buildInstallSingleVersion.bat [yes|no]     yes: skip the (re)installation
REM
REM Author: Johannes Gerstmayr
REM Date: 2022-04-01, 2026-09-14 (portable paths)

setlocal
set "skipInstall=%~1"

pushd "%~dp0..\.."
call pip wheel . -v -w dist --no-deps

if [%skipInstall%] EQU [yes] goto :skipInstall
call pip uninstall exudyn -y
call pip install --no-index --pre --find-links=dist exudyn
:skipInstall

popd
endlocal
