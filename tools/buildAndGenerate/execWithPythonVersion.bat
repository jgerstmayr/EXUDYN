@echo off
REM Run a command in the conda environment venv<version>, from the repository root - or from
REM python\TestModels for the test runners.
REM
REM Usage:   execWithPythonVersion.bat <command> <script> <version> [arg3 ... arg7]
REM   e.g.   execWithPythonVersion.bat python runTestSuite.py P313 -quiet
REM          execWithPythonVersion.bat call buildInstallSingleVersion.bat P312 no
REM
REM Author: Johannes Gerstmayr
REM Date: 2021-05-02, 2026-09-14 (portable paths)

setlocal
set "pythonVersion=%~3"

set execTests=no
if [%~2] EQU [runTestSuite.py] set execTests=yes
if [%~2] EQU [runTestExamples.py] set execTests=yes
if [%~2] EQU [runPerformanceTests.py] set execTests=yes

REM scripts of this directory are called with their full path
set "script=%~2"
if exist "%~dp0%~2" set "script=%~dp0%~2"

call "%~dp0condaActivate.bat" || exit /b 1

pushd "%~dp0..\.."
if [%execTests%] EQU [yes] cd python\TestModels

echo +++++++++++++++++++++++++++++++++++++++++++++++++++++
echo "Process %1 %2 for Exudyn on Windows with Python %pythonVersion%"
call conda activate venv%pythonVersion%
%1 "%script%" %4 %5 %6 %7 %8
call conda deactivate

popd
endlocal
