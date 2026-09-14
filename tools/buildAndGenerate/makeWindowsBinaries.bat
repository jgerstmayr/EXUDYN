@echo off
REM Build the Windows wheels for all Python versions, or for one.
REM
REM Usage:   makeWindowsBinaries.bat [all|P310|...|P314]     default: all
REM
REM Author: Johannes Gerstmayr
REM Date: 2020-03-04, 2026-09-14 (portable paths; pip wheel instead of setup.py bdist_wheel)

setlocal
set "pythonVersionInstall=all"
if not [%~1]==[] set "pythonVersionInstall=%~1"

if [%pythonVersionInstall%] EQU [all] (
    call "%~dp0execWithAllPythonVersions.bat" call buildInstallSingleVersion.bat yes
) else (
    call "%~dp0execWithPythonVersion.bat" call buildInstallSingleVersion.bat %pythonVersionInstall% yes
)
endlocal
