@echo off
REM Run the test suite for all Python versions, or for one.
REM
REM Usage:   runTestSuite.bat [all|P310|...|P314]     default: all
REM
REM Author: Johannes Gerstmayr
REM Date: 2020-03-04, 2026-09-14 (portable paths)

setlocal
set "pythonVersionTests=all"
if not [%~1]==[] set "pythonVersionTests=%~1"

if [%pythonVersionTests%] EQU [all] call "%~dp0execWithAllPythonVersions.bat" python runTestSuite.py -quiet
if [%pythonVersionTests%] NEQ [all] call "%~dp0execWithPythonVersion.bat" python runTestSuite.py %pythonVersionTests% -quiet
endlocal
