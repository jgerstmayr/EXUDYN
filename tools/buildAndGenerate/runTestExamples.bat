@echo off
REM Run the Examples test set (slow; releases and large steps only).
REM
REM Usage:   runTestExamples.bat [all|P310|...|P314]     default: P312
REM
REM Author: Johannes Gerstmayr
REM Date: 2024-05-11, 2026-09-14 (portable paths)

setlocal
set "pythonVersionTests=P312"
if not [%~1]==[] set "pythonVersionTests=%~1"

if [%pythonVersionTests%] EQU [all] call "%~dp0execWithAllPythonVersions.bat" python runTestExamples.py -quiet
if [%pythonVersionTests%] NEQ [all] call "%~dp0execWithPythonVersion.bat" python runTestExamples.py %pythonVersionTests% -quiet
endlocal
