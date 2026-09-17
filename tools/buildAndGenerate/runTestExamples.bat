@echo off
REM Run the Examples test set (slow; releases and large steps only).
REM
REM Usage:   runTestExamples.bat [all|P310|...|P314]     default: P312
REM
REM Arguments after the version are passed on to runTestExamples.py, for example:
REM   runTestExamples.bat P312 --serial
REM   runTestExamples.bat P312 --overwrite-log
REM NOTE %~dp0 must be read BEFORE the shift loop below: shift moves %1 into %0, so
REM afterwards %~dp0 is the CURRENT directory and not this script (checked 2026-09-17).
REM
REM Author: Johannes Gerstmayr
REM Date: 2024-05-11, 2026-09-14 (portable paths)

setlocal enabledelayedexpansion
set "pythonVersionTests=P312"
if not [%~1]==[] set "pythonVersionTests=%~1"

set "scriptDir=%~dp0"   REM before any shift; see the note above
set "extraArgs="
shift
:collectArgs
if not [%~1]==[] (
    set "extraArgs=!extraArgs! %~1"
    shift
    goto collectArgs
)

if [%pythonVersionTests%] EQU [all] call "!scriptDir!execWithAllPythonVersions.bat" python runTestExamples.py -quiet !extraArgs!
if [%pythonVersionTests%] NEQ [all] call "!scriptDir!execWithPythonVersion.bat" python runTestExamples.py %pythonVersionTests% -quiet !extraArgs!
endlocal
