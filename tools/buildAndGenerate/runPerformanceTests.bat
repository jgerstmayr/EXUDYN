@echo off
REM Run the performance tests for all Python versions, or for one.
REM
REM Usage:   runPerformanceTests.bat [all|P310|...|P314]     default: all
REM
REM Arguments after the version are passed on to runPerformanceTests.py, for example:
REM   runPerformanceTests.bat P313 --fast-module   measure exudynCPPfast (revision2026 step R5.11)
REM   runPerformanceTests.bat P313 --overwrite-log
REM NOTE %~dp0 must be read BEFORE the shift loop below: shift moves %1 into %0, so
REM afterwards %~dp0 is the CURRENT directory and not this script (checked 2026-09-17).
REM
REM Author: Johannes Gerstmayr
REM Date: 2020-03-04, 2026-09-14 (portable paths)

setlocal enabledelayedexpansion
set "pythonVersionTests=all"
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

if [%pythonVersionTests%] EQU [all] call "!scriptDir!execWithAllPythonVersions.bat" python runPerformanceTests.py -quiet !extraArgs!
if [%pythonVersionTests%] NEQ [all] call "!scriptDir!execWithPythonVersion.bat" python runPerformanceTests.py %pythonVersionTests% -quiet !extraArgs!
endlocal
