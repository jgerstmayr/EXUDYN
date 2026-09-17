@echo off
REM The Exudyn maintainer driver. Everything it does lives in tools/exudev/ - this file only
REM finds a working python and forwards, so that "exudev test --fast" can be typed anywhere.
REM
REM   exudev --help            the commands
REM   exudev <command> --help  the options of one command
REM   exudev -n <command>      print what it would run, and do nothing
REM
REM Elsewhere (WSL, linux, macOS): python tools/exudev
REM
REM The interpreter is PROBED rather than assumed: on a Windows without python on PATH, "python"
REM is the Microsoft Store stub, which prints an advertisement and exits 9009 - so a bare
REM "python ..." here would look like the driver had run and done nothing.
REM
REM Author: Johannes Gerstmayr
REM Date: 2026-09-18 (created; revision2026 step R5.18)

setlocal
set "exudevPython="

python -c "import sys" >nul 2>&1
if not errorlevel 1 set "exudevPython=python"

if not defined exudevPython (
    py -3 -c "import sys" >nul 2>&1
    if not errorlevel 1 set "exudevPython=py -3"
)

if not defined exudevPython (
    echo ERROR: no working python found. Activate a conda environment ^(conda activate venvExuP313^),
    echo        or run the driver directly:  python "%~dp0tools\exudev" --help
    exit /b 1
)

%exudevPython% "%~dp0tools\exudev" %*
endlocal & exit /b %errorlevel%
