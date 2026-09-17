@echo off
REM The Exudyn maintainer driver. Everything it does lives in tools/exudev/ - this file only
REM forwards, so that "exudev test --fast" can be typed from the repository root.
REM
REM   exudev --help            the commands
REM   exudev <command> --help  the options of one command
REM   exudev -n <command>      print what it would run, and do nothing
REM
REM Elsewhere (WSL, linux, macOS): python tools/exudev <command>
REM
REM Author: Johannes Gerstmayr
REM Date: 2026-09-18 (created; revision2026 step R5.18)

setlocal
python "%~dp0tools\exudev" %*
endlocal & exit /b %errorlevel%
