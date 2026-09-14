@echo off
REM Run a command in every supported conda environment (venvP310 ... venvP314), one after the other.
REM
REM Usage:   execWithAllPythonVersions.bat <command> <script> [arg3 ... arg7]
REM   e.g.   execWithAllPythonVersions.bat python runTestSuite.py -quiet
REM
REM Author: Johannes Gerstmayr
REM Date: 2022-04-01, 2026-09-14 (portable paths)

setlocal
for %%v in (P310 P311 P312 P313 P314) do (
    call "%~dp0execWithPythonVersion.bat" %1 %2 %%v %3 %4 %5 %6 %7
)
endlocal
