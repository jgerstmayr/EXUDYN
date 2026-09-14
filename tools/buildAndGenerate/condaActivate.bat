@echo off
REM Activate the conda base installation, so that "conda activate <env>" works afterwards.
REM Used by every script in this directory instead of a hard-coded Anaconda path.
REM
REM The base installation is found, in this order:
REM   1. EXUDYN_CONDA_ROOT, if set (e.g. set EXUDYN_CONDA_ROOT=D:\Miniconda3)
REM   2. CONDA_EXE, which "conda init" defines in every shell (<root>\Scripts\conda.exe)
REM   3. conda.exe / conda.bat found on PATH
REM
REM Usage:   call "%~dp0condaActivate.bat" || exit /b 1
REM
REM Author: Johannes Gerstmayr
REM Date: 2026-09-14 (created; replaces the hard-coded paths in the other scripts)

set "condaRoot="
if defined EXUDYN_CONDA_ROOT set "condaRoot=%EXUDYN_CONDA_ROOT%"
if not defined condaRoot if defined CONDA_EXE for %%i in ("%CONDA_EXE%\..\..") do set "condaRoot=%%~fi"
if not defined condaRoot for /f "delims=" %%i in ('where conda 2^>nul') do if not defined condaRoot for %%j in ("%%~dpi..") do set "condaRoot=%%~fj"

if not defined condaRoot goto :notFound
if not exist "%condaRoot%\Scripts\activate.bat" goto :notFound

call "%condaRoot%\Scripts\activate.bat"
exit /b 0

:notFound
echo ERROR: conda installation not found. Set EXUDYN_CONDA_ROOT to the conda base directory
echo        (the directory containing Scripts\activate.bat), or run "conda init cmd.exe".
exit /b 1
