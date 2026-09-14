@echo off
REM Build the html documentation with sphinx into _build/ (open _build\index.html afterwards).
REM
REM The command, for reference:   sphinx-build -b html . _build -E
REM   -b html   html builder
REM   .         the source directory: the repository root, which holds conf.py and index.rst
REM   _build    the output directory
REM   -E        always read all files (no stale pages from the environment cache)
REM
REM Usage:   makeSphinxDoc.bat [env]      env: the conda environment, default venvExuP313
REM
REM Author: Johannes Gerstmayr
REM Date: 2026-09-14

setlocal
set "condaEnv=venvExuP313"
if not [%~1]==[] set "condaEnv=%~1"

call "%~dp0condaActivate.bat" || exit /b 1
call conda activate %condaEnv%

pushd "%~dp0..\.."
sphinx-build -b html . _build -E
set "result=%errorlevel%"
popd

call conda deactivate
endlocal & exit /b %result%
