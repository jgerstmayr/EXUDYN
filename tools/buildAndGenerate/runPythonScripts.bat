@echo off
REM Regenerate all generated files (C++ headers, stubs, documentation sources) and build the html
REM documentation. The generators, their order and their inputs/outputs are declared in
REM tools/generators/generate.py; tools/regenerate.py runs them and reports drift against the commit.
REM
REM Usage:   runPythonScripts.bat [makeAll] [nodoc]
REM            makeAll   also write docs/theDoc/buildDate.tex (makeAllBinariesScripts.py)
REM            nodoc     skip the sphinx build
REM
REM Author: Johannes Gerstmayr
REM Date: 2022-04-01, 2026-09-14 (uses the generator driver)

setlocal
set makeAllScripts=no
set skipDoc=no
for %%x in (%*) do (
   if [%%~x] EQU [makeAll] set makeAllScripts=yes
   if [%%~x] EQU [nodoc] set skipDoc=yes
)

call "%~dp0condaActivate.bat" || exit /b 1
call conda activate venvExuP313

pushd "%~dp0..\.."
if [%makeAllScripts%] EQU [yes] python src\pythonGenerator\makeAllBinariesScripts.py
python tools\regenerate.py
popd

call conda deactivate

if [%skipDoc%] NEQ [yes] call "%~dp0makeSphinxDoc.bat"
endlocal
