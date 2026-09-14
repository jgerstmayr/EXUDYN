@echo off
REM Regenerate, build and install exudyn for ONE Python version.
REM
REM Usage:   makeInstallBinaries.bat [noscripts] [P310|P311|P312|P313|P314]     default: P313
REM
REM Author: Johannes Gerstmayr
REM Date: 2020-03-04, 2026-09-14 (portable paths)

setlocal
set skipScripts=no
set Pversion=313

for %%x in (%*) do (
   if [%%~x] EQU [noscripts] set skipScripts=yes
   if [%%~x] EQU [P310] set Pversion=310
   if [%%~x] EQU [P311] set Pversion=311
   if [%%~x] EQU [P312] set Pversion=312
   if [%%~x] EQU [P313] set Pversion=313
   if [%%~x] EQU [P314] set Pversion=314
)

echo "remove builds and eggs ..."
call "%~dp0removeBuildsAndEggs.bat"

if [%skipScripts%] EQU [yes] goto :skipScripts
echo "run scripts ..."
call "%~dp0runPythonScripts.bat" nodoc
:skipScripts

call "%~dp0condaActivate.bat" || exit /b 1
echo "activate venvP%Pversion% ..."
call conda activate venvP%Pversion%

pushd "%~dp0..\.."
echo "uninstall exudyn ..."
call pip uninstall -y exudyn
echo "build exudyn ..."
call pip wheel . -v -w dist --no-deps
call pip install --no-index --pre --find-links=dist exudyn
if exist dist\*.egg del dist\*.egg
popd

call conda deactivate
endlocal
