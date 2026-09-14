@echo off
REM Make all Windows wheels, reinstall them, run the test suites for all Python versions, and start
REM the documentation and linux builds in separate windows.
REM
REM Usage:   makeAndTestAllBinaries.bat [nodoc] [noscripts] [notests] [noinstall] [nolinux] [none]
REM
REM Author: Johannes Gerstmayr
REM Date: 2021-05-03, 2026-09-14 (portable paths)

setlocal
set skipDoc=no
set skipScripts=no
set skipTests=no
set skipInstall=no
set skipLinux=no

for %%x in (%*) do (
   if [%%~x] EQU [nodoc] set skipDoc=yes
   if [%%~x] EQU [noscripts] set skipScripts=yes
   if [%%~x] EQU [notests] set skipTests=yes
   if [%%~x] EQU [noinstall] set skipInstall=yes
   if [%%~x] EQU [nolinux] set skipLinux=yes
   if [%%~x] EQU [none] (
      set skipDoc=yes
      set skipScripts=yes
      set skipTests=yes
      set skipInstall=yes
      set skipLinux=yes
   )
)

echo "processing: skipDoc=%skipDoc%, skipScripts=%skipScripts%, skipTests=%skipTests%, skipInstall=%skipInstall%, skipLinux=%skipLinux%"

call "%~dp0removeBuildsAndEggs.bat"

if [%skipScripts%] NEQ [yes] call "%~dp0runPythonScripts.bat" makeAll

REM documentation in a separate window (in background)
if [%skipDoc%] NEQ [yes] start "theDoc" cmd.exe /c "%~dp0makeDoc.bat"

call "%~dp0execWithAllPythonVersions.bat" call buildInstallSingleVersion.bat %skipInstall%

if [%skipTests%] EQU [yes] goto :skipTests
if [%skipInstall%] EQU [yes] goto :skipTests
call "%~dp0runTestSuite.bat"
call "%~dp0runPerformanceTests.bat"
call "%~dp0runTestExamples.bat"
:skipTests

REM linux wheels in a separate window
if [%skipLinux%] NEQ [yes] start "manylinux" cmd.exe /c "%~dp0makeUbuntuManyLinuxWheels.bat"

echo "run addTags.bat after commit to add tags!"
REM avoid closing of shell
pause
endlocal
