@echo off
REM Clean the Windows build directories and eggs created by setup.py / pip wheel.
REM Linux build directories are kept (removing them caused problems for the linux build).
REM dist\ itself is kept; only stray .egg files are deleted from it.
REM
REM Author: Johannes Gerstmayr
REM Date: 2020-08-17, 2026-09-14 (portable paths)

setlocal
pushd "%~dp0..\.."

for /d %%d in (build\lib.win-amd64-* build\temp.win-amd64-* build\bdist.win32 build\bdist.win-amd64) do rd "%%d" /s /q
if exist .eggs rd .eggs /s /q
if exist exudyn.egg-info rd exudyn.egg-info /s /q
if exist python\exudyn.egg-info rd python\exudyn.egg-info /s /q
if exist dist\*.egg del dist\*.egg

popd
endlocal
