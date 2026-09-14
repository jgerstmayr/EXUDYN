@echo off
REM Compile the LaTeX documentation docs/theDoc/theDoc.tex (twice, with bibtex8) into theDoc.pdf.
REM For the html documentation use makeSphinxDoc.bat.
REM
REM Author: Johannes Gerstmayr
REM Date: 2019-11-22, 2026-09-14 (portable paths)

setlocal
pushd "%~dp0..\..\docs\theDoc"
echo first run: compile theDoc.tex
bibtex8.exe theDoc
pdflatex.exe -quiet theDoc.tex
echo second run: compile theDoc.tex
bibtex8.exe theDoc
pdflatex.exe -quiet theDoc.tex
echo compilation finished
popd
endlocal
