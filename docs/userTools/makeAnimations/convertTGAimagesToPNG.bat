REM 2019-12-23, Johannes Gerstmayr
REM
REM helper file for EXUDYN
REM convert all .tga images of current directory to .png images
REM used to create smaller images, applicable to other codes


"C:\Program Files\IrfanView\i_view64.exe" *.tga /convert="*.png"

REM "i_view32.exe c:\temp\testpics\*.tga /resize=(200,100) /aspectratio /resample /convert=c:\temp\testpics\testsmall\*.png"

