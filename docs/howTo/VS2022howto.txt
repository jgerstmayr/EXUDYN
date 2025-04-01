#configuration of VS2022 community edition
#download installer

#install standard with following additional features:
#add Python tools
#add Python 3.9 
#  as this is a naked Python, also install:
	pip install scipy
	pip install matplotlib
	#plots do not stay open, so use plt.show(block=True) after (serveral) plots
#add DesktopDevelopment (3GB) (this is requested by MSVC when loading Exudyn)
#add Windows 10 SDK 
#not sure which SDK one is optimal, but without it does not take >1000 includes like <math.h> and <GL/gl.h> etc.

#virtual environments untested

#inspect .pyd (versions, etc.) file:
& 'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\MSVC\14.37.32822\bin\Hostx64\x64\dumpbin' /version exudynCPP.cp311-win_amd64.pyd
