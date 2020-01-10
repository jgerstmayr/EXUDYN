echo off
REM 2019-12-23, Johannes Gerstmayr
REM
REM helper file for EXUDYN to convert all frame00000.tga, frame00001.tga, ...  files to a video
REM higher quality than standard, using crf option (standard: -crf 23)

IF EXIST animationHQ.mp4 (
    echo "animationHQ.mp4 already exists! rename the file"
) ELSE (
    "C:\Program Files (x86)\FFMPEG\bin\ffmpeg.exe" -r 25 -start_number 0 -i frame%%05d.tga -c:v libx264 -crf 10 -vf "fps=25,format=yuv420p" animationHQ.mp4
)


