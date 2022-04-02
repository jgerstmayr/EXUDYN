echo off
REM 2019-12-23, Johannes Gerstmayr
REM helper file for EXUDYN to convert all frame00000.png, frame00001.png, ...  files to a video
REM for higher quality use crf option (standard: -crf 23, range: 0-51, lower crf value means higher quality)

IF EXIST animation.mp4 (
    echo "animation.mp4 already exists! rename the file"
) ELSE (
    "C:\Program Files (x86)\FFMPEG\bin\ffmpeg.exe" -r 25 -start_number 0 -i frame%%05d.png -c:v libx264 -vf "fps=25,format=yuv420p" animation.mp4
)


