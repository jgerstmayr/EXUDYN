# Converting videos and images with ffmpeg

Recipes used to produce the Exudyn demo animations. See the
[ffmpeg documentation](https://ffmpeg.org/documentation.html) for everything else; the commands are
written for PowerShell, and `./ffmpeg.exe` becomes `ffmpeg` wherever it is on the PATH.

## From Exudyn images to a video

Exudyn writes numbered frames (`frame00000.tga`, `frame00001.tga`, ...) when
`visualizationSettings.exportImages` is used.

```powershell
./ffmpeg.exe -r 25 -start_number 0 -i frame%05d.tga -c:v libx264 -vf "fps=25,format=yuv420p" animation.mp4
./ffmpeg.exe -r 25 -start_number 0 -i frame%05d.png -c:v libx264 -vf "fps=25,format=yuv420p" animation.mp4
```

Reversed, and smaller (`-crf 28`):

```powershell
ffmpeg.exe -r 25 -start_number 0 -i frame%%05d.png -c:v libx264 -crf 28 -vf "fps=25,format=yuv420p" -vf reverse animationHQ.mp4
```

An animated GIF:

```powershell
ffmpeg -i frame%05d.png -framerate 5 test.gif
```

## Frame rate and speed

```powershell
# 4x faster, output at 16 FPS
ffmpeg -i input.mkv -r 16 -filter:v "setpts=0.25*PTS" output.mkv

# 2x faster
ffmpeg -i input.mkv -filter:v "setpts=0.5*PTS" output.mkv

# 4x faster without dropping frames: 10 FPS in, 40 FPS out
ffmpeg -i input.mkv -r 40 -filter:v "setpts=0.25*PTS" output.mkv

# interpolate intermediate frames (slow, but smooth)
ffmpeg -i input.mkv -filter:v "minterpolate='mi_mode=mci:mc_mode=aobmc:vsbmc=1:fps=120'" output.mkv
```

## Cutting and joining

Split out a part, `-ss` start in seconds, `-t` duration in seconds:

```powershell
ffmpeg -i source-file.mp4 -ss 1200 -t 600 file_part.mp4
```

Concatenate with re-encoding:

```powershell
ffmpeg -i part1.mkv -i part2.mkv -filter_complex "[0:v] [0:a] [1:v] [1:a] concat=n=2:v=1:a=1 [v] [a]" -map "[v]" -map "[a]" output.mkv

# three parts, including audio
ffmpeg -i v1c.mkv -i v2c.mkv -i v3c.mkv -filter_complex "[0:v] [0:a] [1:v] [1:a] [2:v] [2:a] concat=n=3:v=1:a=1 [v] [a]" -map "[v]" -map "[a]" -crf 15 output.mkv
```

Concatenate without re-encoding. `test.txt` holds one line per part, each reading
`file 'part1.mkv'`. Note that audio is lost unless both parts were re-encoded with ffmpeg first:

```powershell
ffmpeg -f concat -safe 0 -i test.txt -c copy output.mkv
ffmpeg -f concat -safe 0 -i test.txt output.mkv
```

Concatenate three clips with cross-fades:

```powershell
ffmpeg -y -i v1.avi -i v2.avi  -i v3.avi -f lavfi -i color=black:s=1920x1080 -filter_complex "[0:v]format=pix_fmts=yuva420p,fade=t=out:st=10:d=1:alpha=1,setpts=PTS-STARTPTS[v0]; [1:v]format=pix_fmts=yuva420p,fade=t=in:st=0:d=1:alpha=1,fade=t=out:st=10:d=1:alpha=1,setpts=PTS-STARTPTS+10/TB[v1];  [2:v]format=pix_fmts=yuva420p,fade=t=in:st=0:d=1:alpha=1,fade=t=out:st=10:d=1:alpha=1,setpts=PTS-STARTPTS+20/TB[v2];  [3:v]trim=duration=30[over];  [over][v0]overlay[over1];  [over1][v1]overlay[over2];  [over2][v2]overlay=format=yuv420[outv]" -vcodec libx264 -map [outv] merge.mp4
```

## Text overlays

See the [drawtext documentation](https://ffmpeg.org/ffmpeg-filters.html#drawtext-1).

Text entering from the right, starting at 4.5 seconds:

```powershell
ffmpeg -i input.avi -vf "drawtext=text=string1:fontfile=foo.ttf:y=h-line_h-10:x=w-(t-4.5)*w/5.5:fontcolor=white:fontsize=40:shadowx=2:shadowy=2" output.mp4
```

Text moving up to a position and settling there, centred horizontally. Commas inside the
expression must be escaped as `\,`:

```powershell
ffmpeg -y -i v1b.avi -vf "drawtext=text=Text to be written:fontfile=/windows/fonts/STENCIL.TTF:y=0*h-line_h+if(lt(t\,0.5)\,t*1200\,600+50.*sin((t-0.5)*20)*exp(-6*(t-0.5))):x=(w-text_w)/2:enable=lt(t\,7):fontcolor=gray:fontsize=100:shadowx=2:shadowy=2" test.mkv
```

## Size, scale and crop

`-crf 10` is very high quality, `-crf 40` very low:

```powershell
ffmpeg.exe -i source.mp4 -vcodec libx264 -crf 30 output.mp4

# scale to 720p
ffmpeg -i test.mp4 -vf scale=-1:720 -crf 35 test2.mp4

# crop to width:height:x:y
ffmpeg -i test.mp4 -filter:v "crop=1445:1080:0:0" test2.mp4
```

## Audio

```powershell
# remove
ffmpeg -i input.mkv -c copy -an output.mkv

# extract (for better quality use VLC)
ffmpeg -i output.mkv -q:a 0 -map a output.mp3
ffmpeg -i output.mkv -f mp3 -ab 192000 -vn sound1.mp3

# mix two tracks; put the higher-quality mp3 first
ffmpeg -i sound2.mp3 -i sound1.mp3 -filter_complex amix=inputs=2:duration=longest -ab 192000 soundmix.mp3
ffmpeg -i sound2.mp3 -i sound1.mp3 -filter_complex amix=inputs=2:duration=shortest -ab 192000 soundmix.mp3

# replace the audio of a video
ffmpeg -i file.mkv -i soundmix.mp3 -map 0:v -map 1:a -c:v copy -shortest file.mp4
```

## Compatibility

A video that PowerPoint refuses, or one produced with `reverse`, usually becomes usable again
after a plain re-encode:

```powershell
ffmpeg -i input.mp4 -c:v libx264 -preset slow  -profile:v high -level:v 4.0 -pix_fmt yuv420p -crf 22 -codec:a aac  output.mp4

# maximum compatibility
ffmpeg -i file.mp4 -c:v libx264 -crf 23 -profile:v baseline -level 3.0 -pix_fmt yuv420p -c:a aac -ac 2 -b:a 256k -movflags faststart newFile.mp4
```
