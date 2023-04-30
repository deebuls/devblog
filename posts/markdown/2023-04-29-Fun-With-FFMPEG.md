---
categories:
- fun
date: '2023-04-29'
description: ffmpeg different commands
image: images/bn.png
layout: post
title: FFMPEG - Automating video edition
toc: true

---

## command to remove the audio from a video, speed it up, and add an annotation to indicate the fast forward speed:

```bash
ffmpeg -i input.mp4 -an -filter_complex "[0:v]setpts=0.5*PTS,scale=1280:-2,
drawtext=text='2x Speed':x=(w-text_w)/2:y=10:fontsize=30:fontcolor=white" -c:v libx264 -preset veryfast -crf 18 output.mp4
```

* In this command, -i specifies the input video file, -an removes the audio stream, and -filter_complex applies a filter graph 
that speeds up the video, scales it to a width of 1280 pixels (keeping the aspect ratio), and adds a text annotation indicating 
the fast forward speed.
* The setpts filter speeds up the video by setting the presentation timestamp (PTS) to half of its original value, effectively 
doubling the playback speed. The drawtext filter adds the text "2x Speed" to the center of the video frame with a 
font size of 30 and white color.
* The -c:v libx264 -preset veryfast -crf 18 options specify the video codec and encoding parameters. In this case, 
the video is encoded using the H.264 codec with a CRF (constant rate factor) of 18, which produces a good balance of
 quality and file size. The veryfast preset is used to speed up the encoding process.
* The output file is specified as output.mp4.
* Note that the speedup factor can be adjusted by changing the value in the setpts filter. For example, 
to speed up the video by a factor of 3, you would set the setpts filter to setpts=0.33*PTS.
* the drawtext filter is used to add text to the video. The text option specifies the text to be added, 
and the x and y options specify the coordinates of the bottom left corner of the text. The fontfile, fontsize, and 
fontcolor options are used to specify the font file, font size, and font color of the text.

## Speed up video from timeframe to timeframe
```
ffmpeg -i input.mp4 -filter_complex "[0:v]trim=start=60:end=120,setpts=0.5*PTS[v];[0:a]atrim=start=60:end=120,atempo=2.0[a]" 
-map "[v]" -map "[a]" output.mp4
```

* The trim filter is used to select the specific timeframe of the video, starting at 1 minute (60 seconds) and ending at 2 minutes
 (120 seconds). The setpts filter is then used to adjust the timestamps of the frames in the selected timeframe to achieve the desired 
speedup factor. In this example, the 0.5*PTS value halves the timestamps of the frames, resulting in a speedup factor of 2.
* The atrim and atempo filters are used to select the corresponding audio timeframe and adjust its speed accordingly.
* Finally, the -map option is used to select the video and audio streams to be included in the output file, which is specified as output.mp4.
* You can adjust the values of start, end, and setpts to speed up a different timeframe and to achieve a different speedup factor.

## Different annotation at different time periods

```
ffmpeg -i input.mp4 -vf "drawtext=text='First Text':x=10:y=H-th-10:fontsize=24:fontcolor=white:enable='between(t,5,10)',
drawtext=text='Second Text':x=10:y=H-th-10:fontsize=24:fontcolor=white:enable='between(t,20,25)'" -c:a copy output.mp4
```

## Croping the shape of the video 

```
ffmpeg -i input.mp4 -vf "crop=w=640:h=480:x=320:y=240" output.mp4
```
* In this command, -i specifies the input video file, -vf specifies the video filtergraph, and crop is the filter that crops the video frame.
 The w and h options specify the width and height of the cropped frame, and the x and y options specify the position of the top-left
 corner of the cropped frame relative to the original frame.
* In this example, the crop filter crops the frame to a size of 640x480, starting at position (320, 240) from the top-left corner
 of the original frame. You can adjust the values of w, h, x, and y to crop the frame to your desired size and position.
 
 ## Overlay logo on the image 
 
```
 ffmpeg -i input.mp4 -i logo.png -filter_complex "overlay=10:10" output.mp4
```
* In this command, -i specifies the input video file and -i specifies the logo image file.
 The filter_complex option is used to apply multiple filters to the video.
* The overlay filter is used to overlay the logo image onto the video. 
The 10:10 values specify the position of the top left corner of the logo image 
relative to the top left corner of the video.
* You can adjust the position of the logo image by changing the 10:10 values
 to your desired position. Additionally, you can add other filters to the filter_complex option to apply more effects to the video.
 * Lop top right ``` overlay=W-w-10:10 ``` . W and w refer to the width of the video and the logo image, respectively. The overlay
 filter calculates the position of the logo image by subtracting the width of the logo from the width of the video and offsetting 
it by 10 pixels from the right edge of the video. The 10 value represents the vertical offset of the logo from the top edge of the video.
 
