This is a program to decode video and render to framebuffer.
This program use FFmpeg to parse and hardware decoder(VPU) to decode stream.
You can choose to write the output YUV data to file or render it to framebuffer.

Here is a command example:
./myplay -i sunflower_2B_2ref_WP_40Mbps.mp4 -o test.yuv -f 8

This program now support mp4 format only.
Need more work to support more format.
