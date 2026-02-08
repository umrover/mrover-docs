---
title: "Camera Streaming"
---
**Context**: At URC we are not allowed to look directly at the rover. Instead we are in a "basestation" which is sealed off. Thus we need a camera stream to view what is happening near the rover.

**Problem**: GStreamer is finicky and unreliable. At URC it cut out and we did not regain it. It also is such a general tool that it suffers performance issues - we could get a much faster and robust stream by writing some custom code for known hardware.

**Solution**:

* Use the [NVIDIA Video Encoder API](https://docs.nvidia.com/video-technologies/video-codec-sdk/11.1/nvenc-video-encoder-api-prog-guide/index.html) directly on the Jetson. Encode using H.265 (HVEC) which is better than H.264 (previously used).
* Connect a websocket on the basestation to a POSIX socket on the Jetson. Forward bytes outputed by the encoder to this stream.
* Use [emscripten](https://emscripten.org/) to compile a H.265 C library (such as [libde265](https://github.com/strukturag/libde265)) that parses the input stream and converts it to a video directly in the browser. Draw it to a canvas.