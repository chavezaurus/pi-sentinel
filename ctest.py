#!/usr/bin/env python3
import fcntl
import mmap
import os
import select
import v4l2
import sys
import ctypes


def main():
    DEVICE = "/dev/video2"
    OUTPUT = "output.h264"
    WIDTH = 1920
    HEIGHT = 1080
    NUM_BUFFERS = 4

    print(f"Opening {DEVICE}")
    fd = os.open(DEVICE, os.O_RDWR, 0)

    # -----------------------------
    # Set format
    # -----------------------------
    fmt = v4l2.v4l2_format()
    fmt.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
    fmt.fmt.pix.width = WIDTH
    fmt.fmt.pix.height = HEIGHT
    fmt.fmt.pix.pixelformat = v4l2.V4L2_PIX_FMT_H264
    fmt.fmt.pix.field = v4l2.V4L2_FIELD_NONE

    fcntl.ioctl(fd, v4l2.VIDIOC_S_FMT, fmt)

    # -----------------------------
    # Request buffers
    # -----------------------------
    req = v4l2.v4l2_requestbuffers()
    req.count = NUM_BUFFERS
    req.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
    req.memory = v4l2.V4L2_MEMORY_MMAP

    fcntl.ioctl(fd, v4l2.VIDIOC_REQBUFS, req)

    print(f"Allocated {req.count} buffers")

    # -----------------------------
    # Map buffers
    # -----------------------------
    buffers = []
    for i in range(req.count):

        buf = v4l2.v4l2_buffer()
        buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
        buf.memory = v4l2.V4L2_MEMORY_MMAP
        buf.index = i

        fcntl.ioctl(fd, v4l2.VIDIOC_QUERYBUF, buf)

        mm = mmap.mmap(
            fd,
            buf.length,
            mmap.MAP_SHARED,
            mmap.PROT_READ | mmap.PROT_WRITE,
            offset=buf.m.offset
        )

        buffers.append((mm, buf.length))

        # queue buffer
        fcntl.ioctl(fd, v4l2.VIDIOC_QBUF, buf)

    # -----------------------------
    # Start streaming
    # -----------------------------
    buf_type = ctypes.c_int(v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE)
    fcntl.ioctl(fd, v4l2.VIDIOC_STREAMON, buf_type)

    print("Streaming… writing raw H264 to", OUTPUT)

    # -----------------------------
    # Capture loop
    # -----------------------------
    frame_count = 0
    with open(OUTPUT, "wb") as out:
        try:
            while True:
                select.select([fd], [], [])

                buf = v4l2.v4l2_buffer()
                buf.type = v4l2.V4L2_BUF_TYPE_VIDEO_CAPTURE
                buf.memory = v4l2.V4L2_MEMORY_MMAP

                # dequeue
                fcntl.ioctl(fd, v4l2.VIDIOC_DQBUF, buf)

                mm, _ = buffers[buf.index]
                frame_count += 1
                if buf.bytesused > 0 and frame_count < 300:
                    out.write(mm[:buf.bytesused])

                # requeue
                fcntl.ioctl(fd, v4l2.VIDIOC_QBUF, buf)

                if frame_count % 100 == 0:
                    print(f"Frames: {frame_count}")

        except KeyboardInterrupt:
            print("Stopping…")

    # -----------------------------
    # Stop streaming
    # -----------------------------
    fcntl.ioctl(fd, v4l2.VIDIOC_STREAMOFF, buf_type)
    os.close(fd)


if __name__ == "__main__":
    main()
