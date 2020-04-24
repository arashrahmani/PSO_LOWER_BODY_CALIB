from v4l2 import *
import fcntl
import mmap
import select
import time
import numpy as np
import cv2
import ctypes
vd = open('/dev/video0', 'rb+', buffering=0)

cp = v4l2_capability()
fcntl.ioctl(vd, VIDIOC_QUERYCAP, cp)
fmt = v4l2_format()
fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
fcntl.ioctl(vd, VIDIOC_G_FMT, fmt)  # get current settings
print("width:", fmt.fmt.pix.width, "height", fmt.fmt.pix.height)
# print("pxfmt:", "V4L2_PIX_FMT_YUYV" if fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_YUYV else fmt.fmt.pix.pixelformat)
# print("sizeimage:", fmt.fmt.pix.sizeimage)
fcntl.ioctl(vd, VIDIOC_S_FMT, fmt)  # set whatever default settings we got before

# print(">>> streamparam")  ## somewhere in here you can set the camera framerate
parm = v4l2_streamparm()
parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME
fcntl.ioctl(vd, VIDIOC_G_PARM, parm)
fcntl.ioctl(vd, VIDIOC_S_PARM, parm)  # just got with the defaults

# init mmap capture
req = v4l2_requestbuffers()
req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
req.memory = V4L2_MEMORY_MMAP
req.count = 1  # nr of buffer frames
fcntl.ioctl(vd, VIDIOC_REQBUFS, req)  # tell the driver that we want some buffers 
# print("req.count", req.count)


buffers = []

# print(">>> VIDIOC_QUERYBUF, mmap, VIDIOC_QBUF")
for ind in range(req.count):
    # setup a buffer
    buf = v4l2_buffer()
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
    buf.memory = V4L2_MEMORY_MMAP
    buf.index = ind
    fcntl.ioctl(vd, VIDIOC_QUERYBUF, buf)

    mm = mmap.mmap(vd.fileno(), buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=buf.m.offset)
    buffers.append(mm)

    # queue the buffer for capture
    fcntl.ioctl(vd, VIDIOC_QBUF, buf)


# Start streaming
buf_type = v4l2_buf_type(V4L2_BUF_TYPE_VIDEO_CAPTURE)
fcntl.ioctl(vd, VIDIOC_STREAMON, buf_type)


# Capture image
t0 = time.time()
max_t = 1
ready_to_read, ready_to_write, in_error = ([], [], [])
print(">>> select")
while len(ready_to_read) == 0 and time.time() - t0 < max_t:
    ready_to_read, ready_to_write, in_error = select.select([vd], [], [], max_t)

for _ in range(50):  # capture 50 frames
    buf = v4l2_buffer()
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
    buf.memory = V4L2_MEMORY_MMAP
    # get image from the driver queue
    fcntl.ioctl(vd, VIDIOC_DQBUF, buf)
    mm = buffers[buf.index]
    # image = np.asarray(bytearray(bit for i, bit in enumerate(mm.read()) if not i % 2), dtype="uint8").reshape(fmt.fmt.pix.height, fmt.fmt.pix.width)
    # print(np.frombuffer(mm, dtype=np.uint8))
    frame = np.frombuffer(mm, dtype=np.uint8).reshape(fmt.fmt.pix.height, fmt.fmt.pix.width,2)
    frame = cv2.cvtColor(frame, cv2.COLOR_YUV2GRAY_YUY2)
    # img = np.fromiter(bytes(bit for i, bit in enumerate(mm.read()) if not i % 2), dtype=np.uint64)
    cv2.imshow("this is it",frame)
    mm.seek(0)
    fcntl.ioctl(vd, VIDIOC_QBUF, buf)  # requeue the buffer
    cv2.waitKey(30)

print(">> Stop streaming")
fcntl.ioctl(vd, VIDIOC_STREAMOFF, buf_type)
vd.close()