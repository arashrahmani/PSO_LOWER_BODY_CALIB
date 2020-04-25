from v4l2 import *
import numpy as np
import cv2
import fcntl
import mmap
import time
import select
cameraIndx = 0

sX = 0.0000025
sY = 0.0000033

fX2Pixel = 1389.5804
fY2Pixel = 1391.4165
cX2Pixel = 954.0355
cY2Pixel = 516.0549

k1 = 0.1328
k2 = -0.2744
k3 = 0.1305
p1 = 0.0016
p2 = -0.0017

class camera: 
    def __init__(self ,cameraIndx):
        self.buffers = []
        self.frame = []
        self.index = cameraIndx
        self.video_cap = open('/dev/video' + str(cameraIndx), 'rb+', buffering=0)
        self.cp = v4l2_capability()
        self.fmt = v4l2_format()
        self.frame_size = v4l2_frmsize_discrete()
        self.set_camera_params()
        self.update_frame()
        self.cameraMatrix = np.asarray([
            [fX2Pixel ,0        ,cX2Pixel ],
            [0        ,fY2Pixel ,cY2Pixel ],
            [0        ,0        ,1        ]
        ])
        self.distCoeffs = np.asarray([k1 ,k2 ,p1 ,p2 ,k3])
        self.fX2Meter = fX2Pixel * sX
        self.fY2Meter = fY2Pixel * sY
        self.cX2Meter = cX2Pixel * sX
        self.cY2Meter = cY2Pixel * sY
    def set_camera_params(self):
        fcntl.ioctl(self.video_cap, VIDIOC_QUERYCAP, self.cp)
        self.fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        fcntl.ioctl(self.video_cap, VIDIOC_G_FMT, self.fmt)  # setting YUYV as default for logitechC920
        self.frame_size.width = 1920
        self.frame_size.height = 1080
        fcntl.ioctl(self.video_cap, VIDIOC_S_FMT, self.fmt)  # set whatever default settings we got before
        parm = v4l2_streamparm()
        parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME
        fcntl.ioctl(self.video_cap, VIDIOC_G_PARM, parm)
        fcntl.ioctl(self.video_cap, VIDIOC_S_PARM, parm)  # just got with the defaults
        # init mmap capture
        req = v4l2_requestbuffers()
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        req.memory = V4L2_MEMORY_MMAP
        req.count = 1  # nr of buffer frames
        fcntl.ioctl(self.video_cap, VIDIOC_REQBUFS, req)  # tell the driver that we want some buffers 
        # parm.type = V4L2_CID_FOCUS_AUTO

        for ind in range(req.count):
            # setup a buffer
            buf = v4l2_buffer()
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = V4L2_MEMORY_MMAP
            buf.index = ind
            fcntl.ioctl(self.video_cap, VIDIOC_QUERYBUF, buf)

            mm = mmap.mmap(self.video_cap.fileno(), buf.length, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE, offset=buf.m.offset)
            self.buffers.append(mm)

            # queue the buffer for capture
            fcntl.ioctl(self.video_cap, VIDIOC_QBUF, buf)


        # Start streaming
        self.buf_type = v4l2_buf_type(V4L2_BUF_TYPE_VIDEO_CAPTURE)
        fcntl.ioctl(self.video_cap, VIDIOC_STREAMON, self.buf_type)
        # Capture image
        t0 = time.time()
        max_t = 1
        ready_to_read, ready_to_write, in_error = ([], [], [])
        print(">>> select")
        while len(ready_to_read) == 0 and time.time() - t0 < max_t:
            ready_to_read, ready_to_write, in_error = select.select([self.video_cap], [], [], max_t)
    def update_frame(self):
            buf = v4l2_buffer()
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = V4L2_MEMORY_MMAP

            # get image from the driver queue
            fcntl.ioctl(self.video_cap, VIDIOC_DQBUF, buf)
            mm = self.buffers[buf.index]
            self.frame = np.frombuffer(mm, dtype=np.uint8).reshape(self.fmt.fmt.pix.height, self.fmt.fmt.pix.width,2)
            self.frame = cv2.cvtColor(self.frame,cv2.COLOR_YUV2GRAY_YUY2)
            # img = np.fromiter(bytes(bit for i, bit in enumerate(mm.read()) if not i % 2), dtype=np.uint64)
            mm.seek(0)
            fcntl.ioctl(self.video_cap, VIDIOC_QBUF, buf)  # requeue the buffer
    def stop_streaming(self):
        fcntl.ioctl(self.video_cap, VIDIOC_STREAMOFF, self.buf_type)
        self.video_cap.close()