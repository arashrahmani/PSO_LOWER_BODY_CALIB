import cv2
import numpy as np

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
        self.index = cameraIndx
        self.videoCap = cv2.VideoCapture(cameraIndx)
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
    def update_frame(self):
        _ ,self.frame = self.videoCap.read()
    def set_camera_config(self):
        self.videoCap.set(3, 1920)
        self.videoCap.set(4, 1080)
        self.videoCap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self.videoCap.set(cv2.CAP_PROP_FOCUS, 0)
        self.videoCap.set(cv2.CAP_PROP_AUTO_WB, 0)
        self.videoCap.set(cv2.CAP_PROP_GAIN, 0)