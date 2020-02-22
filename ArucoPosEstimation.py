import cameraConfig
import cv2
import math
import numpy as np
from cv2 import aruco
from numpy.linalg import inv
from square import square

def world_2_image(point3D,focalX,focalY,centerX,centerY,sX,sY):
    col = point3D[0] * focalX / point3D[2]
    col = centerX + col
    col = col / sX
    row = point3D[1] * focalY / point3D[2]
    row = centerY + row
    row = row / sY
    return (row,col)
arucoLength = 0.03
cameraIndx = 0
cam0 = cameraConfig.camera(cameraIndx)
cam0.set_camera_config()
def show_desired_in_image(r_leg_square,l_leg_square,CAMERA = None):
    r_leg_Pts = r_leg_square.get_Pts_for_show()
    l_leg_Pts = l_leg_square.get_Pts_for_show()
    if CAMERA == None:
        tmp_mat = cam0.frame.copy()
    else:
        tmp_mat = CAMERA.frame.copy()
    for i in range(4):
        row,col = world_2_image(r_leg_Pts[i] ,cam0.fX2Meter ,cam0.fY2Meter ,cam0.cX2Meter ,cam0.cY2Meter ,cameraConfig.sX ,cameraConfig.sY)
        desired_r_Pts_on_image = (int(col) ,int(row))
        row,col = world_2_image(l_leg_Pts[i] ,cam0.fX2Meter ,cam0.fY2Meter ,cam0.cX2Meter ,cam0.cY2Meter ,cameraConfig.sX ,cameraConfig.sY)
        desired_l_Pts_on_image = (int(col) ,int(row))
        cv2.circle(tmp_mat ,desired_r_Pts_on_image ,3 ,(10,255,10) ,thickness = -1 ,lineType = 8 ,shift = 0)
        cv2.circle(tmp_mat ,desired_l_Pts_on_image ,3 ,(10 , 255 ,10) ,thickness = -1 ,lineType = 8 ,shift = 0)
    cv2.imshow("<< Desired Squares IMG >>" ,tmp_mat)
    cv2.waitKey(30)
    # cv2.destroyAllWindows()
def get_camera_log():
    cam0.update_frame()
    h ,w = cam0.frame.shape[:2]
    newcameramtx ,roi = cv2.getOptimalNewCameraMatrix(cam0.cameraMatrix ,cam0.distCoeffs ,(w ,h) , 1,(w ,h))
    undistorted = cv2.undistort(cam0.frame ,cam0.cameraMatrix ,cam0.distCoeffs ,newcameramtx)
    gray = cv2.cvtColor(undistorted ,cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters_create()
    corners ,ids ,rejectedImgPoints = aruco.detectMarkers(gray ,aruco_dict ,parameters = parameters)
    cam0.frame = aruco.drawDetectedMarkers(undistorted.copy() ,corners ,ids)
    arucoList = []
    if len(ids) == 2:
        for i in range(len(ids)):
            corner2Pix = corners[i][0]
            corner2metre = np.zeros((4 ,2))
            corners3D = np.zeros((4 ,3))
            kc = np.zeros((4 ,2))
            for row in range(0 ,4):
                corner2metre[row][0] = (corner2Pix[row][0] * cameraConfig.sX) - cam0.cX2Meter
                corner2metre[row][1] = (corner2Pix[row][1] * cameraConfig.sY) - cam0.cY2Meter
                kc[row][0] = corner2metre[row][0] / cam0.fX2Meter
                kc[row][1] = corner2metre[row][1] / cam0.fY2Meter
            Q = [
                [kc[1][0] ,kc[3][0] ,-kc[0][0] ],
                [kc[1][1] ,kc[3][1] ,-kc[0][1] ],
                [    1    ,    1    ,    -1    ]
            ]
            Qinv = np.linalg.inv(Q)
            L0 = [kc[2][0] ,kc[2][1] ,1]
            L = np.zeros(3)
            for j in range(0 ,3):
                for k in range(0 ,3):
                    L[j] += Qinv[j][k] * L0[k]
            M = math.pow((kc[1][0] * L[0]) - (kc[0][0] * L[2]) ,2) + math.pow(kc[1][1] * L[0] - kc[0][1] * L[2] ,2) + math.pow(L[0] - L[2] ,2)
            corners3D[2][2] = arucoLength / math.sqrt(M)
            corners3D[0][2] = L[2] * corners3D[2][2]
            corners3D[1][2] = L[0] * corners3D[2][2]
            corners3D[3][2] = L[1] * corners3D[2][2]
            for cornerNum in range(0 ,4):
                corners3D[cornerNum][0] = kc[cornerNum][0] * corners3D[cornerNum][2]
                corners3D[cornerNum][1] = kc[cornerNum][1] * corners3D[cornerNum][2]
                arucoList.append(square(corner2metre ,corners3D ,arucoLength ,kc,ids[i]))
            # print(corners3D)
        return arucoList
    else:
        return None