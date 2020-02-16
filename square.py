import numpy as np
class square:
    def __init__(self,imageCs,realCs,l,_kc,ID):
        self.imageCorners = imageCs
        self.realCorners = realCs
        self.length = l
        self.kc = _kc
        self.id = ID
    def setImageCorners(self,corners):
        self.imageCorners = corners
    def setRealCorners(self,corners):
        self.realCorners = corners
    def get_r_leg_Pts(self):
        points_list = []
        points_list.append((self.realCorners[2][1], -(self.realCorners[2][0]) ,self.realCorners[2][2]))
        points_list.append((self.realCorners[1][1], -(self.realCorners[1][0]) ,self.realCorners[1][2]))
        points_list.append((self.realCorners[0][1], -(self.realCorners[0][0]) ,self.realCorners[0][2]))
        points_list.append((self.realCorners[3][1], -(self.realCorners[3][0]) ,self.realCorners[3][2]))
        return points_list
    def get_l_leg_Pts(self):
        points_list = []
        points_list.append((self.realCorners[0][1], -(self.realCorners[0][0]) ,self.realCorners[0][2]))
        points_list.append((self.realCorners[3][1], -(self.realCorners[3][0]) ,self.realCorners[3][2]))
        points_list.append((self.realCorners[2][1], -(self.realCorners[2][0]) ,self.realCorners[2][2]))
        points_list.append((self.realCorners[1][1], -(self.realCorners[1][0]) ,self.realCorners[1][2]))
        return points_list
    def get_Pts_for_show(self):
        points_list = []
        points_list.append(( -(self.realCorners[0][1]),self.realCorners[0][0] ,self.realCorners[0][2]))
        points_list.append(( -(self.realCorners[1][1]),self.realCorners[1][0] ,self.realCorners[1][2]))
        points_list.append(( -(self.realCorners[2][1]),self.realCorners[2][0] ,self.realCorners[2][2]))
        points_list.append(( -(self.realCorners[3][1]),self.realCorners[3][0] ,self.realCorners[3][2]))
        return points_list