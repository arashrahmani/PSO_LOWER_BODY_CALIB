import math
import numpy as np

def translate(_input, x,y,z):
    mat = [[0., 0., 0., x],
           [0., 0., 0., y], 
           [0., 0., 0., z], 
           [0., 0., 0., 0.]]
    return _input+ mat 

def rotateX(_input, theta):
    mat = [ [1., 0., 0., 0],
            [0., math.cos(theta), -1*math.sin(theta), 0.], 
            [0., math.sin(theta),    math.cos(theta), 0.], 
            [0., 0., 0., 1.] ] 

    return np.matmul(mat,_input)

def rotateY(_input, theta):
    mat = [ [math.cos(theta), 0., math.sin(theta), 0.], 
            [0., 1., 0, 0.], 
            [-1*math.sin(theta), 0., math.cos(theta), 1.],
            [0., 0., 0., 1.] ]
    return np.matmul(mat,_input)

def rotateZ(_input, theta):
    mat = [ [math.cos(theta), -1*math.sin(theta), 0., 0.],      
            [math.sin(theta),    math.cos(theta), 0., 0.], 
            [0., 0., 1., 0.],
            [0., 0., 0., 1.] ] 
    return np.matmul(mat,_input)

def mdh(_input, alpha, a, theta, d):
    f = translate(_input, a, 0., 0.)
    f = rotateX(f, alpha)
    f = translate(f, 0., 0., d)
    f = rotateZ(f,theta)
    return f 

def calculate_r_Pts(q):
    
    f = np.eye(4, dtype=float)
    f = translate(f,0.05,0.1,-0.0055)
    f = mdh(f,0., 0., q[0], 0.)
    f = mdh(f, math.pi/2., 0., q[1]+(math.pi/2.) ,0.)
    f = mdh(f, math.pi/2, 0, q[2], 0)
    f = mdh(f, 0.,0.120, q[3] ,0.)
    f = mdh(f, 0.,0.120, q[4] ,0.)
    f = mdh(f,-1*math.pi/2, 0., q[5]-(math.pi/2), 0.)
    f = mdh(f,-1*math.pi/2, 0., 0., 0.0405)
    p1 = translate(f,-0.00798,-0.04193,0)
    p2 = translate(f,0.02204,-0.04193,0)
    p3 = translate(f,0.02204,-0.07195,0)
    p4 = translate(f,-0.00798,-0.07195,0)
    
    p1 = p1[0:3,3]
    p2 = p2[0:3,3]
    p3 = p3[0:3,3]
    p4 = p4[0:3,3]
    
    return(p1,p2,p3,p4)
    
def calculate_l_Pts(q):
    
    f = np.eye(4, dtype=float)
    f = translate(f,-0.05,0.1,-0.0055)
    f = mdh(f,0., 0., q[0], 0.)
    f = mdh(f, math.pi/2., 0., q[1]+(math.pi/2.) ,0.)
    f = mdh(f, math.pi/2, 0, q[2], 0)
    f = mdh(f, 0.,0.120, q[3] ,0.)
    f = mdh(f, 0.,0.120, q[4] ,0.)
    f = mdh(f,-1*math.pi/2, 0., q[5]-(math.pi/2), 0.)
    f = mdh(f,-1*math.pi/2, 0., 0., 0.0405)
    p1 = translate(f,-0.00798,-0.04193,0)
    p2 = translate(f,0.02204,-0.04193,0)
    p3 = translate(f,0.02204,-0.07195,0)
    p4 = translate(f,-0.00798,-0.07195,0)
    
    p1 = p1[0:3,3]
    p2 = p2[0:3,3]
    p3 = p3[0:3,3]
    p4 = p4[0:3,3]
    
    return(p1,p2,p3,p4)