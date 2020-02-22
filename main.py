import ActuatorComm
import trajectoryGenerator
import PSO_optimizer
import numpy as np
import F_kine
import time
import ArucoPosEstimation
import json
import math
import cv2
from square import square
r_logs = []
l_logs = []
n_steps = 2
wait_time_sec = 2
r_leg_offsets = [0.0,0.0,0.0,0.0,0.0,0.0]
l_leg_offsets = [0.0,0.0,0.0,0.0,0.0,0.0]
initial = [1.0,-1.0,1.0,-1.0,1.0,-1.0]                        # initial starting location [x1,x2...]
bounds = [(-6.5,6.5),(-6.5,6.5),(-6.5,6.5),(-6.5,6.5),(-6.5,6.5),(-6.5,6.5)]  # input bounds [(x1_min,x1_max),(x2_min,x2_max)...]
def collect_dataSet():
    right_logs = []
    left_logs = []
    time.sleep(wait_time_sec)
    cam_log_squares = ArucoPosEstimation.get_camera_log()
    if cam_log_squares is not None:
        for i in range (len(cam_log_squares)):
            if cam_log_squares[i].id == 2:
                r_logs.append([trajectoryGenerator.init_r_q, cam_log_squares[i].get_leg_Pts_4_kinematics()])
            elif cam_log_squares[i].id == 1:
                l_logs.append([trajectoryGenerator.init_l_q, cam_log_squares[i].get_leg_Pts_4_kinematics()])
        time.sleep(wait_time_sec)
    for key,val in enumerate(trajectoryGenerator.r_leg_states):
        if key != len(trajectoryGenerator.r_leg_states) -1:
            trajectoryGenerator.trajectory_interpolate_record(val,trajectoryGenerator.r_leg_states[key + 1] ,n_steps ,wait_time_sec ,right_logs,None)
    for key,val in enumerate(trajectoryGenerator.l_leg_states):
        if key != len(trajectoryGenerator.l_leg_states) - 2:
            trajectoryGenerator.trajectory_interpolate_record(val,trajectoryGenerator.l_leg_states[key + 1] ,n_steps ,wait_time_sec ,None,left_logs)
    return right_logs,left_logs


def r_cost_func(offsets):
    cost = np.zeros(len(r_logs))
    for i in range(len(r_logs)):
        desired_points = r_logs[i]
        calculated_points = F_kine.calculate_r_Pts(np.add(r_logs[i][0] , offsets))
        for j in range(0,4):
            dist = np.linalg.norm(desired_points[j] - calculated_points[j])
            cost[i] = cost[i] + (dist**2)
    cost = np.sum(cost)/float(len(r_logs))
    return cost
def l_cost_func(offsets):
    cost = np.zeros(len(l_logs))
    for i in range(len(l_logs)):
        desired_points = l_logs[i]
        calculated_points = F_kine.calculate_l_Pts(np.add(l_logs[i][0] , offsets))
        for j in range(0,4):
            dist = np.linalg.norm(desired_points[j] - calculated_points[j])
            cost[i] = cost[i] + (dist**2)
    cost = np.sum(cost)/float(len(l_logs))
    return cost
ActuatorComm.init()
while (True):
    command = input("hello, issue your command my lord >> :D __DELI__")
    if command == 'c':
        r_logs,l_logs = collect_dataSet()
        #--- COST FUNCTION ------------------------------------------------------------+
        # function we are attempting to optimize (minimize)
        cv2.destroyAllWindows()
        print("calibrating right leg ...")
        R_leg_optimizer = PSO_optimizer.PSO(r_cost_func,initial,bounds,num_particles = 20,maxiter = 100)
        print("done !\ncalibrating left leg ...")
        L_leg_optimizer = PSO_optimizer.PSO(l_cost_func,initial,bounds,num_particles = 20,maxiter = 100)
        r_leg_offsets = R_leg_optimizer.best_global_pos
        l_leg_offsets = L_leg_optimizer.best_global_pos
        print('Best Right Leg Params:\n',R_leg_optimizer.best_global_pos)
        print('Best Left Leg Params:\n',L_leg_optimizer.best_global_pos)
    elif command == 't':
        print("test mode ")
        r_leg_offsets = [0.26339805612907735, -1.1908196013683745, 1.2737564840879076, -1.4278649142094448, 0.6466623868587632, 0.14110476928785157]
        l_leg_offsets = [-1.7979332135203556, -0.5225758641408023, 1.865215675904088, -0.9110044505105875, -0.07877740180470048, -0.19298324719643106] 
        all_motors = [0., 0., 90.,  8., -30.]+l_leg_offsets+ r_leg_offsets+[ 90., -8., -30.]
        ActuatorComm.test(all_motors)
    elif command == 'v':
        ActuatorComm.init()
        key = 0
        cam_log_squares = ArucoPosEstimation.get_camera_log()
        # corrected_l_q = np.add(trajectoryGenerator.init_l_q,l_leg_offsets)
        # corrected_r_q = np.add(trajectoryGenerator.init_r_q,r_leg_offsets)
        corrected_l_q = trajectoryGenerator.init_l_q
        corrected_r_q = trajectoryGenerator.init_r_q
        l_leg_calculated_Pts = square(None,F_kine.calculate_l_Pts(corrected_l_q),ArucoPosEstimation.arucoLength,None,1)
        print("asdfasdf")
        r_leg_calculated_Pts = square(None,F_kine.calculate_r_Pts(corrected_r_q),ArucoPosEstimation.arucoLength,None,2)
        while(cv2.waitKey(1) != 27):
            ArucoPosEstimation.show_desired_in_image(r_leg_calculated_Pts,l_leg_calculated_Pts)
            ArucoPosEstimation.cam0.update_frame()
        cv2.destroyAllWindows()  