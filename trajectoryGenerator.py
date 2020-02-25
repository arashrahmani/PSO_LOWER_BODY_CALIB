import ActuatorComm
import ArucoPosEstimation
import numpy as np
import math
import time
import F_kine
import square
#   19,20,		--Head
#   2,4,6,		--LArm
#   8,10,12,14,16,18,--LLeg
#   7,9,11,13,15,17,--RLeg
#   1,3,5,		--RArm

r_leg_states = []
l_leg_states = []
#init state
init_state = np.asarray([0., 0., 90.,  8., -30., -0.,  0.,  -0.,  0., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.])
init_l_q = [(init_state [j] * math.pi / 180) for j in range(5,11,1)]
init_r_q = [(init_state [j] * math.pi / 180) for j in range(11,17,1)]
#Left leg
l_leg_states.append(init_state)
# ===========================================
# l_leg_states.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -10., 10., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))
#===========================================



l_leg_states.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -40., 40., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))
l_leg_states.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -30.,  0., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))
l_leg_states.append(np.asarray([0., 0., 90.,  8., -30., -0.,  5.,  -0.,  0., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))
l_leg_states.append(np.asarray([0., 0., 90.,  8., -30., -0.,  5.,  22.,  0., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))
l_leg_states.append(np.asarray([0., 0., 90.,  8., -30., -0.,  4., -30., 40., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))
l_leg_states.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0.,  -0.,  0., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))

#Right leg
r_leg_states.append(init_state)

#=============================================================================
# r_leg_states.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -0.,  0., -0., 0., -100., -0., -0., 0.,-0, 0., 90., -8., -30.]))

#==============================================
r_leg_states.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -0.,  0., -0., 0., -0., -0., -40., 40.,-0, 0., 90., -8., -30.]))
r_leg_states.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -0.,  0., -0., 0., -0., -0., -30., 0.,-0, 0., 90., -8., -30.]))
r_leg_states.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -0.,  0., -0., 0., -0., -5., -0., 0.,-0, 0., 90., -8., -30.]))
r_leg_states.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -0.,  0., -0., 0., -0., -5., 22., 0.,-0, 0., 90., -8., -30.]))
r_leg_states.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -0.,  0., -0., 0., -0., -4., -30., 0.,-0, 0., 90., -8., -30.]))
r_leg_states.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0.,  -0.,  0., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))


def trajectory_interpolate_record(start_state ,end_state ,n_steps ,wait_time ,r_logs = None,l_logs = None):
    step_size = np.subtract(end_state , start_state) / n_steps
    if l_logs is None:
        for k in range(n_steps):
            start_state = np.add(start_state , step_size)
            ActuatorComm.set_command([(start_state[j] * math.pi / 180)for j in range(len(start_state))])
            time.sleep(wait_time)
            cam_log_squares = ArucoPosEstimation.get_camera_log()
            q = [(start_state[j] * -1.*math.pi / 180) for j in range(11,17,1)]
            r_leg_calculated_Pts = square.square(None,F_kine.calculate_r_Pts(q),ArucoPosEstimation.arucoLength,None,2)
            l_leg_calculated_Pts = square.square(None,F_kine.calculate_l_Pts(init_l_q),ArucoPosEstimation.arucoLength,None,1)
            ArucoPosEstimation.show_desired_in_image(r_leg_calculated_Pts,l_leg_calculated_Pts)
            if cam_log_squares is not None:
                for i in range(len(cam_log_squares)):
                    if cam_log_squares[i].id == 2:
                        r_logs.append([q, cam_log_squares[i].get_leg_Pts_4_kinematics()])
                        print("camera log squares :\n",cam_log_squares[i].get_leg_Pts_4_kinematics())
                        print("right leg from kinematics: :\n",F_kine.calculate_r_Pts(q))
    elif r_logs is None:
        for k in range(n_steps):
            start_state = np.add(start_state , step_size)
            ActuatorComm.set_command([(start_state[i] * math.pi / 180)for i in range(len(start_state))])
            time.sleep(wait_time)
            cam_log_squares = ArucoPosEstimation.get_camera_log()
            q = [(start_state[i] * math.pi / 180) for i in range(5,11,1)]
            l_leg_calculated_Pts = square.square(None,F_kine.calculate_l_Pts(q),ArucoPosEstimation.arucoLength,None,1)
            r_leg_calculated_Pts = square.square(None,F_kine.calculate_r_Pts(init_r_q),ArucoPosEstimation.arucoLength,None,2)
            ArucoPosEstimation.show_desired_in_image(r_leg_calculated_Pts,l_leg_calculated_Pts)
            if cam_log_squares is not None:
                for i in range(len(cam_log_squares)):
                    if cam_log_squares[i].id == 1:
                        l_logs.append([q, cam_log_squares[i].get_leg_Pts_4_kinematics()])


        
