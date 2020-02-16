import ActuatorComm
import ArucoPosEstimation
import numpy as np
import math
import time
#   19,20,		--Head
#   2,4,6,		--LArm
#   8,10,12,14,16,18,--LLeg
#   7,9,11,13,15,17,--RLeg
#   1,3,5,		--RArm

trajectoeies = []
trajectoeies.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0.,  -0.,  0., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))
#Left leg
# trajectoeies.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -40., 40., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))
# trajectoeies.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -30.,  0., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))
# trajectoeies.append(np.asarray([0., 0., 90.,  8., -30., -0.,  5.,  -0.,  0., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))
# trajectoeies.append(np.asarray([0., 0., 90.,  8., -30., -0.,  5.,  22.,  0., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))
# trajectoeies.append(np.asarray([0., 0., 90.,  8., -30., -0.,  4., -30., 40., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))
# trajectoeies.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0.,  -0.,  0., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))


#Right leg
trajectoeies.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -0.,  0., -0., 0., -0., -0., -40., 40.,-0, 0., 90., -8., -30.]))
# trajectoeies.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -0.,  0., -0., 0., -0., -0., -30., 0.,-0, 0., 90., -8., -30.]))
# trajectoeies.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -0.,  0., -0., 0., -0., -5., -0., 0.,-0, 0., 90., -8., -30.]))
# trajectoeies.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -0.,  0., -0., 0., -0., -5., 22., 0.,-0, 0., 90., -8., -30.]))
# trajectoeies.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0., -0.,  0., -0., 0., -0., -4., -30., 0.,-0, 0., 90., -8., -30.]))
# trajectoeies.append(np.asarray([0., 0., 90.,  8., -30., -0.,  0.,  -0.,  0., -0., 0., -0.,  0., -0., 0.,-0, 0., 90., -8., -30.]))


def trajectory_interpolate_record(start_state ,end_state ,n_steps ,wait_time ,logs):
    step_size =  (end_state - start_state) / n_steps
    print('step_size : ' ,step_size)
    for k in range(n_steps):
        start_state = start_state + step_size
        ActuatorComm.set_command(start_state * math.pi / 180)
        time.sleep(wait_time)
        cam_log = ArucoPosEstimation.get_camera_log()
        if len(cam_log) > 0:
            q = [(start_state * math.pi / 180)[i] for i in range(11,17,1)]
            logs.append([q, cam_log[0].getPoint()])
