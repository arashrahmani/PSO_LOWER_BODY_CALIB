import ActuatorComm
import trajectoryGenerator
import PSO_optimizer
import numpy as np
import F_kine
import time
import ArucoPosEstimation
import json
n_steps = 20
wait_time_sec = 2
r_leg_offsets = [0.0,0.0,0.0,0.0,0.0,0.0]
l_leg_offsets = [0.0,0.0,0.0,0.0,0.0,0.0]
initial = [1.0,-1.0,1.0,-1.0,1.0,-1.0]                        # initial starting location [x1,x2...]
bounds=[(-6.5,6.5),(-6.5,6.5),(-6.5,6.5),(-6.5,6.5),(-6.5,6.5),(-6.5,6.5)]  # input bounds [(x1_min,x1_max),(x2_min,x2_max)...]
def collect_dataSet():
    logs = []
    time.sleep(1)
    for key,val in enumerate(trajectoryGenerator.trajectoeies):
        if key != len(trajectoryGenerator.trajectoeies) - 1:
            trajectoryGenerator.trajectory_interpolate_record(val,trajectoryGenerator.trajectoeies[key + 1] ,n_steps ,wait_time_sec ,logs)
    return logs

ActuatorComm.init()

while (True):
    command = input("hello, issue your command my lord >> :D __DELI__")
    if command == 'c':
        logs = collect_dataSet()
        with open('data.txt', 'w') as outfile:
            json.dump(logs, outfile)

        #--- COST FUNCTION ------------------------------------------------------------+
        # function we are attempting to optimize (minimize)
        def cost_func(offsets):
            cost = np.zeros(len(logs))
            for i in range(len(logs)):
                desired_points = logs[i][1]
                calculated_points = F_kine.calculatePoints(np.add(logs[i][0] , offsets))
                for j in range(0,4):
                    dist = np.linalg.norm(desired_points[j] - calculated_points[j])
                    cost[i] = cost[i] + (dist**2)
            cost = np.sum(cost)/float(len(logs))
            return cost
        print("calibrating ...\n")
        R_leg_optimizer = PSO_optimizer.PSO(cost_func,initial,bounds,num_particles=20,maxiter=100)
        print('my best: ',R_leg_optimizer.best_global_pos)
    elif command == 't':
        print("test mode ")
        ActuatorComm.test(r_leg_offsets,l_leg_offsets)
        # [0., 0., 90.,  8., -30., -0.,  0., -0.,  0., -0., 0.,    1.2667777577605168, -1.2075711459265237, 1.4420370383704575, -0.8310515094327424, 0.38372034673184086, 0.2912239340130389     , 90., -8., -30.])
        