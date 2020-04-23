import math
import numpy as np
from shmAPI import pyShm
import trajectoryGenerator
def get_command():
    return pyShm.get("dcmActuator","command",20)

def get_hardness():
    return pyShm.get("dcmActuator","hardness",20)

def set_hardness(hardness):
    pyShm.set("dcmActuator","hardness",hardness)

def set_command(comands):
    # assert(comands!= 20)
    pyShm.set("dcmActuator","command",comands)

def set_torque_enable():
    pyShm.set("dcmActuator","torqueEnable",[1])
    pyShm.set("dcmActuator","torqueEnableChanged",[1])

def set_torque_disable():
    pyShm.set("dcmActuator","torqueEnable",[0])
    pyShm.set("dcmActuator","torqueEnableChanged",[1])

init_hardness = [0.5, 0.5, 0.3, 0.3, 0.3, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.3, 0.3, 0.3]
        
def init():
    set_torque_enable()
    commands = get_command()
    hardnness = set_hardness(init_hardness)
    for k,v in enumerate(trajectoryGenerator.init_state):
        commands[k] = v*math.pi/180
    set_command(commands)
def test(test_set):
    commands = get_command()
    for k,v in enumerate(test_set):
        commands[k] = v*math.pi/180
    set_command(commands)