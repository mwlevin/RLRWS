import numpy as np
from helper import *
import sys


def measurement_fb(
    x,
    pos_ego,
    pos_preced,
    pos_CV,
    param,
    num_CV,  
):  
    spd, _, _ = state2data(x, param)    
    z = np.zeros((num_CV, 1))
    dx = param.dx
    count = 0    
    x_ego = 0 - pos_ego

    x_veh = 0 - pos_ego
    index_spd = int((x_veh - x_ego) / dx)
    alpha_spd = (x_veh - x_ego) / dx - int((x_veh - x_ego) / dx)
    z[count, 0] = (1 - alpha_spd) * spd[index_spd, 0] + (
        alpha_spd
    ) * spd[index_spd + 1, 0]
    count += 1
    
    if pos_preced is not None:
        x_veh = 0 - pos_preced
        index_spd = int((x_veh - x_ego) / dx)
        alpha_spd = (x_veh - x_ego) / dx - int((x_veh - x_ego) / dx)
        z[count, 0] = (1 - alpha_spd) * spd[index_spd, 0] + (
            alpha_spd
        ) * spd[index_spd + 1, 0]
        count += 1
    
    for index in range(len(pos_CV)):
        x_veh = 0 - pos_CV[index]
        index_spd = int((x_veh - x_ego) / dx)
        alpha_spd = (x_veh - x_ego) / dx - int((x_veh - x_ego) / dx)
        z[count, 0] = (1 - alpha_spd) * spd[index_spd, 0] + (
            alpha_spd
        ) * spd[index_spd + 1, 0]
        count += 1    
    return z
