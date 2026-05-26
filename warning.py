
import numpy as np
import matplotlib.pyplot as plt
import os
import pickle
import datetime
from copy import deepcopy
from UKF import UnscentedKalmanFilter
from estimation_param import *
from helper import *
from measurement import measurement_fb
from traffic_dynamics import traffic_dynamics
from est_pred_param import *
from nmpc_rlrws_no_preced import *


def warning(pos_pred_ego, pos_pred_max_ego, pos_pred_min_ego, pos_ego, spd_ego, spd_pred_ego, predicted_tl_state, sim_param):
    
    original_pos_ego = -pos_ego
    original_spd_ego = spd_ego
    num_horizon = 50
    # time step for discretization, amend this when num_horizon is changed
    timestep = 0.2
    
    pos_pred_min_current_ego = np.zeros(num_horizon + 1)
    pos_pred_max_current_ego = np.zeros(num_horizon + 1)
    pos_pred_current_ego = np.zeros(num_horizon + 1)
    spd_pred_current_ego = np.zeros(num_horizon + 1)
    pos_pred_min_current_ego[0] = (
        pos_pred_min_ego[0, 0] + original_pos_ego
    )
    pos_pred_max_current_ego[0] = (
        pos_pred_max_ego[0, 0] + original_pos_ego
    )
    pos_pred_current_ego[0] = pos_pred_ego[0,0] + original_pos_ego
    spd_pred_current_ego[0] = spd_pred_ego[0, 0]
    

    for id in range(0, num_horizon):
        pos_pred_min_current_ego[id + 1] = (
            pos_pred_min_ego[ (id + 1), 0]
            + original_pos_ego
        )
        pos_pred_max_current_ego[id + 1] = (
            pos_pred_max_ego[(id + 1), 0]
            + original_pos_ego
        )
        pos_pred_current_ego[id + 1] = (
            pos_pred_ego[(id+1), 0]+original_pos_ego
        )
        spd_pred_current_ego[id + 1] = spd_pred_ego[
            (id + 1), 0
        ]
    
    if -original_pos_ego >= 0 and -original_pos_ego<=500:
        tl_in_range = [0]
        tl_pos_in_range = [-original_pos_ego]
        tl_status_in_range = [predicted_tl_state[0, 0]]
        tl_cell_id_in_range = [int((-original_pos_ego) / sim_param.dx + 1)]
    else:
        tl_in_range = []
        tl_pos_in_range = []
        tl_status_in_range = []
        tl_cell_id_in_range = []

    
    
    if len(tl_in_range) == 0:
        u_sol = np.zeros((num_horizon, U_DIM_W))
    else:
        tl_index = tl_in_range[
            0
        ]  # first traffic light index in communication range
        tl_position = tl_pos_in_range[
            0
        ]  # first traffic light relative position to original ego vehicle
        tl_cell_id = tl_cell_id_in_range[0]

        tl_status_pred = np.zeros((num_horizon + 1))
        tl_status_pred[0] = tl_status_in_range[0]

        tl_change_index_pred = None
        tl_status_last_pred = 1

        for i in range(0, num_horizon):
            tl_status_in_range_pred = predicted_tl_state[i+1, 0]
            tl_status_pred[i + 1] = predicted_tl_state[i+1, 0]
            # print(tl_status_in_range_pred[0])
            # last step is green, now is red
            if tl_status_last_pred == 1 and tl_status_in_range_pred == 0:
                tl_change_index_pred = i
                tl_status_last_pred = 0
        # no preceding vehicle
        
        # the ego vehicle doesn't reach the last cell ahead of the red signal
        if (
            pos_pred_current_ego[-1]
            # pos_pred_max_current_ego[-1]
            >= tl_position + original_pos_ego - sim_param.dx
            and pos_pred_current_ego[-1]
            # and pos_pred_max_current_ego[-1]
            <= tl_position + original_pos_ego
            and tl_status_pred[-1] == 0
            and abs(pos_pred_current_ego[-1] - pos_pred_current_ego[-2])<=0.2*2
            # and abs(pos_pred_max_current_ego[-1] - pos_pred_max_current_ego[-2])<=0.2*2
        ):
            terminal_constraint = True
        else:
            terminal_constraint = False
        debug_value = False

        # no light change from green to red
        if tl_change_index_pred is None:
            tl_constraint = False
        # at the time change to red, ego vehicle has passed the signal
        elif (
            pos_pred_current_ego[tl_change_index_pred]
            # pos_pred_max_current_ego[tl_change_index_pred]
            >= tl_position + original_pos_ego
        ):  
            tl_constraint = False
        else:
            tl_constraint = True
        
        x_sol, u_sol = nmpc_no_preced(
            num_horizon,
            timestep,
            original_pos_ego,
            original_spd_ego,
            original_pos_ego + tl_position,
            0,
            tl_status_pred,
            sim_param,
            terminal_constraint=terminal_constraint,
            tl_constraint=tl_constraint,
            debug_value=debug_value,
        )
    #print('pos_ego',-original_pos_ego)
    if -original_pos_ego >= 0 and -original_pos_ego<=500:
        # print('pos_ego',-original_pos_ego)
        # print('spd_ego',original_spd_ego)
        # print('tl_status',tl_status_in_range[0])
        if  -original_pos_ego <=60 and original_spd_ego<=4 and tl_status_in_range[0]==0: 
            warning_signal=np.zeros((50))
        else:
            warning_signal = u_sol[:, U_ID_W["sig"]]
            #print('size of warning signal',np.size(warning_signal)) 
    else:
        warning_signal = u_sol[:, U_ID_W["sig"]]
        
    return warning_signal
