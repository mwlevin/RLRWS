import numpy as np
from helper import *
import sys
import os


def traffic_dynamics(
    x,
    sys_param,
    tl_status_in_range,
    spd_target,
    index_target,
    tl_index_in_range,
    tl_close_status,
    id_tl_close_target,
    est_param=True,
    extend_state=3,
    pred=False,
    debug=False,
    is_pass=None,
    tl_index_dict=None,
    tl_pos_dict=None,
):
    """
    Traffic dynamics update for the flow model
    Input: x: current state; sys_param: parameter for the simulation
    est_param: a boolean variable, if True, estimated flow model parameters will be used,
    otherwise, fixed parameters will be used;
    Output: next timestep state
    """
    # a small variable that prevents denominator from 0
    epsilon = 1e-06
    dt = sys_param.dt_simulation
    dx = sys_param.dx
    num_cell = sys_param.num_cell
    spd, den, param = state2data(x, sys_param)
    spd_next, den_next = (np.zeros((num_cell, 1)), np.zeros((num_cell, 1)))
    if est_param:
        [c, c0, tau] = param[0:4, 0]
        rho_max = sys_param.rho_max
        v0 = sys_param.v0
    else:
        [rho_max, c, c0, tau, v0] = [
            sys_param.rho_max,
            sys_param.c,
            sys_param.c0,
            sys_param.tau,
            sys_param.v0,
        ]
    spd_e = v0 * np.ones((num_cell, 1)) 
    lane_v0 = v0 * np.ones((num_cell, 1))
    spd_e[:, 0:1] = (
        np.minimum(
            lane_v0,
            c
            * np.maximum(
                (rho_max / (den[:, 0:1] + epsilon) - 1), np.zeros((num_cell, 1))
            ),
        )
        * 1
    )
    cell_effect = 6          # 8
    
    alpha = np.ones((num_cell, 1))
    
    tl_index_list_tmp = []
    tl_pos_list_tmp = []
    is_pass_list_tmp = []
    for key_tmp in tl_index_dict.keys():
        tl_index_list_tmp.append(tl_index_dict[key_tmp])
        tl_pos_list_tmp.append(tl_pos_dict[key_tmp])
        is_pass_list_tmp.append(is_pass[key_tmp])
    for id_tl in range(len(tl_index_in_range)):
        if is_pass_list_tmp[tl_index_list_tmp.index(tl_index_in_range[id_tl])]:
            pass
        else:
            # red light
            if tl_status_in_range[id_tl] == 0:
                for id_cell in range(cell_effect+1):                    
                    if tl_index_in_range[id_tl] - id_cell<0:
                        pass
                    else:
                        if id_cell == 0:
                            alpha[tl_index_in_range[id_tl] - id_cell] = min(1, alpha[tl_index_in_range[id_tl] - id_cell], id_cell/cell_effect)
                        else:
                            alpha[tl_index_in_range[id_tl] - id_cell] = min(1, alpha[tl_index_in_range[id_tl] - id_cell], (id_cell-1)/cell_effect)
            # yellow light
            if tl_status_in_range[id_tl] == -1:
                for id_cell in range(2,cell_effect+1):
                    if tl_index_in_range[id_tl] - id_cell<0:
                        pass
                    else:
                        if id_cell == 0:
                            alpha[tl_index_in_range[id_tl] - id_cell] = min(1, alpha[tl_index_in_range[id_tl] - id_cell], 1.1*id_cell/cell_effect)
                        else:
                            alpha[tl_index_in_range[id_tl] - id_cell] = min(1, alpha[tl_index_in_range[id_tl] - id_cell], 1.1*(id_cell-1)/cell_effect)
                # for id_cell in range(cell_effect):
                #     if tl_index_in_range[id_tl] - id_cell<0:
                #         pass
                #     else:
                #         alpha[tl_index_in_range[id_tl] - id_cell] = min(1, alpha[tl_index_in_range[id_tl] - id_cell], id_cell/cell_effect)
    
    spd_e[:, 0:1] = deepcopy(spd_e[:, 0:1])*alpha
    
    # cell_effect = 10

    spd_extend, den_extend = get_extend_state(
        spd, den, num_cell, tl_close_status, spd_e, extend_state=extend_state, spd_target=spd_target
    )
    
    

    den_next[:, 0] = den_extend[extend_state : num_cell + extend_state, 0] - (
        dt / dx
    ) * (
        den_extend[extend_state : num_cell + extend_state, 0]
        * spd_extend[extend_state : num_cell + extend_state, 0]
        - den_extend[extend_state - 1 : num_cell + extend_state - 1, 0]
        * spd_extend[extend_state - 1 : num_cell + extend_state - 1, 0]
    )

    spd_next[:, 0] = (
        spd_extend[extend_state : num_cell + extend_state, 0]
        - (dt / dx)
        * spd_extend[extend_state : num_cell + extend_state, 0]
        * (
            spd_extend[extend_state : num_cell + extend_state, 0]
            - spd_extend[extend_state - 1 : num_cell + extend_state - 1, 0]
        )
        + dt
        * (spd_e[:, 0] - spd_extend[extend_state : num_cell + extend_state, 0])
        / tau
        - (dt / dx)
        * c0
        * c0
        * (
            den_extend[extend_state + 1 : num_cell + extend_state + 1, 0]
            - den_extend[extend_state : num_cell + extend_state, 0]
        )
        / ((den_extend[extend_state : num_cell + extend_state, 0] + epsilon))
    )

    den_tmp = deepcopy(den_next[:, 0:1])
    den_tmp = np.clip(den_tmp, den - 0.1*0.1, den + 0.1*0.1)
    den_next_after_clip = deepcopy(den_tmp[:, 0:1])
    den_next_after_clip[:, 0:1] = np.clip(
        deepcopy(den_next[:, 0:1]),
        sys_param.rho_lb * np.ones((num_cell, 1)),
        sys_param.rho_ub * np.ones((num_cell, 1)),
    )
    max_acc = 2
    max_dec = -5
    spd_after_smooth = deepcopy(spd_next[:, 0:1])
    for index in range(1, num_cell):
        if (
            spd_after_smooth[index, 0]
            > (spd_after_smooth[index - 1, 0] ** 2 + 2 * max_acc * dx) ** 0.5
        ):
            spd_after_smooth[index, 0] = (
                spd_after_smooth[index - 1, 0] ** 2 + 2 * max_acc * dx
            ) ** 0.5
        tmp = max(spd_after_smooth[index - 1, 0] ** 2 + 2 * max_dec * dx, 0)
        if spd_after_smooth[index, 0] < (tmp) ** 0.5:
            spd_after_smooth[index, 0] = (tmp) ** 0.5

    spd_after_clip = deepcopy(spd_after_smooth[:, 0:1])
    spd_after_clip[:, 0:1] = np.clip(
        (deepcopy(spd_after_clip[:, 0:1])),
        sys_param.v_lb * np.ones((num_cell, 1)),
        sys_param.v_ub * np.ones((num_cell, 1)),
    )
    for index in range(num_cell):
        if index in tl_index_in_range:
            if is_pass_list_tmp[tl_index_list_tmp.index(index)]:
                pass
            else:
                if tl_status_in_range[tl_index_in_range.index(index)] == 0:
                    spd_after_clip[index, 0] = 0
                    if index - 1 >= 0:
                        spd_after_clip[index-1, 0] = 0
    x_next = data2state(spd_after_clip, den_next_after_clip, param, sys_param)
    return x_next, spd_e