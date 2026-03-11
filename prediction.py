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


def prediction(pos_ego, spd_ego, predicted_tl_state, filter_ukf, ukf_param, sim_param):
    dx = sim_param.dx
    # tl_status_dict: traffic light status; tl_cell_id_dict: cell id of the traffic light; tl_change_dict: if the traffic light changes to red;
    # is_pass: if the target vehicle passes the traffic light
    tl_status_dict, tl_cell_id_dict, tl_change_dict, tl_pos_dict, is_pass_ego = (
        {},
        {},
        {},
        {},
        {},
    )
    if pos_ego >= 0 and pos_ego<=500:
        tl_in_range = [0]
        tl_pos_in_range = [pos_ego]
        tl_status_in_range = [predicted_tl_state[0, 0]]
        tl_cell_id_in_range = [int(pos_ego / dx + 1)]
    else:
        tl_in_range = []
        tl_pos_in_range = []
        tl_status_in_range = []
        tl_cell_id_in_range = []

    # used to save the traffic light information of previous timestep,
    # if the traffic light is not in the range previously, save information to the dictionary
    for tl_id in range(len(tl_in_range)):
        if tl_in_range[tl_id] not in tl_status_dict.keys():
            tl_status_dict[tl_in_range[tl_id]] = tl_status_in_range[tl_id]
            tl_cell_id_dict[tl_in_range[tl_id]] = tl_cell_id_in_range[tl_id]
            tl_pos_dict[tl_in_range[tl_id]] = tl_pos_in_range[tl_id]
            # if the traffic light is changing at the current timestep
            tl_change_dict[tl_in_range[tl_id]] = False
            # if the vehicle pass the traffic light
            is_pass_ego[tl_in_range[tl_id]] = False

        if tl_in_range[tl_id] in tl_status_dict.keys():
            if (
                tl_status_dict[tl_in_range[tl_id]] != 0  # not red signal
                and tl_status_in_range[tl_id] == 0  # red signal
            ):
                tl_change_dict[
                    tl_in_range[tl_id]
                ] = True  # the signal changes to red
            else:
                tl_change_dict[
                    tl_in_range[tl_id]
                ] = False  # the signal not changes to red
            # save the traffic light information at current timestep to the dictionary
            tl_status_dict[tl_in_range[tl_id]] = tl_status_in_range[tl_id]
            tl_cell_id_dict[tl_in_range[tl_id]] = tl_cell_id_in_range[tl_id]
            tl_pos_dict[tl_in_range[tl_id]] = tl_pos_in_range[tl_id]
            if tl_change_dict[tl_in_range[tl_id]]:
                if (
                    tl_pos_dict[tl_in_range[tl_id]] <= 0
                ):  # if the vehicle passes the traffic light, relative position smaller than 0
                    is_pass_ego[
                        tl_in_range[tl_id]
                    ] = True  # at the estimation stage, this value is always False

    tl_status_dict_pred, tl_index_dict_pred = deepcopy(tl_status_dict), deepcopy(
        tl_cell_id_dict
    )
    tl_change_dict_pred, tl_pos_dict_pred = deepcopy(tl_change_dict), deepcopy(
        tl_pos_dict
    )
    is_pass_ego_pred = deepcopy(is_pass_ego)

    # for estimation part, vehicle is always between cell 0 and cell 1
    id_cell_ego = 0

    # relative position of the target vehicle, for the estimation part, always 0
    relative_pos_ego = 0

    # UKF prior update
    filter_ukf.prior_update(
        tl_pos_in_range,
        tl_status_in_range,
        tl_cell_id_in_range,
        spd_ego,
        relative_pos_ego,
        is_pass=is_pass_ego,
        tl_index_dict=tl_cell_id_dict,
        tl_pos_dict=tl_pos_dict,
    )

    pos_CV_in_range = []
    spd_CV_in_range = []

    # UKF posterior update
    filter_ukf.posteriori_update(
        pos_ego, spd_ego, None, None, pos_CV_in_range, spd_CV_in_range
    )
    matrix_P_current = deepcopy(filter_ukf.mat_P)

    # get state vector in original scale
    vec_x = inverse_transform(deepcopy(filter_ukf.vec_x), ukf_param)
    # save state and parameter
    spd, den, param = state2data(vec_x, ukf_param)
    x = data2state(spd, den, param, ukf_param)
    # used to save all predicted states (traffic speed, density and model parameter)
    x_pred = np.zeros((np.size(x, 0), sim_param.horizon_step + 1))
    x_pred[:, 0:1] = x
    # position of the vehicle at the beginning of prediction
    original_pos_ego = deepcopy(-pos_ego)
    original_spd_ego = deepcopy(spd_ego)
    # prediction information for first time step
    pos_pred_ego = np.zeros((sim_param.horizon_step + 1, 1))
    spd_pred_ego = np.zeros((sim_param.horizon_step + 1, 1))
    pos_pred_min_ego = np.zeros((sim_param.horizon_step + 1, 1))
    pos_pred_max_ego = np.zeros((sim_param.horizon_step + 1, 1))

    pos_pred_ego[0, 0] = 0
    pos_pred_min_ego[0, 0] = pos_pred_ego[0, 0]
    pos_pred_max_ego[0, 0] = pos_pred_ego[0, 0]
    spd_pred_ego[0,0]=original_spd_ego
    # covariance matrix for current step
    matrix_P_pred_ego = np.zeros(
        (sim_param.horizon_step, sim_param.horizon_step))
    # target vehicle's index (in which cell) along the prediction horizon
    cell_id_ego = []

    for index_pred in range(0, sim_param.horizon_step):
        # index of cell for target vehicle
        id_cell_ego = int(relative_pos_ego / sim_param.dx + 1)

        if original_pos_ego <= 0 and -original_pos_ego<=500:
            tl_in_range = [0]
            tl_pos_in_range = [-original_pos_ego]
            tl_status_in_range = [predicted_tl_state[index_pred+1, 0]]
            tl_cell_id_in_range = [int((-original_pos_ego) / sim_param.dx + 1)]
            tl_close_status = predicted_tl_state[index_pred+1, 0]
            id_tl_close_target = 0
        else:
            tl_in_range = []
            tl_pos_in_range = []
            tl_status_in_range = []
            tl_cell_id_in_range = []
            tl_close_status = 0
            id_tl_close_target = 0
        for tl_id in range(len(tl_in_range)):
            if tl_in_range[tl_id] in tl_status_dict_pred.keys():
                if (
                    tl_status_dict_pred[tl_in_range[tl_id]] != 0
                    and tl_status_in_range[tl_id] == 0
                ):
                    tl_change_dict_pred[tl_in_range[tl_id]] = True
                else:
                    tl_change_dict_pred[tl_in_range[tl_id]] = False
                tl_status_dict_pred[tl_in_range[tl_id]
                                    ] = tl_status_in_range[tl_id]

        for tl_id in range(len(tl_in_range)):
            if tl_in_range[tl_id] in is_pass_ego_pred.keys():
                if (
                    relative_pos_ego >= tl_pos_dict_pred[tl_in_range[tl_id]]
                    and tl_change_dict_pred[tl_in_range[tl_id]]
                ):
                    is_pass_ego_pred[tl_in_range[tl_id]] = True
        # update traffic dynamics
        (
            x_pred[:, index_pred + 1: index_pred + 2],
            spd_e_previous,
        ) = traffic_dynamics(
            x_pred[:, index_pred: index_pred + 1],
            ukf_param,
            tl_status_in_range,
            spd_ego,
            id_cell_ego,
            tl_cell_id_in_range,
            tl_close_status,
            id_tl_close_target,
            pred=True,
            debug=False,
            is_pass=is_pass_ego_pred,
            tl_index_dict=tl_index_dict_pred,
            tl_pos_dict=tl_pos_dict_pred,
        )
        # spd and den of all cells at current prediction step
        (spd_cell_t, den_cell_t, _) = state2data(
            x_pred[:, index_pred + 1: index_pred + 2], ukf_param
        )
        relative_pos_ego = spd_ego * sim_param.dt_estimation + relative_pos_ego
        # target vehicle's speed at last time step
        spd_ego_old = spd_ego
        # cell index for target
        id_cell_ego = int(relative_pos_ego / sim_param.dx)
        # out of cell range, target speed is same as last cell's speed
        if relative_pos_ego >= sim_param.cell_range:
            spd_ego = spd_cell_t[-1, 0]
        # cell speed is 0, target speed is also zero
        elif spd_cell_t[id_cell_ego + 1, 0] == 0:
            spd_ego = 0
        # linear interpolation of two cell
        else:
            alpha_spd = relative_pos_ego / sim_param.dx - int(
                relative_pos_ego / sim_param.dx
            )
            spd_ego = (1 - alpha_spd) * spd_cell_t[id_cell_ego, 0] + (
                alpha_spd
            ) * spd_cell_t[id_cell_ego + 1, 0]
        # target vehicle's speed should not change very fast
        spd_ego = max(
            spd_ego_old + sim_param.dt_estimation * sim_param.max_dec,
            min(
                spd_ego,
                spd_ego_old + sim_param.dt_estimation * sim_param.max_acc,
            ),
        )
        # if the vehicle didn't pass the intersection when it changed to red signal
        if spd_cell_t[id_cell_ego, 0] == 0:
            if id_cell_ego in tl_cell_id_in_range:
                if (
                    is_pass_ego_pred[
                        tl_in_range[tl_cell_id_in_range.index(id_cell_ego)]
                    ]
                    is False
                    and spd_cell_t[id_cell_ego, 0] == 0
                    and relative_pos_ego
                    >= tl_pos_dict_pred[
                        tl_in_range[tl_cell_id_in_range.index(id_cell_ego)]
                    ]
                ):
                    spd_ego = 0

        if (id_cell_ego + 1) in tl_cell_id_in_range:
            if (
                is_pass_ego_pred[
                    tl_in_range[tl_cell_id_in_range.index(id_cell_ego + 1)]
                ]
                is False
                and relative_pos_ego
                >= tl_pos_dict_pred[
                    tl_in_range[tl_cell_id_in_range.index(id_cell_ego + 1)]
                ]
                and tl_status_dict_pred[
                    tl_in_range[tl_cell_id_in_range.index(id_cell_ego + 1)]
                ]
                == 0
            ):
                spd_ego = 0

        # save the predicted ego speed to the data
        spd_pred_ego[index_pred + 1, 0] = spd_ego
        # save the predicted ego position to the data
        pos_pred_ego[index_pred + 1, 0] = (
            pos_pred_ego[index_pred, 0] + sim_param.dt_estimation * spd_ego
        )
        cell_id_ego.append(
            int(pos_pred_ego[index_pred + 1, 0] / (ukf_param.dx)) + 1
        )
        pos_pred_min_ego[index_pred + 1, 0] = pos_pred_ego[
            index_pred + 1, 0
        ]
        pos_pred_max_ego[index_pred + 1, 0] = pos_pred_ego[
            index_pred + 1, 0
        ]

    # add uncertainty of estimation to the result
    for i in range(0, sim_param.horizon_step):
        for j in range(0, sim_param.horizon_step):
            matrix_P_pred_ego[i, j] = matrix_P_current[
                cell_id_ego[i],
                cell_id_ego[j],
            ]

    for id_pred in range(0, sim_param.horizon_step):
        pos_pred_min_ego[id_pred + 1, 0] = (
            pos_pred_min_ego[id_pred + 1, 0]
            + sim_param.dt_estimation
            * np.sqrt(np.sum(matrix_P_pred_ego[0: id_pred + 1, 0: id_pred + 1]))
            / ukf_param.k_v
        )
        pos_pred_max_ego[id_pred + 1, 0] = (
            pos_pred_max_ego[id_pred + 1, 0]
            - sim_param.dt_estimation
            * np.sqrt(np.sum(matrix_P_pred_ego[0: id_pred + 1, 0: id_pred + 1]))
            / ukf_param.k_v
        )
    return pos_pred_ego, pos_pred_max_ego, pos_pred_min_ego, spd_pred_ego
