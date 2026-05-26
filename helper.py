import numpy as np
from numpy import linalg as la
from numpy import clip
from copy import deepcopy
from constants import *

def get_direction(y):
    """
    Get the driving direction of a vehilce:
    Input: y coordinate of a vehicle
    Output: int, -1 indicates driving towards east (right);
    1 indicates driving towards west (left)
    """
    direction = None
    if y > 0:
        direction = 1
    else:
        direction = -1
    return direction


def get_lane_index(y, direction):
    """
    Get the lane index of a vehicle.
    Input: y: y coordinate of a vehicle; direction:int, driving direction of the target vehicle
    Output: an int variable
    -1 indicates right most lane;
    0 indicates right lane;
    1 indicates left lane;
    2 indicates left most lane.
    """
    if direction * y > 9.6 and direction * y <= 12.8:
        index = -1
    elif direction * y > 6.4 and direction * y <= 9.6:
        index = 0
    elif direction * y > 3.2 and direction * y <= 6.4:
        index = 1
    elif direction * y > 0 and direction * y <= 3.2:
        index = 2
    return index


def in_communication_range(x, x_target, x_max, direction):
    """
    Determine if a CV is in the communication range,
    Input: x: x coordinate of a vehicle; x_target: x coordinate of the target vehicle;
    x_max: maximum x coordinate of the communication range; direction: target vehicle's driving direction
    Ouput: boolean variable
    """
    if direction == -1:
        if x >= x_target and x <= x_max:
            flag = True
        else:
            flag = False
    elif direction == 1:
        if x <= x_target and x >= x_max:
            flag = True
        else:
            flag = False
    return flag


def get_target_info(target_veh, root, dt):
    """
    Parse target vehicle's information
    Input: target_veh: str, target vehicle's name; root: xml tree; dt: timestep of simulation in SUMO
    Output: 
    time_begin/index_begin: time/index that target vehicle begins to exist in the simulation;
    time_end/index_end: time/index that target vehicle ends to exist in the simulation
    """
    # determine if target vehicle has shown in the simulation
    veh_in = False
    index_end = len(root)
    time_end = index_end * dt
    for index_t in range(len(root)):
        # determine if target vehicle is in the simulation at current time step
        veh_exist = False
        # go through all vehicles at current time step
        for index_v in range(len(root[index_t])):
            if root[index_t][index_v].attrib["id"] == target_veh:
                veh_exist = True
                # if the target vehicle shows in the simulation for the first time
                if veh_in is False:
                    veh_in = True
                    index_begin = index_t
                    time_begin = index_begin * dt
        # if the target vehicle leaves the simulation
        if veh_exist is False and veh_in is True:
            index_end = index_t - 1
            time_end = index_end * dt
            veh_in = False
    return time_begin, time_end, index_begin, index_end


def data2state(spd, den, param, est_param):
    """
    Convert traffic's data to state
    """
    num_param = est_param.num_param
    num_cell = est_param.num_cell
    x = np.zeros((num_cell * 2 + num_param, 1))
    x[0:num_cell, 0:1] = spd[:, 0:1]
    x[num_cell : num_cell * 2, 0:1] = den[:, 0:1]
    x[num_cell * 2 : num_cell * 2 + num_param, 0:1] = param[:, 0:1]
    return x


def state2data(x, est_param):
    """
    Convert state to traffic's data
    """
    num_param = est_param.num_param
    num_cell = est_param.num_cell
    # store all states
    # c; c0; tau
    spd, den, param = (
        np.zeros((num_cell, 1)),
        np.zeros((num_cell, 1)),
        np.zeros((num_param, 1)),
    )
    spd = x[0:num_cell, 0:1]
    den = x[num_cell : num_cell * 2, 0:1]
    param = x[num_cell * 2 : num_cell * 2 + num_param, 0:1]
    return spd, den, param


def diag_mat(mat_list):
    """
    Get a diagonal matrix in the form of [A 0 0
                                          0 B 0
                                          0 0 C]
    Input is a list of matrix [A,B,C...]
    Output is a diagonal matrix
    """
    len_list = len(mat_list)
    mat_A = mat_list[0]
    for index in range(1, len_list):
        size_left = len(mat_A)
        size_right = len(mat_list[index])
        left = np.vstack((mat_A, np.zeros((size_right, size_left))))
        right = np.vstack((np.zeros((size_left, size_right)), mat_list[index]))
        mat_A = np.hstack((left, right))
    return mat_A


def clip_param(list_param_orig, est_param, tf):
    """
    Make all parameters between upper and lower bounds
    Input: list_param: a list that contains all parameters for the traffic model;
    est_param: an object that contains all parameters for the estimator
    tf: a boolean varaiable that indicates if transformed values are used
    """
    list_param = deepcopy(list_param_orig)

    if tf:
        list_param[:, 0] = np.clip(
            list_param[:, 0],
            est_param.list_param_lb_tf[:, 0],
            est_param.list_param_ub_tf[:, 0],
        )
    else:
        list_param[:, 0] = np.clip(
            list_param[:, 0],
            est_param.list_param_lb[:, 0],
            est_param.list_param_ub[:, 0],
        )
    return list_param


def transform(x_orig, est_param):
    """
    Do linear transform for state x, get normalized values
    """
    x = deepcopy(x_orig)

    spd, den, param = state2data(x, est_param)
    spd_tf = deepcopy(spd)
    den_tf = deepcopy(den)
    param_tf = deepcopy(param)
    spd_tf = (spd - est_param.b_v) * est_param.k_v
    den_tf = (den - est_param.b_rho) * est_param.k_rho
    param_tf[:, 0] = (
        param[:, 0] - est_param.list_param_b[:, 0]
    ) * est_param.list_param_k[:, 0]
    x_transform = data2state(spd_tf, den_tf, param_tf, est_param)
    return x_transform


def inverse_transform(x_origin, est_param):
    """
    Do inverse linear transform for state x, get true values
    """
    x = deepcopy(x_origin)
    spd, den, param = state2data(x, est_param)
    spd_inverse_tf = deepcopy(spd)
    den_inverse_tf = deepcopy(den)
    param_inverse_tf = deepcopy(param)
    
    spd_inverse_tf = spd / est_param.k_v + est_param.b_v
    
    den_inverse_tf = den / est_param.k_rho + est_param.b_rho
    
    param_inverse_tf[:, 0] = param[:, 0] / est_param.list_param_k[:, 0] + est_param.list_param_b[:, 0]
    x_inv_transform = data2state(spd_inverse_tf, den_inverse_tf, param_inverse_tf, est_param)

    return x_inv_transform


def get_extend_state(spd, den, num_cell, tl_close_status, spd_e, extend_state=3,spd_target=None):
    """
    For traffic model, six additional cells (three before the first cell and three behind the last cell) are added.
    Their densities and speeds are same as the adjacent cell.
    """
    den_extend = np.zeros((num_cell + 2 * extend_state, 1))
    
    spd_extend = np.zeros((num_cell + 2 * extend_state, 1))
    
    den_extend[extend_state : num_cell + extend_state, 0:1] = den

    spd_extend[extend_state : num_cell + extend_state, 0:1] = spd

    for id in range(0, extend_state):
        den_extend[id, 0], den_extend[-1 - id, 0] = 0.01, den[-1, 0] 
        # spd_extend[id, 0], spd_extend[-1 - id, 0] = spd_e[0,0], spd_e[-1,0]
        spd_extend[id, 0], spd_extend[-1 - id, 0] = spd[0,0], spd_e[-1,0]
    return spd_extend, den_extend


def isPD(B):
    """Returns true when input is positive-definite, via Cholesky"""
    try:
        _ = la.cholesky(B)
        return True
    except la.LinAlgError:
        return False


def nearestPD(A):
    """Find the nearest positive-definite matrix to input
    """

    B = (A + A.T) / 2
    _, s, V = la.svd(B)

    H = np.dot(V.T, np.dot(np.diag(s), V))

    A2 = (B + H) / 2

    A3 = (A2 + A2.T) / 2

    if isPD(A3):
        return A3

    spacing = np.spacing(la.norm(A))
    # The above is different from [1]. It appears that MATLAB's `chol` Cholesky
    # decomposition will accept matrixes with exactly 0-eigenvalue, whereas
    # Numpy's will not. So where [1] uses `eps(mineig)` (where `eps` is Matlab
    # for `np.spacing`), we use the above definition. CAVEAT: our `spacing`
    # will be much larger than [1]'s `eps(mineig)`, since `mineig` is usually on
    # the order of 1e-16, and `eps(1e-16)` is on the order of 1e-34, whereas
    # `spacing` will, for Gaussian random matrixes of small dimension, be on
    # othe order of 1e-16. In practice, both ways converge, as the unit test
    # below suggests.
    I = np.eye(A.shape[0])
    k = 1
    while not isPD(A3):
        mineig = np.min(np.real(la.eigvals(A3)))
        A3 += I * (-mineig * k**2 + spacing)
        k += 1

    return A3


def get_CV_info(
    root, index, target_veh, direction_target, communication_range, CV_list
):
    """
    Get all CVs' information that are in the communication range
    Input: root: xml tree of the simulation; index: index in the simulation;
    target_veh: str, target vehicle's name; direction_target: int, target vehicle's driving direction;
    communication range: float, communication range; CV_list: a list of all CVs' names in the simulation
    """
    # CV in the communication range at current time step, first one is the target vehicle
    CV_in_range = []
    # for every CV, information contains x coordinate, y coordinate, lane index and speed
    CV_info = []
    # x coordinate, y coordinate, lane index, speed
    target_info = np.zeros((4, 1))
    # get CVs info at current time step
    for element in root[index]:
        # get target vehilce info at current time step
        if element.attrib["id"] == target_veh:
            CV_in_range.append(element.attrib["id"])
            target_info[0, 0], target_info[1, 0] = float(element.attrib["x"]), float(
                element.attrib["y"]
            )
            target_info[2, 0] = get_lane_index(target_info[1, 0], direction_target)
            target_info[3, 0] = float(element.attrib["speed"])
            x_max = target_info[0, 0] - direction_target * communication_range
            CV_info.append(target_info)
    for element in root[index]:
        if element.attrib["id"] in CV_list:
            if in_communication_range(
                float(element.attrib["x"]), target_info[0, 0], x_max, direction_target
            ):  
                veh_info = np.zeros((4, 1))
                veh_info[0, 0], veh_info[1, 0] = float(element.attrib["x"]), float(
                    element.attrib["y"]
                )
                veh_info[2, 0] = get_lane_index(veh_info[1, 0], direction_target)
                veh_info[3, 0] = float(element.attrib["speed"])
                if veh_info[2, 0] == target_info[2, 0]:
                    CV_in_range.append(element.attrib["id"])
                    CV_info.append(veh_info)
    return CV_info, CV_in_range, target_info


def get_sim_info(sim_begin, sim_end, time_begin, time_end, dt):
    """
    Get the information for the simulation.
    Input: sim_begin: self-defined simulation start time;
    sim_end: self-defined simulation end time;
    time_begin: time that ego vehicle joins the simulation;
    time_end: time that ego vehicle leaves the simulation;
    dt: timestep;
    Output:
    index_begin: index that simulaltion begins;
    index_end: index that simulation ends
    """
    if sim_begin < time_begin:
        sim_begin = time_begin
        index_begin = round(sim_begin / dt)
    else:
        index_begin = round(sim_begin / dt)

    if sim_end > time_end:
        sim_end = time_end
        index_end = round(sim_end / dt)
    else:
        index_end = round(sim_end / dt)
    return index_begin, index_end


def clip_state(x_original, est_param):
    """
    Make all states and parameters between upper and lower bound (before linear transform)
    """
    x = deepcopy(x_original)
    num_cell = est_param.num_cell
    num_param = est_param.num_param
    # param
    x[num_cell * 2  : num_cell * 2 + num_param, 0:1] = clip_param(
        x_original[num_cell * 2 : num_cell * 2 + num_param, 0:1], est_param, False
    )
    
    # spd
    x[0:num_cell, 0:1] = clip(
        x_original[0:num_cell, 0:1],
        np.ones((num_cell, 1)) * est_param.v_lb,
        np.ones((num_cell, 1)) * est_param.v_ub,
    )

    # rho
    x[num_cell : num_cell * 2, 0:1] = clip(
        x_original[num_cell : num_cell * 2, 0:1],
        np.ones((num_cell, 1)) * est_param.rho_lb,
        np.ones((num_cell, 1)) * est_param.rho_ub,
    )
    
    return x

def target_info_from_dict(car_info_dict, target_veh, index):
    lane_target, spd_target, pos_target = (
        car_info_dict[target_veh][index, CAR_ID["id_lane"]],
        car_info_dict[target_veh][index, CAR_ID["v"]],
        car_info_dict[target_veh][index, CAR_ID["x"]],
    )
    return lane_target, spd_target, pos_target

def get_veh_signal_state(signals):
    num_signals = int(signals)
    part_bin_num_signals = bin(num_signals)[2:]
    bin_num_signals = np.zeros((14))
    for id in range(len(part_bin_num_signals)):
        bin_num_signals[14-len(part_bin_num_signals)+id] = int(part_bin_num_signals[id])
    left_sig, right_sig, brake_sig = 0, 0, 0 
    if bin_num_signals[13] == 1:
        right_sig = 1
    if bin_num_signals[12] == 1:
        left_sig = 1
    if bin_num_signals[10] == 1:
        brake_sig = 1
    return left_sig, right_sig, brake_sig

def get_ground_truth_spd(index_begin, index_end, target_veh, car_info_dict, sim_param):
    # ground truth speed
    spd_ground_truth = np.zeros(
        (
            1,
            int(
                (index_end - index_begin) * sim_param.dt_sumo / sim_param.dt_estimation
                + 1
            ),
        )
    )
    count = 0
    for index in range(index_begin, index_end + 1):
        count += 1
        _, spd_target, _ = target_info_from_dict(
            car_info_dict, target_veh, index - index_begin
        )
        spd_ground_truth[0, count - 1] = spd_target
    return spd_ground_truth