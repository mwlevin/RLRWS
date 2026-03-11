X_DIM, U_DIM = 2, 3

X_ID = {"x": 0, "v": 1}

U_ID = {"acc": 0, "F_b": 1, "I": 2}


X_DIM_W, U_DIM_W = 2, 1
X_ID_W = {"x": 0, "v": 1}
U_ID_W = {"sig": 0}


# currently, v_max, v_min, range_tl, lc_prob attributes are not used
CAR_ID = {
    "x": 0,  # position x
    "y": 1,  # position y
    "v": 2,  # longitudinal speed
    "v2v": 3,  # if in the v2v communication range
    "perception": 4,  # if in the perception range
    "d_target": 5,  # longitudinal distance to the target vehicle
    "signal": 6,  # vehicle signal information
    "id_lane": 7,  # lane id, 0 indicates same lane as the target vehicle
    # -1 indicates on the left, 1 indicates on the right
    # information below not used currently
    "angle": 8,
    "lc_prob": 9,
    "v_max": 10,
    "v_min": 11,
    "range_tl": 12,  # if not in the range of any traffic light
}

SIM_ID = {
    "target": 0,
    "CV_list": 1,
    "id_begin": 2,
    "id_end": 3,
    "param": 4,
}

PW_ID = {
    "c": 0,
    "c0": 1,
    "tau": 2,
}
