import sys
import pickle as pkl
import traceback

import datetime
import numpy as np
from matplotlib import pyplot as plt
from time_convert import *
from coord2dist import *
import socket
import os

import datetime
import socket
from cav_data import *
from prediction import *
from UKF import *
from estimation_param import *
from est_pred_param import *
from warning import *
import keyboard
import pandas as pd
from calibrate_gradient_adaptive_step import *
from ILC_constants import *

# for ILC
import time

# Add these before the main while loop
last_calibration_time = time.time()
CALIBRATION_INTERVAL = 10  # seconds

TEST_NAME = "test34"

# we have 3 csv files: 1) record_data just for all recordings,  2) calibrate_data which is pre_filled, used for ILC calibration, 3) ilc_params_log which logs the parameters
safe_test_name = "".join(
    char if char.isalnum() or char in ("-", "_") else "_"
    for char in TEST_NAME.strip()
) or "unnamed_test"
Run_data_dir = os.path.join(
    "run_data",
    f"{safe_test_name}_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}",
)
os.makedirs(Run_data_dir, exist_ok=True)
Record_data = os.path.join(Run_data_dir, "record_data.csv")
Calibrate_data = os.path.join(Run_data_dir, "calibrate_data.csv")
ILC_params_file = os.path.join(Run_data_dir, "ilc_params_log.csv")

# Seed the calibrate CSV from a pre-filled reference so mainIDM has data on first run
CALIBRATE_SEED = "calibrate_data_reference.csv"   # path to your pre-filled file
if os.path.exists(CALIBRATE_SEED) and not os.path.exists(Calibrate_data):
    import shutil
    shutil.copy(CALIBRATE_SEED, Calibrate_data)
    print(f"[ILC] Seeded {Calibrate_data} from {CALIBRATE_SEED}")

Yellow_limit = 30  # at warning = 30, we switch from yellow to red

HOST_REC = ""
PORT_REC = 10889
s_rec = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_rec.bind((HOST_REC, PORT_REC))

HOST_REC1 = ""
PORT_REC1 = 10900
s_rec1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_rec1.bind((HOST_REC1, PORT_REC1))



HOST_SEND = 'localhost'
PORT_SEND = 10890
s_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

spd_ego_prev = 30


current_time = datetime.datetime.now()
last_time = current_time
s_rec.setblocking(False)
sim_param = SimParameter()
warning_list = []
cav_msg_list=[]
prediction_list = []
pos_ego = 2000
ILC_params = []

# set ILC parameters:
a_cal = 1 * 1.2
d_cal = 0.1
c_cal = 0.9
T_cal = 0.863
J_final1 = 10000
v0 =  24.5
delta = IDM_Param().delta

warning_showed = 0

# save ILC params for start
ILC_params.append({
    "timestamp": current_time,
    "a_cal": a_cal,
    "d_cal": d_cal,
    "c_cal": c_cal,
    "T_cal": T_cal
})


def extract_first_tl(tl_str):
    return float(str(tl_str).strip().replace('[','').replace(']','').split()[0])


def append_csv_row(csv_path, row):
    write_header = not os.path.exists(csv_path) or os.path.getsize(csv_path) == 0
    pd.DataFrame([row]).to_csv(csv_path, mode='a', header=write_header, index=False)


def summarize_tl_prediction(predicted_tl_state, dt=0.2, n_preview=20):
    tl_values = np.asarray(predicted_tl_state).reshape(-1)
    if len(tl_values) == 0:
        return None, None, None, ""

    tl_now = float(tl_values[0])
    tl_change_step = None
    for idx, value in enumerate(tl_values[1:], start=1):
        if value != tl_values[0]:
            tl_change_step = idx
            break

    tl_change_time_s = (
        None if tl_change_step is None else round(tl_change_step * dt, 2)
    )
    tl_first_20 = " ".join(
        str(int(value)) if float(value).is_integer() else str(float(value))
        for value in tl_values[:n_preview]
    )

    return tl_now, tl_change_step, tl_change_time_s, tl_first_20


def make_record_row(current_time, pos_ego, spd_ego, warning_signal,
                    predicted_tl_state, warning_showed,
                    current_ref_file=None, current_intersection=None,
                    current_approach=None):
    tl_now, tl_change_step, tl_change_time_s, tl_first_20 = summarize_tl_prediction(
        predicted_tl_state
    )
    return {
        "timestamp": current_time,
        "pos_ego": pos_ego,
        "spd_ego": spd_ego,
        "current_ref_file": current_ref_file,
        "current_intersection": current_intersection,
        "current_approach": current_approach,
        "tl_now": tl_now,
        "tl_change_step": tl_change_step,
        "tl_change_time_s": tl_change_time_s,
        "tl_first_20": tl_first_20,
        "MPC_warning": warning_signal[0],
        "warning_1": warning_signal[1],
        "warning_2": warning_signal[2],
        "warning_3": warning_signal[3],
        "warning_4": warning_signal[4],
        "warning_5": warning_signal[5],
        "predicted_tl_state": predicted_tl_state,
        "ILC_warning": warning_showed,
    }


def make_ilc_param_row(current_time, a_cal, d_cal, c_cal, T_cal, J_final):
    return {
        "timestamp": current_time,
        "a_cal": a_cal,
        "d_cal": d_cal,
        "c_cal": c_cal,
        "T_cal": T_cal,
        "J_final": J_final,
    }


# check if we are very close to intersection, low speed, and red light
def passer(pos_ego, spd_ego, predicted_tl_state):
    original_pos_ego = -pos_ego
    original_spd_ego = spd_ego

    if -original_pos_ego >= 0 and -original_pos_ego <= 500:
        tl_status_in_range = [predicted_tl_state[0, 0]]
    else:
        tl_status_in_range = []

    if -original_pos_ego >= 0 and -original_pos_ego <= 500:
        if -original_pos_ego <= 60 and original_spd_ego <= 4 and tl_status_in_range[0] == 0:
            check_if = True
        else:
            check_if = False
    else:
        check_if = False

    return check_if




def load_ilc_data(csv_path):
    """Read ILC_data.csv and return lists needed for calibration."""
    import re
    df = pd.read_csv(csv_path, quoting=1, engine='python')

    pos_ego_list  = df['pos_ego'].tolist()
    spd_ego_list  = df['spd_ego'].tolist()
    warning_list  = df['warning_2'].tolist()  # primary warning channel (same as before)

    # all 6 warning channels, in case downstream needs them
    warning_cols = ['MPC_warning','warning_1','warning_2',
                'warning_3','warning_4','warning_5']
    warning_matrix = df[warning_cols].values.tolist()   # list of 6-element lists

    computed_warning_list = df['ILC_warning'].tolist()

    def extract_first_tl(tl_str):
        numbers = re.findall(r'[\d.]+', str(tl_str))
        return float(numbers[0]) if numbers else 1.0
    tls_list = df['predicted_tl_state'].apply(extract_first_tl).tolist()

    # acc_ego: delta_spd / 1s, first = 0
    acc_list = [0.0]
    for i in range(1, len(spd_ego_list)):
        acc_list.append(spd_ego_list[i] - spd_ego_list[i-1])

    spd_ego_list, acc_list = smooth_signals(spd_ego_list, acc_list, window=5)

    # spacing = pos_ego (distance to stop bar)
    spacing_list = pos_ego_list

    return warning_list, acc_list, spd_ego_list, spacing_list, tls_list

while True:
    #if pos_ego<-5:  # edited to be able to go to a loop
    #    break
    start = time.time()
    current_time = datetime.datetime.now()
    if ((current_time - last_time).total_seconds()) >= 1:
        last_time = current_time
        newestData_predicted = None
        # newestData_msg = None        # add this line
        keepReceiving = True
        # print(" hheeeeeeyyyyy")
        while keepReceiving:
            try:
                data_predicted, fromAddr = s_rec.recvfrom(500000)
                data_msg, fromAddr = s_rec1.recvfrom(500000)
                if data_predicted:
                    newestData_predicted = data_predicted
                if data_msg:
                    # print("msg received ", data_msg )
                    newestData_msg = data_msg
                    
                
            except socket.error as why:
                keepReceiving = False
                break
        if newestData_msg:
            msg_data=pkl.loads(newestData_msg)
            ff_spd = msg_data.ff_spd
            # update ff_speed
            # ---- NEW: push fresh free-flow speed into UKF params ----
            # ukf_param.update_ff_speed(data_orig.ff_spd)
            
            cav_msg_list.append(msg_data)
        if newestData_predicted:
            data_orig = pkl.loads(newestData_predicted)
            pos_pred_ego = data_orig.pos_pred_ego
            pos_pred_max_ego = data_orig.pos_pred_max_ego
            pos_pred_min_ego = data_orig.pos_pred_min_ego
            spd_pred_ego = data_orig.spd_pred_ego
            predicted_tl_state = data_orig.predicted_tl_state
            pos_ego = data_orig.pos_ego
            spd_ego = data_orig.spd_ego
            current_ref_file = getattr(data_orig, "current_ref_file", None)
            current_intersection = getattr(data_orig, "current_intersection", None)
            current_approach = getattr(data_orig, "current_approach", None)
            if pos_ego<500:
                warning_signal = warning(pos_pred_ego, pos_pred_max_ego, pos_pred_min_ego, pos_ego, spd_ego, spd_pred_ego, predicted_tl_state, sim_param)
            else:
                warning_signal=np.zeros((50))
            warning_signal_obj = pkl.dumps(warning_signal)
            # print('wraning send', warning_signal[0:5])
            warning_list.append(warning_signal)
            prediction_list.append(data_orig)
            
            
            

            
            
            
            acc_ego = (spd_ego - spd_ego_prev) / 1  # calculate acceleration based on current and previous speed, assuming dt=1s for simplicity
            
            # s_send.sendto(warning_signal_obj, (HOST_SEND, PORT_SEND))
            
            
            print('ff_spd',ff_spd)
            
            # find the warning value to be presented to the driver, look 2 time steps ahead (0.4 seconds) to consider reaction time
            warning_showed = rev_warning_calc(v0, delta, warning_signal[2], acc_ego, spd_ego, pos_ego,predicted_tl_state[0][0] , a_cal , d_cal, c_cal, T_cal)
            
            # check where very small warning values
            if warning_signal[0] < 1 and warning_signal[1] < 1 and warning_signal[2] < 1:
                warning_showed = 0
                
            if warning_showed < 0.01:
                warning_showed = 0
            
            if warning_showed > 100:
                warning_showed = 100
            if pos_ego >2.5*30+30**2/(2*3.414):
                warning_showed=0
                
            # if wrong detected to be very close to intersection, we set warning to 0 to avoid false positive
            if pos_ego < 2 and spd_ego > 18:
                warning_showed = 0
                warning_signal[0] = 0
                
                
                
            # enforce passer filter: red message
            if passer(pos_ego, spd_ego, predicted_tl_state):
                warning_signal[0] = Yellow_limit + 5  
                warning_showed = Yellow_limit + 5
                # warning_showed = warning_signal[0]
                
                  
            # send warning value for plotting
            print("MPC warning ", warning_signal[0])
            print('warning_showned',warning_showed)
            # Send warning_showed to port 10890
            #warning_showed_obj = pkl.dumps(warning_showed)
            
            
            # Scalar - just send MPC warning for now, later we should send ILC warning instead, 
            warning_showed_obj = pkl.dumps(warning_showed)
            #warning_showed_obj = pkl.dumps(warning_showed)
            s_send.sendto(warning_showed_obj, (HOST_SEND, PORT_SEND))

            
            # print("warning shown ", warning_showed)
            row = make_record_row(
                current_time,
                pos_ego,
                spd_ego,
                warning_signal,
                predicted_tl_state,
                warning_showed,
                current_ref_file,
                current_intersection,
                current_approach,
            )
            
            # set previous speed for next iteration
            spd_ego_prev = spd_ego
            
            
            # Write to CSV (append mode so we don't lose history)
            append_csv_row(Record_data, row)
            # print(f"[ILC] Row saved to {Calibrate_data}")
            
            # write to calibrate_data only if we have a warning
            print
            if warning_signal[0] > 0.01 and warning_signal[1] > 0.01:
                append_csv_row(Calibrate_data, row)
            
            
            # --- Every 10 seconds: save data + recalibrate ---
            now = time.time()
            
            
            if now - last_calibration_time >= CALIBRATION_INTERVAL:
                last_calibration_time = now
                # Recalibrate
                try:
                    print(os.path.exists(Calibrate_data))
                    if os.path.exists(Calibrate_data):
                        start = time.time()  # ← start timing HERE, right before calibration
                        warning_ilc, acc_ilc, spd_ilc, spacing_ilc, tls_ilc = load_ilc_data(Calibrate_data)
                        # print(" loaded data " , spd_ilc)
                        if len(warning_ilc) > 5:
                            print(f"[ILC] Running calibration on {len(warning_ilc)} samples...")
                            
                            a_cal1, d_cal1, c_cal1, T_cal1, J_final1 = mainIDM(
                                v0, delta,
                                warning_ilc, acc_ilc, spd_ilc, spacing_ilc, tls_ilc
                            )

                            finish = time.time() - start  # ← now this correctly measures mainIDM duration
                            # print(f"time took {finish:.3f}s, current time: {time.time()}")
                            
                            
                            # save ILC params once updated
                            ILC_params.append({
                                "timestamp": current_time,
                                "a_cal": a_cal1,
                                "d_cal": d_cal1,
                                "c_cal": c_cal1,
                                "T_cal": T_cal1
                            })

                            a_cal = (a_cal1*0.5+a_cal*0.5)
                            d_cal = (d_cal*0.5+d_cal1*0.5)
                            c_cal = (c_cal *0.5+ c_cal1* 0.5)
                            T_cal = (T_cal *0.5+ T_cal1 *0.5)
                            print(f"[ILC] Done. a={a_cal:.4f} d={d_cal:.4f} c={c_cal:.4f} T={T_cal:.4f}")
                        else:
                            print("[ILC] Not enough data yet.")
                except Exception as e:
                    print("[ILC] Calibration error:")
                    traceback.print_exc()
                
            
            # --- Save updated ILC params to CSV ---
            append_csv_row(
                ILC_params_file,
                make_ilc_param_row(current_time, a_cal, d_cal, c_cal, T_cal, J_final1),
            )
    







# df_param = pd.DataFrame(ILC_params)
# csv_path = "ILC_log.csv"
# df_param.to_csv(csv_path, index=False)

# print(f"CSV successfully saved to {csv_path}")

            
# with open("record_data18/warning_list.obj", "wb") as handle:
#     pkl.dump(warning_list, handle, protocol=pkl.HIGHEST_PROTOCOL)
# with open("record_data18/cav_msg_list.obj", "wb") as handle:
#     pkl.dump(cav_msg_list, handle, protocol=pkl.HIGHEST_PROTOCOL)
# with open("record_data18/prediction_list.obj", "wb") as handle:
#     pkl.dump(prediction_list, handle, protocol=pkl.HIGHEST_PROTOCOL)
# sys.exit()


# to extract the first value of the predicted traffic light state for plotting
