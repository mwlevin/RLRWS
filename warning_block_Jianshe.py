import sys
import pickle as pkl

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
Calibrate_data = "calibrate_data.csv"


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
rows = []   # will hold data for dataframe


# set ILC parameters:
a_cal = IDM_Param().a[0]
d_cal = IDM_Param().d[0]
c_cal = IDM_Param().c[0]
T_cal = IDM_Param().T[0]
v0 =  21
delta = IDM_Param().delta

def extract_first_tl(tl_str):
    return float(str(tl_str).strip().replace('[','').replace(']','').split()[0])




def load_ilc_data(csv_path):
    """Read ILC_data.csv and return lists needed for calibration."""
    import re
    df = pd.read_csv(csv_path, quoting=1, engine='python')

    pos_ego_list   = df['pos_ego'].tolist()
    spd_ego_list   = df['spd_ego'].tolist()
    warning_list   = df['warning_0'].tolist()

    def extract_first_tl(tl_str):
        numbers = re.findall(r'[\d.]+', str(tl_str))
        return float(numbers[0]) if numbers else 1.0

    tls_list = df['predicted_tl_state'].apply(extract_first_tl).tolist()

    # acc_ego: delta_spd / 1s, first = 0
    acc_list = [0.0]
    for i in range(1, len(spd_ego_list)):
        acc_list.append(spd_ego_list[i] - spd_ego_list[i-1])

    # spacing = pos_ego (distance to stop bar)
    spacing_list = pos_ego_list

    return warning_list, acc_list, spd_ego_list, spacing_list, tls_list

while True:
    if pos_ego<4:
        break
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
            
            warning_signal = warning(pos_pred_ego, pos_pred_max_ego, pos_pred_min_ego, pos_ego, spd_ego, spd_pred_ego, predicted_tl_state, sim_param)
            warning_signal_obj = pkl.dumps(warning_signal)
            print('wraning send', warning_signal[0:5])
            warning_list.append(warning_signal)
            prediction_list.append(data_orig)
            
            
            
            acc_ego = (spd_ego - spd_ego_prev) / 1  # calculate acceleration based on current and previous speed, assuming dt=1s for simplicity
            
            s_send.sendto(warning_signal_obj, (HOST_SEND, PORT_SEND))
            
            
            # already calibrated, so use the parameters to find the warning value
            # read parameters from the calibrated model

            # print(" pred state ", predicted_tl_state[0][0])
            # tls_state = print()
            # or predicted_tl_state[0] if it's a list, adjust as needed
            # find the warning value to be presented to the driver
            warning_showed = rev_warning_calc(v0, delta, warning_signal[0], acc_ego, spd_ego, pos_ego,predicted_tl_state[0][0] , a_cal , d_cal, c_cal, T_cal)
            
            print("warning shown ", warning_showed)
            # save data for dataframe ILC
            rows.append({
            "timestamp": current_time,
            "pos_ego": pos_ego,
            "spd_ego": spd_ego,
            "warning_0": warning_signal[0],
            "warning_1": warning_signal[1],
            "warning_2": warning_signal[2],
            "warning_3": warning_signal[3],
            "warning_4": warning_signal[4],
            "predicted_tl_state": predicted_tl_state  ,
            "computed_warning": warning_showed}    )
            
            # set previous speed for next iteration
            spd_ego_prev = spd_ego
            
            
            # --- Every 10 seconds: save data + recalibrate ---
            now = time.time()
            if now - last_calibration_time >= CALIBRATION_INTERVAL:
                last_calibration_time = now

                # Save current row
                # rows.append({
                #     "timestamp":        current_time,
                #     "pos_ego":          pos_ego,
                #     "spd_ego":          spd_ego,
                #     "warning_0":        warning_signal[0],
                #     "warning_1":        warning_signal[1],
                #     "warning_2":        warning_signal[2],
                #     "warning_3":        warning_signal[3],
                #     "warning_4":        warning_signal[4],
                #     "predicted_tl_state": predicted_tl_state,
                #     "computed_warning": warning_showed
                # })

                # Write to CSV (append mode so we don't lose history)
                df_new = pd.DataFrame([rows[-1]])
                write_header = not os.path.exists(Calibrate_data)
                df_new.to_csv(Calibrate_data, mode='a', header=write_header, index=False)
                print(f"[ILC] Row saved to {Calibrate_data}")

                # Recalibrate
                try:
                    if os.path.exists(Calibrate_data):
                        warning_ilc, acc_ilc, spd_ilc, spacing_ilc, tls_ilc = load_ilc_data(Calibrate_data)
                        if len(warning_ilc) > 5:
                            print(f"[ILC] Running calibration on {len(warning_ilc)} samples...")
                            a_cal, d_cal, c_cal, T_cal, J_final = mainIDM(
                                v0, delta,
                                warning_ilc, acc_ilc, spd_ilc, spacing_ilc, tls_ilc
                            )
                            print(f"[ILC] Done. a={a_cal:.4f} d={d_cal:.4f} c={c_cal:.4f} T={T_cal:.4f} J={J_final:.6f}")
                        else:
                            print("[ILC] Not enough data yet.")
                except Exception as e:
                    print(f"[ILC] Calibration error: {e}")
# save data to csv for ILC training            
df = pd.DataFrame(rows)

csv_path = "warning_log.csv"
df.to_csv(csv_path, index=False)

print(f"CSV successfully saved to {csv_path}")


            
# with open("record_data18/warning_list.obj", "wb") as handle:
#     pkl.dump(warning_list, handle, protocol=pkl.HIGHEST_PROTOCOL)
# with open("record_data18/cav_msg_list.obj", "wb") as handle:
#     pkl.dump(cav_msg_list, handle, protocol=pkl.HIGHEST_PROTOCOL)
# with open("record_data18/prediction_list.obj", "wb") as handle:
#     pkl.dump(prediction_list, handle, protocol=pkl.HIGHEST_PROTOCOL)
# sys.exit()


# to extract the first value of the predicted traffic light state for plotting
