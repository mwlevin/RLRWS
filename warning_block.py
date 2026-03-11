import sys
import pickle as pkl

import datetime
import numpy as np
from matplotlib import pyplot as plt
from time_convert import *
from coord2dist import *
import socket

import datetime
import socket
from cav_data import *
from prediction import *
from UKF import *
from estimation_param import *
from est_pred_param import *
from warning import *
import keyboard



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

HOST_SEND1 = 'localhost'
PORT_SEND1 = 10891
s_send1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HOST_SEND2 = 'localhost'
PORT_SEND2 = 10892
s_send2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HOST_SEND3 = 'localhost'
PORT_SEND3 = 10893
s_send3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


current_time = datetime.datetime.now()
last_time = current_time
s_rec.setblocking(False)
sim_param = SimParameter()
warning_list = []
cav_msg_list=[]
prediction_list = []
pos_ego = 0

while True:
    if pos_ego<=-50:
        break
    current_time = datetime.datetime.now()
    if ((current_time - last_time).total_seconds()) >= 1:
        last_time = current_time
        newestData_predicted = None
        keepReceiving = True
        while keepReceiving:
            try:
                data_predicted, fromAddr = s_rec.recvfrom(500000)
                data_msg, fromAddr = s_rec1.recvfrom(500000)
                if data_predicted:
                    newestData_predicted = data_predicted
                if data_msg:
                    newestData_msg = data_msg
            except socket.error as why:
                # keepReceiving = False
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

            print('current_tl',predicted_tl_state[0])

            # a function to send dynamics for plotting
           # def dynamics_sender():
           #     return pos_ego, spd_ego, predicted_tl_state
            pos_ego_obj=pkl.dumps(pos_ego)
            spd_ego_obj=pkl.dumps(spd_ego)
            predicted_tl_state_obj=pkl.dumps(predicted_tl_state)
            warning_signal = warning(pos_pred_ego, pos_pred_max_ego, pos_pred_min_ego, pos_ego, spd_ego, spd_pred_ego, predicted_tl_state, sim_param)
            warning_signal_obj = pkl.dumps(warning_signal)
            print('wraning send', warning_signal[0:5])
            warning_list.append(warning_signal)
            prediction_list.append(data_orig)
            s_send.sendto(warning_signal_obj, (HOST_SEND, PORT_SEND))
            s_send1.sendto(pos_ego_obj, (HOST_SEND1, PORT_SEND1))
            s_send2.sendto(spd_ego_obj, (HOST_SEND2, PORT_SEND2))
            s_send3.sendto(predicted_tl_state_obj, (HOST_SEND3, PORT_SEND3))
with open("record_data25/warning_list.obj", "wb") as handle:
    pkl.dump(warning_list, handle, protocol=pkl.HIGHEST_PROTOCOL)
with open("record_data25/cav_msg_list.obj", "wb") as handle:
    pkl.dump(cav_msg_list, handle, protocol=pkl.HIGHEST_PROTOCOL)
with open("record_data25/prediction_list.obj", "wb") as handle:
    pkl.dump(prediction_list, handle, protocol=pkl.HIGHEST_PROTOCOL)
sys.exit()