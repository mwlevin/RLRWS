import sys
import pickle as pkl

import datetime
import numpy as np
from matplotlib import pyplot as plt
from time_convert import *
from coord2dist import *
import socket


from cav_data import *
from prediction import *
from UKF import *
from estimation_param import *
from est_pred_param import *
import keyboard

#import pygame

HOST_REC = ""
PORT_REC = 10888
s_rec = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_rec.bind((HOST_REC, PORT_REC))


HOST_SEND = 'localhost'
PORT_SEND = 10889
s_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

HOST_SEND1 = 'localhost'
PORT_SEND1 = 10900
s_send1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


# load simulation parameter
sim_param = SimParameter()
# load the parameter for UKF
ukf_param = UKFParam(
    sim_param.num_cell,
    dt=sim_param.dt_estimation,
    dx=sim_param.dx,
    num_param=3,
    dt_simulation=sim_param.dt_estimation,
)
filter_ukf = UnscentedKalmanFilter(
    sim_param.dt_estimation, ukf_param, traffic_dynamics, measurement_fb
)
filter_ukf.initialize(28)


current_time = datetime.datetime.now()
last_time = current_time

s_rec.setblocking(False)
#cav_msg_list=[]
#prediction_list = []
pos_ego = 1000
while True:
    if pos_ego<=-50:
        break
    # for event in pygame.event.get():
    #     if (event.type == KEYUP):
    #         print( "key pressed")
    #         break
    #     break
    current_time = datetime.datetime.now()
    if ((current_time - last_time).total_seconds()) >= 0.2:
        last_time = current_time
        newestData = None
        keepReceiving = True
        while keepReceiving:
            try:
                data, fromAddr = s_rec.recvfrom(2048)
                if data:
                    newestData = data
            except socket.error as why:
                # keepReceiving = False
                break
        if newestData:
            data_orig = pkl.loads(newestData)
            spd_ego = data_orig.spd
            pos_ego = data_orig.loc
            ff_spd = data_orig.ff_spd
            predicted_tl_state = data_orig.predicted_state
            
            # update ff_speed
            # ---- NEW: push fresh free-flow speed into UKF params ----
            ukf_param.update_ff_speed(data_orig.ff_spd)
            
            pos_pred_ego, pos_pred_max_ego, pos_pred_min_ego, spd_pred_ego = prediction(
                pos_ego, spd_ego, predicted_tl_state, filter_ukf, ukf_param, sim_param
            )
            pred_data = PredData(pos_pred_ego, pos_pred_max_ego, pos_pred_min_ego,
                                 spd_pred_ego, predicted_tl_state, pos_ego, spd_ego)
            pred_data_obj = pkl.dumps(pred_data)
            msg_data_obj=pkl.dumps(data_orig)
          #  cav_msg_list.append(data_orig)
          #  prediction_list.append(pred_data)
          #  print('prediction', pos_pred_ego[0:5,0])
            s_send.sendto(pred_data_obj, (HOST_SEND, PORT_SEND))
            s_send1.sendto(msg_data_obj, (HOST_SEND1, PORT_SEND1))
            print(" sent workss")
#with open("record_data6/cav_msg_list.obj", "wb") as handle:
#    pkl.dump(cav_msg_list, handle, protocol=pkl.HIGHEST_PROTOCOL)
#with open("record_data6/prediction_list.obj", "wb") as handle:
#    pkl.dump(prediction_list, handle, protocol=pkl.HIGHEST_PROTOCOL)
sys.exit()