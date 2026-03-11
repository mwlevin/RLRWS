import socket
import sys
import pickle as pkl
from matplotlib import animation
import datetime
import numpy as np
from matplotlib import pyplot as plt
import matplotlib


# add the function to check if we are in the if condition(if condition is when we are close and stopped)
   



# def move_figure(f, x, y):
#     """Move figure's upper left corner to pixel (x, y)"""
#     backend = matplotlib.get_backend()
#     if backend == 'TkAgg':
#         f.canvas.manager.window.wm_geometry("+%d+%d" % (x, y))
#     elif backend == 'WXAgg':
#         f.canvas.manager.window.SetPosition((x, y))
#     else:
#         # This works for QT and GTK
#         # You can also use window.setGeometry
#         f.canvas.manager.window.move(x, y)


# plt.ion()


# fig = plt.figure(1,(9,4),dpi=250)

# ax = plt.subplot(111)

# move_figure(fig, 50,50)
# plt.sca(ax)

# plt.axis('off')
# plt.tight_layout()

# warning_plot = plt.plot(0,0,'red',alpha=0.4,marker='o', markersize=10)[0]
# plt.show()
HOST_REC = ""
PORT_REC = 10890

s_rec = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_rec.bind((HOST_REC, PORT_REC))
s_rec.setblocking(False)
current_time = datetime.datetime.now()
last_time = current_time
#HOST_REC = ""
#PORT_REC = 10890
HOST_REC1 = ""
PORT_REC1 = 10891
s_rec1 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_rec1.bind((HOST_REC1, PORT_REC1))

HOST_REC2 = ""
PORT_REC2 = 10892
s_rec2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_rec2.bind((HOST_REC2, PORT_REC2))

HOST_REC3 = ""
PORT_REC3 = 10893
s_rec3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s_rec3.bind((HOST_REC3, PORT_REC3))
yellow_limit = 30  # at warning = 30, we switch from yellow to red
#s_rec = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#s_rec.bind((HOST_REC, PORT_REC))
#s_rec.setblocking(False)
#current_time = datetime.datetime.now()
#last_time = current_time
def passer(pos_ego, spd_ego,predicted_tl_state):
    original_pos_ego = -pos_ego
    original_spd_ego = spd_ego

    if -original_pos_ego >= 0 and -original_pos_ego<=500:
        tl_status_in_range = [predicted_tl_state[0, 0]]
    else:
        tl_status_in_range = []
    
    if -original_pos_ego >= 0 and -original_pos_ego<=500:
        if  -original_pos_ego <=60 and original_spd_ego<=4 and tl_status_in_range[0]==0: 
            check_if = True
        else:
            check_if = False
    else:
       check_if = False

    return check_if   
while True:
    
    current_time = datetime.datetime.now()
    # plt.plot(0,0,'green',alpha=0.4,marker='o', markersize=10)[0]
    if ((current_time - last_time).total_seconds()) >= 1:
        last_time = current_time
        newestData = None
        keepReceiving = True
        while keepReceiving:
            # warning_plot.set_color("green")
            # plt.plot(0,0,'green',alpha=0.4,marker='o', markersize=100)[0]
            try:
                data, fromAddr = s_rec.recvfrom(10048)
                data_pos_ego, fromAddr = s_rec1.recvfrom(500000)
                data_spd_ego, fromAddr = s_rec2.recvfrom(500000)
                data_predicted_tl, fromAddr = s_rec3.recvfrom(500000)
                if data:
                    newestData = data
                if data_pos_ego:
                    newestData_pos_ego = data_pos_ego
                if data_spd_ego:
                    newestData_spd_ego = data_spd_ego
                if data_predicted_tl:
                    newestData_predicted_tl = data_predicted_tl
            except socket.error as why:
                keepReceiving = False
                break
        if newestData:
            plt.clf()
            data_orig = pkl.loads(newestData)
            warning_signal_ego = data_orig

            data_orig_pos_ego = pkl.loads(newestData_pos_ego)
            pos_ego = data_orig_pos_ego

            data_orig_spd_ego = pkl.loads(newestData_spd_ego)
            spd_ego = data_orig_spd_ego

            data_orig_predicted_tl = pkl.loads(newestData_predicted_tl)
            predicted_tl_state = data_orig_predicted_tl
            # print('warning',warning_signal_ego[0])

            # read 3 required dynamics
            #pos_ego, spd_ego, predicted_tl_state = dynamics_sender()
            
            # print("position is", pos_ego, "speed is", spd_ego)

            # if condition is activated
            if passer(pos_ego, spd_ego, predicted_tl_state):
                plt.plot(0,0,'red',alpha=0.5,marker='o', markersize= yellow_limit*4+50)[0]   # show red dot
                # print("works works")
            else:
                if warning_signal_ego[0]<=0.001:
                    plt.plot(0,0,'green',alpha=0.5,marker='o', markersize=50)[0]
                elif 0.001<warning_signal_ego[0]<=yellow_limit:
                    plt.plot(0,0,'gold',alpha=0.5,marker='o', markersize=warning_signal_ego[0]*4+50)[0]
                else:
                    plt.plot(0,0,'red',alpha=0.5,marker='o', markersize=warning_signal_ego[0]*4+50)[0]

        plt.pause(0.01)    
        # plt.ioff()