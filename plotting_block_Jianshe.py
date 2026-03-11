import socket
import sys
import pickle as pkl
from matplotlib import animation
import datetime
import numpy as np
from matplotlib import pyplot as plt
import matplotlib



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
while True:
    plt.clf()
    current_time = datetime.datetime.now()
    # plt.plot(0,0,'green',alpha=0.4,marker='o', markersize=10)[0]
    if ((current_time - last_time).total_seconds()) >= 1:
        last_time = current_time
        newestData = None
        keepReceiving = True
        while keepReceiving:
            # warning_plot.set_color("green")
            plt.plot(0,0,'green',alpha=0.4,marker='o', markersize=100)[0]
            try:
                data, fromAddr = s_rec.recvfrom(10048)
                if data:
                    newestData = data
            except socket.error as why:
                keepReceiving = False
                break
        if newestData:
            data_orig = pkl.loads(newestData)
            warning_signal_ego = data_orig
            print('warning',warning_signal_ego[0])
            if warning_signal_ego[0]<=30:
                plt.plot(0,0,'green',alpha=0.4,marker='o', markersize=warning_signal_ego[0]+100)[0]
            elif warning_signal_ego[0]<=70:
                plt.plot(0,0,'yellow',alpha=0.4,marker='o', markersize=warning_signal_ego[0]+100)[0]
            else:
                plt.plot(0,0,'red',alpha=0.4,marker='o', markersize=warning_signal_ego[0]+100)[0]
        plt.pause(0.01)    
        plt.ioff()