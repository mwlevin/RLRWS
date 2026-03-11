import socket
from pycmssdk import Asn1Type, FacMsgType, FacNotifData, asn1_decode, create_cms_api
from time_convert import *
import pickle as pkl
import numpy as np
from cav_data import *
from matplotlib import pyplot as plt


with open("record_data/cav_msg_list.obj", "rb") as handle:
    cav_msg_list = pkl.load(handle)
with open("record_data/warning_list.obj", "rb") as handle:
    warning_list = pkl.load(handle)
# fig, axs = plt.subplots(3)
cav_spd = []
cav_loc = []
cav_warning = []
cav_acc = []
count = 0
for item in cav_msg_list:
    cav_spd.append(item.spd)
    cav_loc.append(item.loc)

    if count >= 1:
        cav_acc.append((cav_spd[count] - cav_spd[count-1])/0.2)

    count += 1
count = 0
for item in warning_list:
    cav_warning.append(item[0])
    # axs[1].plot(np.arange(count,count+len(item[0:5])*0.2,0.2),item[0:5],'green')
    count+=1

# plt.plot(np.arange(0,len(cav_acc)*0.2,0.2),cav_acc)
# plt.xlabel("Time (s)")
# plt.ylabel("Acceleration (m/s$^2$)")
# plt.show()
print(cav_warning)
plt.plot(np.arange(0,len(cav_warning)*0.2,0.2),cav_warning)
plt.xlabel("Time (s)")
plt.ylabel("Warning")
plt.show()
# plt.plot(np.arange(0,len(cav_loc)*0.2,0.2),cav_loc)
# for i in range(len(cav_loc)):
#     if i>=51/0.2 and i<=130/0.2:
#         plt.plot(i*0.2,0,"red",alpha=0.9,marker="o",markersize=1)
#     if i>130.3/0.2:
#         print(i*0.2)
#         plt.plot(i*0.2,0,"green",marker="o",markersize=1)
        

# plt.xlabel("Time (s)")
# plt.ylabel("Location (m)")
# plt.show()


# axs[0].plot(np.arange(0,len(cav_spd)*0.2,0.2),cav_spd)
# font_size = 17
# axs[0].set_xlabel("Time (s)", fontsize=font_size)
# axs[1].set_xlabel("Time (s)", fontsize=font_size)
# axs[2].set_xlabel("Time (s)", fontsize=font_size)
# axs[0].set_ylabel("Speed (m/s)", fontsize=font_size)
# axs[1].set_ylabel("Warning ", fontsize=font_size)
# axs[2].set_ylabel("Location (m)", fontsize=font_size)
# axs[0].set_xticks([40,60,80,100,120,140],["40","60","80","100","120","140"],fontsize=font_size)
# axs[0].set_yticks([0,5,10,15,20,25],["0","5","10","15","20","25"],fontsize=font_size)
# axs[1].set_xticks([40,60,80,100,120,140],["40","60","80","100","120","140"],fontsize=font_size)
# axs[1].set_yticks([0,20,40,60,80,100],["0","20","40","60","80","100"],fontsize=font_size)
# axs[2].set_xticks([40,60,80,100,120,140],["40","60","80","100","120","140"],fontsize=font_size)
# axs[2].set_yticks([0,200,400,600,800,1000,1200],["0","200","400","600","800","1000","1200"],fontsize=font_size)
# # plt.show()
# axs[1].plot(np.arange(0,len(cav_warning)*1,1),cav_warning)
# # plt.show()
# axs[2].plot(np.arange(0,len(cav_loc)*0.2,0.2),cav_loc)
# axs[2].set_xlim([40,140])
# axs[1].set_xlim([40,140])
# axs[0].set_xlim([40,140])
# plt.show()