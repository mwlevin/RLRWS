import pickle
from cav_data import *
import numpy as np
import matplotlib.pyplot as plt 
with open("record_data2/cav_msg_list.obj", "rb") as handle:
    cav_msg=pickle.load(handle)
with open("record_data2/prediction_list.obj", "rb") as handle:
    prediction_list=pickle.load(handle)
with open("record_data2/warning_list.obj", "rb") as handle:
    warning_list=pickle.load(handle)
loc_list=np.zeros((np.size(cav_msg)))
spd_list=np.zeros((np.size(cav_msg)))
pos_list=np.zeros((np.size(prediction_list)))

print('warning_list',len(warning_list))
#print('warning_list',warning_list[61]) 4
print('prediction_list',np.size(prediction_list))
print('msg_list',len(cav_msg))
for i in range(0, np.size(cav_msg)):
    loc_list[i]=cav_msg[i].loc
    spd_list[i]=cav_msg[i].spd
    pos_list[i]=prediction_list[i].pos_ego
warning_spd=spd_list[123:563]

warning_loc=loc_list[123:563]
pos_of_ego=pos_list[123:563]
predicted_traffic_light=np.zeros((440,51))
predicted_pos_ego=np.zeros((440,51))
predicted_spd_ego=np.zeros((440,51))
for i in range(123,563):
    for s in range(0,51):
        predicted_traffic_light[i-123,s]=cav_msg[i].predicted_state[s]
        predicted_pos_ego[i-123,s]=prediction_list[i].pos_ego-prediction_list[i].pos_pred_ego[s]
        predicted_spd_ego[i-123,s]=prediction_list[i].spd_pred_ego[s]
print('pos_of_vehicle',warning_loc[400])
print('spd_of_vehicle',warning_spd[400])
print('predicted_spd_of_vehicle',predicted_spd_ego[400,:])