import pickle
from cav_data import *
import numpy as np
import matplotlib.pyplot as plt 
with open("record_data25/cav_msg_list.obj", "rb") as handle:
    cav_msg=pickle.load(handle)
with open("record_data25/prediction_list.obj", "rb") as handle:
    prediction_list=pickle.load(handle)
with open("record_data25/warning_list.obj", "rb") as handle:
    warning_list=pickle.load(handle)
loc_list=np.zeros((np.size(cav_msg)))
spd_list=np.zeros((np.size(cav_msg)))
pos_list=np.zeros((np.size(prediction_list)))

print('warning_list',len(warning_list))
#print('warning_list',warning_list[61]) 4
print('prediction_list',np.size(prediction_list))
print('msg_list',len(cav_msg))
#predicted_pos=np.zeros((np.size(prediction_list)))

##predicted_pos=prediction_list[300].pos_pred_ego
#print('predicted_pos',predicted_pos)

##predicted_spd=prediction_list[0].spd_pred_ego
#print('predicted_spd',predicted_spd)

##predicted_state=cav_msg[122].predicted_state[50]
#print('predicted_state',predicted_state)
##pos_ego=prediction_list[30].pos_ego
#print('pos_ego',pos_ego)
size_list=len(warning_list)
warning_signal=np.zeros(5*size_list)
predicted_traffic_state=[]
for i in range(0, np.size(cav_msg)):
    loc_list[i]=cav_msg[i].loc
    spd_list[i]=cav_msg[i].spd
    pos_list[i]=prediction_list[i].pos_ego
#    predicted_traffic_state.append(cav_msg[i].predicted_state)
k=0
#41 121 4
print('warning_list',warning_list)
for i in range(0,size_list):
    for s in range(0,5):
        warning_signal[k]=warning_list[i][s]
        k=k+1
print('size_warning_signal',warning_signal)
#print('warning_list',warning_list)
predicted_traffic_light=np.zeros((size_list,51))
predicted_pos_ego=np.zeros((size_list,51))
predicted_spd_ego=np.zeros((size_list,51))
for i in range(0,size_list):
    for s in range(0,51):
        predicted_traffic_light[i,s]=cav_msg[i].predicted_state[s]
        predicted_pos_ego[i,s]=prediction_list[i].pos_ego-prediction_list[i].pos_pred_ego[s]
        predicted_spd_ego[i,s]=prediction_list[i].spd_pred_ego[s]
print('length of predictpos',len(predicted_pos_ego))
#print('predicted_traffic_light',predicted_traffic_light)
print('predicted_pos_ego',predicted_pos_ego)
print('predicted_spd_ego',predicted_spd_ego)
current_traffic_light=predicted_traffic_light[:,0]
print('current_traffic_light',current_traffic_light)
print('predicted_tl',len(predicted_traffic_light[0,:]))
#print('warning_signal',warning_signal)
#print('size_loc',np.size(loc_list))
#print('size_spd',np.size(spd_list))
##print('predicted_warning',warning_list[118])

#print('predicted_state',predicted_traffic_state)
#print('predicted_state',predicted_state)
#warning_spd=spd_list[260:660] 4

warning_loc=loc_list[0:size_list]
warning_spd=spd_list[0:size_list]
#pos_of_ego=pos_list[0:106]
print('spd_list',spd_list)
print('loc_list',loc_list)
print('warning_spd',warning_spd)
#print('warning_loc',warning_loc)
##print('pos_of_vehicle',warning_loc[300])
##print('spd_of_vehicle',warning_spd[300])
##print('predicted_spd_of_vehicle',predicted_spd_ego[300,:])
#t1=np.arange(0,42)
##print('final_loc',pos_list[528])
##print('try_spd',spd_list[528])
size_green=0
size_yellow=0
size_red=0
for i in range(size_list):
    if current_traffic_light[i]==1:
        size_green+=1
    elif current_traffic_light[i]==0:
        size_red+=1
    else:
        size_yellow+=1
green_light=np.zeros((size_green))
yellow_light=np.zeros((size_yellow))
red_light=np.zeros((size_red))
t=np.arange(0,size_list)
print('warning_spd',warning_spd[300:350])

#plt.figure(1)
plt.subplot(4,1,1)
(line0,)=plt.plot(t,warning_spd,label='spd of target vehicle')
for i in range(size_list):
    if current_traffic_light[i] == 0:
        color = 'red'
    if current_traffic_light[i]==1:
        color='green'
    if current_traffic_light[i]==-1:
        color='yellow'
    #plt.scatter(i, 0, color=color, label='signal phase')
    plt.scatter(i, 0, color=color, label='signal phase')
plt.xlim(-5,size_list+15)
plt.legend(handles=[line0])
plt.xlabel('time (s)')
plt.ylabel('speed (m/s)')

#plt.subplot(5,1,2)
#(line1,)=plt.plot(t,predicted_spd_ego[:,0],label='spd')
#plt.legend(handles=[line1])
#plt.xlabel('time (s)')
#plt.ylabel('speed (m/s)')

plt.subplot(4,1,2)
for i in range(0,size_list):
    (line2,)=plt.plot(t,predicted_spd_ego[:,0])
   # for h in range(0,51):
    t2=np.arange(i/0.2,(i/0.2)+51)*0.2
    #print('t2',t2)
    (line3,)=plt.plot(t2,predicted_spd_ego[i,0:51])
for i in range(size_list):
    if current_traffic_light[i] == 0:
        color = 'red'
    if current_traffic_light[i]==1:
        color='green'
    if current_traffic_light[i]==-1:
        color='yellow'
    #plt.scatter(i, 0, color=color, label='signal phase')
    plt.scatter(i, 0, color=color, label='signal phase')
plt.xlim(-5,size_list+15)
plt.xlabel('time (s)')
plt.ylabel('predicted speed (m/s)')

plt.subplot(4,1,3)
for i in range(0,size_list):
    (line4,)=plt.plot(t,predicted_pos_ego[:,0])
   # for h in range(0,51):
    t2=np.arange(i/0.2,(i/0.2)+51)*0.2
    #print('t2',t2)
    (line5,)=plt.plot(t2,predicted_pos_ego[i,0:51])
for i in range(size_list):
    if current_traffic_light[i] == 0:
        color = 'red'
    if current_traffic_light[i]==1:
        color='green'
    if current_traffic_light[i]==-1:
        color='yellow'
    #plt.scatter(i, 0, color=color, label='signal phase')
    plt.scatter(i, 0, color=color, label='signal phase')
plt.xlim(-5,size_list+15)
plt.xlabel('time (s)')
plt.ylabel('position (m)')
#plt.subplot(4,1,3)
#plt.figure(2)
#(line1,)=plt.plot(t,warning_loc,label='pos of target vehicle')
#plt.legend(handles=[line1])
#plt.xlabel('time (s)')
#plt.ylabel('position (m)')
t1=np.arange(0,5*size_list)*0.2
print('size_t1',len(t1))
plt.subplot(4,1,4)
#plt.figure(3)
(line6,)=plt.plot(t1,warning_signal,label='warning_signal')
for i in range(size_list):
    if current_traffic_light[i] == 0:
        color = 'red'
    if current_traffic_light[i]==1:
        color='green'
    if current_traffic_light[i]==-1:
        color='yellow'
    #plt.scatter(i, 0, color=color, label='signal phase')
    plt.scatter(i, 0, color=color, label='signal phase')
plt.xlim(-5,size_list+15)
plt.legend(handles=[line6])
plt.xlabel('time (s)')
plt.ylabel('warning value')

#t1=np.arange(0,80)*0.2
#print('t1',t1)

#plt.subplot(6,1,4)

#print(predicted_traffic_light[70,40])
#for i in range(0,400):
#    (line3,)=plt.plot(t,predicted_traffic_light[:,0])
   # for h in range(0,51):
#    t2=np.arange(i,i+51)*0.2
    #print('t2',t2)
#    (line4,)=plt.plot(t2,predicted_traffic_light[i,0:51])

#plt.subplot(5,1,4)
#plt.figure(4)
#for i in range(0,400):
#    (line5,)=plt.plot(t,predicted_pos_ego[:,0])
   # for h in range(0,51):
#    t2=np.arange(i,i+51)*0.2
    #print('t2',t2)
#    (line6,)=plt.plot(t2,predicted_pos_ego[i,0:51])

#t3=np.arange(0,563)*0.2
#plt.subplot(6,1,6)
#(line7,)=plt.plot(t3,pos_list)


        

plt.show()