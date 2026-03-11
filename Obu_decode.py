import sys
import os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import socket
from pycmssdk import FacMsgType, FacNotifData,create_cms_api
from pycmssdk import Asn1Type, FacMsgType, FacNotifData, asn1_decode, create_cms_api

#from pycmssdk.asn1 import Asn1Type

#from pycmssdk.asn1 import asn1_decode

from time_convert import *
import pickle as pkl
import numpy as np
from cav_data import *
from distance_finder import *

# import keyboard

from distance_finder import detect_reference_trajectory, load_reference_np, distance_to_stopbar

with open("map/cum_distance.obj", "rb") as handle:
    cum_dist_list = pkl.load(handle)
    
with open("map/list_lat.obj", "rb") as handle:
    lat_list = pkl.load(handle)
    
with open("map/list_long.obj", "rb") as handle:
    long_list = pkl.load(handle)
    





red_status = ["stop-And-Remain", "stop-Then-Proceed"]
yellow_status = ["permissive-clearance", "protected-clearance", "caution-Conflicting-Traffic"]
green_status = ["permissive-Movement-Allowed", "pre-Movement", "protected-Movement-Allowed"]
error_status = ["dark", "unavailable"]

CMS_HOST = "192.168.0.54"
cms = create_cms_api(host=CMS_HOST)
def unwrap_and_decode(buf: bytes):
    for off in (0, 4, 8, 12):
        try:
            return asn1_decode(buf[off:], Asn1Type.US_MESSAGE_FRAME)
        except Exception:
            pass
    try:
        out = cms.decode(buf)
        raw = getattr(out, "unsecured_data", None) or getattr(out, "data", None)
        if not raw:
            return None
    except Exception:
        return None
    for off in (0, 4, 8, 12):
        try:
            return asn1_decode(raw[off:], Asn1Type.US_MESSAGE_FRAME)
        except Exception:
            pass
    return None

#signal_id = 1  # from highway:6; to highway 2; from Home Depot 8; to Home Depot 4
UDP_IP_0 = "192.168.0.60"
UDP_PORT_0 = 8000
sock_0 = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock_0.bind((UDP_IP_0, UDP_PORT_0))
sock_0.setblocking(False)
UDP_IP_1 = "192.168.0.60"
UDP_PORT_1 = 7000
sock_1 = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock_1.bind((UDP_IP_1, UDP_PORT_1))
sock_1.setblocking(False)


HOST = 'localhost'
PORT = 10888
sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


# dt=0.2s, 0 is red, -1 is yellow, 1 is green
pred_horizon = 50
predicted_state = np.ones((pred_horizon+1,1))
dt = 0.2

signal_status = None
cum_dist = 0
speed = 0

signal_id = 6
#42/18
#S/N going straightSignal_id=8
#S/N turn left signal_id=3
#W/E Going staightsignal=2
#W/E Turn left signal=5
#E/W left signal=1
#E/W going straight=6
#N/S going straight=4
#N/S turn left=7
#42/21
#S/N going straightSignal_id=
#S/N turn left signal_id=
#W/E Going staightsignal=
#W/E Turn left signal=
#E/W left signal=
#E/W going straight=
#N/S going straight=
#N/S turn left=
#21/16
#S/N going straightSignal_id=
#S/N turn left signal_id=
#W/E Going staightsignal=
#W/E Turn left signal=
#E/W left signal=
#E/W going straight=
#N/S going straight=
#N/S turn left=
#Intersection ID:
#42/18:53852
# added for refernce
dist_counter = 0
live_points = []
idx=0

while True:

    try:        
        newestData = None
        keepReceiving = True
        while keepReceiving:
            try:
                data, fromAddr = sock_0.recvfrom(10824)
                if data:
                    newestData = data
            except socket.error as why:
                # keepReceiving = False
                break
        if newestData:
            decoded_message = unwrap_and_decode(newestData)      
            if decoded_message["value"][0] == "SPAT":
                # --------- choose which intersection you want ----------
                TARGET_INTERSECTION_ID = 53852   #  for different RSUs/intersections

                # decoded_message["value"][1]["intersections"] is a LIST
                intersections = decoded_message["value"][1]["intersections"]

                # pick the intersection dict whose id matches TARGET_INTERSECTION_ID
                SPaT_data = None
                for inter in intersections:
                    # your printed data shows: 'id': {'id': 0}
                    if inter["id"]["id"] == TARGET_INTERSECTION_ID:
                        SPaT_data = inter
                        break
                # ---- IMPORTANT: if not found, skip this message ----
                if SPaT_data is None:
                    # optional debug: show what IDs exist
                    print("Target not found. Available IDs:",
                        [x["id"]["id"] for x in intersections])
                    continue
              #  SPaT_data = decoded_message["value"][1]["intersections"][0]
                print("SPaT intersection ID =", SPaT_data["id"]["id"])
               # SPaT_data = decoded_message["value"][1]["intersections"][0]
              #  if SPaT_data["states"][6]['signalGroup']==7:
            #if len(SPaT_data["states"])==16:
                moy = SPaT_data["moy"]
                moy_hour, moy_minute = moy_to_hour_min(moy)
                time_stamp = SPaT_data["timeStamp"]
                sec_of_minute = ms_to_sec(time_stamp)
                signal_data = SPaT_data["states"][signal_id-1]["state-time-speed"][0]                                
                if signal_data["eventState"] in red_status:
                    # print("signal is red")
                    signal_status = "red"
                elif signal_data["eventState"] in yellow_status:
                    # print("signal is yellow")
                    signal_status = "yellow"
                elif signal_data["eventState"] in green_status:
                    # print("signal is green")
                    signal_status = "green"
                min_end_time = signal_data["timing"]["minEndTime"]
                max_end_time = signal_data["timing"]["maxEndTime"]
                minute_min, sec_min = tenth_s_to_min_sec(min_end_time)
                minute_max, sec_max = tenth_s_to_min_sec(max_end_time)
                sec_to_change_min = calc_change_time(minute_min, sec_min, moy_minute, sec_of_minute)
                
                
                
                
                # print("min time to change the status is after ", sec_to_change_min , " sec")     
                sec_to_change_max = calc_change_time(minute_max, sec_max, moy_minute, sec_of_minute)
                # print("max time to change the status is after ", sec_to_change_max , " sec")        
    except BlockingIOError:
        pass
    try:   
        newestData = None
        keepReceiving = True
        while keepReceiving:
            try:
                data, fromAddr = sock_1.recvfrom(2048)
                if data:
                    newestData = data
            except socket.error as why:
                # keepReceiving = False
                break
        if newestData:
            decoded_message = unwrap_and_decode(newestData)
            if decoded_message["value"][0] == "BasicSafetyMessage":
                latitude = decoded_message["value"][1]["coreData"]["lat"]*(10**(-7))
                longitude = decoded_message["value"][1]["coreData"]["long"]*(10**(-7))
                # print('vehicle latitude',latitude)
                # print('vehicle latitude',latitude)
                speed = decoded_message["value"][1]["coreData"]["speed"]*0.02
                # print("Vehicle Speed: ",speed," m/s")
                
                
                ## find distance to the signal using references
                cum_dist=1000
                if dist_counter <=9:
                    dist_counter += 1
                    live_points.append((latitude, longitude))
                    folder = "traj_ref"
    
                    best_ref = detect_reference_trajectory(live_points, folder)  # find the driving approach
                    cum_dist = 1000
                    print("finding reference, counter: ", dist_counter)
                    
                elif dist_counter == 10: # more than 10 points available
                    # load the csv file of the best match
                    ref_np = load_reference_np(best_ref['file'])
                    print("reference loaded, counter: ", dist_counter)
                    dist_counter += 1
                    
                elif dist_counter > 10:
                    cum_dist, idx = distance_to_stopbar(latitude, longitude, ref_np)
                    print("distance calculated, counter: ", dist_counter, "distance: ", cum_dist, "idx",idx)
                    
                
                # if dist_counter <=9:
                #     dist_counter += 1
                #     live_points.append((latitude, longitude))
                #     folder = "traj_ref"
    
                #     best_ref = detect_reference_trajectory(live_points, folder)  # find the driving approach
                #     cum_dist = 1000
                    
                # elif dist_counter == 10: # more than 10 points available
                #     # load the csv file of the best match
                #     ref_np = load_reference_np(best_ref['file'])
                    
                # elif dist_counter > 10:
                #     cum_dist = distance_to_stopbar(latitude, longitude, ref_np)
                    
                
                
                
                
                # index_pos = np.argmin((lat_list - latitude)**2 + (long_list-longitude)**2)
                # cum_dist = cum_dist_list[index_pos][0]        
    except BlockingIOError:
        pass   
    
    if signal_status is not None:
        step_to_change = int((sec_to_change_min/dt))
        if signal_status == "green":
            if step_to_change>=pred_horizon:
                # always green along prediction
                predicted_state[:,0] = 1
            else:
                predicted_state[0:step_to_change,0] = 1
                if step_to_change+15<=pred_horizon:
                    # yellow is 3 s, 15 step
                    predicted_state[step_to_change:step_to_change+15,0] = -1
                    predicted_state[step_to_change+15:,0] = 0
                else:
                    predicted_state[step_to_change:,0] = -1
        elif signal_status == "yellow":
            if step_to_change>=pred_horizon:
                predicted_state[:,0] = -1
            else:
                predicted_state[0:step_to_change,0] = -1
                predicted_state[step_to_change:,0] = 0
        else:
            if step_to_change>=pred_horizon:
                predicted_state[:,0] = 0
            else:
                predicted_state[0:step_to_change,0] = 0
                predicted_state[step_to_change:,0] = -1
    cav_data = CavData(predicted_state, cum_dist, speed)
    #print('  distance  ', cum_dist,"  idx  ", idx)
    if signal_status is not None:
        print('signal status', signal_status)
        #print('min change', sec_to_change_min)
        # print('max change',sec_to_change_max)
    cav_data_obj = pkl.dumps(cav_data)
    sock_send.sendto(cav_data_obj, (HOST, PORT))
