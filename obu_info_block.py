import socket
from pycmssdk import FacMsgType, FacNotifData,create_cms_api
# from pycmssdk import Asn1Type, FacMsgType, FacNotifData, asn1_decode, create_cms_api

from pycmssdk.asn1 import Asn1Type

from pycmssdk.asn1 import asn1_decode

from distance_finder import *

from time_convert import *
import pickle as pkl
import numpy as np
from cav_data import *
# import keyboard

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


signal_id = 2  # from highway:6; to highway 2; from Home Depot 8; to Home Depot 4
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

signal_id = 2

# added for refernce
dist_counter = 0
live_points = []

while True:

    # try:        
    #     newestData = None
    #     keepReceiving = True
    #     while keepReceiving:
    #         try:
    #             data, fromAddr = sock_0.recvfrom(10824)
    #             if data:
    #                 newestData = data
    #         except socket.error as why:
    #             # keepReceiving = False
    #             break
    #     if newestData:
    #         decoded_message = asn1_decode(newestData, Asn1Type.US_MESSAGE_FRAME)      
    #         if decoded_message["value"][0] == "SPAT":
    #             SPaT_data = decoded_message["value"][1]["intersections"][0]
    #             if SPaT_data["states"][6]['signalGroup']==7:
    #                 moy = SPaT_data["moy"]
    #                 moy_hour, moy_minute = moy_to_hour_min(moy)
    #                 time_stamp = SPaT_data["timeStamp"]
    #                 sec_of_minute = ms_to_sec(time_stamp)
    #                 signal_data = SPaT_data["states"][signal_id-1]["state-time-speed"][0]                                
    #                 if signal_data["eventState"] in red_status:
    #                     # print("signal is red")
    #                     signal_status = "red"
    #                 elif signal_data["eventState"] in yellow_status:
    #                     # print("signal is yellow")
    #                     signal_status = "yellow"
    #                 elif signal_data["eventState"] in green_status:
    #                     # print("signal is green")
    #                     signal_status = "green"
    #                 min_end_time = signal_data["timing"]["minEndTime"]
    #                 max_end_time = signal_data["timing"]["maxEndTime"]
    #                 minute_min, sec_min = tenth_s_to_min_sec(min_end_time)
    #                 minute_max, sec_max = tenth_s_to_min_sec(max_end_time)
    #                 sec_to_change_min = calc_change_time(minute_min, sec_min, moy_minute, sec_of_minute)
                    
                    
                    
                    
    #                 # print("min time to change the status is after ", sec_to_change_min , " sec")     
    #                 sec_to_change_max = calc_change_time(minute_max, sec_max, moy_minute, sec_of_minute)
    #                 # print("max time to change the status is after ", sec_to_change_max , " sec")        
    # except BlockingIOError:
    #     pass
    
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
            print(" recieving BSMMM")
            decoded_message = asn1_decode(newestData, Asn1Type.US_MESSAGE_FRAME)
            if decoded_message["value"][0] == "BasicSafetyMessage":
                latitude = decoded_message["value"][1]["coreData"]["lat"]*(10**(-7))
                longitude = decoded_message["value"][1]["coreData"]["long"]*(10**(-7))
                
                print("Vehicle Latitude: ",latitude , "Vehicle Longitude: ", longitude)
                cum_dist = 1500 
                # print('vehicle latitude',latitude)
                # print('vehicle latitude',latitude)
                speed = decoded_message["value"][1]["coreData"]["speed"]*0.02
                # print("Vehicle Speed: ",speed," m/s")
                
                
                                ## find distance to the signal using references
# This part of the code is responsible for updating the cumulative distance (`cum_dist`) based on the
# vehicle's current position. Here's a breakdown of what each section does:
                ## find distance to the signal using references
                if dist_counter <=9:
                    dist_counter += 1
                    live_points.append((latitude, longitude))
                    folder = "traj_ref"
    
                    best_ref = detect_reference_trajectory(live_points, folder)  # find the driving approach
                    cum_dist = 1000
                    
                elif dist_counter == 10: # more than 10 points available
                    # load the csv file of the best match
                    ref_np = load_reference_np(best_ref['file'])
                    
                elif dist_counter > 10:
                    cum_dist = distance_to_stopbar(latitude, longitude, ref_np)
                
                
                
                # index_pos = np.argmin((lat_list - latitude)**2 + (long_list-longitude)**2)
                # cum_dist = cum_dist_list[index_pos][0]        
    except BlockingIOError:
        pass   
    
    # if signal_status is not None:
    #     step_to_change = int((sec_to_change_min/dt))
    #     if signal_status == "green":
    #         if step_to_change>=pred_horizon:
    #             # always green along prediction
    #             predicted_state[:,0] = 1
    #         else:
    #             predicted_state[0:step_to_change,0] = 1
    #             if step_to_change+15<=pred_horizon:
    #                 # yellow is 3 s, 15 step
    #                 predicted_state[step_to_change:step_to_change+15,0] = -1
    #                 predicted_state[step_to_change+15:,0] = 0
    #             else:
    #                 predicted_state[step_to_change:,0] = -1
    #     elif signal_status == "yellow":
    #         if step_to_change>=pred_horizon:
    #             predicted_state[:,0] = -1
    #         else:
    #             predicted_state[0:step_to_change,0] = -1
    #             predicted_state[step_to_change:,0] = 0
    #     else:
    #         if step_to_change>=pred_horizon:
    #             predicted_state[:,0] = 0
    #         else:
    #             predicted_state[0:step_to_change,0] = 0
    #             predicted_state[step_to_change:,0] = -1
    # cav_data = CavData(predicted_state, cum_dist, speed)
    # # print('distance', cum_dist)
    # # if signal_status is not None:
    #     # print('signal status', signal_status)
    #    # print('min change', sec_to_change_min)
    #     # print('max change',sec_to_change_max)
    # cav_data_obj = pkl.dumps(cav_data)
    # sock_send.sendto(cav_data_obj, (HOST, PORT))
