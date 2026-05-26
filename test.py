import socket
from pycmssdk import Asn1Type, FacMsgType, FacNotifData, asn1_decode, create_cms_api
from time_convert import *
import pickle as pkl
import numpy as np
from cav_data import *
import keyboard

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

signal_id  = 2
import datetime
current_time = datetime.datetime.now()
last_time = current_time
while True:

    try:        
        current_time = datetime.datetime.now()
        if ((current_time - last_time).total_seconds()) >= 0.5:
            last_time = current_time
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
                decoded_message = asn1_decode(newestData, Asn1Type.US_MESSAGE_FRAME)      
                if decoded_message["value"][0] == "SPAT":
                    print('test')
                    # print(decoded_message)
                    SPaT_data = decoded_message["value"][1]["intersections"][0]
                    
                    moy = SPaT_data["moy"]
                    moy_hour, moy_minute = moy_to_hour_min(moy)
                    time_stamp = SPaT_data["timeStamp"]
                    sec_of_minute = ms_to_sec(time_stamp)
                    signal_data = SPaT_data["states"][signal_id-1]["state-time-speed"][0]                                
                    print('signal event', signal_data["eventState"])
                    if signal_data["eventState"] in red_status:
                        # print("signal is red")
                        signal_status = "red"
                        print('red')
                    elif signal_data["eventState"] in yellow_status:
                        # print("signal is yellow")
                        signal_status = "yellow"
                        print('yellow')
                    elif signal_data["eventState"] in green_status:
                        # print("signal is green")
                        signal_status = "green"
                        print('green')
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
    
    # if signal_status is not None:
    #     print('signal status', signal_status)
    #     print('min change', sec_to_change_min)
    #     # print('max change',sec_to_change_max)
