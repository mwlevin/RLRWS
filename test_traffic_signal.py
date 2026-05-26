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

import datetime
current_time = datetime.datetime.now()
last_time = current_time

signal_id =  2

while True:
    current_time = datetime.datetime.now()
    # if ((current_time - last_time).total_seconds()) >= 0.01:
    #     print('time gap', ((current_time - last_time).total_seconds()))
    #     last_time = current_time

    try:        
        newestData = None
        keepReceiving = True
        while keepReceiving:
            try:
                data, fromAddr = sock_0.recvfrom(100048)
                if data:
                    # print('test')
                    newestData = data
            except socket.error as why:
                keepReceiving = False
                # print('test')
                # break
        # newestData, fromAddr = sock_0.recvfrom(2048)
        # print(newestData)
        if newestData:
            # newestData, fromAddr = sock_0.recvfrom(2048)
            decoded_message = unwrap_and_decode(newestData)      
            if decoded_message["value"][0] == "SPAT":
                SPaT_data = decoded_message["value"][1]["intersections"][0]
               # if SPaT_data["states"][6]['signalGroup']==7:
                if len(SPaT_data["states"])==16:
                #print('SPaT data', SPaT_data)
                    moy = SPaT_data["moy"]
                    moy_hour, moy_minute = moy_to_hour_min(moy)
                    time_stamp = SPaT_data["timeStamp"]
                    sec_of_minute = ms_to_sec(time_stamp)
                    # signal_id = 11
                    signal_data = SPaT_data["states"][signal_id-1]["state-time-speed"][0]       
                    # print('signal ground id', SPaT_data["states"][signal_id-1]["state-time-speed"][0])                         
                    # print('moy hour',moy_hour)
                   # print('moy minute',moy_minute)
                   # print('sec of min',sec_of_minute)
                    print('SPaT data', SPaT_data)
                
                
                #print('Spat data', signal_data)

                    for id_tmp in range(11):
                        if id_tmp==1:
                        
                            if SPaT_data["states"][id_tmp]["state-time-speed"][0]["eventState"] in red_status:
                                print('signal id ', SPaT_data["states"][id_tmp]['signalGroup'],' is  red')
                                # print(SPaT_data["states"][signal_id-1])
                            if SPaT_data["states"][id_tmp]["state-time-speed"][0]["eventState"] in green_status:
                                # print('signal id ', id_tmp+1,' is  green')
                                print('signal id ', SPaT_data["states"][id_tmp]['signalGroup'],' is  green')
                            if SPaT_data["states"][id_tmp]["state-time-speed"][0]["eventState"] in yellow_status:
                                # print('signal id ', id_tmp+1,' is  yellow')
                                print('signal id ', SPaT_data["states"][id_tmp]['signalGroup'],' is  yellow')





                # if signal_data["eventState"] in red_status:
                #     print("signal is red")
                #     signal_status = "red"
                # elif signal_data["eventState"] in yellow_status:
                #     print("signal is yellow")
                #     signal_status = "yellow"
                # elif signal_data["eventState"] in green_status:
                #     print("signal is green")
                #     signal_status = "green"
                # min_end_time = signal_data["timing"]["minEndTime"]
                # max_end_time = signal_data["timing"]["maxEndTime"]
                # minute_min, sec_min = tenth_s_to_min_sec(min_end_time)
                # minute_max, sec_max = tenth_s_to_min_sec(max_end_time)
                # sec_to_change_min = calc_change_time(minute_min, sec_min, moy_minute, sec_of_minute)
                
                
                
                
                # # print("min time to change the status is after ", sec_to_change_min , " sec")     
                # sec_to_change_max = calc_change_time(minute_max, sec_max, moy_minute, sec_of_minute)
                # print("max time to change the status is after ", sec_to_change_max , " sec")        
    except BlockingIOError:
        pass
            
    