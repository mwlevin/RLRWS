import pickle as pkl
import time
from pycmssdk import Asn1Type, FacMsgType, FacNotifData, asn1_decode, create_cms_api
import dpkt
import socket
import os
from time_convert import *
from copy import deepcopy
import numpy as np


CMS_HOST = "192.168.0.54"   # OBU IP
cms = create_cms_api(host=CMS_HOST)

def try_decode_us_message_frame(b: bytes):
    # 1) direct decode with offsets (common when UDP payload has small header)
    for off in (0, 2, 4, 8, 12, 16):
        try:
            return asn1_decode(b[off:], Asn1Type.US_MESSAGE_FRAME)
        except Exception:
            pass

    # 2) try to unwrap secured payload (1609.2/CMS) then decode inner bytes
    try:
        out = cms.decode(b)
        raw = getattr(out, "unsecured_data", None) or getattr(out, "data", None)
        if isinstance(raw, memoryview):
            raw = raw.tobytes()
        if isinstance(raw, bytearray):
            raw = bytes(raw)
        if isinstance(raw, bytes):
            for off in (0, 2, 4, 8, 12, 16):
                try:
                    return asn1_decode(raw[off:], Asn1Type.US_MESSAGE_FRAME)
                except Exception:
                    pass
    except Exception:
        pass

    # 3) optional: trim tail (diagnostic—confirms “extra bytes at end”)
    # Comment this out once you identify the real wrapper.
    if isinstance(b, bytes) and len(b) > 20:
        for cut in range(1, min(80, len(b) - 1)):
            try:
                return asn1_decode(b[:-cut], Asn1Type.US_MESSAGE_FRAME)
            except Exception:
                pass

    return None

class DataStructure:
    def __init__(self, timestep, data):
        self.timestep = timestep
        self.data = data
    



red_status = ["stop-And-Remain", "stop-Then-Proceed"]
yellow_status = ["permissive-clearance", "protected-clearance", "caution-Conflicting-Traffic"]
green_status = ["permissive-Movement-Allowed", "pre-Movement", "protected-Movement-Allowed"]
error_status = ["dark", "unavailable"]




# load the pcap file
filename='0302.pcap'
# option 'rb' is needed for windows OS
f = open(filename,'rb')
pcap = dpkt.pcap.Reader(f)

count = 0
ts_initial = None
ts_final = None


time_spat = []
time_bsm = []

data_spat = []
data_bsm = []

time_map= []
data_map= []

for ts,buf in pcap:
    if count == 0:
        ts_initial = ts
    eth=dpkt.ethernet.Ethernet(buf)  
    if eth.type!=dpkt.ethernet.ETH_TYPE_IP:
        pass
    else:
        try:
            ip = eth.data
            transf_data = ip.data    
            
            src_ip = socket.inet_ntoa(ip.src)
            dst_ip = socket.inet_ntoa(ip.dst)
            src_port = transf_data.sport
            dst_port = transf_data.dport
            
            
            
            
            if isinstance(ip.data, dpkt.tcp.TCP): 
                continue

            if isinstance(ip.data, dpkt.udp.UDP):
                if src_ip == "192.168.0.54" and dst_ip == "192.168.0.60" and dst_port in (7000,8000):
                    decoded_message = try_decode_us_message_frame(transf_data.data)
                    if decoded_message is None:
                        continue

                    msg_type = decoded_message["value"][0]

                    # SPaT
                    if msg_type == "SPAT":
                        time_spat.append(ts - ts_initial)
                       # print('decoded_message["value"][1]',decoded_message["value"][1])
                       # break
                        data_spat.append(decoded_message["value"][1]["intersections"][0])

                    # BSM
                    elif msg_type == "BasicSafetyMessage":
                        time_bsm.append(ts - ts_initial)
                        data_bsm.append(decoded_message["value"][1]["coreData"])

                    # MAP
                    elif msg_type =="MapData":
                        time_map.append(ts - ts_initial)
                        data_map.append(decoded_message["value"][1])
            # if isinstance(ip.data, dpkt.udp.UDP):
            #     if dst_port == 8000 and src_ip == "192.168.0.54"and dst_ip=="192.168.0.60":
            #         #decoded_message = asn1_decode(transf_data.data, Asn1Type.US_MESSAGE_FRAME)
            #         decoded_message = try_decode_us_message_frame(transf_data.data)
            #         if decoded_message is None:
            #             continue
            #         if decoded_message["value"][0] == "SPAT":
            #             # data_stru = DataStructure(ts - ts_initial, decoded_message["value"][1]["intersections"][0])
            #             time_spat.append(ts- ts_initial)
            #             data_spat.append(decoded_message["value"][1]["intersections"][0])
                        
                        
            #     elif dst_port == 7000 and src_ip == "192.168.0.54"and dst_ip=="192.168.0.60":
            #        # decoded_message = asn1_decode(transf_data.data, Asn1Type.US_MESSAGE_FRAME)
            #         decoded_message = try_decode_us_message_frame(transf_data.data)
            #         if decoded_message is None:
            #             continue
            #         if decoded_message["value"][0] == "BasicSafetyMessage":
            #             # data_stru = DataStructure(ts - ts_initial, decoded_message["value"][1]["coreData"])
                        
            #             time_bsm.append(ts- ts_initial)
            #             data_bsm.append(decoded_message["value"][1]["coreData"])
        except AttributeError:
            pass
        count+=1
# print(time_bsm)
#print('time_bsm',time_bsm)
std_time = np.arange(0, time_bsm[-1], 0.1)
msg_bsm = {}
msg_spat = {}
msg_map={}


for id in range(len(std_time)):
    id_tmp_0 = np.argmin(np.abs(deepcopy(time_bsm) - std_time[id]))
    # print((deepcopy(time_bsm) - std_time[id])[0:10])
    # print(id_tmp_0)
    # os.system("pause")
    if np.abs(time_bsm[id_tmp_0] - std_time[id])>=0.15:
        msg_bsm[std_time[id]] = None
    else:
        msg_bsm[std_time[id]] = data_bsm[id_tmp_0]
    id_tmp_1 = np.argmin(np.abs(deepcopy(time_spat) - std_time[id]))
    if np.abs(time_spat[id_tmp_1] - std_time[id])>=0.15:
        msg_spat[std_time[id]] = None
    else:
        msg_spat[std_time[id]] = data_spat[id_tmp_1]
    id_tmp_2 = np.argmin(np.abs(deepcopy(time_map) - std_time[id]))
    if np.abs(time_map[id_tmp_2] - std_time[id])>=0.15:
        msg_map[std_time[id]] = None
    else:
        msg_map[std_time[id]] = data_map[id_tmp_2]
with open("data/log1_bsm.obj", "wb") as handle:
    pkl.dump(msg_bsm, handle, protocol=pkl.HIGHEST_PROTOCOL)
with open("data/log1_spat.obj", "wb") as handle:
    pkl.dump(msg_spat, handle, protocol=pkl.HIGHEST_PROTOCOL)

with open("data/log1_map.obj", "wb") as handle:
    pkl.dump(msg_map, handle, protocol=pkl.HIGHEST_PROTOCOL)


# for t, map_msg in msg_map.items():

#     if map_msg is None:
#         continue

#     for inter in map_msg["intersections"]:
#         print(inter["id"]["id"])
for t, spat_msg in msg_spat.items():

    if spat_msg is None:
        continue

    try:
        print(spat_msg["id"]["id"])
    except KeyError:
        pass
    
                    
#mapdata = data_map[8]  # first MAP message
#print('spat',msg_spat[1])

# list how many intersections are inside this MAP
#print("num intersections:", len(mapdata.get("intersections", [])))

# look at the first intersection fields
# inter0 = mapdata["intersections"][0]
# print(inter0.keys())

# # look at the reference point (this is the location)
# print("refPoint:", inter0.get("refPoint", None))