import socket
from pycmssdk import FacMsgType, FacNotifData,create_cms_api
from pycmssdk import Asn1Type, FacMsgType, FacNotifData, asn1_decode, create_cms_api

# from pycmssdk.asn1 import Asn1Type

# from pycmssdk.asn1 import asn1_decode

from distance_finder import *

from time_convert import *
import pickle as pkl
import numpy as np
from cav_data import *
from estimation_param import *
# import keyboard

with open("map/cum_distance.obj", "rb") as handle:
    cum_dist_list = pkl.load(handle)
    
with open("map/list_lat.obj", "rb") as handle:
    lat_list = pkl.load(handle)
    
with open("map/list_long.obj", "rb") as handle:
    long_list = pkl.load(handle)
    


# ---- reference-trajectory state ----------------------------------------
folder              = "traj_ref"
LOOKAHEAD_POINTS    = 5        # bootstrap buffer size for first detect
SWITCH_EVERY_N      = 10        # re-detect every N BSM samples
NEG_DIST_THRESHOLD  = -0.5      # re-detect once we pass the stop bar
PASS_NEAR_STOP_DIST = 8.0       # near-zero distance where GPS can flip sign
PASS_AWAY_DIST      = 80.0      # after passing, old ref may rise from 0 again
PASS_INCREASE_DELTA = 2.0       # increase from near-zero means moving away
PASS_MIN_SPEED      = 5.0       # m/s; ignore stopped/creeping GPS jitter
REF_SWITCH_MIN_SPEED = 5.0      # m/s; do not switch references while stopped
HEADING_MIN_SPEED    = 2.0      # m/s; below this, heading is often stale/noisy
POST_PASS_HEADING_ERROR_DEG = 25.0  # stricter heading check after passing

live_points          = []        # rolling buffer of recent (lat,lon)
current_ref_file     = None      # path to active reference CSV
current_ref_np       = None      # cached Nx3 array for that CSV
current_intersection = None      # active intersection id (for exclusion)
current_approach     = None      # active approach (for ffspd lookup)
ff_speed             = None      # current free-flow speed (m/s)
samples_since_detect = 0         # counter for the every-N rule
bootstrap_done       = False     # have we made the very first detection?
trend_ref_file       = None      # reference used by pass-trend state
prev_cum_dist        = None      # previous raw distance on active reference
near_stop_seen       = False     # have we recently been near the stop bar?
passed_intersection  = None      # suppress old intersection until a new one is found
passed_ref_file      = None      # suppress only the exact reference just passed
idx = 0
# ------------------------------------------------------------------------


# just for csv file reprot
import csv
import os
import time

csv_file = "log.csv"
detection_debug_file = "detection_debug4.csv"

# write header only if file doesn't exist yet
if not os.path.exists(csv_file):
    with open(csv_file, mode="w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["ref_file", "intersection", "approach", "ff_speed", "cum_dist"])

# spat
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
signal_id = 0

signal_status = None
cum_dist = 0
speed = 0
neg_dist_counter = 0
last_missing_spat_log_time = 0
last_no_status_log_time = 0
last_spat_message_time = 0
last_spat_problem = "no SPaT decoded yet"
last_selected_spat_log_time = 0

# signal_id = 2
phaseid = 2
# added for refernce
dist_counter = 0
live_points = []

def find_signal_data(states, phase_id):
    for state in states:
        if state.get("signalGroup") == phase_id:
            return state["state-time-speed"][0]
    return None

def find_intersection(intersections, target_id):
    if isinstance(intersections, dict):
        intersections = [intersections]
    for intersection in intersections:
        if intersection.get("id", {}).get("id") == target_id:
            return intersection
    return None

def intersection_ids(intersections):
    if isinstance(intersections, dict):
        intersections = [intersections]
    return [
        intersection.get("id", {}).get("id")
        for intersection in intersections
        if isinstance(intersection, dict)
    ]

def print_signal_groups(spat_data, selected_phase_id):
    print(
        "[SPAT] intersection",
        spat_data.get("id", {}).get("id"),
        "selected signalGroup",
        selected_phase_id,
        "reference",
        current_intersection,
        current_approach,
    )
    for state in spat_data.get("states", []):
        signal_data = state["state-time-speed"][0]
        print(
            "[SPAT] signalGroup",
            state.get("signalGroup"),
            "eventState",
            signal_data.get("eventState"),
        )

def signal_group_summary(spat_data):
    summary = []
    for state in spat_data.get("states", []):
        signal_data = state["state-time-speed"][0]
        summary.append(
            f"{state.get('signalGroup')}={signal_data.get('eventState')}"
        )
    return ", ".join(summary)

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
            try:
                decoded_message = asn1_decode(newestData, Asn1Type.US_MESSAGE_FRAME)

                if decoded_message is None:
                    continue

                if decoded_message["value"][0] == "SPAT":
                    last_spat_message_time = time.time()
                    # find intersection id
                    
                    # print(" MM ", decoded_message["value"])
                    intersections = decoded_message["value"][1]["intersections"]
                    SPaT_data = find_intersection(intersections, signal_id)
                    #print(" recieving SPAT is ", SPaT_data['id']['id'])
                    #print("detected id is " , signal_id)
                    # print("spat signal id ", decoded_message["value"][1]["intersections"]["id"])
                    if SPaT_data is not None: # check if correct signal
                        moy = SPaT_data["moy"]
                        moy_hour, moy_minute = moy_to_hour_min(moy)
                        time_stamp = SPaT_data["timeStamp"]
                        sec_of_minute = ms_to_sec(time_stamp)
                        signal_data = find_signal_data(SPaT_data["states"], phaseid)
                        if signal_data is None:
                            last_spat_problem = (
                                f"signalGroup {phaseid} not found in intersection {signal_id}"
                            )
                            continue
                        now = time.time()
                        if now - last_selected_spat_log_time >= 1:
                            last_selected_spat_log_time = now
                            print(
                                "[SPAT] selected intersection",
                                signal_id,
                                "signalGroup",
                                phaseid,
                                "eventState",
                                signal_data.get("eventState"),
                                "reference",
                                current_ref_file,
                            )
                            print("[SPAT] all signalGroups:", signal_group_summary(SPaT_data))
                        if signal_data["eventState"] in red_status:
                            # print("signal is red")
                            signal_status = "red"
                        elif signal_data["eventState"] in yellow_status:
                            # print("signal is yellow")
                            signal_status = "yellow"
                        elif signal_data["eventState"] in green_status:
                            # print("signal is green")
                            signal_status = "green"
                        else:
                            last_spat_problem = (
                                f"unknown eventState {signal_data['eventState']}"
                            )
                            continue
                      #  print(" got SPAT in intersection " , signal_id , " status is ", signal_status)
                        min_end_time = signal_data["timing"]["minEndTime"]
                        max_end_time = signal_data["timing"]["maxEndTime"]
                        minute_min, sec_min = tenth_s_to_min_sec(min_end_time)
                        minute_max, sec_max = tenth_s_to_min_sec(max_end_time)
                        sec_to_change_min = calc_change_time(minute_min, sec_min, moy_minute, sec_of_minute)
                        
                        
                        
                        
                        # print("min time to change the status is after ", sec_to_change_min , " sec")     
                        sec_to_change_max = calc_change_time(minute_max, sec_max, moy_minute, sec_of_minute)
                        # print("max time to change the status is after ", sec_to_change_max , " sec")
                        # 
                    else:
                        last_spat_problem = (
                            f"intersection {signal_id} not found; "
                            f"available ids {intersection_ids(intersections)}"
                        )
                        now = time.time()
                        if now - last_missing_spat_log_time >= 5:
                            last_missing_spat_log_time = now
                            print(
                                "[SPAT] target intersection id not found:",
                                signal_id,
                                "available ids:",
                                intersection_ids(intersections),
                                "reference:",
                                current_intersection,
                                current_approach,
                            )
            except Exception as e:
                last_spat_problem = f"SPaT decode/parse error: {e}"
                continue       
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
            try:
                decoded_message = asn1_decode(newestData, Asn1Type.US_MESSAGE_FRAME)

                if decoded_message is None:
                    continue


                # print(" recieving BSMMM")
                # decoded_message = asn1_decode(newestData, Asn1Type.US_MESSAGE_FRAME)

                if decoded_message["value"][0] == "BasicSafetyMessage":
                    core_data = decoded_message["value"][1]["coreData"]
                    latitude  = core_data["lat"]  * 1e-7
                    longitude = core_data["long"] * 1e-7
                    speed     = core_data["speed"] * 0.02
                    heading_raw = core_data.get("heading")
                    heading_deg = None
                    if heading_raw is not None and heading_raw != 28800:
                        heading_deg = (heading_raw * 0.0125) % 360
                    if abs(latitude -44.7740724)<=1e-5 and abs(longitude -(-93.414747))<=1e-5:
                        # print('latitude',latitude)
                        # print('longitude',longitude)
                        continue
                    live_points.append((latitude, longitude))
                    # keep the rolling buffer bounded
                    if len(live_points) > LOOKAHEAD_POINTS:
                        live_points = live_points[-LOOKAHEAD_POINTS-2:]

                    cum_dist = 1500  # default if we can't compute yetç
                    print(latitude)
                    print(longitude,"long")

                    # -------- BOOTSTRAP: collect first LOOKAHEAD_POINTS samples ----------
                    print(" len live is ", len(live_points), " boot ",  bootstrap_done)
                    if not bootstrap_done:
                        print(len(live_points) , " and ", LOOKAHEAD_POINTS)
                        best_ref = detect_reference_trajectory(
                            live_points,
                            folder,
                            n_points=LOOKAHEAD_POINTS,
                            live_heading_deg=heading_deg if speed >= HEADING_MIN_SPEED else None,
                            allow_gps_heading=speed >= HEADING_MIN_SPEED,
                            debug_csv_file=detection_debug_file,
                            debug_context={
                                "timestamp": time.time(),
                                "idx": idx,
                                "detect_stage": "bootstrap",
                                "current_ref_file": os.path.basename(current_ref_file) if current_ref_file else None,
                                "current_intersection": current_intersection,
                                "current_approach": current_approach,
                                "speed_mps": speed,
                                "exclude_ref_file": None,
                            },
                        )
                        if best_ref is not None:
                            current_ref_file     = best_ref["file"]
                            current_ref_np       = load_reference_np(current_ref_file)
                            current_intersection = best_ref["intersection_id"]
                            current_approach     = best_ref["approach_id"]
                            ff_speed             = ffspd_finder(best_ref)
                            phaseid              = phaseid_finder(best_ref)
                            signal_id            = signalid_finder(best_ref)
                            bootstrap_done       = True
                            samples_since_detect = 0
                            # print(f"[init] ref={os.path.basename(current_ref_file)} "
                            #     f"int={current_intersection} app={current_approach} "
                            #     f"ffspd={ff_speed}")
                        # else:
                            # print("[init] no reference matched yet, still buffering")
                    # not enough points yet: just continue feeding the buffer
                    else:
                        # -------- STEADY STATE: decide if we need to re-detect -----------
                        cum_dist = distance_to_stopbar(latitude, longitude, current_ref_np)
                        raw_cum_dist = cum_dist

                        if trend_ref_file != current_ref_file:
                            trend_ref_file = current_ref_file
                            prev_cum_dist = None
                            near_stop_seen = False

                        if 0 <= raw_cum_dist <= PASS_NEAR_STOP_DIST:
                            near_stop_seen = True

                        passed_by_trend = (
                            near_stop_seen
                            and prev_cum_dist is not None
                            and speed >= PASS_MIN_SPEED
                            and raw_cum_dist <= PASS_AWAY_DIST
                            and raw_cum_dist > prev_cum_dist + PASS_INCREASE_DELTA
                        )

                        need_detect = False
                        exclude_id  = None
                        exclude_ref_file = None
                        if samples_since_detect >= SWITCH_EVERY_N:
                            need_detect = True              # periodic refresh
                         
                        # cum distance is 1000 when switching to a new intersection for 5 seocnds    
                        if (cum_dist < NEG_DIST_THRESHOLD or passed_by_trend) and neg_dist_counter <= 0:
                            if passed_by_trend:
                                print(
                                    "[OBU] passed stop bar by trend:",
                                    os.path.basename(current_ref_file) if current_ref_file else None,
                                    "prev",
                                    round(prev_cum_dist, 2),
                                    "now",
                                    round(raw_cum_dist, 2),
                                    "speed",
                                    round(speed, 2),
                                )
                            passed_intersection = current_intersection
                            passed_ref_file = current_ref_file
                            neg_dist_counter = 20
                            cum_dist = 1000 # reset to a large distance to avoid multiple triggers
                            need_detect = True              # we've passed the stop bar
                            exclude_ref_file = passed_ref_file   # don't re-pick same approach
                            near_stop_seen = False
                        if passed_intersection == current_intersection:
                            cum_dist = 1000
                            need_detect = True
                            exclude_ref_file = passed_ref_file
                        if neg_dist_counter > 0:
                            cum_dist = 1000 # reset to a large distance to avoid multiple triggers
                            neg_dist_counter -= 1            # count down until we can detect again

                        can_detect = (
                            need_detect
                            and (
                                speed >= REF_SWITCH_MIN_SPEED
                                or (
                                    passed_intersection is not None
                                    and passed_intersection == current_intersection
                                )
                                or exclude_id is not None
                                or exclude_ref_file is not None
                            )
                        )
                        switched_ref = False
                        if can_detect:
                            heading_error_limit = (
                                POST_PASS_HEADING_ERROR_DEG
                                if exclude_ref_file is not None
                                else 60.0
                            )
                            best_ref = detect_reference_trajectory(
                                live_points, folder,
                                n_points=len(live_points),
                                exclude_intersection=exclude_id,
                                exclude_file=exclude_ref_file,
                                live_heading_deg=heading_deg if speed >= HEADING_MIN_SPEED else None,
                                max_heading_error_deg=heading_error_limit,
                                allow_gps_heading=speed >= HEADING_MIN_SPEED,
                                debug_csv_file=detection_debug_file,
                                debug_context={
                                    "timestamp": time.time(),
                                    "idx": idx,
                                    "detect_stage": "steady",
                                    "current_ref_file": os.path.basename(current_ref_file) if current_ref_file else None,
                                    "current_intersection": current_intersection,
                                    "current_approach": current_approach,
                                    "speed_mps": speed,
                                    "exclude_ref_file": os.path.basename(exclude_ref_file) if exclude_ref_file else None,
                                },
                            )
                            if best_ref is not None and best_ref["file"] != current_ref_file:
                                current_ref_file     = best_ref["file"]
                                current_ref_np       = load_reference_np(current_ref_file)
                                current_intersection = best_ref["intersection_id"]
                                current_approach     = best_ref["approach_id"]
                                ff_speed             = ffspd_finder(best_ref)
                                phaseid              = phaseid_finder(best_ref)
                                signal_id            = signalid_finder(best_ref)
                                switched_ref         = True
                                passed_intersection  = None
                                passed_ref_file      = None
                                neg_dist_counter     = 0
                                trend_ref_file       = current_ref_file
                                prev_cum_dist        = None
                                near_stop_seen       = False
                                # recompute distance against the new reference
                                cum_dist = distance_to_stopbar(latitude, longitude, current_ref_np)
                                
                                
                                
                                # print(f"[switch] ref={os.path.basename(current_ref_file)} "
                                #     f"int={current_intersection} app={current_approach} "
                                #     f"ffspd={ff_speed} dist={cum_dist:.2f}")
                                
                                # save in csv for record
                                with open(csv_file, mode="a", newline="") as f:
                                    writer = csv.writer(f)
                                    writer.writerow([
                                        idx,
                                        os.path.basename(current_ref_file) if current_ref_file else "None",
                                        current_intersection,
                                        current_approach,
                                        ff_speed,
                                        round(cum_dist, 2)
                                    ])
                            samples_since_detect = 0

                        samples_since_detect += 1
                        if not switched_ref:
                            prev_cum_dist = raw_cum_dist

                    idx += 1
                    # print(f"idx={idx}  ref={os.path.basename(current_ref_file) if current_ref_file else 'None'}  "
                    #     f"dist={cum_dist:.2f}  ffspd={ff_speed}")
                                
                
                # index_pos = np.argmin((lat_list - latitude)**2 + (long_list-longitude)**2)
                # cum_dist = cum_dist_list[index_pos][0]        
            except Exception as e:
                # print(f"[decode error] {e}")
                continue
    
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
                if step_to_change+20<=pred_horizon:
                    # yellow is 3 s, 15 step
                    predicted_state[step_to_change:step_to_change+20,0] = -1
                    predicted_state[step_to_change+20:,0] = 0
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
                predicted_state[step_to_change:,0] = 1
    cav_data = CavData(
        predicted_state,
        cum_dist,
        speed,
        ff_speed,
        os.path.basename(current_ref_file) if current_ref_file else None,
        current_intersection,
        current_approach,
    )
    print('distance', cum_dist,  ' reference ', current_ref_file)
    if signal_status is not None:
       print('signal status', signal_status)
       # print('min change', sec_to_change_min)
        # print('max change',sec_to_change_max)
    else:
        now = time.time()
        if now - last_no_status_log_time >= 5:
            last_no_status_log_time = now
            if last_spat_message_time == 0:
                print("[SPAT] signal status unavailable: no SPaT decoded yet")
            else:
                print("[SPAT] signal status unavailable:", last_spat_problem)
    cav_data_obj = pkl.dumps(cav_data)
    sock_send.sendto(cav_data_obj, (HOST, PORT))
