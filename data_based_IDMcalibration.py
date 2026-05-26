###    Load vehicle data and use it to calibrate IDM model parameters
## INPUT: Vehicle trajectory data (csv file)
## OUTPUT: Calibrated IDM parameters (5 parameters)


import pandas as pd
from calibrate_gradient_adaptive_step import mainIDM
from constants import IDM_Param, Veh_Parameter


def calibrate_data():
    df = pd.read_csv('data/log1_bsm.csv')

    time = df.iloc[:,0].tolist()
    speed = df.iloc[:,3].tolist()
    lat = df.iloc[:,1].tolist()
    long = df.iloc[:,2].tolist()
    length = len(speed)



    # make spacing value list
    base_pos = 1000
    max_gap = 80
    dt = 0.2
    spacing = []
    acc_ego = []
    for i in range(length):
        distance = find_position(lat[i], long[i])
        if distance > 0:
            spacing.append(distance)
        else:
            spacing.append(max_gap)

        if i < length - 1:
            acc_ego.append((speed[i+1] - speed[i])/dt)
        else:
            acc_ego.append(0)
    # make acceleration value list

    rel_spd = speed



    # load lb and ub
    max_a = IDM_Param().lb_ac*1.2
    min_a = IDM_Param().lb_dec*1.2
    max_v =  IDM_Param().lb_speed*1.2
    min_s =  Veh_Parameter().min_gap*1.2

    params = mainIDM(spacing, speed, rel_spd, acc_ego, max_a, min_a, max_v, min_s)

    return params



from geopy.distance import geodesic

def find_position(lat, lon):
        # handle both numeric and string inputs
    lat = str(lat)
    lon = str(lon)

    # insert decimal after first two digits (for lat) and first three for lon if needed
    if '.' not in lat and len(lat) > 2:
        lat = lat[:2] + '.' + lat[2:]
    if '.' not in lon and len(lon) > 3:
        lon = lon[:3] + '.' + lon[3:]

    # convert back to float
    lat = float(lat)
    lon = float(lon)



    lat_base = 44.771951
    lon_base = -93.431541

    # Base and point
    point1 = (lat_base, lon_base)
    point2 = (lat, lon)

    # Distance in meters
    distance_m = geodesic(point1, point2).meters

    # Determine direction (sign)
    if lon < lon_base:
        distance_m *= -1   # point is "after" (east of base)

    # if distance_m>0:
        # print("Signed Distance (m):", distance_m)
    return distance_m



params = calibrate_data()
print("Calibrated IDM parameters: S0, a, b , T, v0", params )