import numpy as np
import math

def deg_to_rad(deg):
    rad = np.pi*(deg/180)
    return rad

def coord_to_dist(lon1, lat1, lon2, lat2):
    r = 6371e3
    lat_deg_1 = deg_to_rad(lat1)
    lat_deg_2 = deg_to_rad(lat2)
    d_lat = deg_to_rad(lat2-lat1)
    d_lon = deg_to_rad(lon2-lon1)
    a = np.sin(d_lat/2)*np.sin(d_lat/2)+np.cos(lat_deg_1)*np.cos(lat_deg_2)*np.sin(d_lon/2)*np.sin(d_lon/2)
    c = 2*math.atan2(np.sqrt(a), np.sqrt(1-a))
    d = r * c
    return d