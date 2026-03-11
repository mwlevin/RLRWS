from matplotlib.dates import WE
import numpy
import csv

from dist_helper import haversine, haversine_np

def read_csv_file(filename):
    data = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            data.append(row)
    return data


def write_reference_csv_file(filename):

    cordinates = read_csv_file(filename)
    len_cordinates = len(cordinates)

    cum_distance = 0
    for i in range(len_cordinates-2 , 0 , -1):  # start from last data, move backwards
        lat_2 = float(cordinates[i][2])
        long_2 = float(cordinates[i][3])
        
        lat_1 = float(cordinates[i+1][2])
        long_1 = float(cordinates[i+1][3])
        
        distance = abs(haversine(lat_1, long_1, lat_2, long_2))
        cum_distance += distance
        cordinates[i].append(cum_distance)

    print( cordinates[100:])

    with open('ref_3_18CROSS_NE_SW.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['intersection_id', 'approach_id', 'lat', 'lon', 'dis_to_ref'])
        writer.writerows(cordinates)
        
        
        

#### Approach and intersection detection function  ###
## function that gets the current coordinates, plus reference files, 
## returns: best approach to go with that trajectory, and the distance to the reference point.
import numpy as np
import glob
import pandas as pd


def detect_reference_trajectory(live_points, reference_folder, n_points=10):
    """
    live_points: list of (lat, lon)
    reference_folder: folder containing CSV reference trajectories
    n_points: number of initial points to use
    """

    live_points = np.array(live_points[:n_points])

    best_match = None
    best_distance = np.inf

    for file in glob.glob(f"{reference_folder}/*.csv"):

        df = pd.read_csv(file)

        ref_lat = df["lat"].values
        ref_lon = df["lon"].values

        # compute distance from each live point to all ref points
        total_min_dist = 0

        for lat, lon in live_points:
            dists = haversine_np(lat, lon, ref_lat, ref_lon)
            total_min_dist += np.min(dists)

        avg_dist = total_min_dist / len(live_points)

        if avg_dist < best_distance:
            best_distance = avg_dist
            best_match = {
                "file": file,
                "intersection_id": df["intersection_id"].iloc[0],
                "approach_id": df["approach_id"].iloc[0],
                "avg_distance_m": avg_dist
            }

    return best_match
        
        
        
def load_reference_np(csv_file):
    """
    Load reference CSV and return Nx3 numpy array:
    [lat, lon, dis_to_ref]
    """
    df = pd.read_csv(csv_file)
    return df[["lat", "lon", "dis_to_ref"]].values
        
        
### Find cumulative distance to reference point
## inputs: current point lat & long + reference trajectory file

def distance_to_stopbar(current_lat, current_lon, ref_np):
    """
    current_lat, current_lon : float
    ref_np : numpy array (N x 3)

    returns:
        cumulative distance to stop bar (meters)
        index of closest point
    """

    ref_lats = ref_np[:, 0]
    ref_lons = ref_np[:, 1]
    ref_dist = ref_np[:, 2]

    dists = haversine_np(current_lat, current_lon, ref_lats, ref_lons)
    idx = np.argmin(dists)

    return ref_dist[idx]
        
        
        
        
        
        
if __name__ == "__main__":
    # write_reference_csv_file('trajectories_3_NE_SW.csv')
    
    live_points = [
        # (44.7466456, -93.3972273),
        # (44.7466463, -93.3972375),
        # (44.7466462, -93.3972495),
        # (44.7466461, -93.39726139998999),
        # (44.7466461, -93.39727339998999),
        # (44.746646299999995, -93.3972852),
        # (44.746646299999995, -93.3972973),
        # (44.7466459, -93.3973099),
        # (44.7466461, -93.3973223),
        # (44.746646299999995, -93.39733509999999)
        
        (44.7467007,-93.4286005),
        (44.746700999999995,-93.4285838),
        (44.7467015,-93.428567),
        (44.7467014,-93.4285495),
        (44.746700999999995,-93.4285318),
        (44.746701099999996,-93.42851498999999),
        (44.7467019,-93.4284969),
        (44.746702219999995,-93.4284792),
        (44.7467019,-93.42846087),   
        (44.7467026,-93.4284468)
    ]
    
    folder = "traj_ref"
    
    best_ref = detect_reference_trajectory(live_points, folder)
    print("Best matching reference trajectory:", best_ref['file'])
    
    # load the csv file of the best match
    ref_np = load_reference_np(best_ref['file'])
    
    current_lat , current_long = (44.746618, -93.413493)
    
    # find distance
    distanceee = distance_to_stopbar(current_lat, current_long, ref_np)
    print("Distance to stop bar:", distanceee)