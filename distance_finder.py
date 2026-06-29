"""


This will:
  1. Parse "ID3_CSAH4221_WE" -> intersection_id=3, approach='WE'
  2. Look up the stop-bar reference point from `reference_points`
  3. Read the recorded trajectory CSV (default: <test_name>.csv)
  4. Compute cumulative distance to the stop bar (positive upstream of
     the stop bar, negative past it)
  5. Write a clean output CSV named ref_<id>_<approach>.csv

CHANGES vs. the previous version are flagged with "# >>> CHANGE:" comments.
"""

import csv
import os
import re
import glob

import numpy as np
import pandas as pd

from dist_helper import haversine, haversine_np, mph_to_mps


# -------------------------------------------------------------------
# Reference points (stop bars) for every intersection / approach
# -------------------------------------------------------------------
reference_points = {
    (1, 'NESW'): (44.7743737, -93.415009),
    (1, 'NWSE'): (44.7741739, -93.4154772),
    (1, 'SENW'): (44.7739464, -93.4149001),
    (1, 'SWNE'): (44.7738154, -93.4152551),
    (2, 'WE'):   (44.7676009, -93.4363206),
    (2, 'EW'):   (44.7675662, -93.4356173),
    (2, 'NS'):   (44.7677858, -93.4359438),
    (2, 'SN'):   (44.7673645, -93.4360262),
    (3, 'EW'):   (44.7468854, -93.4390971),
    (3, 'NS'):   (44.7470018, -93.4394849),
    (3, 'SN'):   (44.7465673, -93.4392467),
    (3, 'WE'):   (44.746775,  -93.4396939),
    (4, 'EW'):   (44.7467122, -93.4089453),
    (4, 'NS'):   (44.7468446, -93.4093183),
    (4, 'SN'):   (44.7464225, -93.4091228),
    (4, 'WE'):   (44.7465821, -93.4095095),
    (5, 'NESW'): (44.7717258, -93.411433),
    (5, 'NWSE'): (44.771580, -93.411900),
    (5, 'SENW'): (44.7714374, -93.4114366),
    (5, 'SWNE'): (44.7713506, -93.4118074),
}


# define speed limits for every intersection / approach (m/s)
speed_limits = {
    (1, 'NESW'): mph_to_mps(55),
    (1, 'NWSE'): mph_to_mps(40),
    (1, 'SENW'): mph_to_mps(40),
    (1, 'SWNE'): mph_to_mps(55),
    (2, 'WE'):   mph_to_mps(55),
    (2, 'EW'):   mph_to_mps(55),
    (2, 'NS'):   mph_to_mps(55),
    (2, 'SN'):   mph_to_mps(55),
    (3, 'EW'):   mph_to_mps(55),
    (3, 'NS'):   mph_to_mps(55),
    (3, 'SN'):   mph_to_mps(55),
    (3, 'WE'):   mph_to_mps(55),
    (4, 'EW'):   mph_to_mps(55),
    (4, 'NS'):   mph_to_mps(50),
    (4, 'SN'):   mph_to_mps(30),
    (4, 'WE'):   mph_to_mps(55),
    (5, 'NESW'): mph_to_mps(30),
    (5, 'NWSE'): mph_to_mps(50),
    (5, 'SENW'): mph_to_mps(50),
    (5, 'SWNE'): mph_to_mps(30),
}


signal_group_ids = {
    (1, 'NESW'): 58519,
    (1, 'NWSE'): 58519,
    (1, 'SENW'): 58519,
    (1, 'SWNE'): 58519,
    (2, 'WE'):   41831,
    (2, 'EW'):   41831,
    (2, 'NS'):   41831,
    (2, 'SN'):   41831,
    (3, 'EW'):   54154,
    (3, 'NS'):   54154,
    (3, 'SN'):   54154,
    (3, 'WE'):   54154,
    (4, 'EW'):   53852,
    (4, 'NS'):   53852,
    (4, 'SN'):   53852,
    (4, 'WE'):   53852,
    (5, 'NESW'):  52084,
    (5, 'NWSE'):  52084,
    (5, 'SENW'):  52084,
    (5, 'SWNE'):  52084,
}


signal_phase_ids = {
    (1, 'NESW'): 6,
    (1, 'NWSE'): 4,
    (1, 'SENW'): 8,
    (1, 'SWNE'): 2,
    (2, 'WE'):   4,
    (2, 'EW'):   8,
    (2, 'NS'):   6,
    (2, 'SN'):   2,
    (3, 'EW'):   6,
    (3, 'NS'):   4,
    (3, 'SN'):   8,
    (3, 'WE'):   2,
    (4, 'EW'):   6,
    (4, 'NS'):   4,
    (4, 'SN'):   8,
    (4, 'WE'):   2,
    (5, 'NESW'):  8,
    (5, 'NWSE'):  6,
    (5, 'SENW'):  2,
    (5, 'SWNE'):  4,
}

# -------------------------------------------------------------------
# Helpers
# -------------------------------------------------------------------
def read_csv_file(filename):
    data = []
    with open(filename, 'r') as file:
        reader = csv.reader(file)
        for row in reader:
            data.append(row)
    return data


# >>> CHANGE: NEW FUNCTION
# Parses test names like "ID3_CSAH4221_WE" or "ID1_FOO_NESW" into
# (intersection_id, approach). Tolerant of extra underscores in the
# road-name middle segment.
def parse_test_name(test_name):
    """
    Parse a test name of the form 'ID<num>_<road_name>_<approach>'.

    Returns
    -------
    (intersection_id: int, approach: str)
    """
    parts = test_name.split('_')
    if len(parts) < 3:
        raise ValueError(
            f"Invalid test_name '{test_name}'. "
            "Expected format: 'ID<num>_<road_name>_<approach>' "
            "(e.g. 'ID3_CSAH4221_WE')."
        )

    m = re.match(r'^ID(\d+)$', parts[0])
    if not m:
        raise ValueError(
            f"Could not parse intersection id from '{parts[0]}'. "
            "Expected something like 'ID3'."
        )
    intersection_id = int(m.group(1))
    approach = parts[-1]               # last token is always the approach
    return intersection_id, approach


# -------------------------------------------------------------------
# Core: write reference trajectory CSV
# -------------------------------------------------------------------
# >>> CHANGE: signature now also accepts intersection_id / approach so
# the output rows can be re-stamped with the correct IDs (the raw
# trajectory CSV sometimes has these columns blank or wrong).
def write_reference_csv_file(filename,
                             reference_point,
                             output_filename='ref_trajectory.csv',
                             intersection_id=None,
                             approach=None,
                             max_distance_m=1500.0):
    """
    Inputs
    ------
    filename         : recorded trajectory CSV. Expected columns:
                       intersection_id, approach_id, lat, lon, ...
    reference_point  : (lat, lon) tuple of the stop bar
    output_filename  : where to save the reference trajectory
    intersection_id  : optional int, written into column 0 of every row
    approach         : optional str, written into column 1 of every row
    max_distance_m   : only rows with |dis_to_ref| < this value are kept
                       in the output (default 1000 m). Set to None to
                       disable filtering.
    """
    coordinates = read_csv_file(filename)
    len_coordinates = len(coordinates)
 
    # >>> CHANGE: don't append 'dis_to_ref' to the in-memory header any
    # more — we'll write a fresh, clean header at the end. (Old code
    # appended here AND wrote a separate writerow() header, producing a
    # duplicated/garbled header in the output.)
    header = coordinates[0]
    body = coordinates[1:]
    n_body = len(body)
 
    ref_lat, ref_lon = reference_point
 
    # ---- find the row closest to the stop bar -----------------------
    min_distance = float('inf')
    reference_index = 0
    for i, row in enumerate(body):
        lat = float(row[2])
        lon = float(row[3])
        d = haversine(lat, lon, ref_lat, ref_lon)
        if d < min_distance:
            min_distance = d
            reference_index = i
 
    print(f"Reference point found at body index {reference_index}")
    print(f"Closest point: lat={body[reference_index][2]}, "
          f"lon={body[reference_index][3]}")
    print(f"Distance to reference: {min_distance:.2f} m")
 
    # initialise a distance column
    for row in body:
        row.append(0.0)
 
    # ---- BACKWARD: upstream of stop bar -> positive cum distance ----
    cum_distance = 0.0
    for i in range(reference_index - 1, -1, -1):
        lat_2 = float(body[i][2])
        lon_2 = float(body[i][3])
        lat_1 = float(body[i + 1][2])
        lon_1 = float(body[i + 1][3])
        cum_distance += abs(haversine(lat_1, lon_1, lat_2, lon_2))
        body[i][-1] = cum_distance
 
    # stop bar row itself = 0
    body[reference_index][-1] = 0.0
 
    # ---- FORWARD: past the stop bar -> negative cum distance --------
    cum_distance = 0.0
    for i in range(reference_index, n_body - 1):
        lat_2 = float(body[i][2])
        lon_2 = float(body[i][3])
        lat_1 = float(body[i + 1][2])
        lon_1 = float(body[i + 1][3])
        cum_distance += abs(haversine(lat_1, lon_1, lat_2, lon_2))
        body[i + 1][-1] = -cum_distance
 
    print("Cumulative distances calculated.")
 
    # >>> CHANGE: optionally stamp the correct intersection_id /
    # approach into every row, so downstream tools can rely on them.
    if intersection_id is not None and approach is not None:
        for row in body:
            row[0] = intersection_id
            row[1] = approach
 
    # >>> CHANGE: keep only rows whose |dis_to_ref| is below the
    # threshold (default 1000 m). dis_to_ref is positive upstream of the
    # stop bar and negative past it, so we filter on absolute value.
    if max_distance_m is not None:
        before = len(body)
        body = [row for row in body if abs(float(row[-1])) < max_distance_m]
        print(f"Filtered to |dis_to_ref| < {max_distance_m} m: "
              f"{before} -> {len(body)} rows")
 
    # ---- write output -----------------------------------------------
    # >>> CHANGE: write ONE clean header and only the body, no more
    # duplicated header row in the output file.
    with open(output_filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['intersection_id', 'approach_id',
                         'lat', 'lon', 'dis_to_ref'])
        writer.writerows(body)
 
    print(f"Output saved to {output_filename}")
    return output_filename


# -------------------------------------------------------------------
# >>> CHANGE: NEW high-level wrapper - this is the "one call" entry
# point you asked for. Pass in the test name and it does the rest.
# -------------------------------------------------------------------
def build_reference_from_test_name(test_name,
                                   trajectory_csv=None,
                                   output_dir='.'):
    """
    One-shot pipeline.

    Parameters
    ----------
    test_name      : e.g. 'ID3_CSAH4221_WE'
    trajectory_csv : path to the recorded trajectory CSV.
                     Defaults to '<test_name>.csv'.
    output_dir     : folder for the output reference CSV.

    Returns
    -------
    Path to the reference CSV that was written.
    """
    intersection_id, approach = parse_test_name(test_name)

    key = (intersection_id, approach)
    if key not in reference_points:
        raise KeyError(
            f"No reference point defined for intersection "
            f"{intersection_id}, approach '{approach}'. "
            f"Check reference_points or your test_name."
        )
    reference_point = reference_points[key]

    if trajectory_csv is None:
        trajectory_csv = f"{test_name}.csv"

    output_filename = (
        f"{output_dir.rstrip('/').rstrip(chr(92))}"
        f"/ref_{intersection_id}_{approach}.csv"
    )

    print("=" * 60)
    print(f"Test name        : {test_name}")
    print(f"Intersection ID  : {intersection_id}")
    print(f"Approach         : {approach}")
    print(f"Reference (lat,lon): {reference_point}")
    print(f"Trajectory CSV   : {trajectory_csv}")
    print(f"Output CSV       : {output_filename}")
    print("=" * 60)

    return write_reference_csv_file(
        trajectory_csv,
        reference_point,
        output_filename=output_filename,
        intersection_id=intersection_id,
        approach=approach,
    )


# -------------------------------------------------------------------
# Approach / intersection detection 
# -------------------------------------------------------------------
def bearing_deg(lat1, lon1, lat2, lon2):
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    y = np.sin(dlon) * np.cos(lat2)
    x = np.cos(lat1) * np.sin(lat2) - np.sin(lat1) * np.cos(lat2) * np.cos(dlon)
    return (np.degrees(np.arctan2(y, x)) + 360) % 360


def heading_difference_deg(heading_a, heading_b):
    return abs((heading_a - heading_b + 180) % 360 - 180)


def reference_heading_near(ref_lat, ref_lon, idx, window=5):
    start = max(0, idx - window)
    end = min(len(ref_lat) - 1, idx + window)
    if start == end:
        return None
    return bearing_deg(ref_lat[start], ref_lon[start], ref_lat[end], ref_lon[end])


def detect_reference_trajectory(
        live_points,
        reference_folder,
        n_points=5,
        exclude_intersection=None,
        exclude_file=None,
        live_heading_deg=None,
        min_dist_to_stopbar=-5.0,
        max_dist_to_stopbar=1600.0,
        max_avg_distance_m=75.0,
        max_heading_error_deg=60.0,
        allow_gps_heading=True,
        min_gps_heading_distance_m=3.0,
        debug_csv_file=None,
        debug_context=None):
    live_points = np.array(live_points[-n_points:])   # last N points


    best_match = None
    best_distance = np.inf
    debug_rows = []
    debug_context = debug_context or {}
    live_heading = live_heading_deg
    heading_source = "bsm" if live_heading is not None else "none"
    heading_distance = None
    if live_heading is None and allow_gps_heading and len(live_points) >= 2:
        heading_distance = haversine(
            live_points[0, 0], live_points[0, 1],
            live_points[-1, 0], live_points[-1, 1],
        )
        if heading_distance >= min_gps_heading_distance_m:
            live_heading = bearing_deg(
                live_points[0, 0], live_points[0, 1],
                live_points[-1, 0], live_points[-1, 1],
            )
            heading_source = "gps"

    def add_debug_row(file, candidate_intersection=None, candidate_approach=None,
                      avg_dist=None, inferred_last=None, ref_heading=None,
                      heading_error=None, result="", reason="", selected=False):
        if debug_csv_file is None:
            return
        debug_rows.append({
            **debug_context,
            "candidate_ref_file": os.path.basename(file) if file else None,
            "candidate_intersection": candidate_intersection,
            "candidate_approach": candidate_approach,
            "live_heading_deg": live_heading,
            "heading_source": heading_source,
            "gps_heading_distance_m": heading_distance,
            "ref_heading_deg": ref_heading,
            "heading_error_deg": heading_error,
            "heading_limit_deg": max_heading_error_deg,
            "avg_distance_m": avg_dist,
            "inferred_distance_to_stopbar": inferred_last,
            "result": result,
            "reason": reason,
            "selected": selected,
        })

    for file in glob.glob(f"{reference_folder}/*.csv"):
        df = pd.read_csv(file)

        candidate_intersection = int(df["intersection_id"].iloc[0])
        candidate_approach = df["approach_id"].iloc[0]

        if exclude_file is not None and os.path.abspath(file) == os.path.abspath(exclude_file):
            add_debug_row(
                file,
                candidate_intersection,
                candidate_approach,
                result="reject",
                reason="excluded_same_ref_file",
            )
            continue

        if exclude_intersection is not None and candidate_intersection == exclude_intersection:
            add_debug_row(
                file,
                candidate_intersection,
                candidate_approach,
                result="reject",
                reason="excluded_intersection",
            )
            continue

        ref_lat = df["lat"].values
        ref_lon = df["lon"].values
        ref_dis = df["dis_to_ref"].values
        ref_np  = np.column_stack([ref_lat, ref_lon, ref_dis])

        total_min_dist = 0
        for lat, lon in live_points:
            dists = haversine_np(lat, lon, ref_lat, ref_lon)
            total_min_dist += np.min(dists)

        avg_dist = total_min_dist / len(live_points)
        if avg_dist > max_avg_distance_m:
            add_debug_row(
                file,
                candidate_intersection,
                candidate_approach,
                avg_dist=avg_dist,
                result="reject",
                reason="avg_distance_too_large",
            )
            continue
        
        
        # --- NEW: pairwise wrong-direction filter ----------------
        # Compute inferred dis_to_ref for each live point against this candidate's reference, then compare consecutive pairs:
        # A pair is "approaching" if the second value is smaller than the first (minus a small tolerance for GPS noise).
        inferred = np.array([
            distance_to_stopbar(lat, lon, ref_np)
            for lat, lon in live_points])

        if inferred[-1] < min_dist_to_stopbar or inferred[-1] > max_dist_to_stopbar:
            add_debug_row(
                file,
                candidate_intersection,
                candidate_approach,
                avg_dist=avg_dist,
                inferred_last=inferred[-1],
                result="reject",
                reason="distance_to_stopbar_out_of_range",
            )
            continue

        ref_heading = None
        heading_error = None
        if live_heading is not None:
            closest_idx = np.argmin(
                haversine_np(live_points[-1, 0], live_points[-1, 1], ref_lat, ref_lon)
            )
            ref_heading = reference_heading_near(ref_lat, ref_lon, closest_idx)
            if ref_heading is not None:
                heading_error = heading_difference_deg(live_heading, ref_heading)
                if heading_error > max_heading_error_deg:
                    add_debug_row(
                        file,
                        candidate_intersection,
                        candidate_approach,
                        avg_dist=avg_dist,
                        inferred_last=inferred[-1],
                        ref_heading=ref_heading,
                        heading_error=heading_error,
                        result="reject",
                        reason="heading_error_too_large",
                    )
                    continue
        
        # skip if the last 3 inferred distances are NOT consistently decreasing
        if (len(inferred) >= 3 and
                inferred[-1] > inferred[-2] and
                inferred[-2] > inferred[-3]):
            add_debug_row(
                file,
                candidate_intersection,
                candidate_approach,
                avg_dist=avg_dist,
                inferred_last=inferred[-1],
                ref_heading=ref_heading,
                heading_error=heading_error,
                result="reject",
                reason="distance_increasing_wrong_direction",
            )
            continue

        add_debug_row(
            file,
            candidate_intersection,
            candidate_approach,
            avg_dist=avg_dist,
            inferred_last=inferred[-1],
            ref_heading=ref_heading,
            heading_error=heading_error,
            result="accept",
            reason="passed_all_filters",
        )
        
        if avg_dist < best_distance:
            best_distance = avg_dist
            best_match = {
                "file": file,
                "intersection_id": candidate_intersection,
                "approach_id": candidate_approach,
                "avg_distance_m": avg_dist,
            }

    if debug_csv_file is not None and debug_rows:
        selected_file = os.path.abspath(best_match["file"]) if best_match is not None else None
        for row in debug_rows:
            candidate_file = row.get("candidate_ref_file")
            if selected_file is not None and candidate_file:
                row["selected"] = os.path.basename(selected_file) == candidate_file
        fieldnames = [
            "timestamp",
            "idx",
            "detect_stage",
            "current_ref_file",
            "current_intersection",
            "current_approach",
            "speed_mps",
            "exclude_ref_file",
            "candidate_ref_file",
            "candidate_intersection",
            "candidate_approach",
            "live_heading_deg",
            "heading_source",
            "gps_heading_distance_m",
            "ref_heading_deg",
            "heading_error_deg",
            "heading_limit_deg",
            "avg_distance_m",
            "inferred_distance_to_stopbar",
            "result",
            "reason",
            "selected",
        ]
        file_exists = os.path.exists(debug_csv_file)
        with open(debug_csv_file, mode="a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            if not file_exists:
                writer.writeheader()
            writer.writerows(debug_rows)

    return best_match


def load_reference_np(csv_file):
    """Load reference CSV -> Nx3 array [lat, lon, dis_to_ref]."""
    df = pd.read_csv(csv_file)
    return df[["lat", "lon", "dis_to_ref"]].values


def distance_to_stopbar(current_lat, current_lon, ref_np):
    """Inverse-distance-weighted interpolation of dist-to-stopbar."""
    ref_lats = ref_np[:, 0]
    ref_lons = ref_np[:, 1]
    ref_dist = ref_np[:, 2]

    dists = haversine_np(current_lat, current_lon, ref_lats, ref_lons)
    idx1, idx2 = np.argsort(dists)[:2]
    d1, d2 = dists[idx1], dists[idx2]
    if d1 + d2 == 0:
        return ref_dist[idx1]
    w1 = d2 / (d1 + d2)
    w2 = d1 / (d1 + d2)
    return w1 * ref_dist[idx1] + w2 * ref_dist[idx2]


def ffspd_finder(matched_ref):
    """Free-flow speed lookup (m/s)."""

    id = matched_ref["intersection_id"]
    approach = matched_ref["approach_id"]


    return speed_limits.get((id, approach))


# find the phase id for the detected intersection and approach
def phaseid_finder(matched_ref):
    """Free-flow speed lookup (m/s)."""

    id = matched_ref["intersection_id"]
    approach = matched_ref["approach_id"]


    return signal_phase_ids.get((id, approach))


# find the phase id for the detected intersection and approach
def signalid_finder(matched_ref):
    """intersection id."""

    id = matched_ref["intersection_id"]
    approach = matched_ref["approach_id"]

    return signal_group_ids.get((id, approach))



# -------------------------------------------------------------------
# Demo
# -------------------------------------------------------------------
def main():
    import os
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    reference_folder = os.path.join(SCRIPT_DIR, "traj_ref")
 
    # 50 (lat, lon) points on a straight line from start to end
    drive = [
        (44.746648, -93.406327),
        (44.746652593, -93.406996102),
        (44.746657186, -93.407665204),
        (44.746661779, -93.408334306),
        (44.746666372, -93.409003408),
        (44.746670965, -93.409672510),
        (44.746675559, -93.410341612),
        (44.746680152, -93.411010714),
        (44.746684745, -93.411679816),
        (44.746689338, -93.412348918),
        (44.746693931, -93.413018020),
        (44.746698524, -93.413687122),
        (44.746703117, -93.414356224),
        (44.746707711, -93.415025327),
        (44.746712304, -93.415694429),
        (44.746716897, -93.416363531),
        (44.746721490, -93.417032633),
        (44.746726083, -93.417701735),
        (44.746730676, -93.418370837),
        (44.746735269, -93.419039939),
        (44.746739862, -93.419709041),
        (44.746744456, -93.420378143),
        (44.746749049, -93.421047245),
        (44.746753642, -93.421716347),
        (44.746758235, -93.422385449),
        (44.746762828, -93.423054551),
        (44.746767421, -93.423723653),
        (44.746772014, -93.424392755),
        (44.746776608, -93.425061857),
        (44.746781201, -93.425730959),
        (44.746785794, -93.426400061),
        (44.746790387, -93.427069163),
        (44.746794980, -93.427738265),
        (44.746799573, -93.428407367),
        (44.746804166, -93.429076469),
        (44.746808759, -93.429745571),
        (44.746813353, -93.430414673),
        (44.746817946, -93.431083776),
        (44.746822539, -93.431752878),
        (44.746827132, -93.432421980),
        (44.746831725, -93.433091082),
        (44.746836318, -93.433760184),
        (44.746840911, -93.434429286),
        (44.746845505, -93.435098388),
        (44.746850098, -93.435767490),
        (44.746854691, -93.436436592),
        (44.746859284, -93.437105694),
        (44.746863877, -93.437774796),
        (44.746868470, -93.438443898),
        (44.746873000, -93.439107000),
    ]
 
    SWITCH_EVERY_N = 20        # re-detect every 20 samples
    NEG_DIST_THRESHOLD = -0.5  # re-detect once distance drops below this
    LOOKAHEAD_POINTS = 10      # most-recent samples passed to the detector
 
    current_ref_file = None
    current_ref_np   = None
    current_intersection = None         # NEW
    samples_since_detect = 0
    speed_limit = 0
    live_buffer = []

    for i, (lat, lon) in enumerate(drive):
        live_buffer.append((lat, lon))

        need_detect = False
        exclude_id  = None              # NEW
        if current_ref_file is None:
            need_detect = True
        elif samples_since_detect >= SWITCH_EVERY_N:
            need_detect = True
        else:
            d_peek = distance_to_stopbar(lat, lon, current_ref_np)
            if d_peek < NEG_DIST_THRESHOLD:
                need_detect = True
                exclude_id  = current_intersection   # NEW: don't re-pick this one

        if need_detect:
            recent = live_buffer[-LOOKAHEAD_POINTS:]
            match = detect_reference_trajectory(
                recent, reference_folder,
                n_points=len(recent),
                exclude_intersection=exclude_id,     # NEW
            )
            if match is not None and match["file"] != current_ref_file:
                current_ref_file     = match["file"]
                current_ref_np       = load_reference_np(current_ref_file)
                current_intersection = match["intersection_id"]   # NEW
                speed_limit = ffspd_finder(match)
            samples_since_detect = 0

        d = distance_to_stopbar(lat, lon, current_ref_np)
        print(f"{i:3d}  ref={os.path.basename(current_ref_file)}  dist={d:8.2f} m speed_limit={speed_limit:.1f} m/s")
        samples_since_detect += 1
 

# if __name__ == "__main__":
#     # >>> CHANGE: example end-to-end run driven by a test_name.
#     # test_name = "ID5_CSAH18CROSS_NESW"     # ID<id>_<road>_<approach>
#     main()

    # If your trajectory CSV is named differently than '<test_name>.csv'
    # pass it explicitly:
    #   build_reference_from_test_name(test_name,
    #                                  trajectory_csv='trajectories_3_WE.csv')
    # build_reference_from_test_name(test_name)
