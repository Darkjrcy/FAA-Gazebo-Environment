import enum
from logging.config import valid_ident
from math import pi
import pandas as pd 
import json
import numpy as np
import os

# Things you need to change depending on the camera characteristics
v_fov = 50 # Vertical field of view of the camera in case it was chanegd
positions_points_path = "/home/adcl/AirplanePathFollower/DATA/MIT_Recognition/Encounters_2501-5000/filtered_points.json" # json file of the data of position and attitude of airplanes
detection_path = "/home/adcl/AirplanePathFollower/DATA/MIT_Recognition/Encounters_2501-5000/Exp_Fog_0.01_Camera_Noise_0.05/detection_data"
detection_file = os.path.join(detection_path,"Detection_process.csv")
detection_df = pd.read_csv(detection_file) # Detection Process csv file that wants to be cleaned
os.path.join(detection_path,"Detection_process_without_false_positives.csv")
output_cleaned_csv_path = os.path.join(detection_path,"Detection_process_without_false_positives.csv")
false_negatives_output_path = os.path.join(os.path.dirname(output_cleaned_csv_path),"false_positives_log.json")

# Camera limits to see if the detection corresponds to the camera 
min_camera_limits = [-25.714,-128.5714,-180,-282.8571,-334.2857]
max_camera_limits = [25.714,-25.714,-128.5714,-180,-282.8571]

# Open and load the JSON file
with open(positions_points_path, 'r') as file:
    data = json.load(file)

# Define the encounter number
Ecounters  = data["encounter_number"]
# Save Each point of the own ship:
x_own = data["x_own"]
y_own = data["y_own"]
z_own = data["z_own"]
roll_own = data["roll_own"]
pitch_own = data["pitch_own"]
yaw_own = data["yaw_own"]
# Save Each point of the own intruder:
x_in = data["x_in"]
y_in = data["y_in"]
z_in = data["z_in"]

# Save the cameras_where each position is going to be visible:
cameras_vis = []

# Helper: Convert degrees to radians
deg2rad = lambda deg: deg * np.pi / 180.0

# Start to see the encounter bounds:
for i in range(len(Ecounters)):
    own_pos = np.array([x_own[i]*0.3048, y_own[i]*0.3048, z_own[i]*0.3048])
    in_pos = np.array([x_in[i]*0.3048, y_in[i]*0.3048, z_in[i]*0.3048])
    vec = in_pos - own_pos  # Vector from ownship to intruder

    # Update the roll, pitch, yaw:
    yaw = yaw_own[i]
    pitch = -pitch_own[i]
    roll = roll_own[i]

    # Build rotation matrix (ZYX convention)
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw),  np.cos(yaw), 0],
        [0,            0,           1]
    ])
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0,             1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    Rx = np.array([
        [1, 0,            0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll),  np.cos(roll)]
    ])

    # Full rotation matrix
    R = Rz @ Ry @ Rx  # World ← Camera
    vec_local = R.T @ vec  # Rotate world vector into camera frame

    # Compute azimuth & elevation in camera frame
    x, y, z = vec_local
    az = np.arctan2(y, x) * 180 / np.pi   # degrees
    el = np.arctan2(z, np.sqrt(x**2 + y**2)) * 180 / np.pi  # degrees

    visible_cameras = []  # Store multiple cameras per point

    for j in range(len(min_camera_limits)):
        az_min = min_camera_limits[j] - 7
        az_max = max_camera_limits[j] + 7
        el_min = -v_fov/2 - 0.5
        el_max = v_fov/2 + 0.5

        az_wrapped = (az + 360) % 360
        az_min_wrapped = (az_min + 360) % 360
        az_max_wrapped = (az_max + 360) % 360

        if az_min_wrapped < az_max_wrapped:
            in_az = az_wrapped >= az_min_wrapped and az_wrapped <= az_max_wrapped
        else:
            in_az = az_wrapped >= az_min_wrapped or az_wrapped <= az_max_wrapped

        in_el = el >= el_min and el <= el_max

        if in_az and in_el:
            visible_cameras.append(j + 1)  # Add camera ID (1-indexed)

    cameras_vis.append(visible_cameras)

# Save teh encountered false positivies in a json file
false_positives = []

# Loop through each position
for i in range(len(Ecounters)):
    visible = cameras_vis[i]  # Cameras that should see the target
    row = detection_df.iloc[i]

    for cam_id in range(1, 6):  # Cameras 1 through 5
        conf_col = f"Camera{cam_id}_Confidence_level"
        if conf_col in row:
            confidence = row[conf_col]

            # A detection was reported
            if confidence > 0:
                # Case 1: No camera should have seen it
                # Case 2: This camera shouldn't have seen it
                if (not visible) or (cam_id not in visible):
                    false_positives.append({
                        "encounter": Ecounters[i],
                        "Row_number": i+2,
                        "camera": cam_id,
                        "confidence": confidence,
                        "visible_cameras": visible
                    })

print(false_positives)
# Save the false positives json file:
with open(false_negatives_output_path, 'w') as f:
    json.dump(false_positives, f, indent=4)


# Now using the Deetection process file obtian the rows that shouldn't have 
cleaned_df = detection_df.copy()

for i,visible in enumerate(cameras_vis):
    has_valid_detection =False

    for cam_id in range(1,6):
        conf_col = f"Camera{cam_id}_Confidence_level"
        if conf_col in cleaned_df.columns:
            confidence = cleaned_df.at[i, conf_col]
            if confidence>0:
                if cam_id not in visible:
                    cleaned_df.at[i,conf_col]=0
                else:
                    has_valid_detection = True
    
    if not has_valid_detection:
        for suffix in ["Preprocess_time_ms", "Inference_time_ms", "Postprocess_time_ms"]:
            time_col = suffix
            if time_col in cleaned_df.columns:
                cleaned_df.at[i, time_col] = 0


# Export the cleanend version without the false positives.
cleaned_df.to_csv(output_cleaned_csv_path, index=False)
        





