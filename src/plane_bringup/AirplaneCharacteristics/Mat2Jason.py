# Open the MATLAB file that contains the starting position and name of the Airplane:
import scipy.io as spio
import matlab.engine
from launch.substitutions import LaunchConfiguration, TextSubstitution
# Save it in json file:
import json
import os
import glob

######### Detect the matlab files:
# Path to the current file
current_file_path = os.path.dirname(os.path.abspath(__file__))
# Search for the airplane mat existing files:
mat_files_path = glob.glob(os.path.join(current_file_path,"*.mat"))
# Get only the names:
mat_files = [os.path.splitext(os.path.basename(f))[0] for f in mat_files_path]


######## Pass .mat files to .json files:
# Start the engine
eng = matlab.engine.start_matlab()
# open each file
for mat_file in mat_files:
    # Open the matlab file
    structure = eng.load(os.path.join(current_file_path,mat_file))
    # Change rot and rot:
    pose_1 = structure['Airplane']['pose']
    rot_1= structure['Airplane']['rot']
    # Position and rotation:
    pose = [pose_1[0][0], pose_1[0][1], pose_1[0][2]]
    rot = [rot_1[0][0], rot_1[0][1], rot_1[0][2]]

    # Save the information as:
    Airplane = {
        "pose" : pose,
        "rot": rot,
        "robot_name" : structure['Airplane']['name'],
        "robot_imu" : structure['Airplane']['imu'],
    }
    # Define the json_filename
    json_filename = f"{mat_file}.json"
    print(Airplane)

    # Save the dictionary
    with open(os.path.join(current_file_path,json_filename), 'w') as json_file:
        json.dump(Airplane,json_file,indent=4)
        


