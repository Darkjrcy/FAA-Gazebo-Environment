import pandas as pd 

path_name = "/home/adcl/AirplanePathFollower/DATA/MIT_Recognition/Encounters_2501-5000/Exp_Fog_0.01_Camera_Noise_0.05/detection_data/Detection_process.csv"
df = pd.read_csv(path_name)
confidence_cols = [
    "Camera1_Confidence_level",
    "Camera2_Confidence_level",
    "Camera3_Confidence_level",
    "Camera4_Confidence_level",
    "Camera5_Confidence_level"
]

df[confidence_cols] =df[confidence_cols].astype('float')

# Find rows where more than one camera has a non-zero confidence level
multi_detect_rows = df[df[confidence_cols].gt(0).sum(axis=1) > 1]

# Print them
print(multi_detect_rows["Encounter_number"])

# Function to find all the rows with more than one confidence level:
def retain_max_confidence(row):
    conf_values = row[confidence_cols].astype(float)  # ensure numeric
    max_conf_col = conf_values.idxmax()  # get column with highest confidence
    for col in confidence_cols:
        if col != max_conf_col:
            row[col] = 0.0  # zero out other confidence values
    return row

# Apply the function to every row
df = df.apply(retain_max_confidence, axis=1)

# Threshold for filtering
threshold = 0.55

# Columns to reset when confidence is too low
extra_cols_to_erase = [
    "Preprocess_time_ms",
    "Inference_time_ms",
    "Postprocess_time_ms"
]

# Combine confidence + time columns
cols_to_erase = confidence_cols + extra_cols_to_erase

# Function to zero out all confidence values if none are >= threshold
def erase_low_confidence(row):
    if (row[confidence_cols] >= threshold).any():
        return row  # keep row as is
    else:
        for col in cols_to_erase:
            row[col] = 0.0  # erase all confidence values
        return row

# Apply the function
df = df.apply(erase_low_confidence, axis=1)


# Save the result (optional)
df.to_csv(path_name)