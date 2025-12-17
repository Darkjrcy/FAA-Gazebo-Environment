import pandas as pd
import csv
import io


# Define the path with the summary of detections of hte 7500 encounters:
encounter_summary_path = "/home/adcl/AirplanePathFollower/src/plane_bringup/Detection_encounters/Detection_process.csv"

# Open the csv file:
df = pd.read_csv(encounter_summary_path)
# Sort by the detection confidance rate
sorted_df = df.sort_values(
    by = "Camera_Confidence_level",
    ascending= False
).reset_index(drop=True)

# Save only the encounters number:
ordered_encounter_numbers = sorted_df["Encounter_number"].tolist()
ordered_encounter_numbers = [int(s.split('_')[-1]) for s in ordered_encounter_numbers]

# Use this encoutners to identify the CPA time:
# ENcounter information summary:
encounter_info_path = "/home/adcl/AirplanePathFollower/src/plane_bringup/Detection_encounters/terminal_encounter_info_20200630.csv"
encounter_info_df = pd.read_csv(encounter_info_path)
encounter_info_df = encounter_info_df.rename(columns=lambda c: c.strip())
encounter_info_df["id"]   = pd.to_numeric(encounter_info_df["id"], errors="coerce").astype("Int64")
encounter_info_df["tcpa"] = pd.to_numeric(encounter_info_df["tcpa"], errors="coerce")

# Left DataFrame = your ordered ids (this preserves order after merge)
left = pd.DataFrame({"id": ordered_encounter_numbers})

# Merge to get tcpa, drop rows with missing tcpa (or keep them if you prefer)
merged = left.merge(encounter_info_df[["id", "tcpa"]], on="id", how="left")
merged = merged.dropna(subset=["tcpa"]).astype({"id": int})

# Save as JSON: [{"id": 3059, "tcpa": 119.0}, ...]
out_json_path = "/home/adcl/AirplanePathFollower/src/plane_bringup/Detection_encounters/encounters_tcpa.json"
merged.to_json(out_json_path, orient="records", indent=2)
print(f"Saved {len(merged)} rows to {out_json_path}")





print(merged)

