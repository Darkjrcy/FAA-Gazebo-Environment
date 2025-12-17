#!/bin/bash
echo "==== Downloading models.zip from Google Drive ===="

# Dowload and unzip the Models folder
# Create the folder (if it already exists, it's fine)
mkdir -p src/plane_bringup/models

# Google Drive file ID: 1612k0ajAOZc07ogzQaVJj58sSxcUIGob
MODELS_ID="1612k0ajAOZc07ogzQaVJj58sSxcUIGob"
gdown --id ${MODELS_ID} -O models.zip

echo "=== Unzipping the files into the models folder ==="
unzip -o models.zip -d src/plane_bringup/models/


# Dowload and unzip the data YOLO folder
# Google Drive file ID: 1chHuow8cOx5b5a7lb4I7q1iILFkzfIup
DATA_ID="1chHuow8cOx5b5a7lb4I7q1iILFkzfIup"
gdown --id ${DATA_ID} -O data.zip

echo "=== Unzipping the files into the data folder ==="
unzip -o data.zip -d src/airplane_recognition/

# Delete the zip files:
echo "=== clean both zip files==="
rm models.zip
rm data.zip
