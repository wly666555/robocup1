#!/bin/bash

cd `dirname $0`

pt_file=./model/det.pt

real_pt_file=$(realpath "$pt_file")

mkdir -p exported_model
exported_model_path=$(pwd)/exported_model
echo $exported_model_path

# Check if conda environment 'trt' exists
if conda env list | grep -q '^trt'; then
    echo "Conda environment 'trt' exists."
else
    echo "Creating conda environment 'trt' from requirements.txt..."
    conda create -y -n trt --file model/requirements.txt -c pytorch -c conda-forge
fi

# Activate the 'trt' environment
source ~/miniconda3/bin/activate trt
# pip install -r scripts/model/requirements.txt
conda info --env

echo 'start wts converting'
# convert pt file to wts script
python3 model/gen_wts.py -w $pt_file -o $exported_model_path/model.wts -t detect
echo 'finish wts converting'

# convert wts to engine
source ../../install/setup.bash
ros2 run vision yolov8_det -s $exported_model_path/model.wts $exported_model_path/model.engine s


