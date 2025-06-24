# Football Detection Project

[**中文文档**](README_ZH.md)

Real-time image acquisition using Realsense D455 camera and detection of football, goalposts, and field feature lines through the [yolov11](https://docs.ultralytics.com/zh/models/yolo11/) network. 
Finally, the detected information is published via DDS. The detection effect is shown in the figure:


![](https://doc-cdn.unitree.com/static/2025/1/7/1bc97fcafb384421b8179c17e4f047db_640x480.png)

### Code Directory

```
football_detect_cpp/
├── dds/                      # DDS generated CPP type message format and idl description files
├── src/                      # Files related to detection logic
├── utils/                    # Utility class files
│   ├── pt_to_onnx.py         # Script to convert training weights to onnx format
│   ├── save_image.py         # Script to read and save camera images
│   ├── spilt_data.py         # Dataset splitting script
│   ├── yolo_image_label.py   # Label new images using trained weights
├── weights/                  # Weights storage
├── football_build.sh         # Build script file, note to modify the corresponding library paths in CmakeList.txt
├── football_onnx2trt.sh      # Script to convert onnx weights to trt format
├── football_run.sh           # Script to execute the executable file
└── main.cpp                  # Main file
```

### Dependencies
The code has been tested in the following environments:

#### 1. Ubuntu 22.4
    1. CUDA version 12.8
    2. TensorRT version 10.9.0.34    Refer to https://developer.nvidia.com/tensorrt/download/10x for installation
    3. OpenCV version 4.8.1
    4. Realsense SDK 2.55.1 Refer to https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide for installation
    5. unitree_sdk2 Refer to https://github.com/unitreerobotics/unitree_sdk2.git for installation

#### 2. Ubuntu 20.4 (NVIDIA Orin NX Developer Kit (P3767-000))
    1. CUDA version 11.4.315
    2. TensorRT version 8.5.2.2
    3. OpenCV version 4.2.0 (CUDA support: NO)
    4. Realsense SDK 2.55.1 Refer to https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide for installation
    5. unitree_sdk2 Refer to https://github.com/unitreerobotics/unitree_sdk2.git for installation

### Quick Start
    git clone http://xxxxxxx/football_detect_cpp
    cd football_detect_cpp
    Execute ./football_build.sh for compilation
    Execute ./football_onnx2trt.sh for model conversion
    Execute ./football_run.sh for inference detection    #No display by default, set IMAGE_SHOWER="1" if display is needed

### Compilation
    1. Download the project
    git clone http://xxxxxxx/football_detect_cpp

    2. Compile:
    cd football_detect_cpp
    mkdir build
    cd build
    cmake ../
    make

### Running
    All operations below should be performed in the build directory
    1. Model conversion (from onnx to TensorRT engine)
    ./football_detect ../weight/weight.onnx 
    2. Inference detection
    ./football_detect ../weight/weight.engine  0 #No image display
    or
    ./football_detect ../weight/weight.engine  1 #Display image

### Results Display and Explanation
- Subscription Display

    Use ```cyclonedds subscribe detectionresults``` to subscribe. If objects are detected, data in the following format will appear. If no objects are detected, ```DetectionResults(results=[])``` will be displayed.

```
DetectionResults(
    results=[
        DetectionResult(
            class_id='3',
            class_name='L',
            box=[
                269.4150695800781,
                267.6305236816406,
                315.8921813964844,
                293.1822814941406
            ],
            score=0.5962997674942017,
            xyz=[-0.10796776413917542, 0.1056944727897644, 2.518663167953491],
            offset=[-27.34637451171875, 40.406402587890625],
            offset_fov=[-3.674669027328491, -4.79826021194458]
        ),
        DetectionResult(
            class_id='4',
            class_name='T',
            box=[
                434.28106689453125,
                274.1331481933594,
                479.90887451171875,
                299.8746643066406
            ],
            score=0.6660580635070801,
            xyz=[0.33687829971313477, 0.0781024768948555, 1.4782800674438477],
            offset=[137.094970703125, 47.00390625],
            offset_fov=[18.422136306762695, -5.581714153289795]
        )
    ]
)
```

- Result Explanation
    - class_id: Object ID number
    - class_name: Corresponding object name
    - box: (x, y) coordinates of the top-left and bottom-right corners of the object box
    - score: Object confidence
    - xyz: 3D coordinates (x, y, z) in camera coordinate system
    - offset: Pixel offset (dx, dy) of the object box center from the image center. Calculation method: For a 640x480 image, the image center coordinates are (320,240); when calculating the offset, subtract the image center coordinates from the box center coordinates (x, y).
    - offset_fov: FOV offset (degrees) of the object box center from the image center

### Subscription Test
    After running the detection, run ./sub in the build directory for subscription testing; test if there is data output.

### Retraining
- Data Acquisition
Yolo format dataset can be downloaded [here](https://huggingface.co/datasets/unitreerobotics/RoboCupFootball_Dataset).
- Training
Training can refer to the [yolov11](https://docs.ultralytics.com/zh/modes/train/#introduction) official tutorial, or follow the steps below.
    - Environment setup
        ```
        pip install torch==2.4.1 torchvision==0.19.1 torchaudio==2.4.1 --index-url https://download.pytorch.org/whl/cu121
        pip install ultralytics
        pip install opencv-python
        pip install onnxruntime
        ```
    - Clone ultralytics
      ```
      git clone https://github.com/ultralytics/ultralytics.git
      ```
    - Set up training data
      ```
      cd ultralytics

      vim ultralytics/cfg/datasets/robocupfootball.yaml 
      
      ```
      Fill in the following content in robocupfootball.yaml

      ```
      # Dataset path
      train: /home/unitree/Code/data/RoboCupFootball/trainset
      val: /home/unitree/Code/data/RoboCupFootball/testset
      
      # Number of classes
      nc: 5     
      
      # Class names
      names: ["ball","goalpost","X-Intersection","L-Intersection","T-Intersection"]
      
      ```
      - Training
      Use the following command for training
      ```
       yolo detect train data=./ultralytics/cfg/datasets/robocupfootball.yaml model=yolo11s.yaml epochs=250 imgsz=640 device=0 batch=64
      
      ```
      - Model Conversion and Deployment
        You can use pt_to_onnx.py to convert the model to onnx format, then refer to the **Running Detection** instructions for execution.

### Other Tools Description
- pt_to_onnx.py: Convert trained weight files to onnx model format
- yolo_image_label.py: Use the trained model to label new images. First modify model_path to fill in the correct model address and input_image_folder image directory. During labeling, you need to judge whether to keep the label of the current image, press 's' key to keep, press 'space' key to discard.
- spilt_data.py: Split the newly labeled data into training and test sets, need to modify the ratio.
- save_image.py: Read images from camera and save to local.

### Notes:
    1. It is strongly recommended to perform model conversion on your own platform, as TensorRT models are platform-dependent.
    2. In the processing_loop function in main.cpp, the specific_class_id variable is used to control keeping only the highest confidence result for the corresponding id class in one detection; all detection results only keep those with confidence greater than conf_flag.
    3. MAX_RECONNECT: Used to control the number of reconnection attempts after camera disconnection