# 足球检测项目  

[​**​English Documentation​**​](README.md)  


使用Realsense D455相机实时获取图像并通过[yolov11](https://docs.ultralytics.com/zh/models/yolo11/)网络进行足球、球门以及球场功能线检测。最后通过DDS把检测到的信息发布出去。检测效果如图所示:
![](https://doc-cdn.unitree.com/static/2025/1/7/1bc97fcafb384421b8179c17e4f047db_640x480.png)

### 代码目录

```
football_detect_cpp/
├── dds/                      # dds生成的CPP类型的消息格式以及idl描述文件
├── src/                      # 存放检测逻辑相关文件
├── utils/                    # 存放一些工具类文件
│   ├── pt_to_onnx.py         # 把训练权重转成onnx类型脚本
│   ├── save_image.py         # 读取相机图像保存脚本
│   ├── spilt_data.py         # 分割数据集脚本
│   ├── yolo_image_label.py   # 使用训练好的权重标注新图像
├── weights/                  # 存放权重
├── football_build.sh         # 进行编译的脚本文件，注意修改CmakeList.txt中对应库的路径
├── football_onnx2trt.sh      # 将onnx权重转成trt的脚本文件
├── football_run.sh           # 执行可执行文件的脚本
└── main.cpp                  # 主文件

```
### 依赖
代码的测试全是在以下环境下完成
#### 1、ubuntu 22.4
    1、CUDA 版本 12.8
    2、TensorRT 版本 10.9.0.34    参考https://developer.nvidia.com/tensorrt/download/10x安装
    3、OpenCV 版本   4.8.1
    4、Realsense SDK 2.55.1 参考https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide安装
    5、unitree_sdk2 参考https://github.com/unitreerobotics/unitree_sdk2.git 安装

### 2、ubuntu 20.4(NVIDIA Orin NX Developer Kit (P3767-000))
    1、CUDA 版本 11.4.315
    2、TensorRT 版本 8.5.2.2 
    3、OpenCV 版本   4.2.0（CUDA 支持：​NO）
    4、Realsense SDK 2.55.1 参考https://dev.intelrealsense.com/docs/compiling-librealsense-for-linux-ubuntu-guide安装
    5、unitree_sdk2 参https://github.com/unitreerobotics/unitree_sdk2.git 安装

### 快速开始
    git clone http://xxxxxxx/football_detect_cpp
    cd football_detect_cpp
    执行 ./football_build.sh  进行编译
    执行 ./football_onnx2trt.sh 进行模型转换
    执行 ./football_run.sh  进行推理检测         #默认不显示，如需要显示图形请设置IMAGE_SHOWER="1"

### 编译
    1、下载项目
    git clone http://xxxxxxx/football_detect_cpp

    2、编译:
    cd football_detect_cpp
    mkdir build
    cd build
    cmake ../
    make
### 运行
    以下操作全部在build目录下完成
    1、模型转换（由onnx转成TensorRT的engine）
    ./football_detect ../weight/weight.onnx 
    2、推理检测
    ./football_detect ../weight/weight.engine  0 #不显示图像
    或
    ./football_detect ../weight/weight.engine  1 #显示图像




###  结果显示与解释
- 订阅显示

    使用```cyclonedds subscribe detectionresults``` 订阅，如果检测到物体会出现如下格式的数据。如果没有检测到物体则显示```DetectionResults(results=[])```。

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
 - 结果解释

        - class_id：物体对应的id编号
        - class_name：对应的物体名称
        - box： 物体box的左上角和右下角的（x，y）坐标
        - score：物体置信度
        - xyz：相机坐标系下的三维坐标（x，y，z）
        - offset：物体box中心点距图像中心的偏移像素量（dx，dy）。计算方式：假如640x480的图像，其图像中心坐标是（320,240）;计算偏移量时使用box的中心坐标（x，y）减图像的中心坐标。
        - offset_fov：物体box中心点距图像中心的偏移fov(度)
### 订阅测试
    运行检测后在build目录中运行./sub 进行订阅测试;测试是否有数据输出。


### 重新训练
- 数据获取
Yolo格式的数据集可以在[此处](https://huggingface.co/datasets/unitreerobotics/RoboCupFootball_Dataset)下载.
- 训练
训练可参考[yolov11](https://docs.ultralytics.com/zh/modes/train/#introduction)官方教程、或者按照下面步骤进行。
    - 环境安装
        ```
        pip install torch==2.4.1 torchvision==0.19.1 torchaudio==2.4.1 --index-url https://download.pytorch.org/whl/cu121

        pip install ultralytics
        pip install opencv-python
        pip install onnxruntime
        ```
    - 克隆ultralytics
      ```
      git clone https://github.com/ultralytics/ultralytics.git
      ```
    - 设置训练数据
      ```
      cd ultralytics

      vim ultralytics/cfg/datasets/robocupfootball.yaml 
      
      ```
      把下面内容填写到robocupfootball.yaml 中

      ```
      # 数据集路径
      train: /home/unitree/Code/data/RoboCupFootball/trainset
      val: /home/unitree/Code/data/RoboCupFootball/testset
      
      #类别个数
      nc: 5     
      
      # 类别名称
      names: ["ball","goalpost","X-Intersection","L-Intersection","T-Intersection"]
      
      ```
      - 训练
      使用下面的命令进行训练
      ```
       yolo detect train data=./ultralytics/cfg/datasets/robocupfootball.yaml model=yolo11s.yaml epochs=250 imgsz=640 device=0 batch=64
      
      ```
      - 模型转换与部署
        可以使用pt_to_onnx.py把模型转换成onnx类型，然后参考**运行检测**说明进行运行。

### 其他工具说明
- pt_to_onnx.py: 把训练好的权重文件转变成onnx模型类型
- yolo_image_label.py: 使用训练好的模型对新图像进行标注。首先修改model_path填写正确的模型地址和input_image_folder图像目录。标注期间需要自己判断是否保留本张图像的标注，如果保留需要按下's'键、如果不保留按下'空格'键。
- spilt_data.py: 把上面新标注的数据分割为训练集和测试集、需要修改比例关系。
- save_image.py: 通过相机读取图像并保存到本地。


### 注意事项:
    1、强烈建议在自己的平台上进行模型转换，因为TensorRT的模型与平台相关。
    2、在main.cpp文件中的processing_loop函数中的specific_class_id变量用于控制在一次检测中对应id的的类别只保留置信度最高的；所有检测的结果只保留置信度大于conf_flag。
    3、MAX_RECONNECT:用于控制相机掉线后尝试重连的次数