
6/30

新建脚本----一键运行 已完成


首先启动 g1_comp_servo_service 舵机控制接口

```
cd g1_comp_servo_service/build/
sudo ./main eth0
```
接下来启动 football_detectcpp 目标检测接口
```
cd football_detectcpp/
sudo ./football_run.sh
```

接下来启动 robocup_locator_v1.0 定位接口
```
cd robocup_locator_v1.0/build/
sudo ./test_location eth0
```

最后运行 roboCup_sdk 例程，以运行 halfFieldKickExample 例程为例
```
cd roboCup_sdk/build/
sudo ./test/halfFieldKickExample eth0
```

yolo窗口关不掉


---------------



在trackball只是粗略的追踪球,球往哪里跑,相机就随动,最多只能保持在视野的相对位置不变,如果在视野边缘检测到,就一直在边缘了.
是否有必要修改node.cpp中camtrackball部分,例如加入kalman滤波+死区限制,实现保持在视野中央区域,不用加入多目标的判断.
--------------
camTrackBall 节点修改逻辑:计算球与视野中心偏移量,利用高频率(30fps)进行回正.


    加入死区


--------------






