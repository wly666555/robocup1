#!/bin/bash

# 舵机控制节点启动脚本

echo "Starting servo control node..."

# 设置ROS2环境
source /opt/ros/foxy/setup.bash

# 设置Unitree SDK环境（如果存在）
if [ -f "/home/unitree/unitree_sdk2/setup.bash" ]; then
    source /home/unitree/unitree_sdk2/setup.bash
fi

# 设置ROS包路径（使用相对路径）
export ROS_PACKAGE_PATH="$(pwd)/src:$ROS_PACKAGE_PATH"

# 设置配置文件路径环境变量
export SERVO_CONFIG_PATH="$(pwd)/src/servo_control/config/config.yaml"

# 设置DDS配置文件路径
DDS_CONFIG_PATH="$(pwd)/config/dds_config.xml"

# 检查DDS配置文件是否存在
if [ ! -f "$DDS_CONFIG_PATH" ]; then
    echo "Warning: DDS config file not found at $DDS_CONFIG_PATH"
    echo "Creating default DDS config..."
    mkdir -p "$(dirname "$DDS_CONFIG_PATH")"
    cat > "$DDS_CONFIG_PATH" << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles" >
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>UdpTransport</transport_id>
            <type>UDPv4</type>
            <interfaceWhiteList>
                <address>127.0.0.1</address>
            </interfaceWhiteList>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="udp_transport_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>UdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF
fi

# 启动节点（带DDS配置）
echo "Launching servo control node with DDS config: $DDS_CONFIG_PATH"
cd build
./servo_control_node "$DDS_CONFIG_PATH"

echo "Servo control node stopped." 