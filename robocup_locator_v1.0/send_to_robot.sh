current_dir=`pwd`

REMOTE_USER="unitree"     # 替换为远程设备的用户名
REMOTE_HOST="192.168.123.164"    # 替换为远程设备的IP地址或主机名
REMOTE_PORT="22"           # 如果使用了非默认的SSH端口，请修改为正确的端口号

# # 设置本地文件路径和远程目标路径
LOCAL_FILE="$current_dir/../../locate/"           # 替换为本地文件的完整路径
REMOTE_DESTINATION="/home/unitree/" # 替换为远程设备上接收文件的目标路径

# # 使用scp命令发送文件
scp -r "$REMOTE_PORT" "$LOCAL_FILE" "$REMOTE_USER"@"$REMOTE_HOST":"$REMOTE_DESTINATION"
