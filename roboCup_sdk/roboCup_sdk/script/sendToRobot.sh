current_dir=`pwd`

REMOTE_USER="unitree"     
REMOTE_HOST="192.168.123.164"   
REMOTE_PORT="22"          

LOCAL_FILE="$current_dir/../../roboCup_sdk/"          
REMOTE_DESTINATION="/home/unitree/" 

scp -r "$REMOTE_PORT" "$LOCAL_FILE" "$REMOTE_USER"@"$REMOTE_HOST":"$REMOTE_DESTINATION"
