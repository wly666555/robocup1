from unitree_dl_utils.robots.aloha.aloha_trajectory import AlohaJointTrajectory
import time

aloha_cmd = AlohaJointTrajectory()

def send_once():
    aloha_cmd.schedule_waypoints(
        [time.time() + 2, time.time() + 5.],
        [[0, 0, 0, 0, 0, 0, 1], 
        [0, 0, 0, 0, 0, 0, 0]]
    )


print("Start running...")
send_once() # 暂不清楚为什么第一次发送不生效
time.sleep(1)
send_once()
time.sleep(1)