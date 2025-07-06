from unitree_dl_utils.robots.aloha.aloha import AlohaCtrl, MotorStates
import time

# d2 = MotorStates("aloha_mini", window_size=20)
d2 = MotorStates("aloha_mini")
d2.wait_for_connection()

aloha = AlohaCtrl()
aloha.enable()

aloha.movej(d2.q)
print("Start running...")

while True:
    aloha.motorcmd.q = d2.q.copy()
    aloha.motorcmd.write()
    time.sleep(1./66.)