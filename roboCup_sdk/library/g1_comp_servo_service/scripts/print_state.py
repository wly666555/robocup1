from unitree_dl_utils.robots.aloha.aloha import MotorStates
import time

aloha_state = MotorStates(namespace="aloha_mini")

while True:
    time.sleep(0.01)
    for i in range(7):
        aloha_state.q[i] = aloha_state.msg.states[i].q
    print(aloha_state.q)