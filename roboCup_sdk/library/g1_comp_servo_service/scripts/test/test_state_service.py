from unitree_dl_utils.robots.aloha.aloha_trajectory import AlohaState
import time
import cv2

aloha_state = AlohaState()

while True:
    time.sleep(0.01)
    s = aloha_state.get()
    if not s:
        print("No state received")
        continue

    print(s["times"], time.time())
    print(s["times"][1] - s["times"][0])
    # print(s["points"])

    cv2.imshow("image", s["colors"][1][0])
    cv2.waitKey(1)