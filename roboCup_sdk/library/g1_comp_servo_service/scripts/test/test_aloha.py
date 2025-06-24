from unitree_dl_utils.robots.aloha.aloha import AlohaCtrl
import numpy as np
import time

aloha = AlohaCtrl("aloha1")
aloha.enable()

aloha.movej(np.zeros(7))

aloha.enable(0)
time.sleep(1)