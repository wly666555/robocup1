"""
@file: aloha_service.py
@description: 
    1. 实现一个cmd服务，接受waypoints消息，不断插值控制机械臂的运动
    2. 实现一个state服务，记录当前时刻的关节状态，并发布最新的两组关节状态
"""

from unitree_dl_utils.robots.utils.joint_trajectory_interpolator import JointTrajcetoryInterpolator
from unitree_dl_utils.robots.aloha.aloha import AlohaCtrl

import time
import numpy as np
np.set_printoptions(suppress=True, threshold=np.inf)
import threading
import msgpack      
import zmq
import base64
import cv2

from unitree_dl_utils.robots.aloha.aloha_trajectory import JointTrajectoryDataType, AlohaStateDataType
from unitree_dl_utils.dataset.replay_buffer import ReplayBuffer
from unitree_dl_utils.device.camera.realsense import RealSenseCamera


aloha = AlohaCtrl(ns='aloha')

class JointTrajectory:
    """
    实现一个关节插值器, 根据当前时间t，返回对应的关节角度命令
    订阅joint_trajectory消息，在每次接受到新的消息时，更新插值器

    ; init_q: 当前关节角度
    """
    def __init__(self, init_q):
        self.q_interp = JointTrajcetoryInterpolator(times=[time.time()], points=[init_q])
        
        # zmq
        self._context = zmq.Context()
        self._subscriber = self._context.socket(zmq.SUB)
        self._subscriber.setsockopt(zmq.SUBSCRIBE, b"")
        self._subscriber.connect("tcp://127.0.0.1:8881")

        # Start the listener thread
        self._lock = threading.Lock()
        self._listener = threading.Thread(target=self._listen_cmd)
        self._listener.setDaemon(True)
        self._listener.start()

    def __call__(self, t):
        """
        返回当前时间t对应的关节角度
        """
        with self._lock:
            return self.q_interp(t)

    def _listen_cmd(self):
        """
        在后台线程中更新规划器
        """
        while True:
            try:
                msg = self._subscriber.recv()
                msg = msgpack.unpackb(msg, raw=False)
                data = JointTrajectoryDataType()
                data.get(msg)

                assert np.all(data.times[1:] >= data.times[:-1])
                assert len(data.times) == len(data.points)

                with self._lock:
                    self.q_interp = self.q_interp.schedule_waypoints(data.times, data.points, time.time(), 0.4)
            except Exception as e:
                print(e)


class AlohaCmdService:
    """
    实现一个Aloha服务，接受waypoints消息，不断插值控制机械臂的运动
    """
    def __init__(self):
        self.q_interp = JointTrajectory(aloha.motorcmd.q)

    def run(self):
        aloha.motorcmd.q = self.q_interp(time.time())
        aloha.motorcmd.write()

class AlohaStateService:
    def __init__(self):
        # Init camera
        self.cam0 = RealSenseCamera(serial_number="134222071164") # primart camera
        self.cam1 = RealSenseCamera(serial_number="218622274879") # left camera
        # test
        # self.cam0 = RealSenseCamera(serial_number="213322074261") # primart camera
        # self.cam1 = RealSenseCamera(serial_number="213722070772") # left camera

        self._replay_buffer = ReplayBuffer()

        # zmq
        self._context = zmq.Context()
        self._publisher = self._context.socket(zmq.PUB)
        self._publisher.bind("tcp://127.0.0.1:8882")

    def add_and_publish(self):
        """
        1. 记录当前时刻的状态
        2. 发布最新的两组状态
        """
        # 使用base64编码图像
        color_image0, depth_image0 = self.cam0.get_frame()
        _, color_buffer0 = cv2.imencode('.jpg', color_image0)
        color_image0 = base64.b64encode(color_buffer0)
        _, depth_buffer0 = cv2.imencode('.png', depth_image0.astype(np.uint16))
        depth_image0 = base64.b64encode(depth_buffer0)

        color_image1, depth_image1 = self.cam1.get_frame()
        _, color_buffer1 = cv2.imencode('.jpg', color_image1)
        color_image1 = base64.b64encode(color_buffer1)
        _, depth_buffer1 = cv2.imencode('.png', depth_image1.astype(np.uint16))
        depth_image1 = base64.b64encode(depth_buffer1)
        with aloha.motorstate.lock:
            new_state = {
                "time": aloha.motorstate._last_recv_time,
                "q": aloha.motorstate.q.copy().tolist(),
                "color": [color_image0, color_image1],
                "depth": [depth_image0, depth_image1],
            }
        self._replay_buffer.add(new_state)
        if self._replay_buffer.size() < 2:
            return
        
        state = self._replay_buffer.get(batch_size=2)
        data = AlohaStateDataType()
        data.times = [s["time"] for s in state]
        data.points = [s["q"] for s in state]
        data.colors = [s["color"] for s in state]
        data.depths = [s["depth"] for s in state]

        data_pack = msgpack.packb(vars(data))
        self._publisher.send(data_pack)

if __name__ == "__main__":
    aloha.enable()
    cmd_service = AlohaCmdService()
    state_service = AlohaStateService()

    cmd_dt = 1./66. # 控制机械臂的频率
    pub_cnt = 0 # 控制发布频率

    print("Start running...")
    while True:
        start_t = time.time()

        cmd_service.run()
        pub_cnt += 1
        if pub_cnt >= 2: # 每执行两次发布一次state
            state_service.add_and_publish()
            pub_cnt = 0

        elasped_t = time.time() - start_t
        if elasped_t < cmd_dt:
            time.sleep(cmd_dt - elasped_t)
            # 实际上发布状态的那次肯定超时了; 小问题不影响