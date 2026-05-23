# -*- coding: utf-8 -*-
"""
相机采集线程
在单独线程中直接调用相机 SDK，不通过 ROS
"""

import threading
import queue
import time
from typing import Dict

from camera_interface import CameraInterface


class CameraCaptureThread(threading.Thread):
    """
    独立相机采集线程
    在单独线程中直接调用相机 SDK，不通过 ROS
    """

    def __init__(self,
                 cameras: Dict[str, CameraInterface],
                 frame_queue: queue.Queue,
                 fps: int = 30):
        """
        初始化相机采集线程

        Args:
            cameras: 相机字典 {'camera_name': CameraInterface}
            frame_queue: 帧数据队列
            fps: 目标帧率
        """
        super().__init__(daemon=True)
        self.cameras = cameras
        self.frame_queue = frame_queue
        self.frame_interval = 1.0 / fps
        self.running = False
        self._lock = threading.Lock()

    def run(self):
        """采集循环"""
        self.running = True
        self.paused = False
        last_time = 0.0

        while self.running:
            current_time = time.time()

            # FPS 控制
            if current_time - last_time < self.frame_interval:
                time.sleep(0.001)
                continue

            # 如果暂停则跳过
            if self.paused:
                time.sleep(0.01)
                continue

            # 从所有相机获取帧
            frames = {}
            valid = True

            with self._lock:
                for name, camera in self.cameras.items():
                    try:
                        color, depth, meta = camera.get_frame()
                        frames[name] = {
                            'color': color,
                            'depth': depth,
                            'metadata': meta
                        }
                    except Exception as e:
                        print(f"[{name}] 获取帧失败: {e}")
                        frames[name] = None

            if valid and frames:
                try:
                    self.frame_queue.put_nowait((frames, current_time))
                except queue.Full:
                    pass

            last_time = current_time

    def pause(self):
        """暂停采集"""
        self.paused = True

    def resume(self):
        """恢复采集"""
        self.paused = False

    def stop(self):
        """停止采集线程"""
        self.running = False