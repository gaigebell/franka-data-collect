# -*- coding: utf-8 -*-
"""
Orbbec 相机驱动
直接调用 pyorbbecsdk，不通过 ROS
"""

import sys
import time
import cv2
import numpy as np
from typing import Tuple, Optional, Dict, Any

from camera_interface import CameraInterface

from orbbec_camera_v6 import OrbbecCamera


class OrbbecCameraDriver(CameraInterface):
    """Orbbec 相机驱动 - 直接调用 pyorbbecsdk"""

    def __init__(self,
                 color_width: int = 1280,
                 color_height: int = 720,
                 depth_width: int = 640,
                 depth_height: int = 576,
                 fps: int = 30,
                 enable_alignment: bool = True,
                 extrinsics=None):
        """
        初始化 Orbbec 相机

        Args:
            color_width: 彩色图宽度
            color_height: 彩色图高度
            depth_width: 深度图宽度
            depth_height: 深度图高度
            fps: 帧率
            enable_alignment: 是否启用深度对齐到彩色
            extrinsics: 外参矩阵 (暂不支持)
        """
        self.camera = OrbbecCamera(
            color_width=color_width,
            color_height=color_height,
            depth_width=depth_width,
            depth_height=depth_height,
            color_fps=fps,
            depth_fps=fps,
            enable_alignment=enable_alignment,
            extrinsics=extrinsics
        )
        self.frame_count = 0
        self.config = {
            'color_enable': True,
            'depth_enable': True,
            'pointcloud_enable': False,
        }

    def get_frame(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Dict[str, Any]]:
        """
        获取单帧彩色图和深度图

        Returns:
            color_image: RGB格式 HxWx3 uint8
            depth_image: mm格式 HxW uint16
            metadata: {'timestamp', 'frame_id', 'pointcloud'}
        """
        color_frame, depth_frame = self.camera.get_frames()
        color_img, depth_img = self.camera.get_images_from_frames(color_frame, depth_frame)

        # Orbbec 返回的是 BGR 格式，转换为 RGB
        if color_img is not None:
            color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)

        metadata = {
            'timestamp': time.time(),
            'frame_id': self.frame_count,
            'pointcloud': None
        }

        # 点云生成（仅当启用时）
        if self.config.get('pointcloud_enable', False):
            metadata['pointcloud'] = self.camera.get_aligned_point_cloud(depth_img)

        self.frame_count += 1
        return color_img, depth_img, metadata

    def get_color_intrinsics(self) -> Dict[str, Any]:
        """获取彩色相机内参"""
        return self.camera.get_color_intrinsics()

    def get_depth_intrinsics(self) -> Dict[str, Any]:
        """获取深度相机内参"""
        return self.camera.get_depth_intrinsics()

    @property
    def camera_name(self) -> str:
        """相机名称"""
        return "orbbec"

    def start(self):
        """启动相机（OrbbecCamera 已在 __init__ 时启动）"""
        pass

    def stop(self):
        """停止相机"""
        self.camera.stop()