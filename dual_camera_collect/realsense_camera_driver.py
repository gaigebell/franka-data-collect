# -*- coding: utf-8 -*-
"""
RealSense 相机驱动
直接调用 pyrealsense2，不通过 ROS
"""

import pyrealsense2 as rs
import numpy as np
import time
from typing import Tuple, Optional, Dict, Any

from camera_interface import CameraInterface


class RealSenseCameraDriver(CameraInterface):
    """RealSense 相机驱动 - 直接调用 pyrealsense2"""

    def __init__(self,
                 color_width: int = 640,
                 color_height: int = 480,
                 depth_width: int = 640,
                 depth_height: int = 480,
                 fps: int = 30):
        """
        初始化 RealSense 相机

        Args:
            color_width: 彩色图宽度
            color_height: 彩色图高度
            depth_width: 深度图宽度
            depth_height: 深度图高度
            fps: 帧率
        """
        self.pipeline = rs.pipeline()
        self.config_rs = rs.config()

        self.config_rs.enable_stream(
            rs.stream.color, color_width, color_height,
            rs.format.rgb8, fps
        )
        self.config_rs.enable_stream(
            rs.stream.depth, depth_width, depth_height,
            rs.format.z16, fps
        )

        self.pipeline.start(self.config_rs)
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
        frames = self.pipeline.wait_for_frames()

        color_img = None
        depth_img = None
        metadata = {'timestamp': time.time(), 'frame_id': self.frame_count, 'pointcloud': None}

        if self.config.get('color_enable', True):
            color_frame = frames.get_color_frame()
            if color_frame:
                color_img = np.asanyarray(color_frame.get_data())

        if self.config.get('depth_enable', True):
            depth_frame = frames.get_depth_frame()
            if depth_frame:
                depth_img = np.asanyarray(depth_frame.get_data())

        if self.config.get('pointcloud_enable', False) and depth_img is not None:
            # TODO: 使用内参生成点云
            pass

        self.frame_count += 1
        return color_img, depth_img, metadata

    def get_color_intrinsics(self) -> Dict[str, Any]:
        """获取彩色相机内参"""
        profile = self.pipeline.get_active_profile()
        color_profile = profile.get_stream(rs.stream.color)
        intr = color_profile.as_video_stream_profile().get_intrinsics()
        return {
            'width': intr.width, 'height': intr.height,
            'fx': intr.fx, 'fy': intr.fy,
            'cx': intr.ppx, 'cy': intr.ppy
        }

    def get_depth_intrinsics(self) -> Dict[str, Any]:
        """获取深度相机内参"""
        profile = self.pipeline.get_active_profile()
        depth_profile = profile.get_stream(rs.stream.depth)
        intr = depth_profile.as_video_stream_profile().get_intrinsics()
        return {
            'width': intr.width, 'height': intr.height,
            'fx': intr.fx, 'fy': intr.fy,
            'cx': intr.ppx, 'cy': intr.ppy
        }

    @property
    def camera_name(self) -> str:
        """相机名称"""
        return "realsense"

    def start(self):
        """启动相机（已在 __init__ 时启动）"""
        pass

    def stop(self):
        """停止相机"""
        self.pipeline.stop()