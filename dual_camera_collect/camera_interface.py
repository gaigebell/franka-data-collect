# -*- coding: utf-8 -*-
"""
相机抽象基类
定义所有相机驱动的统一接口
"""

from abc import ABC, abstractmethod
import numpy as np
from typing import Tuple, Optional, Dict, Any


class CameraInterface(ABC):
    """所有相机的抽象基类"""

    @abstractmethod
    def start(self):
        """启动相机"""
        pass

    @abstractmethod
    def stop(self):
        """停止相机"""
        pass

    @abstractmethod
    def get_frame(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Dict[str, Any]]:
        """
        获取单帧数据

        Returns:
            color_image: HxWx3 uint8 RGB格式 (不可用时为None)
            depth_image: HxW uint16 mm格式 (不可用时为None)
            metadata: {
                'timestamp': float,
                'frame_id': int,
                'pointcloud': np.ndarray (可选, HxWx3 float32)
            }
        """
        pass

    @abstractmethod
    def get_color_intrinsics(self) -> Dict[str, Any]:
        """获取彩色相机内参"""
        pass

    @abstractmethod
    def get_depth_intrinsics(self) -> Dict[str, Any]:
        """获取深度相机内参"""
        pass

    @property
    @abstractmethod
    def camera_name(self) -> str:
        """相机名称"""
        pass