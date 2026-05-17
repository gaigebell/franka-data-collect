#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class SimpleViewer(Node):
    def __init__(self):
        super().__init__('simple_camera_viewer')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_rect_raw',  # 确认你的话题名
            self.listener_callback,
            10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        try:
            # 转换消息
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 显示图像
            cv2.imshow('RealSense D405 View', cv_image)
            
            # 等待 1ms 并检查是否按下 'q' 退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main():
    rclpy.init()
    node = SimpleViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()