#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import os
import random

class SpoofNode(Node):
    def __init__(self):
        super().__init__('spoof_node')
        self.bridge = CvBridge()
        # Camera: subscribe to orig, publish to real topic
        self.img_sub = self.create_subscription(Image, '/camera_masked', self.cb_image, 10)
        self.img_pub = self.create_publisher(Image, '/camera_spoofed', 10)
        # LiDAR: same pattern
        self.lidar_sub = self.create_subscription(PointCloud2, '/lidar', self.cb_lidar, 10)
        self.lidar_pub = self.create_publisher(PointCloud2, '/lidar_spoof', 10)
        self.p_spoof = float(os.environ.get('SPOOF_RATE', '0.3'))

    def cb_image(self, msg: Image):
        '''
        Callback for image spoofing
        '''
        if random.random() >= self.p_spoof:
            return self.img_pub.publish(msg)
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            h, w, _ = img.shape
            # overlay a translucent red square
            overlay = img.copy()
            size = min(h, w)//4
            cv2.rectangle(overlay, ((w-size)//2, (h-size)//2),
                          ((w+size)//2, (h+size)//2), (0,0,255), -1)
            img = cv2.addWeighted(overlay, 0.3, img, 0.7, 0)
            spoofed = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            self.img_pub.publish(spoofed)
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')

    def cb_lidar(self, msg: PointCloud2):
        '''
        Callback for LiDAR spoofing
        '''
        if random.random() >= self.p_spoof:
            return self.lidar_pub.publish(msg)
        pts = list(pc2.read_points(msg, skip_nans=True))
        # add 50 ghost points 1m ahead
        ghost = [(1.0, 0.0, 0.0)] * 50
        new_pts = pts + ghost
        spoofed = pc2.create_cloud_xyz32(msg.header, new_pts)
        self.lidar_pub.publish(spoofed)

    def on_shutdown(self): 
            if rclpy.ok():
                self.get_logger().info(f'Shutting down {self.get_name()}...')
            self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SpoofNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()