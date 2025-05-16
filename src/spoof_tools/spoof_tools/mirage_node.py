import os
import random
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class MirageNode(Node):
    def __init__(self):
        super().__init__('mirage_node')
        self.bridge = CvBridge()
        self.p_mirage = float(os.environ.get('MIRAGE_RATE'))
        self.get_logger().info(f'Mirage rate: {self.p_mirage}')
        self.detectable_colors = [
            (0, 0, 255),   # red
            (255, 0, 0),   # blue
            (0, 255, 255), # yellow
            (128, 0, 128)  # purple
        ]
        # Subscribe to raw camera frames
        self.sub = self.create_subscription(Image, '/camera', self.cb, 10)
        # Publish miraged frames
        self.pub = self.create_publisher(Image, '/camera_mirage', 10)

    def cb(self, msg: Image):
        '''
        Callback for camera mirage
        '''
        # Convert ROS image to OpenCV
        cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # With probability p_mirage draw a shape
        if random.random() < self.p_mirage:
            h, w, _ = cv_img.shape
            color = random.choice(self.detectable_colors)
            if random.choice([True, False]):
                # Rectangle
                pt1 = (random.randint(0, w//2), random.randint(0, h//2))
                pt2 = (random.randint(w//2, w), random.randint(h//2, h))
                cv2.rectangle(cv_img, pt1, pt2, color, thickness=-1)
            else:
                # Circle
                center = (random.randint(0, w), random.randint(0, h))
                radius = random.randint(10, min(w,h)//4)
                cv2.circle(cv_img, center, radius, color, thickness=-1)
        # Convert back and publish
        out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
        out_msg.header = msg.header
        self.pub.publish(out_msg)

    def on_shutdown(self): 
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MirageNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
