import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class CameraCaptionNode(Node):
    def __init__(self):
        super().__init__('camera_caption_node')
        self.publisher_ = self.create_publisher(String, '/camera_caption', 10)
        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
        self.threshold = int(os.getenv('AREA_THRESHOLD'))
        self.bridge = CvBridge()
        self.window_name = 'Masked View'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.current_pos_desc = None  # last positional description
        # Memory: list of tuples (positional_description, timestamp)
        self.memory = []
        # Latest detection
        self.latest_positions = {}
        # Timer to publish every second
        self.caption_timer = self.create_timer(float(os.getenv('PUBLISH_INTERVAL')), self.timer_callback)

    def detect_objects(self, image):
        """
        Detect colored blobs using HSV thresholding and perform basic shape recognition
        Returns:
          - objects: list of detected objects with label, shape, and bounding box
          - display_masked_image: visualization image
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        objects = []
        display_masked_image = np.ones_like(image) * 255
        color_ranges = {
            'red': [((0, 100, 100), (10, 255, 255)), ((160, 100, 100), (180, 255, 255))],
            'blue': [((100, 150, 0), (140, 255, 255))],
            'yellow': [((20, 100, 100), (30, 255, 255))],
            'purple': [((130, 50, 50), (160, 255, 255))],
        }
        color_bgr = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'yellow': (0, 255, 255),
            'purple': (128, 0, 128),
        }
        for color, ranges in color_ranges.items():
            combined_mask = None
            for lower, upper in ranges:
                mask = cv2.inRange(hsv, np.array(lower, 'uint8'), np.array(upper, 'uint8'))
                combined_mask = mask if combined_mask is None else cv2.bitwise_or(combined_mask, mask)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            display_masked_image[mask != 0] = color_bgr[color]
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid = [c for c in contours if cv2.contourArea(c) > self.threshold]
            if valid:
                merged = np.vstack(valid)
                x, y, w, h = cv2.boundingRect(merged)
                peri = cv2.arcLength(merged, True)
                approx = cv2.approxPolyDP(merged, 0.04 * peri, True)
                shape = 'unidentified'
                if len(approx) == 3: shape = 'triangle'
                elif len(approx) == 4: shape = 'rectangle'
                elif len(approx) == 5: shape = 'pentagon'
                elif len(approx) > 5: shape = 'circle'
                objects.append({'label': color, 'shape': shape, 'bbox': (x, y, w, h)})
        for obj in objects:
            x, y, w, h = obj['bbox']
            cv2.rectangle(display_masked_image, (x, y), (x+w, y+h), (0,0,0), 2)
            cv2.putText(display_masked_image, f"{obj['label']} {obj['shape']}",
                        (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 2)
        return objects, display_masked_image

    def analyze_positions(self, objects, image_width):
        '''
        Analyze the positions of detected objects and classify them as left, center, or right
        '''
        positions = {}
        for obj in objects:
            x, y, w, h = obj['bbox']
            center = x + w/2
            if center < image_width/3: pos='left'
            elif center > 2*image_width/3: pos='right'
            else: pos='center'
            positions[f"{obj['label']} {obj['shape']}"] = pos
        return positions

    def image_callback(self, msg):
        '''
        Callback for image messages
        '''
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        objs, disp = self.detect_objects(img)
        self.latest_positions = self.analyze_positions(objs, img.shape[1])
        cv2.imshow(self.window_name, disp)
        cv2.waitKey(1)

    def timer_callback(self):
        '''
        Timer callback to publish the current caption and manage memory
        '''
        # Build current description
        if self.latest_positions:
            pos_desc = ', '.join([f"{d} at the {p}" for d,p in self.latest_positions.items()])
        else:
            pos_desc = 'nothing visible'
        # Always add previous description to memory if it exists and differs from last memory entry
        if self.current_pos_desc:
            if not self.memory or self.memory[-1][0] != self.current_pos_desc:
                self.memory.append((self.current_pos_desc, time.time()))
        # Update current description
        self.current_pos_desc = pos_desc
        # Build history (exclude center, age>1s)
        now = time.time()
        history = []
        for desc, ts in reversed(self.memory):
            if ' at the center' in desc:
                continue
            delta = int(now - ts)
            if delta > 1:
                history.append(f"{desc} last seen {delta}s ago")
        hist_str = '; '.join(history)
        full_msg = f"{pos_desc}. History: {hist_str}" if hist_str else pos_desc
        # Truncate to 1024 chars
        full_msg = full_msg[:1024]
        self.get_logger().info(f'Caption: {full_msg}')
        self.publisher_.publish(String(data=full_msg))

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
        self.destroy_node()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = CameraCaptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()