import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import json

class CameraCaptionNode(Node):
    def __init__(self):
        super().__init__('camera_caption_node')
        self.publisher_ = self.create_publisher(String, '/camera_caption', 10)
        self.mask_pub_ = self.create_publisher(Image, '/camera_masked', 10)
        self.subscription = self.create_subscription(Image, '/camera_mirage', self.image_callback, 10)
        # Configuration
        self.area_threshold = int(os.getenv('AREA_THRESHOLD'))  # min blob area
        self.match_tol = float(os.getenv('AREA_MATCH_TOL'))     # relative area tolerance
        self.max_tracked = int(os.getenv('MAX_TRACKED'))        # max objects to track
        # State
        self.bridge = CvBridge()
        self.current_objects = []
        self.latest_detections = []
        # Precomputed color ranges
        self.color_ranges = {
            'red': [((0,100,100),(10,255,255)), ((160,100,100),(180,255,255))],
            'blue': [((100,150,0),(140,255,255))],
            'yellow': [((20,100,100),(30,255,255))],
            'purple': [((130,50,50),(160,255,255))],
        }
        self.color_bgr = {
            'red': (0,0,255),
            'blue': (255,0,0),
            'yellow': (0,255,255),
            'purple': (128,0,128),
        }
        self.kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))

    def detect_objects(self, image):
        '''
        Detect colored blobs via HSV thresholding
        '''
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detections = []
        vis = np.ones_like(image) * 255
        # Use precomputed maps
        for label, ranges in self.color_ranges.items():
            mask = None
            for low, high in ranges:
                m = cv2.inRange(hsv, np.array(low, 'uint8'), np.array(high, 'uint8'))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            # Clean and overlay
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kern)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kern)
            vis[mask != 0] = self.color_bgr[label]
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for c in cnts:
                area = cv2.contourArea(c)
                if area < self.area_threshold:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                detections.append({'label': label, 'area': area, 'bbox': (x, y, w, h)})
                cv2.rectangle(vis, (x, y), (x+w, y+h), (0,0,0), 2)
        return detections, vis

    def analyse_positions(self, objects, width):
        '''
        Assign left/center/right based on quarters:
        - left:   x_center < width/4
        - center: width/4 <= x_center <= 3*width/4
        - right:  x_center > 3*width/4
        '''
        for obj in objects:
            x, y, w, h = obj['bbox']
            cx = x + w/2
            if cx < width/4:
                pos = 'left'
            elif cx > 3*width/4:
                pos = 'right'
            else:
                pos = 'center'
            obj['pos'] = pos
        return objects

    def image_callback(self, msg):
        '''
        Process each frame: detect, match to memory, and publish masked image
        '''
        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')  # assume success
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        # Detect & position
        dets, vis = self.detect_objects(img)
        self.latest_detections = self.analyse_positions(dets, img.shape[1])
        now = time.time()  # cache timestamp once
        co = self.current_objects
        # Match & update
        for det in self.latest_detections:
            matched = False
            for obj in co:
                if det['label'] == obj['label'] and abs(obj['area'] - det['area'])/obj['area'] < self.match_tol:
                    obj.update(area=det['area'], pos=det['pos'], timestamp=now)
                    matched = True
                    break
            if not matched:
                # New object
                if len(self.current_objects) < self.max_tracked:
                    # Room to add
                    self.current_objects.append({
                        'label': det['label'],
                        'area': det['area'],
                        'pos': det['pos'],
                        'timestamp': now
                    })
                else:
                    # Memory full, then replace oldest duplicate‐color first
                    same = [o for o in self.current_objects if o['label'] == det['label']]
                    if same:
                        # Find oldest of that color
                        oldest = min(same, key=lambda o: o['timestamp'])
                        oldest['area'] = det['area']
                        oldest['pos'] = det['pos']
                        oldest['timestamp'] = now
                    else:
                        # No duplicate, then replace absolute oldest
                        oldest_all = min(self.current_objects, key=lambda o: o['timestamp'])
                        idx = self.current_objects.index(oldest_all)
                        self.current_objects[idx] = {
                            'label': det['label'],
                            'area': det['area'],
                            'pos': det['pos'],
                            'timestamp': now
                        }
        # Publish masked visualization
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            self.mask_pub_.publish(mask_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Mask pub error: {e}')
        # Publish camera_caption with current time
        full_msg = json.dumps({
            "current_time": now,
            "objects": self.current_objects
        })
        self.publisher_.publish(String(data=full_msg))

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
        self.destroy_node()

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