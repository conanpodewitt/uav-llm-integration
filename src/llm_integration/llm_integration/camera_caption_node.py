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
        self.mask_pub_ = self.create_publisher(Image, '/camera_masked', 10)
        self.subscription = self.create_subscription(Image, '/camera', self.image_callback, 10)
        # Configuration
        self.area_threshold = int(os.getenv('AREA_THRESHOLD', '500'))       # min blob area
        self.match_tol = float(os.getenv('AREA_MATCH_TOL', '0.2'))         # relative area tolerance
        self.max_tracked = int(os.getenv('MAX_TRACKED', '5'))             # max objects to track
        interval = float(os.getenv('SYSTEM_INTERVAL', '1.0'))             # publish interval
        # State
        self.bridge = CvBridge()
        self.current_objects = []  # list of tracked objects: dicts with label, area, pos, timestamp
        self.latest_detections = []
        # Start timer for publishing only
        self.caption_timer = self.create_timer(interval, self.timer_callback)

    def detect_objects(self, image):
        '''
        Detect colored blobs via HSV thresholding
        Returns list of dicts: {label, area, bbox}
        and a visualization image with color overlays
        '''
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detections = []
        vis = np.ones_like(image) * 255

        color_ranges = {
            'red': [((0,100,100),(10,255,255)),((160,100,100),(180,255,255))],
            'blue': [((100,150,0),(140,255,255))],
            'yellow': [((20,100,100),(30,255,255))],
            'purple': [((130,50,50),(160,255,255))],
        }
        color_bgr = {
            'red': (0,0,255),
            'blue': (255,0,0),
            'yellow': (0,255,255),
            'purple': (128,0,128),
        }
        for label, ranges in color_ranges.items():
            mask = None
            for low, high in ranges:
                m = cv2.inRange(hsv, np.array(low, 'uint8'), np.array(high, 'uint8'))
                mask = m if mask is None else cv2.bitwise_or(mask, m)
            # Clean mask
            kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5,5))
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kern)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kern)
            # Overlay color
            vis[mask != 0] = color_bgr[label]
            # Find contours
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
        - left: x_center < width/4
        - center: width/4 <= x_center <= 3*width/4
        - right: x_center > 3*width/4
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
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return
        # Detect and position
        dets, vis = self.detect_objects(img)
        self.latest_detections = self.analyse_positions(dets, img.shape[1])
        # Update memory each frame
        now = time.time()
        # Match detections to existing
        for det in self.latest_detections:
            matched = False
            for obj in self.current_objects:
                if det['label'] == obj['label']:
                    rel = abs(obj['area'] - det['area']) / det['area']
                    if rel < self.match_tol:
                        obj['area'] = det['area']
                        obj['pos'] = det['pos']
                        obj['timestamp'] = now
                        matched = True
                        break
            if not matched and len(self.current_objects) < self.max_tracked:
                self.current_objects.append({
                    'label': det['label'],
                    'area': det['area'],
                    'pos': det['pos'],
                    'timestamp': now
                })
        # Publish masked visualisation
        try:
            mask_msg = self.bridge.cv2_to_imgmsg(vis, encoding='bgr8')
            self.mask_pub_.publish(mask_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'Mask pub error: {e}')

    def timer_callback(self):
        '''
        Publish caption and history; memory already updated per frame
        '''
        now = time.time()
        # Build caption from current_objects
        descs = [f"{o['label']} at the {o['pos']}" for o in self.current_objects]
        caption = ', '.join(descs) if descs else 'nothing visible'
        # Build history excluding center and entries <=1s old
        history = []
        for o in self.current_objects:
            if o['pos'] == 'center':
                continue
            dt = int(now - o['timestamp'])
            if dt > 1:
                history.append(f"{o['label']} last seen {dt}s ago")
        hist_str = '; '.join(history)
        full_msg = f"{caption}. History: {hist_str}" if hist_str else caption
        self.publisher_.publish(String(data=full_msg))
        # self.get_logger().info(f'Current memory: {self.current_objects}')

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