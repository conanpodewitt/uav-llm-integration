import ast
import os
import json
import re
import collections
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
import requests
import numpy as np

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        self.cmd_pub = self.create_publisher(Twist, '/llm_cmd', 10)
        self.plan_pub = self.create_publisher(String, '/plan', 10)
        self.idx_pub = self.create_publisher(Int32, '/plan_index', 10)
        self.text_sub = self.create_subscription(String, '/text_in', self.text_callback, 10)
        self.caption_sub = self.create_subscription(String, '/camera_caption', self.caption_callback, 10)
        self.poisoned_plan_sub = self.create_subscription(String, '/plan_poisoned', self.poisoned_plan_callback, 10)
        # Internal state
        self.latest_text = ''
        self.latest_caption = ''
        self.latest_caption_time = None
        self.max_retries = int(os.getenv('LLM_MAX_RETRIES'))
        self.plan = []
        self.plan_index = 0
        self.paused = False
        self.plan_history = collections.deque(maxlen=3)
        self.detected_objects = []
        self.target_object = None
        self.approaching_target = False  # Flag to indicate already approaching a target
        self.approach_cooldown = 0.0     # Timer to prevent rapid re-detection
        self.last_target_pos = None      # Store the last position of the target
        # Load drive parameters
        self.forward_speed = float(os.getenv('MAX_FORWARD_SPEED'))
        self.backward_speed = float(os.getenv('MAX_REVERSE_SPEED'))
        self.turn_left_speed = float(os.getenv('MAX_TURN_LEFT_SPEED'))
        self.turn_right_speed = float(os.getenv('MAX_TURN_RIGHT_SPEED'))
        # Action primitives
        self.action_params = {
            'small_forward':  {'linear': self.forward_speed,  'angular': 0.0,   'duration': 1.0},
            'big_forward':    {'linear': self.forward_speed,  'angular': 0.0,   'duration': 1.5},
            'small_backward': {'linear': self.backward_speed, 'angular': 0.0,   'duration': 1.0},
            'big_backward':   {'linear': self.backward_speed, 'angular': 0.0,   'duration': 1.5},
            'small_left':     {'linear': 0.0, 'angular': self.turn_left_speed,  'duration': 0.5},
            'big_left':       {'linear': 0.0, 'angular': self.turn_left_speed,  'duration': 1.0},
            'small_right':    {'linear': 0.0, 'angular': self.turn_right_speed, 'duration': 0.5},
            'big_right':      {'linear': 0.0, 'angular': self.turn_right_speed, 'duration': 1.0},
            'search':         {'linear': 0.0, 'angular': self.turn_right_speed * 2.0, 'duration': 10.0},
        }
        # Precompute action keys and regex patterns for performance
        self.action_keys = list(self.action_params.keys())
        colors = ['red', 'blue', 'yellow', 'purple']
        self.colors = colors
        self.target_regex = re.compile(
            r'\b(?:' + '|'.join(colors) + r')\b',
            re.IGNORECASE
        )
        self.json_pattern = re.compile(r'\{.*?\}', re.DOTALL)
        # LLM settings
        self.api_key = os.getenv('LLM_API_KEY')
        self.llm_url = os.getenv('LLM_URL')
        self.model = os.getenv('LLM_MODEL')
        self.temperature = float(os.getenv('LLM_TEMPERATURE'))
        self.system_interval = float(os.getenv('SYSTEM_INTERVAL'))
        self.system_prompt = self.load_system_prompt('uav-llm-integration/system_prompt.txt')
        self.time_diff_threshold = float(os.getenv('TIME_DIFF_THRESHOLD'))
        # Timers
        self.plan_timer = self.create_timer(self.system_interval, self.replan_callback)
        self.exec_timer = None
        self.session = requests.Session()
        self.cooldown_timer = self.create_timer(0.1, self.update_cooldown)

    def load_system_prompt(self, filename):
        '''
        Load system prompt from file
        '''
        try:
            with open(filename, 'r') as f:
                return f.read().strip()
        except Exception as e:
            self.get_logger().error(f'Failed to load system prompt: {e}')
            return ''

    def text_callback(self, msg: String):
        '''
        Handle new user command or emergency stop
        '''
        text = msg.data.strip()
        if text == 'LLM_STOP':
            self.get_logger().warn('Emergency stop: pausing LLM')
            self.paused = True
            if self.exec_timer: self.exec_timer.cancel()
            if self.plan_timer: self.plan_timer.cancel()
            self.cmd_pub.publish(Twist())
            return
        if self.paused:
            self.get_logger().info('Resuming after stop')
            self.paused = False
            self.plan_timer = self.create_timer(self.system_interval, self.replan_callback)
        self.latest_text = text
        self.get_logger().info(f"Received command: '{self.latest_text}'")
        self.target_object = self.extract_target(self.latest_text)
        self.get_logger().info(f"Target object set to: '{self.target_object}'")
        self.generate_plan()
    
    def extract_target(self, command):
        '''
        Extract target object from user command based on predefined colors
        '''
        match = self.target_regex.search(command)
        return match.group(0).lower() if match else ''

    def caption_callback(self, msg: String):
        '''
        Update camera caption and check for target object and timestamp skew
        '''
        if not self.paused:
            text = msg.data.strip()
            data = ast.literal_eval(text)
            # Unpack new structure
            self.latest_caption_time = data.get('current_time')
            # Check for timestamp difference
            now = self.get_clock().now().nanoseconds / 1e9
            if abs(self.latest_caption_time - now) > self.time_diff_threshold:
                self.get_logger().warn(
                    f'Caption timestamp {self.latest_caption_time:.2f}s vs now {now:.2f}s exceeds '
                    f'{self.time_diff_threshold}s – triggering replan'
                )
                # Stop any ongoing action
                if self.exec_timer:
                    self.exec_timer.cancel()
                self.cmd_pub.publish(Twist())
                # Immediate replan
                self.replan_callback()
                return
            self.detected_objects = data.get('objects', [])
            # include time in caption passed to LLM
            self.latest_caption = (
                f"Current time: {self.latest_caption_time}. "
                f"Detected objects: {self.detected_objects}"
            )
            if self.target_object and self.exec_timer:
                self.check_target()
    
    def update_cooldown(self):
        '''
        Decrement the approach cooldown timer
        '''
        if self.approach_cooldown > 0:
            self.approach_cooldown -= 0.1

    def check_target(self):
        '''
        Check if target object is detected and interrupt current action if so
        '''
        if not self.target_object or self.approach_cooldown > 0:
            return
        target_words = self.target_object.lower().split()
        # Check if the target object is in view
        for obj in self.detected_objects:
            obj_label = obj.get('label', '').lower()
            # Check if any of the target words are in the object label
            if any(word in obj_label for word in target_words):
                position = obj.get('pos', 'unknown')
                # If we're already approaching this target in the same position, don't interrupt
                if self.approaching_target and self.last_target_pos == position:
                    return
                # Check if the target has moved significantly (if we were already approaching)
                significant_change = False
                if self.approaching_target and self.last_target_pos != position:
                    significant_change = True
                # Only interrupt if we're not already approaching or there's significant change
                if not self.approaching_target or significant_change:
                    self.get_logger().info(f'Target "{obj_label}" detected at {position} - interrupting current action')
                    # Stop the current action
                    self.cmd_pub.publish(Twist())
                    # Cancel the execution timer
                    if self.exec_timer:
                        self.exec_timer.cancel()
                    # Set the approaching flag and record position
                    self.approaching_target = True
                    self.last_target_pos = position
                    # Set a cooldown to prevent rapid re-detection
                    self.approach_cooldown = 2.0  # 2 second cooldown
                    # Update the caption sent to the LLM for this interrupt
                    self.latest_caption = f"Detected objects: {self.detected_objects}"
                    # Generate a new approach plan with position info
                    self.generate_approach(obj)
                return

    def generate_plan(self):
        '''
        Generate and publish initial plan
        '''        
        if self.paused or not self.latest_text:
            return
        actions = self.action_keys
        history_str = ''
        if self.plan_history:
            history_str = f"Previous plans: {list(self.plan_history)}\n"
        user_msg = (
            f'{history_str}'
            f'User command: {self.latest_text}\n'
            f'Image caption: {self.latest_caption}\n'
            f'Respond with EXACTLY one JSON object: {{\"plan\": [list]}} using actions from {actions}.'
        )
        plan = self.call_api(user_msg)
        if plan:
            self.plan_history.append(plan.copy())
            self.plan = plan
            self.plan_index = 0
            self.publish_plan()
            # start_execution()
        else:
            self.get_logger().warn('LLM returned empty plan')

    def generate_approach(self, detected_object):
        '''
        Generate a specialized plan to approach the detected object
        '''
        if self.paused:
            return
        actions = self.action_keys
        history_str = ''
        if self.plan_history:
            history_str = f"Previous plans: {list(self.plan_history)}\n"
        position = detected_object.get('pos', 'unknown')
        area = detected_object.get('area', 0)
        user_msg = (
            f'{history_str}'
            f'User command: {self.latest_text}\n'
            f'Target "{detected_object["label"]}" detected at position "{position}" with area {area}\n'
            f'Image caption: {self.latest_caption}\n'
            f'Respond with EXACTLY one JSON object: {{\"plan\": [list]}} to approach the target using actions from {actions}.\n'
            f'If the target is in the center, consider moving forward. If left/right, turn in that direction first.'
        )
        plan = self.call_api(user_msg)
        if plan:
            self.plan_history.append(plan.copy())
            self.plan = plan
            self.plan_index = 0
            self.publish_plan()
            # start_execution()
        else:
            self.get_logger().warn('LLM returned empty approach plan')
            self.approaching_target = False  # Reset flag if plan generation failed

    def poisoned_plan_callback(self, msg: String):
        '''
        Override internal plan with poisoned version and begin execution
        '''
        data = json.loads(msg.data)
        poisoned = data.get('plan', [])
        if poisoned:
            old_plan = self.plan
            if poisoned != old_plan:
                self.get_logger().info(f'Using poisoned plan: {poisoned}')
            self.plan = poisoned
            self.plan_index = 0
            self.publish_index()
            self.start_execution()
        else:
            self.get_logger().warn('Received empty poisoned plan')

    def replan_callback(self):
        '''
        Periodic replan check based on current state
        '''        
        if self.paused or not self.plan:
            return
        remaining = self.plan[self.plan_index:]
        actions = self.action_keys
        history_str = ''
        if self.plan_history:
            history_str = f"Previous plans: {list(self.plan_history)}\n"
        user_msg = (
            f'{history_str}'
            f'User command: {self.latest_text}\n'
            f'Image caption: {self.latest_caption}\n'
            f'Remaining plan: {remaining}\n'
            f'Respond with EXACTLY one JSON object: {{\"plan\": [list]}} adjusting actions from {actions}.'
        )
        plan = self.call_api(user_msg)
        if plan:
            self.plan_history.append(plan.copy())
            self.plan = self.plan[:self.plan_index] + plan
            self.publish_plan()

    def publish_plan(self):
        '''
        Publish the full plan as JSON
        '''        
        msg = String(data=json.dumps({'plan': self.plan}))
        self.plan_pub.publish(msg)
        self.publish_index()

    def publish_index(self):
        '''
        Publish the current plan index
        '''        
        idx_msg = Int32(data=self.plan_index)
        self.idx_pub.publish(idx_msg)

    def call_api(self, user_msg) -> list:
        '''
        Call LLM and parse JSON plan, retry immediately on error
        '''    
        messages = [{'role':'system','content':self.system_prompt}] if self.system_prompt else []
        messages.append({'role':'user','content':user_msg})
        body = {'model':self.model, 'messages':messages, 'temperature':self.temperature}
        headers = {'Authorization':f'Bearer {self.api_key}', 'Content-Type':'application/json'}
        for attempt in range(int(self.max_retries)):
            try:
                r = self.session.post(self.llm_url, json=body, headers=headers)
                r.raise_for_status()
                content = r.json()['choices'][0]['message']['content']
                match = self.json_pattern.search(content)
                if not match:
                    self.get_logger().error(f'No JSON in response: {content}')
                    return []
                data = json.loads(match.group(0))
                return data.get('plan', []) if isinstance(data.get('plan', []), list) else []
            except Exception as e:
                self.get_logger().error(f'API error (attempt {attempt+1}/{self.max_retries}): {e}')
                if attempt == self.max_retries - 1:
                    return []
                # retry immediately
        return []

    def start_execution(self):
        '''
        Execute plan primitives sequentially
        '''    
        if self.paused: return
        if self.exec_timer: self.exec_timer.cancel()
        self.exec_timer = self.create_timer(0.1,self.execute_next_action)

    def execute_next_action(self):
        '''
        Publish twist for next action, schedule stop based on duration
        '''    
        if self.paused: return
        if self.plan_index >= len(self.plan):
            self.generate_plan()
            return
        action = self.plan[self.plan_index]
        params = self.action_params.get(action)
        if not params:
            self.plan_index+=1
            return
        # Create and publish the twist command
        twist = Twist(
            linear=type(Twist().linear)(x=params['linear']), 
            angular=type(Twist().angular)(z=params['angular'])
        )
        self.cmd_pub.publish(twist)
        # Use the explicit duration parameter if available, otherwise calculate it
        if 'duration' in params:
            duration = params['duration']
        else:
            # Fallback to old calculation method
            duration = (params.get('distance', params.get('angle', 1.0)) /
                    (abs(self.forward_speed) if 'distance' in params else abs(params['angular']) or 1.0))
        self.get_logger().info(f"Executing action: {action} for {duration:.2f} seconds")
        # Cancel any existing timer and create a new one
        if self.exec_timer:
            self.exec_timer.cancel()
        self.exec_timer = self.create_timer(duration, self.on_action_complete)

    def on_action_complete(self):
        '''
        Stop and advance plan index
        '''    
        self.cmd_pub.publish(Twist())
        self.plan_index += 1
        self.publish_index()
        # If we've completed all actions in the approach plan, reset the approaching flag
        if self.approaching_target and self.plan_index >= len(self.plan):
            self.approaching_target = False
            self.last_target_pos = None
        self.start_execution()

    def on_shutdown(self): 
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
        if self.exec_timer:
            self.exec_timer.cancel()
        if self.plan_timer:
            self.plan_timer.cancel()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LLMNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()