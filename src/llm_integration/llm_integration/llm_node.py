import os
import json
import re
import collections
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import requests
import numpy as np

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        # Publishers and subscriptions
        self.cmd_pub = self.create_publisher(Twist, '/llm_cmd', 10)
        self.text_sub = self.create_subscription(String, '/text_in', self.text_callback, 10)
        self.caption_sub = self.create_subscription(String, '/camera_caption', self.caption_callback, 10)
        # Internal state
        self.latest_text = ''
        self.latest_caption = ''
        self.plan = []
        self.plan_index = 0
        self.paused = False
        # History of past plans (keep last 3)
        self.plan_history = collections.deque(maxlen=3)
        # Load drive parameters
        self.forward_speed = float(os.getenv('MAX_FORWARD_SPEED'))
        self.backward_speed = float(os.getenv('MAX_REVERSE_SPEED'))
        self.turn_left_speed = float(os.getenv('MAX_TURN_LEFT_SPEED'))
        self.turn_right_speed = float(os.getenv('MAX_TURN_RIGHT_SPEED'))
        # Define action primitives
        self.action_params = {
            'forward':   {'linear': self.forward_speed,  'angular': 0.0,                  'distance': 0.5},
            'backward':  {'linear': self.backward_speed, 'angular': 0.0,                  'distance': 0.5},
            'turn_left': {'linear': 0.0,                 'angular': self.turn_left_speed,  'angle': np.pi/4},
            'turn_right':{'linear': 0.0,                 'angular': self.turn_right_speed, 'angle': np.pi/4},
        }
        # Load LLM parameters
        self.api_key = os.getenv('LLM_API_KEY')
        self.llm_url = os.getenv('LLM_URL')
        self.model = os.getenv('LLM_MODEL')
        self.temperature = float(os.getenv('LLM_TEMPERATURE', '0.7'))
        self.system_interval = float(os.getenv('SYSTEM_INTERVAL', '2.5'))
        # Load system prompt
        self.system_prompt = self.load_system_prompt('uav-llm-integration/setup.txt')
        # Timers
        self.plan_timer = self.create_timer(self.system_interval, self.replan_callback)
        self.exec_timer = None
        # HTTP session
        self.session = requests.Session()

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
        Handle new user command and emergency stop
        '''
        text = msg.data.strip()
        # Emergency stop command
        if text == 'LLM_STOP':
            self.get_logger().warn('Emergency stop received: pausing LLM')
            self.paused = True
            # cancel timers
            if self.exec_timer: self.exec_timer.cancel()
            if self.plan_timer: self.plan_timer.cancel()
            # publish immediate stop
            self.cmd_pub.publish(Twist())
            return
        # If paused, resume on any other command
        if self.paused:
            self.get_logger().info('Resuming LLM after emergency stop')
            self.paused = False
            # restart plan timer
            self.plan_timer = self.create_timer(self.system_interval, self.replan_callback)
        # Normal command handling
        self.latest_text = text
        self.get_logger().info(f'Received command: "{self.latest_text}"')
        self.generate_plan()

    def caption_callback(self, msg: String):
        '''
        Update latest camera caption (ignored when paused)
        '''
        if not self.paused:
            self.latest_caption = msg.data.strip()

    def generate_plan(self):
        '''
        Generate initial plan via LLM, including history of past plans
        '''
        if self.paused:
            return
        if not self.latest_text:
            self.get_logger().warn('No user command to plan')
            return
        actions = list(self.action_params.keys())
        # Build history context
        history = list(self.plan_history)
        history_str = ''
        if history:
            history_str = f'Previous plans: {history}\n'
        user_msg = (
            f'{history_str}'
            f'User command: {self.latest_text}\n'
            f'Image caption: {self.latest_caption}\n'
            f'Respond with EXACTLY one JSON object: {{\"plan\": [list of actions]}} using only actions from {actions}.'
        )
        plan = self.call_api_for_plan(user_msg)
        if plan:
            # Save to history before replacing
            self.plan_history.append(plan.copy())
            self.plan = plan
            self.plan_index = 0
            self.get_logger().info(f'New plan: {self.plan}')
            self.start_execution()
        else:
            self.get_logger().warn('LLM returned empty plan')

    def replan_callback(self):
        '''
        Periodically adjust remaining plan with plan history context
        '''
        if self.paused or not self.plan:
            return
        remaining = self.plan[self.plan_index:]
        actions = list(self.action_params.keys())
        history = list(self.plan_history)
        history_str = ''
        if history:
            history_str = f'Previous plans: {history}\n'
        user_msg = (
            f'{history_str}'
            f'User command: {self.latest_text}\n'
            f'Image caption: {self.latest_caption}\n'
            f'Remaining plan: {remaining}\n'
            f'Respond with EXACTLY one JSON object: {{\"plan\": [list of actions]}} adjusting actions from {actions}.'
        )
        plan = self.call_api_for_plan(user_msg)
        if plan:
            # Append the adjusted plan to history
            self.plan_history.append(plan.copy())
            self.plan = self.plan[:self.plan_index] + plan
            self.get_logger().info(f'Updated plan: {self.plan}')

    def call_api_for_plan(self, user_msg) -> list:
        '''
        Send system+user messages, parse JSON response
        '''    
        messages = []
        if self.system_prompt:
            messages.append({'role': 'system', 'content': self.system_prompt})
        messages.append({'role': 'user', 'content': user_msg})
        body = {
            'model': self.model,
            'messages': messages,
            'temperature': self.temperature
        }
        self.get_logger().debug(f'Sending API request with messages: {messages}')
        headers = {'Authorization': f'Bearer {self.api_key}', 'Content-Type': 'application/json'}
        try:
            r = self.session.post(self.llm_url, json=body, headers=headers)
            r.raise_for_status()
            choice = r.json()['choices'][0]['message']['content']
            self.get_logger().debug(f'API raw content: {choice}')
            match = re.search(r"\{.*?\}", choice, re.DOTALL)
            if not match:
                self.get_logger().error(f'No JSON found in LLM response: {choice}')
                return []
            data = json.loads(match.group(0))
            if 'plan' not in data or not isinstance(data['plan'], list):
                self.get_logger().error(f'Invalid plan format: {data}')
                return []
            return data['plan']
        except Exception as e:
            self.get_logger().error(f'Plan API error: {e}')
            return []

    def start_execution(self):
        '''
        Begin executing plan
        '''    
        if self.paused:
            return
        if self.exec_timer:
            self.exec_timer.cancel()
        self.exec_timer = self.create_timer(0.1, self.execute_next_action)

    def execute_next_action(self):
        '''
        Execute one primitive, schedule stop
        '''    
        if self.paused:
            return
        if self.plan_index >= len(self.plan):
            self.get_logger().info('Plan complete, regenerating...')
            self.generate_plan()
            return
        action = self.plan[self.plan_index]
        params = self.action_params.get(action)
        if not params:
            self.get_logger().warn(f'Unknown action: {action}')
            self.plan_index += 1
            return
        twist = Twist()
        twist.linear.x = params['linear']
        twist.angular.z = params['angular']
        self.cmd_pub.publish(twist)
        if 'distance' in params:
            duration = params['distance'] / abs(self.forward_speed)
        else:
            duration = params['angle'] / abs(params['angular']) if params.get('angular') else 1.0
        self.exec_timer.cancel()
        self.exec_timer = self.create_timer(duration, self.on_action_complete)

    def on_action_complete(self):
        '''
        Stop robot and move to next
        '''    
        self.cmd_pub.publish(Twist())
        self.plan_index += 1
        self.get_logger().info(f'Action {self.plan_index} complete')
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