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
        # Internal state
        self.latest_text = ''
        self.latest_caption = ''
        self.plan = []
        self.plan_index = 0
        self.paused = False
        self.plan_history = collections.deque(maxlen=3)
        # Load drive parameters
        self.forward_speed = float(os.getenv('MAX_FORWARD_SPEED', '0.5'))
        self.backward_speed = float(os.getenv('MAX_REVERSE_SPEED', '-0.5'))
        self.turn_left_speed = float(os.getenv('MAX_TURN_LEFT_SPEED', '0.25'))
        self.turn_right_speed = float(os.getenv('MAX_TURN_RIGHT_SPEED', '-0.25'))
        # Action primitives
        self.action_params = {
            'forward':   {'linear': self.forward_speed,  'angular': 0.0,                  'distance': 0.5},
            'backward':  {'linear': self.backward_speed, 'angular': 0.0,                  'distance': 0.5},
            'turn_left': {'linear': 0.0,                 'angular': self.turn_left_speed,  'angle': np.pi/4},
            'turn_right':{'linear': 0.0,                 'angular': self.turn_right_speed, 'angle': np.pi/4},
        }
        # LLM settings
        self.api_key = os.getenv('LLM_API_KEY')
        self.llm_url = os.getenv('LLM_URL')
        self.model = os.getenv('LLM_MODEL')
        self.temperature = float(os.getenv('LLM_TEMPERATURE', '0.7'))
        self.system_interval = float(os.getenv('SYSTEM_INTERVAL', '2.5'))
        self.system_prompt = self.load_system_prompt('setup.txt')
        # Timers
        self.plan_timer = self.create_timer(self.system_interval, self.replan_callback)
        self.exec_timer = None
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
        self.generate_plan()

    def caption_callback(self, msg: String):
        '''
        Update camera caption if not paused
        '''
        if not self.paused:
            self.latest_caption = msg.data.strip()

    def generate_plan(self):
        '''
        Generate and publish initial plan
        '''        
        if self.paused or not self.latest_text:
            return
        actions = list(self.action_params.keys())
        history_str = ''
        if self.plan_history:
            history_str = f"Previous plans: {list(self.plan_history)}\n"
        user_msg = (
            f'{history_str}'
            f'User command: {self.latest_text}\n'
            f'Image caption: {self.latest_caption}\n'
            f'Respond with EXACTLY one JSON object: {{\"plan\": [list]}} using actions from {actions}.'
        )
        plan = self.call_api_for_plan(user_msg)
        if plan:
            self.plan_history.append(plan.copy())
            self.plan = plan
            self.plan_index = 0
            self.publish_plan()
            self.start_execution()
        else:
            self.get_logger().warn('LLM returned empty plan')

    def replan_callback(self):
        '''
        Adjust and publish updated plan
        '''        
        if self.paused or not self.plan:
            return
        remaining = self.plan[self.plan_index:]
        actions = list(self.action_params.keys())
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
        plan = self.call_api_for_plan(user_msg)
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

    def call_api_for_plan(self, user_msg) -> list:
        '''
        Call LLM and parse JSON plan
        '''    
        messages = [{'role':'system','content':self.system_prompt}] if self.system_prompt else []
        messages.append({'role':'user','content':user_msg})
        body = {'model':self.model,'messages':messages,'temperature':self.temperature}
        headers = {'Authorization':f'Bearer {self.api_key}','Content-Type':'application/json'}
        try:
            r = self.session.post(self.llm_url,json=body,headers=headers)
            r.raise_for_status()
            content = r.json()['choices'][0]['message']['content']
            match = re.search(r"\{.*?\}",content,re.DOTALL)
            if not match:
                self.get_logger().error(f'No JSON in response: {content}')
                return []
            data = json.loads(match.group(0))
            return data.get('plan',[]) if isinstance(data.get('plan',[]), list) else []
        except Exception as e:
            self.get_logger().error(f'API error: {e}')
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
        Publish twist for next action, schedule stop
        '''    
        if self.paused: return
        if self.plan_index >= len(self.plan):
            self.generate_plan()
            return
        action = self.plan[self.plan_index]
        params = self.action_params.get(action)
        if not params:
            self.plan_index+=1; return
        twist = Twist(linear=type(Twist().linear)(x=params['linear']), angular=type(Twist().angular)(z=params['angular']))
        self.cmd_pub.publish(twist)
        duration = (params.get('distance',params.get('angle',1.0)) /
                    (abs(self.forward_speed) if 'distance' in params else abs(params['angular']) or 1.0))
        self.exec_timer.cancel()
        self.exec_timer = self.create_timer(duration,self.on_action_complete)

    def on_action_complete(self):
        '''
        Stop and advance plan index
        '''    
        self.cmd_pub.publish(Twist())
        self.plan_index+=1
        self.publish_index()
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