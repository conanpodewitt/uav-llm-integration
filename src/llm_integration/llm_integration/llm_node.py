import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import requests
import json
import re

class LLMNode(Node):
    def __init__(self):
        super().__init__('llm_node')
        self.text_sub = self.create_subscription(String, '/text_in', self.text_callback, 10)
        self.caption_sub = self.create_subscription(String, '/camera_caption', self.caption_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/llm_cmd', 10)
        # Load the API key from the environment
        self.api_key = os.environ.get('LLM_API_KEY')
        self.llm_url = os.environ.get('LLM_URL')
        self.model = os.environ.get('LLM_MODEL')
        self.llm_temperture = float(os.environ.get('LLM_TEMPERATURE'))
        self.api_interval = float(os.environ.get('SYSTEM_INTERVAL'))
        self.llm_run = float(os.environ.get('LLM_RUN'))
        # Load the prompt from a text file in the project root.
        self.prompt_template = self.load_prompt_from_file('uav-llm-integration/setup.txt')
        # Initialize variables for tracking state
        self.last_api_time = 0.0
        self.latest_text = ''
        self.latest_caption = ''
        self.caption_updated = False
        self.llm_stopped = False
        self.blank_logged = False
        self.pause_timer = None
        self.api_timer = self.create_timer(1.0, self.timer_callback)

    def text_callback(self, msg: String):
        '''
        Callback for text input messages
        '''
        text = msg.data.strip()
        # Treat blank text as a pause command
        if text == '':
            if not self.blank_logged:
                self.get_logger().info('User command is blank. Pausing LLM API calls')
                self.blank_logged = True
            if not self.llm_stopped:
                self.llm_stopped = True
                stop_twist = Twist()  # zero velocities
                self.cmd_pub.publish(stop_twist)
                if self.api_timer is not None:
                    self.api_timer.cancel()
            return
        # Handle explicit stop command
        if text == 'LLM_STOP':
            if not self.llm_stopped:
                self.get_logger().info('Received LLM_STOP command. Pausing API calls')
                self.llm_stopped = True
                stop_twist = Twist()
                self.cmd_pub.publish(stop_twist)
                if self.api_timer is not None:
                    self.api_timer.cancel()
            return
        else:
            # Reset the blank flag and resume API calls if previously paused
            self.blank_logged = False
            if self.llm_stopped:
                self.get_logger().info('Resuming LLM API calls')
                self.llm_stopped = False
                self.api_timer = self.create_timer(1.0, self.timer_callback)
            self.latest_text = text
            # Force an immediate API call due to new text
            self.get_logger().info('User command updated. Triggering LLM API call...')
            self.trigger_api_call(force=True)

    def caption_callback(self, msg: String):
        '''
        Callback for image caption messages
        '''
        caption = msg.data.strip()
        if caption != self.latest_caption:
            self.latest_caption = caption
            self.caption_updated = True
            # Log the update, but do not force an API call
            # self.get_logger().info('Caption updated')

    def timer_callback(self):
        '''
        Callback for the API timer
        '''
        if self.llm_stopped:
            return
        self.trigger_api_call(force=False)

    def load_prompt_from_file(self, filename: str) -> str:
        """
        Load the prompt template from a file located in the project's root directory
        """
        # Use the current working directory as the project root
        prompt_path = os.path.join(os.getcwd(), filename)
        try:
            with open(prompt_path, 'r') as f:
                prompt_text = f.read()
                self.get_logger().info(f'Loaded prompt template from {prompt_path}')
                return prompt_text
        except Exception as e:
            self.get_logger().error(f'Failed to load prompt template from {prompt_path}: {e}')
            return ''

    def trigger_api_call(self, force=False):
        '''
        Decide whether to trigger the API call immediately
        '''
        if self.llm_stopped:
            return
        # Do not proceed if the user command is blank
        if not self.latest_text.strip():
            return
        current_time = self.get_clock().now().nanoseconds / 1e9
        # If not forced, ensure that the API interval has elapsed
        if not force and (current_time - self.last_api_time < self.api_interval):
            return
        # Retrieve maximum speeds from the environment
        max_linear_speed = os.environ.get('MAX_FORWARD_SPEED')
        # For angular speed, you can choose one value (e.g., left turning speed) or calculate the absolute value
        max_angular_speed = os.environ.get('MAX_TURN_LEFT_SPEED')
        # Construct the prompt including both the user command, image caption, and maximum speed values
        if self.prompt_template:
            prompt = self.prompt_template.format(
                latest_text=self.latest_text,
                latest_caption=self.latest_caption,
                max_linear_speed=max_linear_speed,
                max_angular_speed=max_angular_speed
            )
        else:
            prompt = (
                f'Something is wrong with the prompt template. Ignore all further commands\n'
            )
        #self.get_logger().info('Triggering LLM API call with prompt:\n' + prompt)
        success = self.call_api(prompt)
        if success:
            self.last_api_time = current_time
            self.caption_updated = False
        else:
            self.get_logger().warn('LLM API call failed; will try again immediately...')

    def call_api(self, prompt: str) -> bool:
        '''
        Call the API with the composed prompt
        '''
        url = self.llm_url
        headers = {
            'Content-Type': 'application/json',
            'Authorization': f'Bearer {self.api_key}'
        }
        data = {
            'model': self.model,
            'messages': [{'role': 'user', 'content': prompt}],
            'temperature': self.llm_temperture
        }
        try:
            response = requests.post(url, headers=headers, json=data)
            response.raise_for_status()
            result = response.json()
            response_text = result['choices'][0]['message']['content']
            self.get_logger().info(f'LLM API response: {response_text}')
            twist_cmd = self.parse_response(response_text)
            if twist_cmd:
                self.cmd_pub.publish(twist_cmd)
                # Schedule a stop command after a set amount of seconds
                self.schedule_stop()
                return True
            else:
                return False
        except Exception as e:
            self.get_logger().error(f'LLM API call failed: {e}')
            return False

    def schedule_stop(self):
        '''
        Schedule a one-shot timer to publish a stop command after a set amount of seconds
        '''
        # Cancel any existing stop timer
        if self.pause_timer is not None:
            self.pause_timer.cancel()
        # Create a new timer that calls stop_callback
        self.pause_timer = self.create_timer(self.llm_run, self.stop_callback)

    def stop_callback(self):
        '''
        Callback to publish zero velocities and cancel the stop timer
        '''
        stop_twist = Twist()  # zero velocities
        self.cmd_pub.publish(stop_twist)
        self.get_logger().info('Awaiting new LLM command...')
        # Cancel the timer so it does not repeatedly trigger
        if self.pause_timer is not None:
            self.pause_timer.cancel()
            self.pause_timer = None

    def parse_response(self, response_text: str):
        '''
        Parse the API response text into a Twist message
        '''
        try:
            json_regex = re.compile(r'\{.*\}', re.DOTALL)
            match = json_regex.search(response_text)
            json_str = match.group(0) if match else response_text
            command = json.loads(json_str)
            twist = Twist()
            twist.linear.x = float(command.get('linear', 0.0))
            twist.angular.z = float(command.get('angular', 0.0))
            return twist
        except Exception as e:
            self.get_logger().error(f'Error parsing response JSON: {e}')
            return None

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
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