import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import json

class DeadmanNode(Node):
    def __init__(self):
        super().__init__('deadman_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/lidar', self.lidar_callback, 10)
        self.custom_joy_sub = self.create_subscription(String, '/custom_joy_cmd', self.joy_callback, 10)
        self.llm_sub = self.create_subscription(Twist, '/llm_cmd', self.llm_callback, 10)
        self.too_close = False
        self.safety_distance = float(os.environ.get('SAFETY_STOP_DISTANCE'))
        self.joy_state = {}           # Will store JSON parsed from custom joy node
        self.latest_llm_cmd = Twist() # Fallback command (STOP)
        # Timer to publish command at 10 Hz.
        self.timer = self.create_timer(0.1, self.publish_command)
        # Load environment variables for speed limits
        self.max_forward_speed = float(os.environ.get('MAX_FORWARD_SPEED'))
        self.max_reverse_speed = float(os.environ.get('MAX_REVERSE_SPEED'))
        self.max_turn_left_speed = float(os.environ.get('MAX_TURN_LEFT_SPEED'))
        self.max_turn_right_speed = float(os.environ.get('MAX_TURN_RIGHT_SPEED'))

    def lidar_callback(self, msg: LaserScan):
        '''
        Callback for lidar messages
        '''
        try:
            min_distance = min(msg.ranges)
            # Ignore noisy readings that report near-zero (assumed to be noise)
            if min_distance < 0.01:
                return
            new_too_close = min_distance < self.safety_distance
            if new_too_close != self.too_close:
                self.too_close = new_too_close
                if self.too_close:
                    self.get_logger().warn(f'Obstacle detected (min distance: {min_distance:.2f}m)')
                else:
                    self.get_logger().info('Obstacle cleared')
        except Exception as e:
            self.get_logger().error(f'Lidar error: {e}')

    def joy_callback(self, msg: String):
        '''
        Callback for custom joy node messages
        '''
        try:
            self.joy_state = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f'JSON parse error in joy_callback: {e}')

    def llm_callback(self, msg: Twist):
        '''
        Callback for LLM command messages
        '''
        self.latest_llm_cmd = msg

    def publish_command(self):
        '''
        Publish the final command based on deadman (joystick) state, user input, lidar safety, and LLM commands
        Priority ordering (if deadman is pressed):
          1. User (joystick) input overrides everything
          2. If no user input, and if an obstacle is too close then stop
          3. Otherwise, use the LLM command
        If deadman is not pressed, the UAV is stopped
        '''
        final_cmd = Twist()  # Default: STOP
        # Check deadman switch using custom joy node JSON
        buttons = self.joy_state.get('buttons', {})
        axes = self.joy_state.get('axes', {})
        deadman_pressed = (buttons.get('KEY_310', 0) == 1 and buttons.get('KEY_311', 0) == 1)
        # The UAV should only move if the deadman switch is pressed.
        if not deadman_pressed:
            self.cmd_pub.publish(final_cmd)
            return
        # If deadman is pressed, first check for user override via joystick input
        hat_y = axes.get('ABS_HAT0Y', 0)  # Forward (-1), Reverse (1), Neutral (0)
        hat_x = axes.get('ABS_HAT0X', 0)  # Left (-1), Right (1), Neutral (0)
        if hat_y != 0 or hat_x != 0:
            # User input is available. Use it even if lidar detects an obstacle.
            linear_speed = 0.0
            angular_speed = 0.0
            if hat_y == -1:  # Forward
                linear_speed = self.max_forward_speed
            elif hat_y == 1:  # Reverse
                linear_speed = self.max_reverse_speed
            if hat_x == -1:  # Left
                angular_speed = self.max_turn_left_speed
            elif hat_x == 1:  # Right
                angular_speed = self.max_turn_right_speed
            final_cmd.linear.x = linear_speed
            final_cmd.angular.z = angular_speed
            self.cmd_pub.publish(final_cmd)
            return
        # If no user override, then the LLM command is only used if there is no obstacle
        if self.too_close:
            # Lidar obstacle: ignore LLM commands and force stop
            self.cmd_pub.publish(final_cmd)
            return
        # Otherwise, if LLM command is non-zero, use it
        if abs(self.latest_llm_cmd.linear.x) > 0.01 or abs(self.latest_llm_cmd.angular.z) > 0.01:
            final_cmd = self.latest_llm_cmd
        self.cmd_pub.publish(final_cmd)

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DeadmanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()