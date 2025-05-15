import json
import random
import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PoisonNode(Node):
    def __init__(self):
        super().__init__('poison_node')
        self.sub = self.create_subscription(String, '/plan', self.cb, 10)
        self.pub = self.create_publisher(String, '/plan_poisoned', 10)
        self.p_poison = float(os.environ.get('POISON_RATE', 0.0))
        self.skip_next = False
        self.get_logger().info(f'Poison rate: {self.p_poison}')
        # Define malicious replacements
        self.malicious_actions = ['small_backward', 'big_backward']

    def cb(self, msg: String):
        '''
        Callback for plan poisoning
        '''
        if self.skip_next:
            self.get_logger().info('Skipping plan due to cooldown')
            self.skip_next = False
            self.pub.publish(msg)
            return
        data = json.loads(msg.data)
        plan = data.get('plan', [])
        plan_poisoned = False
        for i, act in enumerate(plan):
            if random.random() < self.p_poison:
                old = plan[i]
                plan[i] = random.choice(self.malicious_actions)
                plan_poisoned = True
                self.get_logger().warn(f'Replaced action "{old}" with "{plan[i]}"')
                break
        if plan_poisoned:
            self.skip_next = True
        poisoned = json.dumps({'plan': plan})
        self.pub.publish(String(data=poisoned))

    def on_shutdown(self): 
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PoisonNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()