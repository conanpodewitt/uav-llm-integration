import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import tkinter as tk
from tkinter import filedialog
import json

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')
        # Publisher for user commands
        self.cmd_pub = self.create_publisher(String, '/text_in', 10)
        # Subscribers for plans and progress
        self.plan_sub = self.create_subscription(String, '/plan', self.plan_callback, 10)
        self.index_sub = self.create_subscription(Int32, '/plan_index', self.index_callback, 10)

        # Instruction plan state
        self.current_plan = []
        self.current_index = 0

        # Tkinter setup
        self.root = tk.Tk()
        self.root.title('LLM Command UI')

        # Input frame
        input_frame = tk.Frame(self.root)
        input_frame.pack(padx=10, pady=5)
        self.entry = tk.Entry(input_frame, width=50)
        self.entry.pack(side=tk.LEFT, padx=(0,5))
        send_button = tk.Button(input_frame, text='Send Command', command=self.send_command)
        send_button.pack(side=tk.LEFT)
        self.entry.bind('<Return>', lambda e: self.send_command())

        # Control buttons
        ctrl_frame = tk.Frame(self.root)
        ctrl_frame.pack(padx=10, pady=5)
        stop_button = tk.Button(ctrl_frame, text='LLM Stop', command=self.stop_llm)
        stop_button.pack(side=tk.LEFT, padx=5)
        load_button = tk.Button(ctrl_frame, text='Load from File', command=self.load_from_file)
        load_button.pack(side=tk.LEFT, padx=5)

        # Plan display
        plan_frame = tk.Frame(self.root)
        plan_frame.pack(padx=10, pady=5, fill=tk.BOTH, expand=True)
        tk.Label(plan_frame, text='Current Plan:').pack(anchor=tk.W)
        self.listbox = tk.Listbox(plan_frame, width=50, height=10)
        self.listbox.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar = tk.Scrollbar(plan_frame, orient=tk.VERTICAL, command=self.listbox.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox.config(yscrollcommand=scrollbar.set)

        # Shutdown protocol
        self.root.protocol('WM_DELETE_WINDOW', self.on_shutdown)

    def plan_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.current_plan = data.get('plan', [])
            self.current_index = 0
            self.update_listbox()
        except Exception as e:
            self.get_logger().error(f'Error parsing plan: {e}')

    def index_callback(self, msg: Int32):
        self.current_index = msg.data
        self.highlight_current()

    def update_listbox(self):
        self.listbox.delete(0, tk.END)
        for i, action in enumerate(self.current_plan):
            self.listbox.insert(tk.END, action)
        self.highlight_current()

    def highlight_current(self):
        self.listbox.config(selectbackground='green', selectforeground='white')
        self.listbox.selection_clear(0, tk.END)
        if 0 <= self.current_index < len(self.current_plan):
            self.listbox.selection_set(self.current_index)
            self.listbox.activate(self.current_index)

    def send_command(self):
        cmd = self.entry.get().strip()
        if not cmd:
            return
        msg = String(data=cmd)
        self.cmd_pub.publish(msg)
        self.entry.delete(0, tk.END)

    def stop_llm(self):
        msg = String(data='LLM_STOP')
        self.cmd_pub.publish(msg)
        self.get_logger().info('Published emergency stop')

    def load_from_file(self):
        filename = filedialog.askopenfilename(title='Select Instruction File', filetypes=[('Text files','*.txt')])
        if filename:
            try:
                with open(filename,'r') as f:
                    content = f.read().strip()
                self.entry.delete(0, tk.END)
                self.entry.insert(0, content)
                self.get_logger().info(f'Loaded instructions from {filename}')
            except Exception as e:
                self.get_logger().error(f'Error loading file: {e}')

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
        self.destroy_node()
        self.root.quit()

    def spin(self):
        try:
            while rclpy.ok():
                self.root.update()
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.on_shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    try:
        node.spin()
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()