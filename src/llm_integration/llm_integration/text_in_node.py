import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import filedialog

class TextInputGUI(Node):
    def __init__(self):
        super().__init__('text_in_node')
        self.publisher_ = self.create_publisher(String, '/text_in', 10)
        self.root = tk.Tk()
        self.root.title('LLM Command Input')
        # Create a frame for the text entry and send button
        input_frame = tk.Frame(self.root)
        input_frame.pack(padx=20, pady=10)
        self.entry = tk.Entry(input_frame, width=50)
        self.entry.pack(side=tk.LEFT, padx=(0, 10))
        send_button = tk.Button(input_frame, text='Send Command', command=self.send_command)
        send_button.pack(side=tk.LEFT)
        # Bind the Return key to the send_command function
        self.entry.bind('<Return>', lambda event: self.send_command())
        # Create a second frame for additional buttons
        button_frame = tk.Frame(self.root)
        button_frame.pack(padx=20, pady=10)
        llm_stop_button = tk.Button(button_frame, text='LLM Stop', command=self.stop_llm)
        llm_stop_button.pack(side=tk.LEFT, padx=10)
        load_file_button = tk.Button(button_frame, text='Load from File', command=self.load_from_file)
        load_file_button.pack(side=tk.LEFT, padx=10)
        self.root.protocol('WM_DELETE_WINDOW', self.on_shutdown)

    def send_command(self):
        '''
        Publish the text in the entry field to the /text_in topic
        '''
        command = self.entry.get()
        msg = String()
        msg.data = command
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published command: {command}')
        self.entry.delete(0, tk.END)

    def stop_llm(self):
        '''
        Publish the LLM_STOP command to the /text_in topic
        '''
        msg = String()
        msg.data = 'LLM_STOP'
        self.publisher_.publish(msg)
        self.get_logger().info('Published LLM_STOP command')

    def load_from_file(self):
        '''
        Load text from a file and insert it into the entry field
        '''
        filename = filedialog.askopenfilename(title='Select Instruction File', filetypes=[('Text files', '*.txt')])
        if filename:
            try:
                with open(filename, 'r') as f:
                    content = f.read()
                self.entry.delete(0, tk.END)
                self.entry.insert(0, content)
                self.get_logger().info(f'Loaded instructions from {filename}')
            except Exception as e:
                self.get_logger().error(f'Failed to load file: {e}')

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
        self.destroy_node()
        self.root.quit()  # Quit Tkinter event loop

    def spin(self):
        try:
            while rclpy.ok():
                self.root.update()
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.on_shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TextInputGUI()
    try:
        node.spin()
    except KeyboardInterrupt:
        node.on_shutdown()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
