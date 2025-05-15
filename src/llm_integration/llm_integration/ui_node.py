import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image
import ast
import cv2
from cv_bridge import CvBridge, CvBridgeError
import json
import PIL.Image, PIL.ImageTk
import tkinter as tk
import tkinter.ttk as ttk
from tkinter import filedialog

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')
        self.cmd_pub = self.create_publisher(String, '/text_in', 10)
        self.plan_sub = self.create_subscription(String, '/plan_poisoned', self.plan_callback, 10)
        self.index_sub = self.create_subscription(Int32, '/plan_index', self.index_callback, 10)
        self.mask_sub = self.create_subscription(Image, '/camera_masked', self.mask_callback, 10)
        self.memory_sub = self.create_subscription(String, '/camera_caption', self.memory_callback, 10)
        # State
        self.current_plan = []
        self.current_index = 0
        self.memory_list = []
        # Tkinter setup
        self.root = tk.Tk()
        self.root.title('LLM Command UI')
        # Global keybindings
        self.root.bind('<Return>', lambda e: self.send_command())
        self.root.bind('<space>',  lambda e: self.stop_llm())
        # Two-column layout
        main_frame = tk.Frame(self.root)
        main_frame.grid(row=0, column=0, sticky='nsew')
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        # Left: fixed width for controls
        left_panel = tk.Frame(main_frame)
        left_panel.grid(row=0, column=0, sticky='ns')
        main_frame.columnconfigure(0, weight=0)
        # Right: camera view stretches
        cam_panel = tk.Frame(main_frame)
        cam_panel.grid(row=0, column=1, sticky='nsew')
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)
        # LEFT PANEL
        # Input + buttons
        input_frame = tk.Frame(left_panel)
        input_frame.pack(fill=tk.X, padx=5, pady=5)
        self.entry = tk.Entry(input_frame, width=30)
        self.entry.pack(side=tk.LEFT, padx=(0,5))
        # Placeholder logic
        self.entry.insert(0, 'Enter a command')
        self.entry.config(fg='grey')
        self.entry.bind('<FocusIn>', self._clear_placeholder)
        self.entry.bind('<FocusOut>', self._add_placeholder)
        send_button = tk.Button(
            input_frame, text='Send Command', width=15,
            bg='green', fg='white',
            activebackground='#00cc00', activeforeground='white',
            command=self.send_command
        )
        send_button.pack(side=tk.LEFT, padx=5)
        stop_button = tk.Button(
            input_frame, text='LLM Stop', width=15,
            bg='red', fg='white',
            activebackground='#cc0000', activeforeground='white',
            command=self.stop_llm
        )
        stop_button.pack(side=tk.LEFT, padx=5)
        sep1 = ttk.Separator(left_panel, orient='horizontal')
        sep1.pack(fill='x', padx=5, pady=5)
        # Plan display
        tk.Label(left_panel, text='Current Plan:').pack(anchor=tk.W, padx=5)
        self.listbox = tk.Listbox(left_panel, width=40, height=6)
        self.listbox.pack(fill=tk.X, padx=5)
        sep2 = ttk.Separator(left_panel, orient='horizontal')
        sep2.pack(fill='x', padx=5, pady=5)
        # Memory display
        tk.Label(left_panel, text='Memory:').pack(anchor=tk.W, padx=5)
        self.memory_listbox = tk.Listbox(left_panel, width=40, height=10, exportselection=False)
        self.memory_listbox.pack(fill=tk.X, padx=5)
        # ─── RIGHT PANEL ───
        tk.Label(cam_panel, text='Masked Camera View:').pack(anchor=tk.W, padx=5, pady=(5,0))
        self.image_label = tk.Label(cam_panel)
        self.image_label.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        cam_panel.rowconfigure(1, weight=1)
        cam_panel.columnconfigure(0, weight=1)
        # Start the combined Tk/RCLPY spin
        self.root.protocol('WM_DELETE_WINDOW', self.on_shutdown)

    def mask_callback(self, msg: Image):
        '''
        Convert ROS Image to Tkinter-compatible format and update the label
        '''
        try:
            cv_img = CvBridge().imgmsg_to_cv2(msg, 'bgr8')
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGBA)
            pil_img = PIL.Image.fromarray(cv_img)
            imgtk = PIL.ImageTk.PhotoImage(image=pil_img)
            self.image_label.imgtk = imgtk
            self.image_label.config(image=imgtk)
        except Exception as e:
            self.get_logger().error(f'Error in mask_callback: {e}')

    def plan_callback(self, msg: String):
        '''
        Parse the incoming plan JSON and update the listbox
        '''
        try:
            data = json.loads(msg.data)
            self.current_plan = data.get('plan', [])
        except Exception as e:
            self.get_logger().error(f'Error parsing plan JSON: {e}')
            self.current_plan = []
        self.update_listbox()

    def index_callback(self, msg: Int32):
        '''
        Update the current index of the plan
        '''
        self.current_index = msg.data
        self.highlight_current()

    def memory_callback(self, msg: String):
        """
        Parse either valid JSON or Python repr lists for memory,
        ignore 'current_time' if present
        """
        text = msg.data.strip()
        try:
            parsed = json.loads(text)
            if isinstance(parsed, dict) and 'objects' in parsed:
                data_list = parsed['objects']
            elif isinstance(parsed, list):
                data_list = parsed
            else:
                data_list = []
        except Exception:
            try:
                data_list = ast.literal_eval(text)
            except Exception:
                data_list = []
        self.memory_list = data_list
        self.update_memory_listbox()

    def update_listbox(self):
        '''
        Update the listbox with the current plan
        '''
        self.listbox.delete(0, tk.END)
        for action in self.current_plan:
            self.listbox.insert(tk.END, action)
        self.listbox.see(tk.END)
        self.highlight_current()

    def highlight_current(self):
        '''
        Highlight the current action in the listbox
        '''
        self.listbox.selection_clear(0, tk.END)
        if 0 <= self.current_index < len(self.current_plan):
            self.listbox.selection_set(self.current_index)
            self.listbox.activate(self.current_index)

    def update_memory_listbox(self):
        '''
        Update the memory listbox with the current memory
        '''
        self.memory_listbox.delete(0, tk.END)
        for o in self.memory_list:
            entry = f"{o.get('label')} @ {o.get('pos')} ({int(o.get('area',0))} px)"
            self.memory_listbox.insert(tk.END, entry)
        self.memory_listbox.see(tk.END)

    def send_command(self):
        '''
        Send the command from the entry box to the LLM
        '''
        text = self.entry.get().strip()
        # Ignore placeholder
        if not text or text == 'Enter a command':
            return
        self.cmd_pub.publish(String(data=text))
        self.entry.delete(0, tk.END)

    def stop_llm(self):
        '''
        Send an emergency stop command to the LLM
        '''
        self.cmd_pub.publish(String(data='LLM_STOP'))
        self.get_logger().info('Published emergency stop')

    def _clear_placeholder(self, event):
        '''
        Clear the placeholder text when the entry is focused
        '''
        if self.entry.get() == 'Enter a command':
            self.entry.delete(0, tk.END)
            self.entry.config(fg='black')

    def _add_placeholder(self, event):
        '''
        Add the placeholder text if the entry is empty
        '''
        if not self.entry.get().strip():
            self.entry.insert(0, 'Enter a command')
            self.entry.config(fg='grey')

    def spin(self):
        '''
        Main loop for the Tkinter UI and RCLPY
        '''
        try:
            while rclpy.ok():
                self.root.update()
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.on_shutdown()

    def on_shutdown(self):
        if rclpy.ok():
            self.get_logger().info(f'Shutting down {self.get_name()}...')
        self.destroy_node()
        self.root.quit()

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