import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox
import threading
import math
import time
import random

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tier4_external_api_msgs.srv import Engage

# --- Constants ---
GOAL_TOLERANCE = 2.0
NEAR_SPOT_DISTANCE = 7.0

def get_yaw_from_quaternion(q):
    """
    Helper function to convert quaternion (x,y,z,w) to yaw (z-rotation)
    """
    # roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    
    # yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return yaw

class ParkingApp(tk.Tk):
    def __init__(self, ros_node, *args, **kwargs):
        tk.Tk.__init__(self, *args, **kwargs)
        self.ros_node = ros_node
        self.ros_node.app = self

        self.title_font = tkfont.Font(family='Helvetica', size=18, weight="bold")
        self.title("Klinikum Parking Assistant")
        self.geometry("500x800") # Increased height for legend and tools

        container = tk.Frame(self)
        container.pack(side="top", fill="both", expand=True)
        container.grid_rowconfigure(0, weight=1)
        container.grid_columnconfigure(0, weight=1)

        self.frames = {}
        for F in (WelcomeView, SelectionView, NavigationView):
            page_name = F.__name__
            frame = F(parent=container, controller=self)
            self.frames[page_name] = frame
            frame.grid(row=0, column=0, sticky="nsew")

        self.show_frame("WelcomeView")

    def show_frame(self, page_name):
        frame = self.frames[page_name]
        frame.tkraise()

class WelcomeView(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller

        label = tk.Label(self, text="Welcome to the Parking Garage", font=controller.title_font)
        label.pack(side="top", fill="x", pady=20)

        entry_label = tk.Label(self, text="Enter your last name:")
        entry_label.pack(pady=5)
        self.last_name_entry = tk.Entry(self)
        self.last_name_entry.pack(pady=5)

        button1 = tk.Button(self, text="Find My Reservation", command=self.find_reservation, height=2, width=25)
        button1.pack(pady=10)

        button2 = tk.Button(self, text="I have no reservation",
                            command=lambda: controller.show_frame("SelectionView"), height=2, width=25)
        button2.pack(pady=10)

        # --- DEVELOPER TOOLS SECTION ---
        dev_frame = tk.LabelFrame(self, text="Developer Tools", padx=10, pady=10)
        dev_frame.pack(side="bottom", fill="x", padx=20, pady=20)
        
        coord_btn = tk.Button(dev_frame, text="Get Vehicle Coordinates (from Autoware)", 
                              command=self.get_coordinates, bg="#d3d3d3")
        coord_btn.pack(fill="x")

    def find_reservation(self):
        last_name = self.last_name_entry.get().lower().strip()
        if not last_name:
            messagebox.showwarning("Input Error", "Please enter a last name.")
            return
        
        user_info = self.controller.ros_node.find_reservation(last_name)
        if user_info:
            nav_frame = self.controller.frames["NavigationView"]
            nav_frame.set_welcome_message(f"Welcome, {user_info['full_name']}!")
            self.controller.ros_node.start_navigation(user_info['spot_id'])
            self.controller.show_frame("NavigationView")
        else:
            messagebox.showerror("Not Found", "Name not found under reservations.")
            self.last_name_entry.delete(0, tk.END)

    def get_coordinates(self):
        pose = self.controller.ros_node.robot_current_pose
        if pose:
            x = pose.position.x
            y = pose.position.y
            yaw_rad = get_yaw_from_quaternion(pose.orientation)
            yaw_deg = math.degrees(yaw_rad)
            
            msg = (f"Current Vehicle Pose:\n\n"
                   f"X: {x:.4f}\n"
                   f"Y: {y:.4f}\n"
                   f"Yaw (Rad): {yaw_rad:.4f}\n"
                   f"Yaw (Deg): {yaw_deg:.2f}\n\n"
                   f"Use these values for your map configuration.")
            
            # Print to terminal as well for easy copying
            self.controller.ros_node.get_logger().info(f"\n[COORDINATE CHECK]\n   x: {x}\n   y: {y}\n   yaw: {yaw_rad}")
            
            messagebox.showinfo("Coordinates", msg)
        else:
            messagebox.showwarning("No Data", "No Odometry received yet.\n\nPlease set a '2D Pose Estimate' in RViz first.")

class SelectionView(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller
        self.current_filter = "All"
        
        label = tk.Label(self, text="Select an Available Spot", font=controller.title_font)
        label.pack(side="top", fill="x", pady=10)
        
        # --- Filter Buttons ---
        filter_frame = tk.Frame(self)
        filter_frame.pack(pady=5)
        tk.Label(filter_frame, text="Filter:").pack(side="left", padx=5)
        tk.Button(filter_frame, text="All", command=lambda: self.set_filter("All")).pack(side="left")
        tk.Button(filter_frame, text="Disabled", command=lambda: self.set_filter("Disabled")).pack(side="left")
        tk.Button(filter_frame, text="Women", command=lambda: self.set_filter("Women")).pack(side="left")
        tk.Button(filter_frame, text="EV Only ⚡", command=lambda: self.set_filter("EV")).pack(side="left")

        # --- Legend (Key) ---
        legend_frame = tk.Frame(self)
        legend_frame.pack(pady=5)
        self.create_legend_item(legend_frame, "#87CEFA", "Disabled").pack(side="left", padx=5)
        self.create_legend_item(legend_frame, "#FFB6C1", "Women").pack(side="left", padx=5)
        self.create_legend_item(legend_frame, "#90EE90", "General").pack(side="left", padx=5)
        tk.Label(legend_frame, text="⚡ = Charger").pack(side="left", padx=10)

        self.spot_frame = tk.Frame(self)
        self.spot_frame.pack(pady=10, fill="both", expand=True)
        
        button = tk.Button(self, text="Go Back",
                           command=lambda: controller.show_frame("WelcomeView"))
        button.pack(pady=20)

    def create_legend_item(self, parent, color, text):
        frame = tk.Frame(parent)
        tk.Label(frame, bg=color, width=2).pack(side="left")
        tk.Label(frame, text=text).pack(side="left")
        return frame

    def set_filter(self, spot_type):
        self.current_filter = spot_type
        self.controller.ros_node.update_ui()

    def update_spots(self, spots):
        for widget in self.spot_frame.winfo_children():
            widget.destroy()

        for spot_id, spot_data in sorted(spots.items()):
            # --- Filter Logic ---
            if self.current_filter == "Disabled" and spot_data.get('type') != "Disabled": continue
            if self.current_filter == "Women" and spot_data.get('type') != "Women": continue
            if self.current_filter == "EV" and not spot_data.get('is_ev', False): continue

            # --- Color Logic ---
            color = "#90EE90" # Default Green (General)
            
            if spot_data.get('type') == 'Disabled':
                color = "#87CEFA" # Light Sky Blue
            elif spot_data.get('type') == 'Women':
                color = "#FFB6C1" # Light Pink
            
            # Status Overrides (Occupied/Reserved take priority over type colors)
            state = "normal"
            if spot_data['status'] == 'occupied':
                color = "#F08080" # Light Coral (Red)
                state = "disabled"
            elif spot_data['status'] == 'reserved':
                color = "#FFD700" # Gold (Yellow)
                state = "disabled"
            
            # --- Text Logic ---
            spot_text = f"Spot {spot_id}\n{spot_data.get('type', 'General')}"
            if spot_data.get('is_ev', False):
                spot_text += " ⚡"

            btn = tk.Button(self.spot_frame, text=spot_text, bg=color, state=state, height=3, width=20,
                            command=lambda id=spot_id: self.select_spot(id))
            btn.pack(pady=5)

    def select_spot(self, spot_id):
        nav_frame = self.controller.frames["NavigationView"]
        nav_frame.set_welcome_message(f"Proceeding to Spot {spot_id}")
        self.controller.ros_node.start_navigation(spot_id)
        self.controller.show_frame("NavigationView")

class NavigationView(tk.Frame):
    def __init__(self, parent, controller):
        tk.Frame.__init__(self, parent)
        self.controller = controller
        
        self.welcome_label = tk.Label(self, text="", font=controller.title_font)
        self.welcome_label.pack(side="top", fill="x", pady=10)
        
        self.instruction_label = tk.Label(self, text="Calculating route...", font=("Helvetica", 16, "bold"), wraplength=350)
        self.instruction_label.pack(pady=20)

        self.info_frame = tk.Frame(self)
        self.info_frame.pack(pady=10)
        self.distance_label = tk.Label(self.info_frame, text="Distance: N/A", font=("Helvetica", 12))
        self.distance_label.pack()
        self.phase_label = tk.Label(self.info_frame, text="Phase: Idle", font=("Helvetica", 12))
        self.phase_label.pack()
        
        self.cancel_button = tk.Button(self, text="Cancel Navigation", command=self.go_back)
        button = tk.Button(self, text="Go Back to Main Menu", command=self.go_back)
        button.pack(pady=20)

    def set_welcome_message(self, message):
        self.welcome_label.config(text=message)
    
    def set_instruction(self, instruction):
        self.instruction_label.config(text=instruction)

    def update_nav_info(self, distance, phase):
        self.distance_label.config(text=f"Distance to Goal: {distance:.2f} m")
        self.phase_label.config(text=f"Phase: {phase}")
        if phase != "Idle" and not self.cancel_button.winfo_ismapped():
            self.cancel_button.pack(pady=20)
        elif phase == "Idle" and self.cancel_button.winfo_ismapped():
            self.cancel_button.pack_forget()

    def go_back(self):
        self.controller.ros_node.cancel_navigation()
        self.controller.show_frame("WelcomeView")

class ParkingManagementNode(Node):
    def __init__(self):
        super().__init__('parking_management_node_tkinter')
        self.get_logger().info("Tkinter Parking Management Node started.")
        self.app = None

        self.reservations = {
            "name1": {"full_name": "user name1", "spot_id": 3},
            "name2": {"full_name": "user name2", "spot_id": 5},
            "name3": {"full_name": "user name3", "spot_id": 8},
        }

        self.parking_spots = {
            # Disabled Spots (1 & 2)
            1: {"id": 1, "floor": "P1", "area": "Right", "side": "Right-Hand Side", "type": "Disabled", "is_ev": True,  "x": 69.8124, "y": 56.2120, "yaw": -2.9004, "status": "available"},
            2: {"id": 2, "floor": "P1", "area": "Left", "side": "Right-Hand Side", "type": "Disabled", "is_ev": False, "x": 76.5557, "y": 55.8214, "yaw": -0.1826, "status": "available"},
            
            # Women's Spots (3-6)
            3: {"id": 3, "floor": "P1", "area": "Right", "side": "Left-Hand Side", "type": "Women", "is_ev": True,  "x": 69.5355, "y": 52.5298, "yaw": -2.9054, "status": "available"},
            4: {"id": 4, "floor": "P1", "area": "Left", "side": "Left-Hand Side", "type": "Women", "is_ev": True,  "x": 76.8895, "y": 52.6976, "yaw": -0.1771, "status": "available"},
            5: {"id": 5, "floor": "P1", "area": "Right", "side": "Left-Hand Side", "type": "Women", "is_ev": False, "x": 69.0713, "y": 48.7766, "yaw": -2.9431, "status": "available"},
            6: {"id": 6, "floor": "P1", "area": "Left", "side": "Right-Hand Side", "type": "Women", "is_ev": False, "x": 76.7348, "y": 48.7503, "yaw": -0.1950, "status": "available"},
            
            # General Spots (7-10)
            7: {"id": 7, "floor": "P1", "area": "Right", "side": "Right-Hand Side", "type": "General", "is_ev": False, "x": 69.6403, "y": 45.1934, "yaw": -2.8840, "status": "available"},
            8: {"id": 8, "floor": "P1", "area": "Left", "side": "Left-Hand Side", "type": "General", "is_ev": False, "x": 76.8160, "y": 45.0976, "yaw": -0.1679, "status": "available"},
            9: {"id": 9, "floor": "P1", "area": "Right", "side": "Left-Hand Side", "type": "General", "is_ev": False, "x": 69.4512, "y": 41.3414, "yaw": -2.9248, "status": "available"},
            10:{"id": 10,"floor": "P1", "area": "Left", "side": "Left-Hand Side", "type": "General", "is_ev": False, "x": 76.8137, "y": 41.1137, "yaw": -0.1858, "status": "available"},
        }
        for user in self.reservations.values():
            if user['spot_id'] in self.parking_spots:
                self.parking_spots[user['spot_id']]['status'] = 'reserved'

        self.navigating_to_spot_id = None
        self.robot_current_pose = None
        self.navigation_phase = "Idle"

        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10)
        self.engage_client = self.create_client(Engage, '/api/autoware/set/engage')
        self.odom_subscriber = self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        
        self.ui_updater = self.create_timer(1.0, self.update_ui)
        self.digital_twin_updater = self.create_timer(5.0, self.simulate_occupancy_change)

    def find_reservation(self, last_name):
        return self.reservations.get(last_name)

    def odom_callback(self, msg: Odometry):
        self.robot_current_pose = msg.pose.pose
        if self.navigating_to_spot_id is None:
            if self.app:
                self.app.frames["NavigationView"].update_nav_info(0.0, "Idle")
            return

        spot_data = self.parking_spots[self.navigating_to_spot_id]
        dx = spot_data['x'] - self.robot_current_pose.position.x
        dy = spot_data['y'] - self.robot_current_pose.position.y
        dist = math.hypot(dx, dy)

        instruction = ""
        if self.navigation_phase == 'DRIVING' and dist < NEAR_SPOT_DISTANCE:
            self.navigation_phase = 'APPROACHING'
            instruction = f"Your spot is approaching.\nPark in Spot {spot_data['id']}."
        elif self.navigation_phase == 'APPROACHING' and dist < GOAL_TOLERANCE:
            self.navigation_phase = 'ARRIVED'
            self.parking_spots[self.navigating_to_spot_id]['status'] = 'occupied'
            instruction = "✅ Parked Successfully!"
            self.navigating_to_spot_id = None
        
        if self.app:
            if instruction:
                self.app.frames["NavigationView"].set_instruction(instruction)
            self.app.frames["NavigationView"].update_nav_info(dist, self.navigation_phase or "Idle")

    def set_initial_pose(self):
        pwc = PoseWithCovarianceStamped()
        pwc.header.frame_id = 'map'
        pwc.header.stamp = self.get_clock().now().to_msg()
        pwc.pose.pose.position.x = 72.8894
        pwc.pose.pose.position.y = 66.3720
        yaw_rad = math.radians(-89.18)
        qz = math.sin(yaw_rad / 2.0); qw = math.cos(yaw_rad / 2.0)
        pwc.pose.pose.orientation.z = qz; pwc.pose.pose.orientation.w = qw
        self.initial_pose_publisher.publish(pwc)
        
        empty_goal = PoseStamped()
        empty_goal.header.stamp = self.get_clock().now().to_msg()
        empty_goal.header.frame_id = 'map'
        self.goal_pose_publisher.publish(empty_goal)
        self.get_logger().info("Initial pose set and goal cleared.")

    def start_navigation(self, spot_id):
        if self.navigating_to_spot_id is not None: return
        
        self.set_initial_pose()
        time.sleep(1.0)

        self.navigation_phase = 'DRIVING'
        spot_data = self.parking_spots[spot_id]
        #instruction = f"Proceed to {spot_data['floor']}.\nYour spot is in the {spot_data['area']} Area, on the {spot_data['side']}."
        instruction = f"Proceed to {spot_data['floor']}.\nYour spot is on the {spot_data['area']} side."
        
        if self.app:
            self.app.frames["NavigationView"].set_instruction(instruction)

        self.navigating_to_spot_id = spot_id
        if self.parking_spots[spot_id]['status'] == 'available':
            self.parking_spots[spot_id]['status'] = 'reserved'
        
        nav_thread = threading.Thread(target=self.navigate_to_spot, args=(spot_id,))
        nav_thread.start()

    def cancel_navigation(self):
        self.get_logger().info("Navigation cancelled.")
        self.navigating_to_spot_id = None
        self.navigation_phase = "Idle"
        self.set_initial_pose()
        if self.app:
            self.app.frames["NavigationView"].set_instruction("Navigation Cancelled.")

    def navigate_to_spot(self, spot_id):
        spot_data = self.parking_spots[spot_id]
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = float(spot_data['x'])
        goal_msg.pose.position.y = float(spot_data['y'])
        qz = math.sin(spot_data["yaw"] / 2.0); qw = math.cos(spot_data["yaw"] / 2.0)
        goal_msg.pose.orientation.z = qz; goal_msg.pose.orientation.w = qw
        self.goal_pose_publisher.publish(goal_msg)
        time.sleep(2.0)
        
        request = Engage.Request()
        request.engage = True
        self.engage_client.call_async(request)

    def update_ui(self):
        if self.app:
            self.app.frames["SelectionView"].update_spots(self.parking_spots)

    def simulate_occupancy_change(self):
        changeable_spots = [id for id, data in self.parking_spots.items() if data['status'] != 'reserved']
        if not changeable_spots:
            return
        spot_to_change = random.choice(changeable_spots)
        if self.parking_spots[spot_to_change]['status'] == 'available':
            if random.random() < 0.1:
                self.parking_spots[spot_to_change]['status'] = 'occupied'
                self.get_logger().info(f"Spot {spot_to_change} is now OCCUPIED.")
        elif self.parking_spots[spot_to_change]['status'] == 'occupied':
            if random.random() < 0.3:
                self.parking_spots[spot_to_change]['status'] = 'available'
                self.get_logger().info(f"Spot {spot_to_change} is now AVAILABLE.")

def main(args=None):
    rclpy.init(args=args)
    
    node = ParkingManagementNode()
    app = ParkingApp(node)
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        app.mainloop()
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()
