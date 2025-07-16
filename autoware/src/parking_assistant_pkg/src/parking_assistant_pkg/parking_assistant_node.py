# File: parking_assistant_node.py
import threading, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tkinter as tk
#notRequired from autoware_auto_planning_msgs.srv import SetGoal #new addition for BPP

class ParkingAssistantNode(Node):
    def __init__(self):
        super().__init__('parking_assistant_node')
        # Publisher to Autoware planning goal topic
        self.goal_pub = self.create_publisher(PoseStamped, '/planning/goal_pose', 10)
        
        #notRequired # Service client to Autoware's Behavior Path Planner
        #notRequired self.goal_client = self.create_client(SetGoal, '/planning/behavior_path_planner/set_goal')
        #notRequired if not self.goal_client.wait_for_service(timeout_sec=10.0):
        #notRequired     self.get_logger().error("BPP service not available. Make sure Autoware is running.")

        
        # Subscriber to vehicle pose (localization)
        self.current_pos = None
        self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10)
        # List of predefined parking spots (x, y, yaw in radians)
        self.spots = [
            {"name": "Spot A", "x": 3728.0, "y":  73753.0, "yaw": 0.0},
            {"name": "Spot B", "x": 3726.0, "y":  73751.0, "yaw": 0.0},
            {"name": "Spot C", "x": 3723.0, "y":  73750.0, "yaw": 0.0},
            {"name": "Spot D", "x": 3720.0, "y":  73749.0, "yaw": 0.0},
            {"name": "Spot E", "x": 3717.0, "y":  73747.0, "yaw": 0.0},
            {"name": "Spot F (Reverse)", "x": 5.0, "y":  4.0, "yaw": math.pi},
        ]
        self.goal = None   # currently selected goal (name and coords)

    def odom_callback(self, msg):
        # Update current vehicle position (simple 2D)
        self.current_pos = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def send_goal(self, spot):
        # Publish a goal PoseStamped for the given spot (dict with x,y,yaw)
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'  # Use the map frame
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = spot["x"]
        pose_msg.pose.position.y = spot["y"]
        # Convert yaw to quaternion (z,w)
        qz = math.sin(spot["yaw"]/2.0)
        qw = math.cos(spot["yaw"]/2.0)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)
        
        self.goal_pub.publish(pose_msg)
        self.goal = spot  # remember the goal for GUI
        self.get_logger().info(f"Published goal for {spot['name']} at ({spot['x']}, {spot['y']})")
        
        #notRequired # Create and call the service request
        #notRequired request = SetGoal.Request()
        #notRequired request.goal_pose = pose_msg

        #notRequired future = self.goal_client.call_async(request)

        #notRequired def done_callback(fut):
            #notRequired try:
                #notRequired response = fut.result()
                #notRequired if response.error == response.SUCCESS:
                    #notRequired self.get_logger().info(f"Goal '{spot['name']}' accepted.")
                #notRequired else:
                    #notRequired self.get_logger().warn(f"Goal rejected with error code: {response.error}")
            #notRequired except Exception as e:
                #notRequired self.get_logger().error(f"Service call failed: {e}")

        #notRequired future.add_done_callback(done_callback)
        #notRequired self.goal = spot  # remember the goal for GUI

def main(args=None):
    rclpy.init(args=args)
    node = ParkingAssistantNode()

    # Start ROS spin in a background thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()

    # --- Build Tkinter GUI ---
    root = tk.Tk()
    root.title("Parking Assistant")
    # Labels to show status
    pos_label = tk.Label(root, text="Current Position: (unknown)")
    pos_label.pack(padx=10, pady=5)
    goal_label = tk.Label(root, text="No goal selected")
    goal_label.pack(padx=10, pady=5)
    dist_label = tk.Label(root, text="")
    dist_label.pack(padx=10, pady=5)

    # Create buttons for each parking spot
    def make_send_callback(spot):
        return lambda: send_and_display(spot)
    for spot in node.spots:
        btn = tk.Button(root, text=spot["name"], width=20, 
                        command=make_send_callback(spot))
        btn.pack(pady=2)

    def send_and_display(spot):
        # Publish goal via ROS2
        node.send_goal(spot)
        goal_label.config(text=f"Goal: {spot['name']} at ({spot['x']:.1f}, {spot['y']:.1f})")
        dist_label.config(text="Distance to goal: calculating...")

    # Periodic GUI update (every 100ms)
    def update_gui():
        if node.current_pos and node.goal:
            cx, cy = node.current_pos
            gx, gy = node.goal["x"], node.goal["y"]
            dist = math.hypot(gx - cx, gy - cy)
            pos_label.config(text=f"Current Position: ({cx:.2f}, {cy:.2f})")
            if dist < 0.5:
                dist_label.config(text=f"Reached goal '{node.goal['name']}'!")
            else:
                dist_label.config(text=f"Distance to goal: {dist:.2f} m")
        root.after(100, update_gui)
    update_gui()

    root.mainloop()
    # On GUI close
    rclpy.shutdown()

if __name__ == '__main__':
    main()
