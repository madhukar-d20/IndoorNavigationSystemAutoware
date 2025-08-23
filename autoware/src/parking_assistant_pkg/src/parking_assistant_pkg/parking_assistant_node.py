# File: parking_assistant_node.py
import threading, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tkinter as tk
from tf_transformations import euler_from_quaternion  # pip install transforms3d or from ROS tf2

# angle thresholds (degrees)
STRAIGHT_DEG = 30.0
REVERSE_DEG = 150.0

class ParkingAssistantNode(Node):
    def __init__(self):
        super().__init__('parking_assistant_node')

        # Publisher to Autoware goal topic
        self.goal_pub = self.create_publisher(PoseStamped, '/planning/goal_pose', 10)

        # Subscriber to localization (pose + orientation)
        self.current_pos = None
        self.current_yaw = None
        self.create_subscription(Odometry,
                                 '/localization/kinematic_state',
                                 self.odom_callback, 10)

        # Predefined parking spots
        self.spots = [
            {"name": "Spot A", "x": 3728.0, "y": 73753.0, "yaw": 0.0},
            {"name": "Spot B", "x": 3726.0, "y": 73751.0, "yaw": 0.0},
            {"name": "Spot C", "x": 3723.0, "y": 73750.0, "yaw": 0.0},
            {"name": "Spot D", "x": 3720.0, "y": 73749.0, "yaw": 0.0},
            {"name": "Spot E", "x": 3717.0, "y": 73747.0, "yaw": 0.0},
        ]
        self.goal = None

    def odom_callback(self, msg: Odometry):
        # position
        self.current_pos = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        # orientation → yaw
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

    def send_goal(self, spot):
        # publish goal pose
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = spot["x"]
        pose_msg.pose.position.y = spot["y"]
        # orientation (only for completeness, not used for instruction)
        qz = math.sin(spot["yaw"]/2.0)
        qw = math.cos(spot["yaw"]/2.0)
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw

        self.goal_pub.publish(pose_msg)
        self.goal = spot
        self.get_logger().info(f"Published goal for {spot['name']} at ({spot['x']}, {spot['y']})")

def main(args=None):
    rclpy.init(args=args)
    node = ParkingAssistantNode()

    # spin ROS in background
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    # --- Build Tkinter GUI ---
    root = tk.Tk()
    root.title("Parking Assistant")

    # Labels
    pos_label         = tk.Label(root, text="Current Position: (unknown)")
    goal_label        = tk.Label(root, text="No goal selected")
    dist_label        = tk.Label(root, text="")
    instruction_label = tk.Label(root, text="", font=('Helvetica', 16, 'bold'))

    for lbl in (pos_label, goal_label, dist_label, instruction_label):
        lbl.pack(padx=10, pady=5)

    # Button callback
    def on_click(spot):
        node.send_goal(spot)
        goal_label.config(text=f"Goal: {spot['name']} at ({spot['x']:.1f}, {spot['y']:.1f})")
        dist_label.config(text="Distance to goal: calculating...")
        instruction_label.config(text="")  # reset

    # Create buttons
    for spot in node.spots:
        btn = tk.Button(root,
                        text=spot["name"],
                        width=20, height=2,
                        command=lambda s=spot: on_click(s))
        btn.pack(pady=2)

    # Main update loop
    def update_gui():
        if node.current_pos and node.current_yaw is not None:
            cx, cy = node.current_pos
            pos_label.config(text=f"Current Position: ({cx:.2f}, {cy:.2f})")

            if node.goal:
                gx, gy = node.goal["x"], node.goal["y"]
                # distance
                dist = math.hypot(gx - cx, gy - cy)
                if dist < 1.5:
                    dist_label.config(text="Goal reached!")
                    instruction_label.config(text="🎉")
                else:
                    dist_label.config(text=f"Distance: {dist:.2f} m")
                    # desired heading
                    desired = math.atan2(gy - cy, gx - cx)  # [-π, π]
                    # error = desired - current, normalized to [-π, π]
                    err = (desired - node.current_yaw + math.pi) % (2*math.pi) - math.pi
                    deg = math.degrees(err)
                    # choose instruction
                    if abs(deg) < STRAIGHT_DEG:
                        instr = "Go straight"
                    elif abs(deg) > REVERSE_DEG:
                        instr = "Reverse"
                    elif deg > 0:
                        instr = "Turn left"
                    else:
                        instr = "Turn right"
                    instruction_label.config(text=instr)

        root.after(50, update_gui)

    update_gui()
    root.mainloop()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
