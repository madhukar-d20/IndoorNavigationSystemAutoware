import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from flask import Flask, render_template, jsonify, request
import threading
import os
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tier4_external_api_msgs.srv import Engage

import math
import time

# --- Global variables & Constants ---
ros_node = None
current_instruction = "Welcome! Please enter your last name."
GOAL_TOLERANCE = 2.0
NEAR_SPOT_DISTANCE = 7.0

# --- Flask Web Application ---
web_dir = os.path.join(get_package_share_directory('parking_system'), 'web')
app = Flask(__name__, template_folder=web_dir)

@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/get_status')
def get_status():
    if ros_node:
        return jsonify(list(ros_node.parking_spots.values()))
    return jsonify([])

@app.route('/get_instruction')
def get_instruction():
    global current_instruction
    return jsonify({"instruction": current_instruction})

@app.route('/check_reservation', methods=['POST'])
def check_reservation():
    data = request.get_json()
    last_name = data.get('last_name', '').lower()
    if ros_node:
        user_info = ros_node.find_reservation(last_name)
        if user_info:
            return jsonify({"found": True, "user": user_info})
    return jsonify({"found": False})

@app.route('/navigate_to_spot', methods=['POST'])
def navigate_to_spot():
    data = request.get_json()
    spot_id = data.get('spot_id')
    if ros_node:
        ros_node.start_navigation(spot_id)
        return jsonify({"success": True, "message": "Navigation initiated."})
    return jsonify({"success": False, "message": "ROS node not available."})

@app.route('/set_initial_pose', methods=['POST'])
def set_initial_pose():
    if ros_node:
        ros_node.set_initial_pose()
        return jsonify({"success": True, "message": "Initial pose command sent."})
    return jsonify({"success": False, "message": "ROS node not available."})

@app.route('/cancel_navigation', methods=['POST'])
def cancel_navigation():
    if ros_node:
        ros_node.cancel_navigation()
        return jsonify({"success": True})
    return jsonify({"success": False})

# --- ROS 2 Node ---
class ParkingManagementNode(Node):
    def __init__(self):
        super().__init__('parking_management_node')
        self.get_logger().info("Parking Management Node (with Web Dashboard) started.")
        self.cbg = ReentrantCallbackGroup()

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
        self.navigation_phase = None

        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10, callback_group=self.cbg)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10, callback_group=self.cbg)
        self.engage_client = self.create_client(Engage, '/api/autoware/set/engage', callback_group=self.cbg)
        self.odom_subscriber = self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10, callback_group=self.cbg)
        self.get_logger().info("Node setup complete.")

    def find_reservation(self, last_name):
        return self.reservations.get(last_name)

    def odom_callback(self, msg: Odometry):
        global current_instruction
        self.robot_current_pose = msg.pose.pose

        if self.navigating_to_spot_id is None or self.navigation_phase is None:
            return

        spot_data = self.parking_spots[self.navigating_to_spot_id]
        dx = spot_data['x'] - self.robot_current_pose.position.x
        dy = spot_data['y'] - self.robot_current_pose.position.y
        dist = math.hypot(dx, dy)

        if self.navigation_phase == 'DRIVING' and dist < NEAR_SPOT_DISTANCE:
            self.navigation_phase = 'APPROACHING'
            #current_instruction = f"Your spot is approaching. Park on the {spot_data['side']} with ID {spot_data['id']}."
            current_instruction = f"Your spot is approaching. Park on the {spot_data['area']} with ID {spot_data['id']}."
        elif self.navigation_phase == 'APPROACHING' and dist < GOAL_TOLERANCE:
            self.navigation_phase = 'ARRIVED'
            self.parking_spots[self.navigating_to_spot_id]['status'] = 'occupied'
            current_instruction = "Parked Successfully!"
            self.navigating_to_spot_id = None
            self.navigation_phase = None

    def set_initial_pose(self):
        global current_instruction
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
        current_instruction = "Vehicle position has been reset."
        self.get_logger().info("Initial pose set and goal cleared.")

    def start_navigation(self, spot_id):
        global current_instruction
        if self.navigating_to_spot_id is not None: return
        
        self.set_initial_pose()
        time.sleep(1.0) # Give a moment for the pose to settle

        self.navigation_phase = 'DRIVING'
        spot_data = self.parking_spots[spot_id]
        #current_instruction = f"Proceed to {spot_data['floor']}. Your spot is in the {spot_data['area']} Area, on the {spot_data['side']}."
        current_instruction = f"Proceed to {spot_data['floor']}. Your spot is on the {spot_data['area']}side."
        self.get_logger().info(f"Starting navigation to spot {spot_id}.")
        
        self.navigating_to_spot_id = spot_id
        if self.parking_spots[spot_id]['status'] == 'available':
            self.parking_spots[spot_id]['status'] = 'reserved'
        
        nav_thread = threading.Thread(target=self.navigate_to_spot, args=(spot_id,))
        nav_thread.start()

    def cancel_navigation(self):
        self.get_logger().info("Navigation cancelled by user.")
        self.navigating_to_spot_id = None
        self.navigation_phase = None
        self.set_initial_pose()

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

def main(args=None):
    rclpy.init(args=args)
    global ros_node
    ros_node = ParkingManagementNode()
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    app.run(host='0.0.0.0', port=8080, use_reloader=False)
    
    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()
