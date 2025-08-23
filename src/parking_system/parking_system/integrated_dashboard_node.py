import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from flask import Flask, render_template, jsonify, request
import threading
import os
from ament_index_python.packages import get_package_share_directory

from parking_system_interfaces.srv import BookSpot
from std_srvs.srv import Trigger

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from tier4_external_api_msgs.srv import Engage

import math
import time

# --- Global variables ---
ros_node = None
current_instruction = "Please select a spot."

# --- Flask Web Application ---
web_dir = os.path.join(get_package_share_directory('parking_system'), 'web')
app = Flask(__name__, template_folder=web_dir)

@app.route('/')
def index():
    return render_template('dashboard.html')

@app.route('/get_status')
def get_status():
    if ros_node:
        spots_list = [spot for spot in ros_node.parking_spots.values()]
        return jsonify(spots_list)
    return jsonify([])

@app.route('/get_instruction')
def get_instruction():
    global current_instruction
    return jsonify({"instruction": current_instruction})

@app.route('/set_initial_pose', methods=['POST'])
def set_initial_pose():
    if ros_node:
        ros_node.set_initial_pose()
        return jsonify({"success": True, "message": "Initial pose command sent."})
    return jsonify({"success": False, "message": "ROS node not available."})

@app.route('/navigate_to_spot', methods=['POST'])
def navigate_to_spot():
    data = request.get_json()
    spot_id = data.get('spot_id')
    if ros_node:
        success, message = ros_node.book_spot_and_navigate(spot_id)
        return jsonify({"success": success, "message": message})
    return jsonify({"success": False, "message": "ROS node not available."})

@app.route('/free_spot', methods=['POST'])
def free_spot():
    data = request.get_json()
    spot_id = data.get('spot_id')
    if ros_node:
        success, message = ros_node.free_spot_callback(spot_id)
        return jsonify({"success": success, "message": message})
    return jsonify({"success": False, "message": "ROS node not available."})


# --- ROS 2 Node ---
class ParkingManagementNode(Node):
    def __init__(self):
        super().__init__('parking_management_node')
        self.get_logger().info("Parking Management Node (Integrated with Flask) started.")
        self.cbg = ReentrantCallbackGroup()

        self.parking_spots = {
            1: {"id": 1, "x": 3728.0, "y": 73753.0, "z": 0.0, "yaw": 1.57, "status": "available"},
            2: {"id": 2, "x": 3726.0, "y": 73751.0, "z": 0.0, "yaw": 1.57, "status": "available"},
            3: {"id": 3, "x": 3723.0, "y": 73750.0, "z": 0.0, "yaw": 1.57, "status": "available"},
            4: {"id": 4, "x": 3720.0, "y": 73749.0, "z": 0.0, "yaw": 1.57, "status": "available"},
            5: {"id": 5, "x": 3717.0, "y": 73747.0, "z": 0.0, "yaw": 1.57, "status": "available"},
        }
        self.booked_spot_id = None
        self.robot_current_pose = None

        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10, callback_group=self.cbg)
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, '/planning/mission_planning/goal', 10, callback_group=self.cbg)
        self.engage_client = self.create_client(
            Engage, '/api/autoware/set/engage', callback_group=self.cbg)
        if not self.engage_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Engage service is not available.")
            
        self.odom_subscriber = self.create_subscription(
            Odometry, '/localization/kinematic_state', self.odom_callback, 10, callback_group=self.cbg)

        self.get_logger().info("Node setup complete.")

    def odom_callback(self, msg: Odometry):
        global current_instruction
        self.robot_current_pose = msg.pose.pose

        if self.booked_spot_id is None:
            current_instruction = "Please select a spot."
            return

        spot_data = self.parking_spots[self.booked_spot_id]
        dx = spot_data['x'] - self.robot_current_pose.position.x
        dy = spot_data['y'] - self.robot_current_pose.position.y
        dist = math.hypot(dx, dy)

        if dist < 2.0:
            if self.parking_spots[self.booked_spot_id]['status'] == 'reserved':
                self.get_logger().info(f"Vehicle has arrived at Spot {self.booked_spot_id}.")
                self.parking_spots[self.booked_spot_id]['status'] = 'occupied'
                self.booked_spot_id = None
            current_instruction = "Arrived!"
            return

        q = self.robot_current_pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        desired_angle = math.atan2(dy, dx)
        angle_error = (desired_angle - yaw + math.pi) % (2 * math.pi) - math.pi
        deg_error = math.degrees(angle_error)

        if abs(deg_error) < 30.0: current_instruction = "Go Straight"
        elif abs(deg_error) > 160.0: current_instruction = "Reverse"
        elif deg_error > 0: current_instruction = "Turn Left"
        else: current_instruction = "Turn Right"

    def set_initial_pose(self):
        self.get_logger().info("Setting initial pose via service call...")
        pwc = PoseWithCovarianceStamped()
        pwc.header.frame_id = 'map'
        pwc.header.stamp = self.get_clock().now().to_msg()
        pwc.pose.pose.position.x = 3711.23
        pwc.pose.pose.position.y = 73718.04
        pwc.pose.pose.orientation.w = 1.0
        pwc.pose.covariance[0] = 0.25; pwc.pose.covariance[7] = 0.25; pwc.pose.covariance[35] = 0.068
        self.initial_pose_publisher.publish(pwc)
        self.get_logger().info("Initial pose published.")

    def book_spot_and_navigate(self, spot_id):
        self.get_logger().info(f"Received booking request for spot {spot_id}.")
        if spot_id not in self.parking_spots:
            return False, f"Spot {spot_id} does not exist."
        if self.parking_spots[spot_id]["status"] == "available":
            self.parking_spots[spot_id]["status"] = "reserved"
            self.booked_spot_id = spot_id
            nav_thread = threading.Thread(target=self.navigate_to_spot, args=(spot_id,))
            nav_thread.start()
            return True, f"Spot {spot_id} reserved! Navigating now."
        else:
            return False, f"Spot {spot_id} is currently {self.parking_spots[spot_id]['status']}."

    def free_spot_callback(self, spot_id):
        if spot_id in self.parking_spots:
            self.parking_spots[spot_id]['status'] = 'available'
            self.get_logger().info(f"Spot {spot_id} has been manually set to 'available'.")
            return True, f"Spot {spot_id} is now available."
        return False, f"Spot {spot_id} does not exist."

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
        self.get_logger().info(f"Published goal for spot {spot_id}.")
        time.sleep(2.0)
        self.call_engage_service()

    def call_engage_service(self):
        if not self.engage_client.service_is_ready():
            self.get_logger().error("Engage service not ready.")
            return
        request = Engage.Request()
        request.engage = True
        future = self.engage_client.call_async(request)
        future.add_done_callback(self.engage_callback)
        self.get_logger().info("Engage service called.")

    def engage_callback(self, future):
        try:
            response = future.result()
            if response.status.success:
                self.get_logger().info("Vehicle engaged successfully!")
            else:
                self.get_logger().error(f"Failed to engage vehicle: {response.status.message}")
        except Exception as e:
            self.get_logger().error(f"Engage service call failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    global ros_node
    ros_node = ParkingManagementNode()
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    time.sleep(2.0)
    ros_node.get_logger().info("Flask web server is starting...")
    app.run(host='0.0.0.0', port=8080, use_reloader=False)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
