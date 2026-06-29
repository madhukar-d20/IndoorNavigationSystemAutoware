import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from flask import Flask, render_template, jsonify, request
import threading
import os
import math
import time
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tier4_external_api_msgs.srv import Engage

GOAL_TOLERANCE = 1.0
NEAR_SPOT_DISTANCE = 3.0

# --- Flask Web Application ---
try:
    web_dir = os.path.join(get_package_share_directory('parking_system'), 'web')
    app = Flask(__name__, template_folder=web_dir)
except Exception:
    # Fallback if package not found
    app = Flask(__name__)

ros_node = None
current_instruction = "System Ready. Waiting for Navigation Command..."

@app.route('/')
def index():
    try:
        return render_template('dashboard.html')
    except:
        return "Dashboard Active. Use POST /navigate to move robot"

@app.route('/get_instruction')
def get_instruction():
    global current_instruction
    return jsonify({"instruction": current_instruction})

@app.route('/navigate', methods=['POST'])
def navigate_to_coordinates():
    data = request.get_json()
    
    #Extract coordinates with defaults
    x = data.get('x')
    y = data.get('y')
    yaw = data.get('yaw', 0.0)
    
    if x is None or y is None:
        return jsonify({"success": False, "message": "Missing x or y coordinates"}), 400

    if ros_node:
        #Trigger navigation in the ROS node
        threading.Thread(target=ros_node.start_coord_navigation, args=(float(x), float(y), float(yaw))).start()
        return jsonify({
            "success": True, 
            "message": f"Navigation started to X:{x}, Y:{y}, Yaw:{yaw}"
        })
    
    return jsonify({"success": False, "message": "ROS Node not initialized"}), 500

@app.route('/cancel', methods=['POST'])
def cancel_navigation():
    if ros_node:
        ros_node.cancel_navigation()
        return jsonify({"success": True, "message": "Navigation Cancelled"})
    return jsonify({"success": False}), 500

# --- ROS 2 Node ---
class ParkingManagementNode(Node):
    def __init__(self):
        super().__init__('parking_management_node')
        self.get_logger().info("Parking Management Node (HTTP Mode) started.")
        self.cbg = ReentrantCallbackGroup()

        # Navigation State
        self.target_coordinates = None  # Stores {'x': float, 'y': float}
        self.robot_current_pose = None
        self.navigation_phase = None

        # ROS Communications
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10, callback_group=self.cbg)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, '/planning/mission_planning/goal', 10, callback_group=self.cbg)
        self.engage_client = self.create_client(Engage, '/api/autoware/set/engage', callback_group=self.cbg)
        self.odom_subscriber = self.create_subscription(Odometry, '/localization/kinematic_state', self.odom_callback, 10, callback_group=self.cbg)

    def odom_callback(self, msg: Odometry):
        global current_instruction
        self.robot_current_pose = msg.pose.pose

        if self.target_coordinates is None or self.navigation_phase is None:
            return

        # Calculate Euclidean distance to target
        dx = self.target_coordinates['x'] - self.robot_current_pose.position.x
        dy = self.target_coordinates['y'] - self.robot_current_pose.position.y
        dist = math.hypot(dx, dy)

        if self.navigation_phase == 'DRIVING' and dist < NEAR_SPOT_DISTANCE:
            self.navigation_phase = 'APPROACHING'
            current_instruction = "Approaching Target Location..."
            self.get_logger().info(current_instruction)
        
        elif self.navigation_phase == 'APPROACHING' and dist < GOAL_TOLERANCE:
            self.navigation_phase = 'ARRIVED'
            current_instruction = "Vehicle Arrived Successfully!"
            self.get_logger().info(current_instruction)
            self.target_coordinates = None # Clear target
            self.navigation_phase = None

    def set_initial_pose(self):
        pwc = PoseWithCovarianceStamped()
        pwc.header.frame_id = 'map'
        pwc.header.stamp = self.get_clock().now().to_msg()
        
        # New initial pose
        pwc.pose.pose.position.x = 80.9224
        pwc.pose.pose.position.y = 71.2413
        
        # Calculate Quaternion for Initial Yaw (180.00 degrees)
        yaw_rad = math.radians(180.00) 
        qz = math.sin(yaw_rad / 2.0)
        qw = math.cos(yaw_rad / 2.0)
        pwc.pose.pose.orientation.z = qz
        pwc.pose.pose.orientation.w = qw
        
        self.initial_pose_publisher.publish(pwc)
        self.get_logger().info("Initial Pose Reset.")

    def cancel_navigation(self):
        self.target_coordinates = None
        self.navigation_phase = None
        # Send empty goal to stop planning
        empty_goal = PoseStamped()
        empty_goal.header.stamp = self.get_clock().now().to_msg()
        empty_goal.header.frame_id = 'map'
        self.goal_pose_publisher.publish(empty_goal)
        self.get_logger().info("Navigation Cancelled.")
    
    def start_coord_navigation(self, x, y, yaw):
        global current_instruction
        
        # Reset Pose
        self.set_initial_pose()
        time.sleep(2.0) # Wait for localization to settle
    
        # Update State
        self.target_coordinates = {'x': x, 'y': y, 'yaw':yaw}
        self.navigation_phase = 'DRIVING'
        current_instruction = f"Navigating to X:{x:.2f}, Y:{y:.2f}, Yaw:{yaw:.2f}"
        self.get_logger().info(current_instruction)

        # Publish Goal Pose
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        
        # Convert Target Yaw (radians) to Quaternion
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        goal_msg.pose.orientation.z = qz
        goal_msg.pose.orientation.w = qw
        
        self.get_logger().info(f"GOAL TO AUTOWARE => X: {x}, Y: {y}, Yaw(rad): {yaw}, Qz: {qz:.6f}, Qw: {qw:.6f}")
        
        # Publish goal TWICE with a small delay
        self.goal_pose_publisher.publish(goal_msg)
        time.sleep(0.5)
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose_publisher.publish(goal_msg)
        time.sleep(1.0)
        
        # Engage Autoware
        if self.engage_client.wait_for_service(timeout_sec=2.0):
            request = Engage.Request()
            request.engage = True
            self.engage_client.call_async(request)
            self.get_logger().info("Vehicle Engaged.")
        else:
            self.get_logger().error("Engage service not available!")

def main(args=None):
    rclpy.init(args=args)
    global ros_node
    ros_node = ParkingManagementNode()

    print("Registered Routes:")
    for rule in app.url_map.iter_rules():
        print(f"{rule} -> {rule.methods}")

    # Create Executor to handle ROS callbacks in background
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    
    # Run ROS spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
   
    # Run Flask App in the main thread 
    print("Starting Flask Server on port 8081...")
    app.run(host='0.0.0.0', port=8081, use_reloader=False)
    
    # Cleanup
    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()
