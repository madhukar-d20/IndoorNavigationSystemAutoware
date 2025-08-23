import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Your custom messages and services
from parking_system_interfaces.msg import ParkingSpot, ParkingStatusArray
from parking_system_interfaces.srv import BookSpot

# Standard ROS 2 and Autoware messages
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger # For the dashboard's initial pose button

# Correct Autoware service for engaging
from tier4_external_api_msgs.srv import Engage

import math
import random
import time

class ParkingManagementNode(Node):
    def __init__(self):
        super().__init__('parking_management_node')
        self.get_logger().info("Parking Management Node (with Web Dashboard Logic) started.")

        self.cbg = ReentrantCallbackGroup()

        # --- Parking Spot Data ---
        self.parking_spots = {
            1: {"x": 3728.0, "y": 73753.0, "z": 0.0, "yaw": 0.0, "status": "available"},
            2: {"x": 3726.0, "y": 73751.0, "z": 0.0, "yaw": 0.0, "status": "available"},
            3: {"x": 3723.0, "y": 73750.0, "z": 0.0, "yaw": 0.0, "status": "occupied"},
            4: {"x": 3720.0, "y": 73749.0, "z": 0.0, "yaw": 0.0, "status": "available"},
            5: {"x": 3717.0, "y": 73747.0, "z": 0.0, "yaw": 0.0, "status": "available"},
        }
        self.booked_spot_id = None
        self.robot_current_pose = None

        # --- Publishers ---
        self.parking_status_publisher = self.create_publisher(
            ParkingStatusArray, '/parking_status', 10, callback_group=self.cbg)
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10, callback_group=self.cbg)
        self.goal_pose_publisher = self.create_publisher(
            PoseStamped, '/planning/mission_planning/goal', 10, callback_group=self.cbg)

        # --- Service Client ---
        self.engage_client = self.create_client(
            Engage, '/api/autoware/set/engage', callback_group=self.cbg)
        if not self.engage_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Engage service '/api/autoware/set/engage' is not available.")

        # --- Subscribers ---
        self.robot_pose_subscriber = self.create_subscription(
            Odometry, '/localization/kinematic_state', self.robot_pose_callback, 10, callback_group=self.cbg)

        # --- Services (for Web Dashboard) ---
        self.book_spot_service = self.create_service(
            BookSpot, '/book_spot', self.book_spot_callback, callback_group=self.cbg)
        self.set_initial_pose_service = self.create_service(
            Trigger, '/set_initial_pose', self.set_initial_pose_callback, callback_group=self.cbg)

        # --- Timers ---
        self.status_publish_timer = self.create_timer(1.0, self.publish_parking_status)
        self.occupancy_simulation_timer = self.create_timer(10.0, self.simulate_occupancy_change)

        self.get_logger().info("Node setup complete. Ready to receive booking requests.")

    def publish_parking_status(self):
        msg = ParkingStatusArray()
        for spot_id, data in self.parking_spots.items():
            spot_msg = ParkingSpot()
            spot_msg.id = spot_id
            spot_msg.x = float(data["x"])
            spot_msg.y = float(data["y"])
            spot_msg.z = float(data["z"])
            spot_msg.yaw = float(data["yaw"])
            spot_msg.status = data["status"]
            msg.spots.append(spot_msg)
        self.parking_status_publisher.publish(msg)

    def simulate_occupancy_change(self):
        available_spots_to_change = [id for id, data in self.parking_spots.items() if id != self.booked_spot_id]
        if not available_spots_to_change:
            return
        spot_id = random.choice(available_spots_to_change)
        if self.parking_spots[spot_id]["status"] == "available":
            self.parking_spots[spot_id]["status"] = "occupied"
        elif self.parking_spots[spot_id]["status"] == "occupied":
            self.parking_spots[spot_id]["status"] = "available"
        self.get_logger().info(f"Simulated change: Spot {spot_id} is now {self.parking_spots[spot_id]['status']}")

    def robot_pose_callback(self, msg: Odometry):
        self.robot_current_pose = msg.pose.pose

    def book_spot_callback(self, request, response):
        spot_id = request.spot_id
        self.get_logger().info(f"Received booking request for spot {spot_id}.")
        if spot_id not in self.parking_spots:
            response.success = False
            response.message = f"Spot {spot_id} does not exist."
            return response
        if self.parking_spots[spot_id]["status"] == "available":
            self.parking_spots[spot_id]["status"] = "reserved"
            self.booked_spot_id = spot_id
            response.success = True
            response.message = f"Spot {spot_id} successfully reserved! Navigating now."
            self.get_logger().info(response.message)
            self.navigate_to_spot(spot_id)
        else:
            response.success = False
            response.message = f"Spot {spot_id} is currently {self.parking_spots[spot_id]['status']}."
            self.get_logger().warn(response.message)
        return response

    def navigate_to_spot(self, spot_id):
        spot_data = self.parking_spots[spot_id]
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = float(spot_data['x'])
        goal_msg.pose.position.y = float(spot_data['y'])
        qz = math.sin(spot_data["yaw"] / 2.0)
        qw = math.cos(spot_data["yaw"] / 2.0)
        goal_msg.pose.orientation.z = qz
        goal_msg.pose.orientation.w = qw
        self.goal_pose_publisher.publish(goal_msg)
        self.get_logger().info(f"Published goal for spot {spot_id}.")
        time.sleep(2.0)
        self.call_engage_service()

    def call_engage_service(self):
        if not self.engage_client.service_is_ready():
            self.get_logger().error("Engage service is not ready. Cannot engage.")
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
                self.get_logger().info("✅ Vehicle engaged successfully!")
            else:
                self.get_logger().error(f"❌ Failed to engage vehicle: {response.status.message}")
        except Exception as e:
            self.get_logger().error(f"Engage service call failed: {e}")

    def set_initial_pose_callback(self, request, response):
        """Service callback for the dashboard button to set the initial pose."""
        try:
            self.get_logger().info("Setting initial pose via service call...")
            pwc = PoseWithCovarianceStamped()
            pwc.header.frame_id = 'map'
            pwc.header.stamp = self.get_clock().now().to_msg()
            pwc.pose.pose.position.x = 3711.23
            pwc.pose.pose.position.y = 73718.04
            pwc.pose.pose.orientation.w = 1.0
            pwc.pose.covariance[0] = 0.25
            pwc.pose.covariance[7] = 0.25
            pwc.pose.covariance[35] = 0.068
            self.initial_pose_publisher.publish(pwc)
            
            response.success = True
            response.message = "Initial pose published successfully."
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to publish initial pose: {e}"
            self.get_logger().error(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    executor = MultiThreadedExecutor()
    node = ParkingManagementNode()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
