import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
import numpy as np

from tv_sim.core.controllers.torque_vectoring import VehicleLQR
from tv_sim.core.controllers.drive_intent import ReferenceModel
from tv_sim.core.utils.config import VehicleConfig



class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.cfg = VehicleConfig()
        
        # Initialize 
        self.controller = VehicleLQR(self.cfg)
        self.reference = ReferenceModel(self.cfg)
        
        
        # State tracking
        self.current_vx = 0.0
        self.current_vy=0
        self.current_yaw_rate = 0.0
        self.current_steering = 0.0
        self.cmd_torque=np.zeros(4)
        self.last_time = self.get_clock().now()

        # Subscribers
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 2)
        self.steer_sub = self.create_subscription(Float64MultiArray, '/cmd_steering', self.steer_callback, 2)
       

        # Publishers
        self.mz_pub = self.create_publisher(Float64, '/requested_mz', 10)
        self.target_yaw_pub = self.create_publisher(Float64, '/target_yaw_rate', 10)

        self.get_logger().info('TV Controller Node (Modular) initialized')

        self.timer_period = 0.01  # seconds
        self.timer = self.create_timer(self.timer_period, self.control_loop)

    def steer_callback(self, msg):
        self.current_steering = msg.data[0]

    def odom_callback(self, msg):
        # Update current state from the Plant
        self.current_vx = msg.twist.twist.linear.x
        self.current_vx = msg.twist.twist.linear.y
        self.current_yaw_rate = msg.twist.twist.angular.z


        


    def control_loop(self):

        target_yaw = self.reference.get_target_yaw_rate(self.current_vx, self.current_steering)

    
        mz_req = self.controller.solve(
            self.current_vx, 
            self.current_vy, 
            self.current_yaw_rate,self.current_steering,self.reference.get_target_yaw_rate)
        


        self.mz_pub.publish(Float64(data=mz_req))
        self.target_yaw_pub.publish(Float64(data=target_yaw))

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()