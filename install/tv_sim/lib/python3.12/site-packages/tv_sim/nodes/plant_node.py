import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
import numpy as np
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped



from tv_sim.core.models.ev_18dof import Vehicle18DOF

from tv_sim.core.utils.config import VehicleConfig
from tv_sim.core.controllers.torque_allocator import TorqueAllocator

class PlantNode(Node):
    def __init__(self):
        super().__init__('plant_node')
        
        # init
        self.cfg = VehicleConfig()
        self.vehicle = Vehicle18DOF(self.cfg)

        self.allocator=TorqueAllocator(self.cfg)
        
        #state
        self.dt = 0.01  # 100Hz simulation rate
        self.current_torques = np.zeros(4)
        self.current_steering = 0.0
        self.current_throttle = 0.0
        self.requested_mz=0
        self.vx=0
        
        # Suscriber(inputs)

        self.steer_sub = self.create_subscription(Float64MultiArray, '/cmd_steering', self.steer_callback, 10)
        self.cmd_throttle_sub = self.create_subscription(Float64MultiArray, '/cmd_throttle', self.throttle_callback, 10)
        self.mz_sub = self.create_subscription(Float64MultiArray , '/requested_mz', self.requested_mz_callback,10)
    

            
        # Publishers(outputs)
   
        self.odom_pub = self.create_publisher(Odometry, 'odom', 5)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        #sim timmer callback
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info('Vehicle Plant Node started at 100Hz')

    def throttle_callback(self, msg):

        self.current_throttle = msg.data[0] 
        

    def steer_callback(self, msg):
        self.current_steering = msg.data[0]

    def requested_mz_callback(self,msg):
        self.requested_mz=msg.data[0]
        

    def timer_callback(self):
        #solver step
        self.current_torques=self.allocator.distribute(self.current_throttle,self.requested_mz,self.dt,self.vx)
        state = self.vehicle.step(
            self.current_torques, 
            self.current_steering, 
            self.dt,substeps=10
        )
        self.vx=state[0]
       
        
        #publish state
        self.publish_odometry(state)

    def publish_odometry(self, state):
        vx, vy, r, x, y, psi = state[0:6]
        now=self.get_clock().now().to_msg()
        
        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # Position
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        
        # Orientation (Euler to Quaternion)

        t = TransformStamped()
        t.header.stamp =now
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Set position
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, psi)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transform
        
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        
        # Velocities (Twist)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = r
        
        self.odom_pub.publish(msg)
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = PlantNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()