import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys, select, termios, tty
import numpy as np
from tv_sim.core.utils.config import VehicleConfig
class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        
        # Publishers
        self.steer_pub = self.create_publisher(Float64MultiArray, '/cmd_steering', 10)
        self.cmd_throttle_pub = self.create_publisher(Float64MultiArray, '/cmd_throttle', 10)
        
        # State
        self.steering = 0.0
        self.throttle = 0.0
       
        
        # Settings
        self.steer_step = 0.05  # Radians
        self.throttle_step = 0.01# 0 to 1.0 scale

        self.cfg=VehicleConfig()

        self.get_logger().info("Teleop Node Started. Use WASD: W/S for Throttle, A/D for Steering.")

    def run_loop(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while True:
                key = self.get_key(settings)
                if key == 'w': self.throttle = min(1.0, self.throttle + self.throttle_step)
                elif key == 's': self.throttle = max(-0.2, self.throttle - self.throttle_step)
                elif key == 'a': self.steering += self.steer_step
                elif key == 'd': self.steering -= self.steer_step
                elif key == ' ': # Spacebar to reset
                    self.steering = 0.0
                    self.throttle = 0.0
                elif key == '\x03': break # Ctrl+C

                # Publish current intent
                
                self.steering = np.clip(self.steering, -self.cfg.max_steer, self.cfg.max_steer)
                
                self.steer_pub.publish(Float64MultiArray(data=[self.steering]))     
                self.cmd_throttle_pub.publish(Float64MultiArray(data=[self.throttle]))
                
                
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def get_key(self, settings):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    node.run_loop()# activily polling 
    node.destroy_node()
    rclpy.shutdown()