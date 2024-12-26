import rclpy
import DR_init
from datetime import datetime
import time
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import threading

class force_sub(Node):
    def __init__(self):
        super().__init__("force_sub_node")
        self.force_sub = self.create_subscription(Float64MultiArray, "dsr01/msg/tool_force", self.force_callback, 10)

    def force_callback(self, msg):
        if msg.data[1] >= 4.0:
            print(msg.data)

def main(args=None):
    rclpy.init(args = args)    

    force_node = force_sub()

    rclpy.spin(force_node)

if __name__ == "__main__":
    main()