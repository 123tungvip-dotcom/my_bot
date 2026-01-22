#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ObstacleMover(Node):
    def __init__(self):
        super().__init__('obstacle_mover')
        
        # Publisher cho Hộp 1
        self.pub1 = self.create_publisher(Twist, '/obstacle1/cmd_vel', 8)
        
        # Publisher cho Hộp 2 (Mới)
        self.pub2 = self.create_publisher(Twist, '/obstacle2/cmd_vel', 8)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.dir1 = 1 # Hướng của hộp 1
        self.dir2 = -1 # Hướng của hộp 2 (cho đi ngược nhau)
        self.counter = 0

    def timer_callback(self):
        # --- Điều khiển Hộp 1 (Đi dọc X) ---
        msg1 = Twist()
        msg1.linear.x = 0.5 * self.dir1
        self.pub1.publish(msg1)

        # --- Điều khiển Hộp 2 (Đi ngang Y) ---
        msg2 = Twist()
        msg2.linear.y = 0.25 * self.dir2
        self.pub2.publish(msg2)
        
        # Logic đổi chiều (Giữ nguyên)
        self.counter += 1
        if self.counter > 200:
            self.dir1 *= -1
            self.dir2 *= -1
            self.counter = 0

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
