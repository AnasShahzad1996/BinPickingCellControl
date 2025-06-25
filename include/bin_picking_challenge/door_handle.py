#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class DoorHandlePublisher(Node):
    def __init__(self):
        super().__init__('door_handle_publisher')
        self.publisher_ = self.create_publisher(Bool, 'door_handle', 10)
        
        self.current_state = True
        self.publish_duration_sec = 10.0  # how long to keep each state
        
        # Timer ticks every 1 second to publish message
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Keep track of time in current state
        self.elapsed_time = 0.0
        
    def timer_callback(self):
        msg = Bool()
        msg.data = self.current_state
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        
        self.elapsed_time += 1.0
        if self.elapsed_time >= self.publish_duration_sec:
            # Switch state and reset timer
            self.current_state = not self.current_state
            self.elapsed_time = 0.0

def main(args=None):
    rclpy.init(args=args)
    node = DoorHandlePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
