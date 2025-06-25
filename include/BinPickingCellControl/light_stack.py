#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Int32

class LightStackPublisher(Node):
    """
        Light Stack Publisher class gathers information and publishes feedback on the picking cell's current state
    Args:
        Node
    """
    def __init__(self):
        super().__init__('light_stack_publisher')
        self.publisher_ = self.create_publisher(Int32, 'light_stack', 10)

        # Topics to subscribe
        self.latest_request_msg, self.latest_estop_msg, self.latest_door_handle_msg = "", "", ""
        self.requests_sub = self.create_subscription(String, 'cell_state', self.requests_callback, 10)
        self.e_stop_sub = self.create_subscription(Bool, 'e_stop_pressed', self.estop_callback, 10)
        self.door_handle_sub = self.create_subscription(Bool, 'door_handle', self.door_handle_callback, 10)
                
        # Timer ticks every 1 second to publish message
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Keep track of time in current state
        self.elapsed_time = 0.0

    def requests_callback(self, msg):
        self.latest_request_msg= msg.data

    def estop_callback(self, msg):
        self.latest_estop_msg = msg.data

    def door_handle_callback(self, msg):
        self.latest_door_handle_msg = msg.data

    def timer_callback(self):
        """
            Check e-stop, door handle and requests
            If e-stop is pressed send -1
            If door handle is pressed send 1
            If requests are not coming then send 2
            If bin is functional and picking then send 1 
        """     
        msg = Int32()
        if self.latest_estop_msg == False:
            msg.data = -1
        elif self.latest_door_handle_msg == False:
            msg.data = 1
        elif len(self.latest_request_msg.split("|")[0]) == 0:
            msg.data = 2
        else:
            msg.data = 0
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
    
def main(args=None):
    rclpy.init(args=args)
    node = LightStackPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()