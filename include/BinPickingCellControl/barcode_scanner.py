#!/usr/bin/env python3
import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class BarcodePublisherWithService(Node):
    """
    Publishes a random array of 5 barcodes and exposes a rosservice as well
    """

    def __init__(self):
        super().__init__("barcode_publisher_with_service")

        self.publisher_ = self.create_publisher(String, "barcode", 10)
        self.srv = self.create_service(
            Trigger, "get_latest_barcode", self.handle_get_latest_barcode
        )

        self.barcode_history = []  # Use a simple list

        self.timer = self.create_timer(1.0, self.timer_callback)  # Publish every second

        self.get_logger().info("Barcode publisher node started.")

    def timer_callback(self):
        # Generate random 5-digit barcode as string
        barcode = "".join(str(random.randint(0, 9)) for _ in range(5))

        # Keep only the last 5 barcodes
        self.barcode_history.append(barcode)
        if len(self.barcode_history) > 5:
            self.barcode_history.pop(0)

        # Publish the list as a comma-separated string
        msg = String()
        msg.data = ",".join(self.barcode_history)
        self.publisher_.publish(msg)

        self.get_logger().info(f"Published barcodes: {self.barcode_history}")

    def handle_get_latest_barcode(self, request, response):
        if not self.barcode_history:
            response.success = False
            response.message = "No barcode published yet."
        else:
            latest_barcode = self.barcode_history[-1]
            response.success = True
            response.message = latest_barcode
            self.get_logger().info(
                f"Service request: returning latest barcode {latest_barcode}"
            )
        return response


def main(args=None):
    rclpy.init(args=args)
    node = BarcodePublisherWithService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
