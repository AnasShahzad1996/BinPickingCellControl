#!/usr/bin/env python3

##################################################################################
### Helper class to start e-stop, door handle, light stack and barcode scanner ###
##################################################################################

import rclpy
from rclpy.executors import MultiThreadedExecutor

from BinPickingCellControl.barcode_scanner import BarcodePublisherWithService
from BinPickingCellControl.door_handle import DoorHandlePublisher
from BinPickingCellControl.emergency_button import EmergencyButtonPublisher
from BinPickingCellControl.light_stack import LightStackPublisher


def main():
    rclpy.init()

    # Create node instances
    barcode_node = BarcodePublisherWithService()
    door_node = DoorHandlePublisher()
    emergency_node = EmergencyButtonPublisher()
    light_stack_node = LightStackPublisher()

    # Create executor that can handle multiple threads
    executor = MultiThreadedExecutor()

    # Add nodes to the executor
    executor.add_node(barcode_node)
    executor.add_node(door_node)
    executor.add_node(emergency_node)
    executor.add_node(light_stack_node)

    print("All publishers started using MultiThreadedExecutor")

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Clean up nodes
        for node in [barcode_node, door_node, emergency_node, light_stack_node]:
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
