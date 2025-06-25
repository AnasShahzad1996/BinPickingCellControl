#!/usr/bin/env python3
import rclpy
import threading
import json
import requests
from flask import Flask, request
from rclpy.node import Node
from std_msgs.msg import String, Bool

class CellServer:
    def __init__(self, wms_info, response_info):
        self.wms_info = wms_info
        self.response_info = response_info
        # Initialize ROS node and publisher
        rclpy.init(args=None)
        self.ros_node = rclpy.create_node('cell_server_node')
        self.cell_state_publisher = self.ros_node.create_publisher(String, 'cell_state', 10)
        
        # Topics to subscribe
        self.latest_barcode_msg, self.latest_estop_msg, self.latest_door_handle_msg = "", "", ""
        self.last_used_barcode = ""
        self.barcode_sub = self.ros_node.create_subscription(String, 'barcode', self.barcode_callback, 10)
        self.e_stop_sub = self.ros_node.create_subscription(Bool, 'e_stop_pressed', self.estop_callback, 10)
        self.door_handle_sub = self.ros_node.create_subscription(Bool, 'door_handle', self.door_handle_callback, 10)

        # Initialize client
        self.client = Flask(__name__)
        self._register_routes()

        # ROS spinning thread
        self._ros_spin_thread = threading.Thread(target=self._ros_spin)
        self._ros_spin_thread.daemon = True
        self._ros_spin_thread.start()

    def _ros_spin(self):
        """Spin ROS node in a separate thread."""
        try:
            while rclpy.ok():
                rclpy.spin_once(self.ros_node, timeout_sec=0.1)
        except Exception as e:
            print(f"[ROS] Spin error: {e}")

    def barcode_callback(self, msg):
        self.latest_barcode_msg = msg.data[-1]

    def estop_callback(self, msg):
        self.latest_estop_msg = msg.data

    def door_handle_callback(self, msg):
        self.latest_door_handle_msg = msg.data

    def check_cell_state(self):
        """Check the cell state of the robot

        Returns:
            (bool, str): Result, Reason of result
        """
        print (self.latest_door_handle_msg, "|", self.latest_estop_msg)
        if self.latest_estop_msg == False:
            return False, "E-stop is pressed"
        if self.latest_door_handle_msg == False:
            return False, "Door is open"
        else:
            return True, ""

    def _register_routes(self):
        @self.client.route(self.wms_info["topic"], methods=["POST"])
        def handle_pick_request():
            """Handles pick request by checking available barcodes
            Sends Failure message if emergency button is pressed or door is open
            
            Returns:
                (str, int): ACK message
            """
            pick_request = request.get_json()
            print (f"Picking Cell received request with id {pick_request['pickId']}")

            pick_request_str = json.dumps(pick_request)
            confirm_pick_str = ""

            # Check if state of cell is okay
            state, reason = self.check_cell_state()
            
            
            # If picking cell is inoperable
            if state == False:
                payload = {
                    "pickId": pick_request["pickId"],
                    "pickSuccessful": False,
                    "errorMessage": reason,
                    "itemBarcode": None
                }
                confirm_pick_str = str(payload)
                try:
                    requests.post(f"http://localhost:{self.response_info['port']}/{self.response_info['topic']}", json = payload)
                except Exception as e:
                    print (f"Failed to send pick request with pickId: {id} due to the following error {str(e)}")
            else:
                if self.last_used_barcode == "" and self.last_used_barcode == self.latest_barcode_msg:
                    payload = {
                        "pickId": pick_request["pickId"],
                        "pickSuccessful": False,
                        "errorMessage": "Barcode scanner not available",
                        "itemBarcode": None
                    }
                    confirm_pick_str = str(payload)
                    try:
                        requests.post(f"http://localhost:{self.response_info['port']}/{self.response_info['topic']}", json = payload)
                    except Exception as e:
                        print (f"Failed to send pick request with pickId: {id} due to the following error {str(e)}")
                else:
                    self.last_used_barcode = self.latest_barcode_msg
                    payload = {
                        "pickId": pick_request["pickId"],
                        "pickSuccessful": True,
                        "errorMessage": None,
                        "itemBarcode": self.last_used_barcode
                    }
                    confirm_pick_str = str(payload)
                    try:
                        requests.post(f"http://localhost:{self.response_info['port']}/{self.response_info['topic']}", json = payload)
                    except Exception as e:
                        print (f"Failed to send pick request with pickId: {id} due to the following error {str(e)}")

            # Publish string message to ROS topic 'cell_state'
            msg = String()
            msg.data = pick_request_str + "|" + confirm_pick_str
            self.cell_state_publisher.publish(msg)
            print(f"Published to 'cell_state': {pick_request_str}")

            # Send ACK message was recieved
            return "", 204

    def run_client(self):
        """Runs flask client for picking cell
        """
        try:
            self.client.run(port=self.wms_info["port"])
        except Exception as e:
            print (f"Exception in client {str(e)}")
        finally:
            print ("Shutting down client thread")
                    

def main(args=None):
    print ("Robot picking cell")
    wms_info = {
        "port": 8080,
        "topic": "/pick"
    }
    response_info = {
        "port": 8081,
        "topic": "/confirmPick"
    }

    cell_server = CellServer(wms_info=wms_info, response_info=response_info)
    cell_server.run_client()
    
if __name__=="__main__":
    main()