#!/usr/bin/env python3

import threading
import time

import requests
from flask import Flask, request

CELL_URL = "http://localhost:8080/pick"
CELL_MSG = {"port": 8081, "topic": "/confirmPick"}
app = Flask(__name__)


@app.route(CELL_MSG["topic"], methods=["POST"])
def receive_client_reply():
    """
    Receives picking response from cell
    """
    data = request.get_json()
    print(f"Received reply from client: {data}")
    return "", 204


def pick_request_sender():
    """
    Sends HTTP POST requests at 8080/pick at 10 seconds intervals
    """
    id = 0
    while True:
        time.sleep(10)
        payload = {"pickId": id, "quantity": 1}
        try:
            print(f"Sending pick request with pickId: {id} to the client.")
            requests.post(CELL_URL, json=payload)
        except Exception as e:
            print(
                f"Failed to send pick request with pickId: {id} due to the following error {str(e)}"
            )
        id += 1


if __name__ == "__main__":
    print("WMS system for robot")
    threading.Thread(target=pick_request_sender, daemon=True).start()
    app.run(port=CELL_MSG["port"])
