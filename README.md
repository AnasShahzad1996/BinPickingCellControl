# 🤖 Bin Picking Cell Control System

A mini-project that simulates control software for a **Robotic Bin Picking Cell** using ROS 2, Flask, and a PyQt5-based Human-Machine Interface (HMI). This system replicates common safety and operation features of an industrial robot cell including API interaction, ROS 2 nodes for sensors, and a real-time HMI — all without the actual robot!

---

## 📺 Demo

[![Watch the Video](https://github.com/AnasShahzad1996/BinPickingCellControl/blob/main/media/image.png)](https://github.com/AnasShahzad1996/BinPickingCellControl/blob/main/media/demo.mp4)

---

## 📦 Features

| Module | Description |
|--------|-------------|
| 🌐 **API Handler** | Flask-based WMS server and bin-picking client to exchange HTTP POST requests and simulate task execution. |
| 📷 **Scanner Node** | Publishes random barcodes and provides a service to get the last scanned barcode. |
| 🚪 **Door Handle Node** | Publishes door status and allows toggling via service. |
| 🛑 **Emergency Button Node** | Publishes e-stop state; service allows "pressing" and "releasing" the emergency button. |
| 🚦 **Stack-Light Node** | Publishes system status (0: Operational, 1: Paused, -1: Emergency) based on door and e-stop. |
| 🖥 **HMI (Qt GUI)** | Displays real-time cell status, color-coded shapes for stack-light, and I/O fields for WMS/Cell messages. |

---

## 🧠 System Overview

```text
+-------------+         HTTP          +---------------+          ROS 2          +--------------+
| WMS Server  |  <---------------->   | Picking Cell  |  <----------------->   |  ROS 2 Nodes |
| (Flask API) |                      | Client API    |                         | (Scanner, etc.)
+-------------+                      +---------------+                         +--------------+

                                     +--------------+
                                     | Qt HMI (GUI) |
                                     +--------------+
```
## 🚀 Installation

### 📦 Requirements

- **Ubuntu 22.04**
- **ROS 2 Humble** installed and sourced (see below)
- **Python 3.10+**

---

### 🧰 1. Install ROS 2 Humble

Follow the official installation guide here: [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation.html)

Or, in short:

```bash
sudo apt update && sudo apt install curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```
# Source ROS 2 in your shell
```echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 🐍 2. Install Python Dependencies

Install dependencies using the provided `requirements.txt` file:

```bash
# (Optional) Create a virtual environment
python3 -m venv venv
source venv/bin/activate

# Install dependencies
pip install -r requirements.txt
```

### 📁 3. Build and Source ROS 2 Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```


# 🚀 Run code
```
# In four separate terminals execute the following commands:
ros2 run BinPickingCellControl cell_sensors.py
ros2 run BinPickingCellControl client.py
ros2 run BinPickingCellControl server.py
ros2 run BinPickingCellControl hmi.py
```
