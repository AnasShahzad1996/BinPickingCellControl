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
