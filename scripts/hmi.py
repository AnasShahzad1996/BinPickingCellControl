#!/usr/bin/env python3

import sys
import math
import rclpy
import threading
from std_msgs.msg import Bool, String, Int32
from PyQt5.QtWidgets import (
    QApplication, QWidget, QLabel, QVBoxLayout, QHBoxLayout, QLineEdit, QFrame, QTextEdit
)
from PyQt5.QtGui import QPainter, QBrush, QColor, QPolygon
from PyQt5.QtCore import Qt, QTimer, QPoint

class ShapeWidget(QFrame):
    def __init__(self):
        super().__init__()
        self.shape = 'circle'  # can be: 'circle', 'square', 'triangle'
        self.setMinimumHeight(120)
        self.setMinimumWidth(120)

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)

        rect = self.rect().adjusted(10, 10, -10, -10)

        if self.shape == 'circle':
            painter.setBrush(QBrush(QColor('green')))
            painter.drawEllipse(rect)
        elif self.shape == 'square':
            painter.setBrush(QBrush(QColor('yellow')))
            painter.drawRect(rect)
        elif self.shape == 'triangle':
            painter.setBrush(QBrush(QColor('red')))
            points = QPolygon([
                QPoint(rect.center().x(), rect.top()),
                QPoint(rect.left(), rect.bottom()),
                QPoint(rect.right(), rect.bottom())
            ])
            painter.drawPolygon(points)        
        elif self.shape == 'star':
            painter.setBrush(QBrush(QColor('blue')))

            cx = rect.center().x()
            cy = rect.center().y()
            r_outer = min(rect.width(), rect.height()) // 2 - 5
            r_inner = r_outer // 2.5

            star_points = []
            for i in range(10):
                angle_deg = i * 36
                angle_rad = angle_deg * 3.14159 / 180
                r = r_outer if i % 2 == 0 else r_inner
                x = cx + int(r * math.cos(angle_rad))
                y = cy - int(r * math.sin(angle_rad))
                star_points.append(QPoint(x, y))
            painter.drawPolygon(QPolygon(star_points))

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        
        # Set GUI geometry
        self.setWindowTitle("Robot Cell HMI")
        self.resize(1000, 600)
        qr = self.frameGeometry()
        cp = QApplication.primaryScreen().availableGeometry().center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

        rclpy.init(args=None)
        self.ros_node = rclpy.create_node('hmi_node')

        self.latest_pick_cell_msg, self.latest_estop_msg, self.latest_door_handle_msg, self.latest_light_stack_msg = "", "", "", ""
        self.pick_cell_sub = self.ros_node.create_subscription(String, 'cell_state', self.pick_cell_callback, 10)
        self.e_stop_sub = self.ros_node.create_subscription(Bool, 'e_stop_pressed', self.estop_callback, 10)
        self.door_handle_sub = self.ros_node.create_subscription(Bool, 'door_handle', self.door_handle_callback, 10)
        self.light_stack_sub = self.ros_node.create_subscription(Int32, 'light_stack', self.light_stack_callback, 10)

        # ROS spinning thread
        self._ros_spin_thread = threading.Thread(target=self._ros_spin)
        self._ros_spin_thread.daemon = True
        self._ros_spin_thread.start()

        # Initialize UI
        self.init_ui()

    def _ros_spin(self):
        """Spin ROS node in a separate thread."""
        try:
            while rclpy.ok():
                rclpy.spin_once(self.ros_node, timeout_sec=0.1)
        except Exception as e:
            print(f"[ROS] Spin error: {e}")
        
    def pick_cell_callback(self, msg):
        self.latest_pick_cell_msg = msg.data
    
    def estop_callback(self, msg):
        self.latest_estop_msg = msg.data
        
    def door_handle_callback(self, msg):
        self.latest_door_handle_msg = msg.data
        
    def light_stack_callback(self, msg):
        self.latest_light_stack_msg = msg.data

    def init_ui(self):
        """Adding widgets to display information
        """
        layout = QVBoxLayout()

        input_layout = QVBoxLayout()
        wms_layout = QHBoxLayout()
        wms_layout.addWidget(QLabel("WMS request:"))
        self.wms_req_label = QTextEdit()
        self.wms_req_label.setFixedHeight(60)  # Optional: control height
        wms_layout.addWidget(self.wms_req_label)

        # Picking Cell response
        cell_layout = QHBoxLayout()
        cell_layout.addWidget(QLabel("Picking Cell response:"))
        self.cell_res_label = QTextEdit()
        self.cell_res_label.setFixedHeight(60)
        cell_layout.addWidget(self.cell_res_label)

        # Add both rows to the main input layout
        input_layout.addLayout(wms_layout)
        input_layout.addLayout(cell_layout)
        layout.addLayout(input_layout)
        
        self.label1 = QLabel("E-stop")
        self.label2 = QLabel("Door Handle")
        for label in [self.label1, self.label2]:
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("background-color: red; color: black; padding: 10px;")
        layout.addWidget(self.label1)
        layout.addWidget(self.label2)

        self.shape = ShapeWidget()
        shape_layout = QHBoxLayout()
        shape_layout.addWidget(self.shape)
        layout.addLayout(shape_layout)

        self.setLayout(layout)

        # Timers for animation
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(500)

    def update_ui(self):
        """Updates UI with values read from ros-messages
        """
        color1 = 'white' if self.latest_estop_msg else 'red'
        color2 = 'white' if self.latest_door_handle_msg else 'red'
        self.label1.setStyleSheet(f"background-color: {color1}; color: black; padding: 10px;")
        self.label2.setStyleSheet(f"background-color: {color2}; color: black; padding: 10px;")
        
        if len(self.latest_pick_cell_msg) > 0 :
            wms_req = self.latest_pick_cell_msg.split("|")[0]
            bin_req = self.latest_pick_cell_msg.split("|")[1]
            self.wms_req_label.setText(wms_req)
            self.cell_res_label.setText(bin_req)

            if self.latest_light_stack_msg == 0:
                self.shape.shape = 'circle'
            elif self.latest_light_stack_msg == 1:
                self.shape.shape = 'square'
            elif self.latest_light_stack_msg == -1:
                self.shape.shape = 'triangle'
            elif self.latest_light_stack_msg == 2:
                self.shape.shape = 'star'
            self.shape.update()

        else:
            self.shape.shape = 'star'
            self.shape.update()
        
if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
