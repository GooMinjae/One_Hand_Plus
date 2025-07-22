import os
import sys

from ament_index_python.resources import get_resource
from geometry_msgs.msg import Twist
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QKeySequence, QPixmap
from python_qt_binding.QtWidgets import QShortcut, QWidget, QMainWindow, QApplication
# from python_qt_binding import uic

import one_hand_plus.plastic_bottle_test as plastic
import one_hand_plus.glass_test as glass
import one_hand_plus.bread as bread
# import one_hand_plus.vegetable as vegetable



import rclpy
from rclpy.qos import QoSProfile
from std_srvs.srv import SetBool
from std_msgs.msg import String


class WindowClass(QMainWindow):
    def __init__(self):
        super().__init__()

        pkg_name = 'one_hand_plus'
        ui_filename = 'one_hand_plus_ui.ui'
        style = """
                QPushButton {
                    background-color: rgb(222, 221, 218);
                    border-radius: 5px;
                }
                QPushButton:hover {
                    background-color: rgb(200, 200, 200);
                }
                QPushButton:pressed {
                    background-color: rgb(170, 170, 170);
                }
            """
        
        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)
        img_dir = os.path.join(package_path, 'share', pkg_name, 'resource', 'img')  # ✅ 이미지 디렉토리

        rclpy.init(args=None)
        self.node = rclpy.create_node("ui_task_sender")
        self.cmd_pub = self.node.create_publisher(String, "/robot_task_cmd", 10)

        self.node.create_subscription(String, "/task_status", self.status_callback, 10)

        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.ros_spin_once)
        self.ros_timer.start(50) 


        # self.setupUi(self)
        self.setWindowTitle("One Hand+")

        self.plastic_bottle_img.setPixmap(QPixmap(os.path.join(img_dir, 'bottle_water.png')).scaled(201, 281, Qt.KeepAspectRatio))
        self.glass_img.setPixmap(QPixmap(os.path.join(img_dir, 'glass.png')).scaled(201, 281, Qt.KeepAspectRatio))
        self.bread_img.setPixmap(QPixmap(os.path.join(img_dir, 'bread.png')).scaled(201, 281, Qt.KeepAspectRatio))

        self.plastic_bottle_btn.setStyleSheet(style)
        self.glass_btn.setStyleSheet(style)
        self.bread_btn.setStyleSheet(style)

        self.plastic_bottle_btn.clicked.connect(self.send_plastic_cmd)
        self.glass_btn.clicked.connect(self.send_glass_cmd)
        self.bread_btn.clicked.connect(self.send_bread_cmd)

        self.running_label.setVisible(False)

    def ros_spin_once(self):
        rclpy.spin_once(self.node, timeout_sec=0)

    def status_callback(self, msg):
        if msg.data == "running":
            self.running_label.setVisible(True)
        else:
            self.running_label.setVisible(False)
        print('callback')

    def send_cmd(self, cmd_str):
        msg = String()
        msg.data = cmd_str
        self.cmd_pub.publish(msg)
        print(f"[UI] Published command: {cmd_str}")

    def send_plastic_cmd(self):
        # if plastic.is_task_running:
        #     self.running_label.setVisible(True)
        # else:
        self.send_cmd("plastic")
        # if plastic.is_task_running:
        #     self.running_label.setVisible(True)
        # # self.running_label.setVisible(True)

    def send_glass_cmd(self):
        self.send_cmd("glass")

    def send_bread_cmd(self):
        self.send_cmd("bread")


def main():
    app = QApplication(sys.argv)
    window = WindowClass()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
