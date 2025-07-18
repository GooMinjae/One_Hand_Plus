# import sys
# import os
# from PyQt5.QtWidgets import QMainWindow, QApplication
# from PyQt5.QtCore import *
# from PyQt5 import uic
# from PyQt5.QtGui import QPixmap

import os
import sys

from ament_index_python.resources import get_resource
from geometry_msgs.msg import Twist
from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtGui import QKeySequence, QPixmap
from python_qt_binding.QtWidgets import QShortcut, QWidget, QMainWindow, QApplication
# from python_qt_binding import uic


import rclpy
from rclpy.qos import QoSProfile
from std_srvs.srv import SetBool

# # High DPI Scaling 비활성화 및 고정 배율 설정
# os.environ["QT_AUTO_SCREEN_SCALE_FACTOR"] = "0"  # 자동 스케일링 비활성화
# os.environ["QT_ENABLE_HIGHDPI_SCALING"] = "0"   # DPI Scaling 비활성화
# os.environ["QT_SCALE_FACTOR"] = "1"             # 배율을 1로 고정

# '''PyInstaller로 프로그램을 생성하였을 때, 코드에서 호출하는 파일을 상대경로로 호출하기 위한 함수입니다.'''
# @(lambda f: f())
# def _(): os.chdir(getattr(sys, '_MEIPASS', os.path.dirname(os.path.abspath(__file__))))

# # UI 파일 로드
# form_class = uic.loadUiType("one_hand_plus_ui.ui")[0]



class WindowClass(QMainWindow):
    def __init__(self):
        super().__init__()

        pkg_name = 'one_hand_plus'
        ui_filename = 'one_hand_plus_ui.ui'
        # topic_name = 'cmd_vel'
        # service_name = 'led_control'
        
        _, package_path = get_resource('packages', pkg_name)
        ui_file = os.path.join(package_path, 'share', pkg_name, 'resource', ui_filename)
        loadUi(ui_file, self)
        img_dir = os.path.join(package_path, 'share', pkg_name, 'resource', 'img')  # ✅ 이미지 디렉토리

        # self.setupUi(self)
        self.setWindowTitle("One Hand+")

        self.plastic_bottle_img.setPixmap(QPixmap(os.path.join(img_dir, 'bottle_water.png')).scaled(121, 161, Qt.KeepAspectRatio))
        self.glass_img.setPixmap(QPixmap(os.path.join(img_dir, 'glass.png')).scaled(121, 161, Qt.KeepAspectRatio))
        self.vegetable_img.setPixmap(QPixmap(os.path.join(img_dir, 'vegetable_img.jpg')).scaled(121, 161, Qt.KeepAspectRatio))
        self.bread_img.setPixmap(QPixmap(os.path.join(img_dir, 'bread.png')).scaled(121, 161, Qt.KeepAspectRatio))


        self.plastic_bottle_btn.clicked.connect(self.plastic_task)
        self.glass_btn.clicked.connect(self.glass_task)

        self.vegetable_btn.clicked.connect(self.vegetabel_task)
        self.bread_btn.clicked.connect(self.bread_task)



    def plastic_task(self):
        print('open plastic')
        pass

    def glass_task(self):
        print('open glass')
        pass

    def vegetabel_task(self):
        print('vegetable')
        pass

    def bread_task(self):
        print('bread')
        pass

def main():
    app = QApplication(sys.argv)
    window = WindowClass()
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
