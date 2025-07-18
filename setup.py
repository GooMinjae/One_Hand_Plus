from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'one_hand_plus'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS 2 필수 정보
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # UI 파일 포함
        ('share/' + package_name + '/resource', ['resource/one_hand_plus_ui.ui']),
        # 이미지 파일 전체 포함 (glob)
        ('share/' + package_name + '/resource/img', glob('resource/img/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='goominjae',
    maintainer_email='999aldwo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plastic=one_hand_plus.plastic_bottle_test:main',
            'glass=one_hand_plus.glass_test:main',
            'ui=one_hand_plus.ui_sample:main'
        ],
    },
)
