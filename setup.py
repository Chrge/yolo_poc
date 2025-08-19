from setuptools import setup
from glob import glob

package_name = 'yolo_poc'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # Python-Package-Verzeichnis: yolo_poc/
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # alle Launchfiles installieren
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # alle RViz-Configs installieren
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        # alle Welten installieren (.world für Gazebo Classic, .sdf für Gazebo Sim)
        ('share/' + package_name + '/worlds', glob('worlds/*.world') + glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='YOLOv8 detector for ROS 2 images',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_poc.yolo_node:main',
        ],
    },
)
