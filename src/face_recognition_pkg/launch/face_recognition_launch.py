# face_recognition_pkg/launch/face_recognition_launch.py

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 定义face_detection_server节点
    face_detection_server = Node(
        package='face_recognition_pkg',
        executable='face_detection_server',
        name='face_detection_server',
        output='screen',
        parameters=[
            # 如果需要，可以在这里添加参数文件或字典
            # os.path.join(get_package_share_directory('face_recognition_pkg'), 'config', 'server_params.yaml'),
        ],
    )

    # 定义face_detection_client节点
    face_detection_client = Node(
        package='face_recognition_pkg',
        executable='face_detection_client',
        name='face_detection_client',
        output='screen',
        parameters=[
            # 如果需要，可以在这里添加参数文件或字典
            # os.path.join(get_package_share_directory('face_recognition_pkg'), 'config', 'client_params.yaml'),
        ],
    )

    # 返回LaunchDescription对象，包含所有要启动的节点
    return LaunchDescription([
        face_detection_server,
        face_detection_client,
    ])