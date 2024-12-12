from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 定义系统状态发布节点
    publisher_node = Node(
        package='system_status_monitoring',  # 替换为您的包名
        executable='system_status_publisher',  # 发布节点的可执行文件名
        name='system_status_publisher',  # 节点名称
        output='screen',
        parameters=[
            # 如果发布节点需要参数，可以在这里添加
            # {'param_name': param_value},
        ]
    )

    # 定义系统状态订阅节点
    subscriber_node = Node(
        package='system_status_monitoring',  # 替换为您的包名
        executable='system_status_subscriber',  # 订阅节点的可执行文件名
        name='system_status_subscriber',  # 节点名称
        output='screen',
        parameters=[
            # 如果订阅节点需要参数，可以在这里添加
            # {'param_name': param_value},
        ]
    )

    return LaunchDescription([
        publisher_node,
        subscriber_node
    ])