from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 첫 번째 그룹
        Node(
            package='launch_example',
            namespace='launch_example1',
            executable='pub.py',
            name='pub'
        ),
        Node(
            package='launch_example',
            namespace='launch_example1',
            executable='sub.py',
            name='sub'
        ),
        # 두 번째 그룹
        Node(
            package='launch_example',
            namespace='launch_example2',
            executable='pub.py',
            name='pub'
        ),
        Node(
            package='launch_example',
            namespace='launch_example2',
            executable='sub.py',
            name='sub'
        )
    ])