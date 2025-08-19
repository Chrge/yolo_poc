from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    image_topic = LaunchConfiguration('image_topic')
    model_path = LaunchConfiguration('model_path')
    device = LaunchConfiguration('device')
    conf = LaunchConfiguration('conf')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('model_path', default_value='yolov8n.pt'),
        DeclareLaunchArgument('device', default_value='cpu'),  # oder 'cuda:0'
        DeclareLaunchArgument('conf', default_value='0.25'),

        Node(
            package='yolo_poc',
            executable='yolo_node',
            name='yolo_node',
            output='screen',
            parameters=[{
                'image_topic': image_topic,
                'model_path': model_path,
                'device': device,
                'conf': conf,
                'publish_annotated': True,
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', ['$(find-pkg-share yolo_poc)', '/rviz/yolo_poc.rviz']]
        )
    ])