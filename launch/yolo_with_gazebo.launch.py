from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = FindPackageShare('yolo_poc')

    world_path = PathJoinSubstitution([
        pkg_share, 'worlds', 'gazebo-world_truck-and-trafficlight.sdf'
    ])

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('gazebo_ros'), 'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true'
        }.items()
    )

    rviz_cfg = PathJoinSubstitution([pkg_share, 'rviz', 'rviz2-config_truck-and-trafficlight-with-yolo-annotation.rviz'])

    yolo_and_rviz = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='yolo_poc',
                executable='yolo_node',
                name='yolo_node',
                output='screen',
                parameters=[{
                    'input_image_topic': '/camera/image_raw',
                    'model': 'yolov8n.pt',
                    'device': 'cpu',
                    'conf_thres': 0.25,
                    'publish_annotated': True,
                }]
            ),
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_cfg],
                output='screen'
            )
        ]
    )

    return LaunchDescription([gz, yolo_and_rviz])
