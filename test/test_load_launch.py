from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def test_launch_file_is_importable():
    src = PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('yolo_poc'), 'launch', 'yolo_with_gazebo.launch.py'])
    )
    assert src is not None
