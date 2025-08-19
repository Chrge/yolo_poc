import os, xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory

def test_world_xml_is_valid():
    share = get_package_share_directory('yolo_poc')
    candidates = ['camera.world', 'gazebo-world_truck-and-trafficlight.sdf']
    paths = [os.path.join(share, 'worlds', c) for c in candidates if os.path.exists(os.path.join(share,'worlds',c))]
    assert paths, 'no world file found in share/worlds'
    for p in paths:
        ET.parse(p)
