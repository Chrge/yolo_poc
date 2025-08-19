import os
from ament_index_python.packages import get_package_share_directory

def test_assets_present():
    share = get_package_share_directory('yolo_poc')
    assert os.path.exists(os.path.join(share, 'worlds')), 'missing worlds/'
    assert os.path.exists(os.path.join(share, 'rviz')), 'missing rviz/'
