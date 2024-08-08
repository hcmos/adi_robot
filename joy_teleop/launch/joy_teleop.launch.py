import os
# import subprocess
# import yaml
import launch
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # パラメータファイルのパス設定
    config_file_path = os.path.join(
        get_package_share_directory('joy_teleop'),
        'config',
        'params.yaml'
    )

    # 起動ノードの作成
    diff_node = Node(
        package = 'joy_teleop',
        executable = 'diff_driver_node',
        parameters = [config_file_path],
        output='screen'
    )
    joy_node = Node(
        package = 'joy',
        executable = 'joy_node',
        parameters = [config_file_path],
        output='screen'
    )
    teleop_node = Node(
        package = 'teleop_twist_joy',
        executable = 'teleop_node',
        parameters = [config_file_path],
        output='screen'
    )

    # 起動の作成
    tmcl_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('adi_tmcl'), 'launch/'),
            'tmcm_2611.launch.py'])
    )

    # 起動エンティティクラスの作成
    launch_discription = LaunchDescription()

    # 起動の追加
    launch_discription.add_entity(diff_node)
    launch_discription.add_entity(joy_node)
    launch_discription.add_entity(teleop_node)
    # launch_discription.add_action(tmcl_launch)

    return launch_discription
