import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
import launch
import launch_ros.actions
import xacro

from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import Node


# share_dir_path = os.path.join(get_package_share_directory('urdf'))

xacro_path = os.path.join("/home/auv/_urdf_ws/src/urdf_test/urdf/agv2.xacro")#ここを変更
urdf_path = os.path.join("/home/auv/_urdf_ws/src/urdf_test/urdf/test.urdf")

def generate_launch_description():
    # xacroをロード
    doc = xacro.process_file(xacro_path)
    # xacroを展開してURDFを生成
    robot_desc = doc.toprettyxml(indent='  ')
    # urdf_pathに対してurdfを書き出し
    f = open(urdf_path, 'w')
    f.write(robot_desc)
    f.close()
    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  #node_executable='robot_state_publisher',
                                  output='both',
                                  # argumentsでURDFを出力したパスを指定
                                  arguments=[urdf_path])


    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        #parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    return launch.LaunchDescription([

        rsp,
        robot_state_publisher_node,
                Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf_path]),

        ])