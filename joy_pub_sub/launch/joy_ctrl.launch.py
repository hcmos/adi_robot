from launch import LaunchDescription
from launch_ros.actions import Node

#launch fileを使用するときは、各ノードのコンソール出力を消す

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def generate_launch_description():
    joy_pub_sub_node = Node(
        package='joy_pub_sub',
        executable='joy_pub_sub',
        #output="screen"
    )

    print(f"{bcolors.OKGREEN}[Launch]joy_pub_sub Node ready {bcolors.ENDC}")

    joy_linux_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        #output="screen"

    )

    print(f"{bcolors.OKGREEN}[Launch]joy_linux Node ready {bcolors.ENDC}")

    return LaunchDescription(
        [
            joy_pub_sub_node,
            joy_linux_node
        ]
    )
