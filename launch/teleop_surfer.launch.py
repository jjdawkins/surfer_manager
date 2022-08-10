import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

agents = ["billy"]
#agents = ["hulk","alpha","beta"]
def generate_launch_description():
    ld = LaunchDescription()
    i = 0
    for agent_name in agents:

        #agent_name = "hulk"
        surfer_urdf =os.path.join(get_package_share_directory('surfer_description'),'urdf/','surfer.urdf')
        joynode = Node(
            name="joystick_driver",
            package="joy",
            executable="joy_node"
        )

        teleop = Node(
            name="surfer_teleop_node",
            package="teleop_twist_joy",
            executable="teleop_node",
            parameters=['/home/arty/ros2_ws/src/surfer_manager/config/surfer_teleop_config.yaml'],
            remappings=[
            ('/cmd_vel', '/'+agent_name+'/cmd_force'),
            ]
        )

        i = i+1

        ld.add_action(joynode)
        ld.add_action(teleop)

    return ld
