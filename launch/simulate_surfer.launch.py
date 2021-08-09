import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

#agents = ["alpha"]
agents = ["hulk","alpha","beta"]
def generate_launch_description():
    ld = LaunchDescription()
    i = 0
    for agent_name in agents:

        #agent_name = "hulk"
        surfer_urdf =os.path.join(get_package_share_directory('surfer_description'),'urdf/','surfer.urdf')
        manager = Node(
            namespace = agent_name,
            name="manager_node",
            package="surfer_manager",
            executable="surfer_manager_node",
            parameters=[{'name': agent_name}, {'type': 'sim_boat'},{'group': 'avengers'},{'behaviors':['waypoint', 'path']}],
            output='screen'
        )

        autonomy = Node(
            namespace=agent_name,
            name="autonomy_node",
            package="surfer_autonomy",
            executable="surfer_autonomy_node",
            parameters=['/home/arty/ros2_ws/src/surfer_manager/config/surfer_params.yaml']
        )

        gazebo = Node(
            namespace=agent_name,
            name="spawn_agent",
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=['-x', '5', '-y', str(5+i) ,'-z', '0.2',
            '-robot_namespace',agent_name,'-entity', agent_name, '-file', surfer_urdf]
        )
        i = i+1

        ld.add_action(manager)
        ld.add_action(autonomy)
        ld.add_action(gazebo)

    return ld
