import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

#agents = ["alpha"]
#agents = ["Bob"]

def generate_launch_description():
    ld = LaunchDescription()

    agent_name = os.environ.get('MY_NAME')

    bridge = Node(
        namespace=agent_name,
        name="ros_bridge",
        package="rosbridge_server",
        executable="rosbridge_websocket",
        parameters=[{'port': 9091}],
        output='screen'
    )

    manager = Node(
        namespace = agent_name,
        name="manager_node",
        package="surfer_manager",
        executable="surfer_manager_node",
        parameters=[{'name': agent_name}, {'type': 'sim_boat'},'/home/ubuntu/ros2_ws/src/surfer_manager/config/surfer_params.yaml'],
        #parameters=[{'name': agent_name}, {'type': 'sim_boat'},{'group': 'avengers'},{'behaviors':['waypoint', 'path']}],
        output='screen'
    )


    interface = Node(
        namespace=agent_name,
        name="surfer_interface",
        package="surfer_control",
        executable="surfer_interface",
        parameters=['/home/ubuntu/ros2_ws/src/surfer_manager/config/surfer_params.yaml']
    )

    controller = Node(
        namespace=agent_name,
        name="surfer_controller",
        package="surfer_control",
        executable="surfer_controller",
        parameters=['/home/ubuntu/ros2_ws/src/surfer_manager/config/surfer_params.yaml'],
        #remappings=[('/'+agent_name+'/odom','/model/'+agent_name+'/odometry')],
        output='screen'
    )


    autonomy = Node(
        namespace=agent_name,
        name="autonomy_node",
        package="surfer_autonomy",
        executable="surfer_autonomy_node",
        parameters=['/home/ubuntu/ros2_ws/src/surfer_manager/config/surfer_params.yaml'],
        #remappings=[('/'+agent_name+'/odom','/model/'+agent_name+'/odometry')],
        output='screen'
    )


    ld.add_action(bridge)
    ld.add_action(manager)
    ld.add_action(interface)
    ld.add_action(controller)
    ld.add_action(autonomy)

    return ld
