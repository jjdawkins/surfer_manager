import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

#agents = ["alpha"]
agents = ["Alice","Bob","Carol","Dave"]
#agents = ["Alice"]

def generate_launch_description():
    ld = LaunchDescription()
    i = 0
    for agent_name in agents:

        surfer_urdf =os.path.join(get_package_share_directory('surfer_description'),'urdf/','surfer.sdf')
        manager = Node(
            namespace = agent_name,
            name="manager_node",
            package="surfer_manager",
            executable="surfer_manager_node",
            parameters=[{'name': agent_name}, {'type': 'sim_boat'},'/home/arty/ros2_ws/src/surfer_manager/config/surfer_params.yaml'],
            #parameters=[{'name': agent_name}, {'type': 'sim_boat'},{'group': 'avengers'},{'behaviors':['waypoint', 'path']}],
            output='screen'
        )

        bridge = Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            arguments=['/model/'+agent_name+'/joint/propeller_joint_fr/cmd_thrust@std_msgs/msg/Float64@ignition.msgs.Double',
                       '/model/'+agent_name+'/joint/propeller_joint_fl/cmd_thrust@std_msgs/msg/Float64@ignition.msgs.Double',
                       '/model/'+agent_name+'/joint/propeller_joint_br/cmd_thrust@std_msgs/msg/Float64@ignition.msgs.Double',
                       '/model/'+agent_name+'/joint/propeller_joint_bl/cmd_thrust@std_msgs/msg/Float64@ignition.msgs.Double',
                       '/model/'+agent_name+'/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
            remappings=[('/model/'+agent_name+'/joint/propeller_joint_fr/cmd_thrust','/'+agent_name+'/propeller_joint_fr/cmd_thrust'),
                        ('/model/'+agent_name+'/joint/propeller_joint_fl/cmd_thrust','/'+agent_name+'/propeller_joint_fl/cmd_thrust'),
                        ('/model/'+agent_name+'/joint/propeller_joint_br/cmd_thrust','/'+agent_name+'/propeller_joint_br/cmd_thrust'),
                        ('/model/'+agent_name+'/joint/propeller_joint_bl/cmd_thrust','/'+agent_name+'/propeller_joint_bl/cmd_thrust'),
                        ('/model/'+agent_name+'/odometry','/'+agent_name+'/odom')],
            output='screen'
        )

        interface = Node(
            namespace=agent_name,
            name="surfer_interface",
            package="surfer_control",
            executable="surfer_interface",
            parameters=['/home/arty/ros2_ws/src/surfer_manager/config/surfer_params.yaml']
        )

        controller = Node(
            namespace=agent_name,
            name="surfer_controller",
            package="surfer_control",
            executable="surfer_controller",
            parameters=['/home/arty/ros2_ws/src/surfer_manager/config/surfer_params.yaml'],
            #remappings=[('/'+agent_name+'/odom','/model/'+agent_name+'/odometry')],
            output='screen'
        )


        autonomy = Node(
            namespace=agent_name,
            name="autonomy_node",
            package="surfer_autonomy",
            executable="surfer_autonomy_node",
            parameters=['/home/arty/ros2_ws/src/surfer_manager/config/surfer_params.yaml'],
            #remappings=[('/'+agent_name+'/odom','/model/'+agent_name+'/odometry')],
            output='screen'
        )

        ignition = Node(
            namespace=agent_name,
            name="spawn_agent",
            package="ros_ign_gazebo",
            executable="create",
            arguments=['-x', '5', '-y', str(5+i) ,'-z', '0.2','-name',agent_name, '-file', surfer_urdf]
        )
        i = i+1

        ld.add_action(manager)
        ld.add_action(interface)
        ld.add_action(controller)
        #ld.add_action(autonomy)
        #ld.add_action(ignition)
        ld.add_action(bridge)
    return ld
