import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

agents = ["Alice","Bob"]

def generate_launch_description():
    ld = LaunchDescription()
    i = 0

    for agent_name in agents:

        ign_bridge = Node(
            namespace=agent_name,
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

        ld.add_action(ign_bridge)
    return ld
