from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    manager_1 = Node(
        namespace = "ironman",
        name="manager",
        package="surfer_manager",
        executable="surfer_manager_node",
        parameters=[{'name': 'ironman'}, {'type': 'drone'},{'group': 'avengers'}]
    )
    manager_2 = Node(
        namespace="hulk",
        name="manager",
        package="surfer_manager",
        executable="surfer_manager_node",
        parameters=[{'name': 'hulk'}, {'type': 'boat'},{'group': 'avengers'}]

    )
    manager_3 = Node(
        namespace="thor",
        name="manager",
        package="surfer_manager",
        executable="surfer_manager_node",
        parameters=[{'name': 'thor'}, {'type': 'rover'},{'group': 'avengers'}]
    )

    autonomy_2 = Node(
        namespace="hulk",
        name="autonomy",
        package="surfer_autonomy",
        executable="basic_autonomy",
        parameters=[]
    )
    #ld.add_action(manager_1)
    ld.add_action(manager_2)
    ld.add_action(autonomy_2)
    #ld.add_action(manager_3)
    return ld
