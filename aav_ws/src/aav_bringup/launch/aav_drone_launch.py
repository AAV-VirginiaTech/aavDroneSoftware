from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Add AAV Software nodes you want to launch here

    location_logger = Node(
        package="aav_software",
        executable="location_logger",
    )

    manavs_magic_code = Node(
        package="aav_software",
        executable="manavs_magic_code",
    )
    
    object_alignment_controller = Node(
        package="aav_software",
        executable="object_alignment_controller",
    )
    
    topic_converter = Node(
        package="aav_software",
        executable="topic_converter",
    )

    ld.add_action(location_logger)
    ld.add_action(manavs_magic_code)
    ld.add_action(object_alignment_controller)
    ld.add_action(topic_converter)


    return ld