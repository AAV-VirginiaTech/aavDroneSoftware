from aav_software.object_alignment_controller import Mission
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
        parameters=[
            {"current_mission": Mission.PACKAGE_DELIVERY_CUASC.value},
            {"descent_alignment_altitude": 5.0},
            {"hardcoded_drop_altitude": 3.0},
        ],
    )

    topic_converter = Node(
        package="aav_software",
        executable="topic_converter_for_drone",
    )

    ld.add_action(location_logger)
    ld.add_action(manavs_magic_code)
    ld.add_action(object_alignment_controller)
    ld.add_action(topic_converter)

    return ld
