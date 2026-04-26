from pathlib import Path

from aav_software.object_alignment_controller import Mission
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()

    aav_bringup = get_package_share_directory("aav_bringup")

    # Add AAV Software nodes you want to launch here

    drone_mavros_and_camera_boot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("aav_bringup"),
                    "launch",
                    "drone_mavros_and_camera_boot_launch.py",
                ]
            )
        )
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
        ],
    )

    topic_converter = Node(
        package="aav_software",
        executable="topic_converter_for_drone",
    )

    # YOLO.
    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("yolo_bringup"),
                        "launch",
                        "yolo.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "model": str(Path(aav_bringup) / "aav_yolo_models" / "bullseye.pt"),
            "input_image_topic": "/siyi_a8/image_raw",
            "image_reliability": "2",
        }.items(),
    )

    ld.add_action(drone_mavros_and_camera_boot_launch)
    ld.add_action(manavs_magic_code)
    ld.add_action(object_alignment_controller)
    ld.add_action(topic_converter)
    ld.add_action(yolo)

    return ld
