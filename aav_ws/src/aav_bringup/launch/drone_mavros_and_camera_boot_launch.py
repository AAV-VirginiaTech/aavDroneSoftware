from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    net_iface = LaunchConfiguration("net_iface")
    jetson_ip = LaunchConfiguration("jetson_ip")
    camera_ip = LaunchConfiguration("camera_ip")
    rtsp_url = LaunchConfiguration("rtsp_url")

    return LaunchDescription(
        [
            DeclareLaunchArgument("fcu_url", default_value="/dev/ttyACM0:115200"),
            DeclareLaunchArgument("net_iface", default_value="eth0"),
            DeclareLaunchArgument("jetson_ip", default_value="192.168.144.30/24"),
            DeclareLaunchArgument("camera_ip", default_value="192.168.144.25"),
            DeclareLaunchArgument(
                "rtsp_url",
                default_value="rtsp://192.168.144.25:8554/main.264",
            ),
            Node(
                package="mavros",
                executable="mavros_node",
                output="screen",
                parameters=[
                    {
                        "fcu_url": fcu_url,
                        "gcs_url": "",
                        "tgt_system": 1,
                        "tgt_component": 1,
                        "plugin_allowlist": [
                            "sys_status",
                            "global_position",
                            "local_position",
                            "setpoint_position",
                            "command",
                        ],
                    }
                ],
            ),
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "service",
                            "call",
                            "/mavros/set_stream_rate",
                            "mavros_msgs/srv/StreamRate",
                            "{stream_id: 0, message_rate: 10, on_off: true}",
                        ],
                        output="screen",
                    )
                ],
            ),
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    [
                        "ip addr flush dev ",
                        net_iface,
                        " || true; ip addr add ",
                        jetson_ip,
                        " dev ",
                        net_iface,
                        "; ip link set ",
                        net_iface,
                        " up",
                    ],
                ],
                output="screen",
            ),
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=["ping", "-c", "3", camera_ip],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package="gscam",
                        executable="gscam_node",
                        name="siyi_a8_gscam",
                        output="screen",
                        parameters=[
                            {
                                "gscam_config": [
                                    "rtspsrc location=",
                                    rtsp_url,
                                    " protocols=tcp latency=100 ! "
                                    "rtph264depay ! h264parse ! "
                                    "nvv4l2decoder ! nvvidconv ! videoconvert ! "
                                    "video/x-raw,format=RGB",
                                ],
                                "camera_name": "a8_mini",
                                "frame_id": "a8_mini_optical_frame",
                                "image_encoding": "rgb8",
                                "use_gst_timestamps": True,
                                "sync_sink": False,
                                "use_sensor_data_qos": True,
                            }
                        ],
                        remappings=[
                            ("/camera/image_raw", "/siyi_a8/image_raw"),
                            ("/camera/camera_info", "/siyi_a8/camera_info"),
                        ],
                    )
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="aavrviz",
                output="screen",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [FindPackageShare("aav_bringup"), "launch", "aavrviz.rviz"]
                    ),
                ],
            ),
        ]
    )
