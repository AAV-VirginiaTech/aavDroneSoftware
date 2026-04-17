from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fcu_url = LaunchConfiguration("fcu_url")
    stream_id = LaunchConfiguration("stream_id")
    message_rate = LaunchConfiguration("message_rate")
    on_off = LaunchConfiguration("on_off")

    net_iface = LaunchConfiguration("net_iface")
    jetson_ip = LaunchConfiguration("jetson_ip")
    camera_ip = LaunchConfiguration("camera_ip")

    rtsp_url = LaunchConfiguration("rtsp_url")
    image_topic = LaunchConfiguration("image_topic")
    caminfo_topic = LaunchConfiguration("caminfo_topic")

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "fcu_url",
            default_value="/dev/ttyACM0:115200",
            description="MAVROS FCU URL",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "stream_id",
            default_value="0",
            description="MAVROS stream ID",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "message_rate",
            default_value="10",
            description="MAVLink stream rate in Hz",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "on_off",
            default_value="true",
            description="Enable or disable MAVLink streaming",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "net_iface",
            default_value="eth0",
            description="Network interface connected to the camera",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "jetson_ip",
            default_value="192.168.144.30/24",
            description="Static IP to assign to the Jetson network interface",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "camera_ip",
            default_value="192.168.144.25",
            description="Camera IP address",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "rtsp_url",
            default_value="rtsp://192.168.144.25:8554/main.264",
            description="RTSP URL for SIYI camera",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "image_topic",
            default_value="/siyi_a8/image_raw",
            description="Camera image topic",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "caminfo_topic",
            default_value="/siyi_a8/camera_info",
            description="Camera info topic",
        )
    )

    mavros_node = Node(
        package="mavros",
        executable="mavros_node",
        name="mavros",
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
    )

    set_stream_rate = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/mavros/set_stream_rate",
                    "mavros_msgs/srv/StreamRate",
                    [
                        "{stream_id: ",
                        stream_id,
                        ", message_rate: ",
                        message_rate,
                        ", on_off: ",
                        on_off,
                        "}",
                    ],
                ],
                output="screen",
            )
        ],
    )

    configure_camera_network = ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            [
                "sudo ip addr flush dev ",
                net_iface,
                " || true; sudo ip addr add ",
                jetson_ip,
                " dev ",
                net_iface,
                "; sudo ip link set ",
                net_iface,
                " up",
            ],
        ],
        output="screen",
    )

    ping_camera = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ping",
                    "-c",
                    "3",
                    camera_ip,
                ],
                output="screen",
            )
        ],
    )

    gscam_node = TimerAction(
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
                            "rtph265depay ! h265parse ! "
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
                    ("/camera/image_raw", image_topic),
                    ("/camera/camera_info", caminfo_topic),
                ],
            )
        ],
    )

    ld.add_action(mavros_node)
    ld.add_action(set_stream_rate)
    ld.add_action(configure_camera_network)
    ld.add_action(ping_camera)
    ld.add_action(gscam_node)

    return ld
