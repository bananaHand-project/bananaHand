from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baud", default_value="115200"),

        Node(
            package="banana_serial_bridge",
            executable="serial_bridge",
            name="serial_bridge",
            output="screen",
            parameters=[{
                "port": port,
                "baud": baud,
                "publish_rate_hz": 100.0,
                "joint_names": [f"joint_{i}" for i in range(8)],
            }],
        ),

        # Future nodes (uncomment when created)
        # create using ros2 pkg create banana_webcam --build-type ament_python --dependencies rclpy sensor_msgs
        # Node(package="banana_webcam", executable="webcam_node", name="webcam", output="screen"),
        # Node(package="banana_perception", executable="perception_node", name="perception", output="screen"),
        # Node(package="banana_tracking", executable="tracking_node", name="tracking", output="screen"),
    ])
