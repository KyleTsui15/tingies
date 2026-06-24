from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def controller_node(namespace: str, device: str) -> Node:
    return Node(
        package='ros_robot_controller',
        executable='ros_robot_controller',
        name='ros_robot_controller',
        namespace=namespace,
        output='screen',
        parameters=[
            {
                'device': device,
                'baudrate': 1000000,
                'serial_timeout': 5.0,
                'imu_frame': f'{namespace}/imu_link',
            }
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        controller_node(
            namespace='arm_a',
            device='/dev/rrc_arm_a', #CHANGE
        ),
        controller_node(
            namespace='arm_b',
            device='/dev/rrc_arm_b', #CHANGE
        ),
    ])

if __name__ == '__main__':
    # Create a LaunchDescription object. 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
