import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    compiled = os.environ.get('need_compile', 'False')

    namespace = LaunchConfiguration('namespace')
    base_frame = LaunchConfiguration('base_frame')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for this arm, e.g. arm_a or arm_b. Leave empty for the old global one-arm layout.',
    )
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='',
        description='Base frame prefix/string used by the servo controller.',
    )

    if compiled == 'True':
        servo_controller_package_path = get_package_share_directory('servo_controller')
    else:
        servo_controller_package_path = '/home/ubuntu/ros2_ws/src/driver/servo_controller'

    servo_controller_node = Node(
        package='servo_controller',
        executable='servo_controller',
        namespace=namespace,
        output='screen',
        parameters=[
            os.path.join(servo_controller_package_path, 'config/servo_controller.yaml'),
            {'base_frame': base_frame},
        ],
    )

    grasp_node = Node(
        package='servo_controller',
        executable='grasp',
        namespace=namespace,
        output='screen',
    )

    return LaunchDescription([
        namespace_arg,
        base_frame_arg,
        servo_controller_node,
        grasp_node,
    ])

if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
