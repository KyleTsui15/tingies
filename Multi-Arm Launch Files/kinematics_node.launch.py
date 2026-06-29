from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='ROS namespace for this arm, e.g. arm_a or arm_b. Leave empty for the old global one-arm layout.',
    )

    kinematics_node = Node(
        package='kinematics',
        executable='search_kinematics_solutions',
        namespace=namespace,
        output='screen',
    )

    return LaunchDescription([
        namespace_arg,
        kinematics_node,
    ])

if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
