import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction

def _csv(value):
    return [item.strip().strip('/') for item in value.split(',') if item.strip()]

def _package_path(package_name, source_path, compiled):
    if compiled == 'True':
        return get_package_share_directory(package_name)
    return source_path

def _as_bool(value):
    return str(value).strip().lower() in ('true', '1', 'yes', 'y', 'on')

def _arm_specs(context):
    namespace = LaunchConfiguration('namespace').perform(context).strip().strip('/')
    device = LaunchConfiguration('device').perform(context).strip()
    arm_names = _csv(LaunchConfiguration('arm_names').perform(context))
    devices = [item.strip() for item in LaunchConfiguration('devices').perform(context).split(',') if item.strip()]

    if arm_names:
        if len(devices) != len(arm_names):
            raise RuntimeError(
                'When arm_names is provided, devices must have the same number of comma-separated entries. '
                'Example: arm_names:=arm_a,arm_b devices:=/dev/rrc_arm_a,/dev/rrc_arm_b'
            )
        return list(zip(arm_names, devices))

    return [(namespace, device)]

def launch_setup(context, *args, **kwargs):
    compiled = os.environ.get('need_compile', 'False')
    chassis_type = os.environ.get('CHASSIS_TYPE', '')

    ros_robot_controller_package_path = _package_path(
        'ros_robot_controller',
        '/home/ubuntu/ros2_ws/src/driver/ros_robot_controller',
        compiled,
    )
    servo_controller_package_path = _package_path(
        'servo_controller',
        '/home/ubuntu/ros2_ws/src/driver/servo_controller',
        compiled,
    )
    kinematics_package_path = _package_path(
        'kinematics',
        '/home/ubuntu/ros2_ws/src/driver/kinematics',
        compiled,
    )
    chassis_package_path = _package_path(
        'chassis',
        '/home/ubuntu/ros2_ws/src/chassis',
        compiled,
    )

    baudrate = int(LaunchConfiguration('baudrate').perform(context))
    serial_timeout = float(LaunchConfiguration('serial_timeout').perform(context))
    start_servo_check = _as_bool(LaunchConfiguration('start_servo_check').perform(context))

    actions = []
    for namespace, device in _arm_specs(context):
        imu_frame = f'{namespace}/imu_link' if namespace else 'imu_link'
        base_frame = namespace

        actions.extend([
            Node(
                package='ros_robot_controller',
                executable='ros_robot_controller',
                name='ros_robot_controller',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'device': device,
                    'baudrate': baudrate,
                    'serial_timeout': serial_timeout,
                    'imu_frame': imu_frame,
                }],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(servo_controller_package_path, 'launch/servo_controller.launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'base_frame': base_frame,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(kinematics_package_path, 'launch/kinematics_node.launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                }.items(),
            ),
        ])

        if start_servo_check:
            actions.append(Node(
                package='sdk',
                executable='check_servo_connection',
                namespace=namespace,
                output='screen',
            ))

    start_chassis_value = LaunchConfiguration('start_chassis').perform(context).strip().lower()
    if start_chassis_value == 'auto':
        start_chassis = chassis_type in ('Mecanum', 'Tank')
    else:
        start_chassis = _as_bool(start_chassis_value)

    if start_chassis:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(chassis_package_path, 'launch/chassis_controller_node.launch.py')
            )
        ))

    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Single-arm namespace. Example: namespace:=arm_a. Ignored when arm_names is used.',
        ),
        DeclareLaunchArgument(
            'device',
            default_value='/dev/rrc',
            description='Single-arm serial device. Example: device:=/dev/rrc_arm_a. Ignored when arm_names/devices are used.',
        ),
        DeclareLaunchArgument(
            'arm_names',
            default_value='',
            description='Comma-separated namespaces for multiple arms. Example: arm_names:=arm_a,arm_b.',
        ),
        DeclareLaunchArgument(
            'devices',
            default_value='',
            description='Comma-separated serial devices matching arm_names. Example: devices:=/dev/rrc_arm_a,/dev/rrc_arm_b.',
        ),
        DeclareLaunchArgument(
            'baudrate',
            default_value='1000000',
            description='Serial baudrate for all arm controllers.',
        ),
        DeclareLaunchArgument(
            'serial_timeout',
            default_value='5.0',
            description='Serial timeout in seconds for all arm controllers.',
        ),
        DeclareLaunchArgument(
            'start_servo_check',
            default_value='false',
            description='Start sdk/check_servo_connection under each arm namespace. Keep false until that script uses relative/namespaced services.',
        ),
        DeclareLaunchArgument(
            'start_chassis',
            default_value='auto',
            description='auto, true, or false. auto preserves the original CHASSIS_TYPE-based behavior.',
        ),
        OpaqueFunction(function=launch_setup),
    ])

if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
