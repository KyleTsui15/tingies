import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _safe_frame_prefix(namespace):
    cleaned = namespace.strip('/')
    return cleaned.replace('/', '_') if cleaned else 'camera'


def launch_setup(context, *args, **kwargs):
    compiled = os.environ.get('need_compile', 'False')
    camera_type = os.environ.get('CAMERA_TYPE', 'USB_CAM')

    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    camera_namespace = LaunchConfiguration('camera_namespace').perform(context)
    video_device = LaunchConfiguration('video_device').perform(context)
    depth_camera_name = LaunchConfiguration('depth_camera_name').perform(context)
    camera_frame = LaunchConfiguration('camera_frame').perform(context)
    camera_link_frame = LaunchConfiguration('camera_link_frame').perform(context)

    prefix = _safe_frame_prefix(camera_namespace)
    if not depth_camera_name:
        depth_camera_name = prefix + '_depth_cam'
    if not camera_frame:
        camera_frame = prefix + '_color_frame'
    if not camera_link_frame:
        camera_link_frame = prefix + '_link'

    if camera_type == 'GEMINI':
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(peripherals_package_path, 'launch/include/gemini.launch.py')),
            launch_arguments={
                'camera_name': depth_camera_name,
            }.items()
        )]

    return [GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(peripherals_package_path, 'launch/include/usb_cam.launch.py')),
            launch_arguments={
                'camera_namespace': camera_namespace,
                'video_device': video_device,
                'camera_name': depth_camera_name,
                'camera_frame': camera_frame,
            }.items()
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            namespace=camera_namespace,
            name='depth_cam_base_link',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--qx', '0',
                '--qy', '0',
                '--qz', '0',
                '--qw', '1',
                '--frame-id', camera_link_frame,
                '--child-frame-id', camera_frame,
            ]
        ),
    ])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_namespace', default_value='depth_cam'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('depth_camera_name', default_value=''),
        DeclareLaunchArgument('camera_frame', default_value=''),
        DeclareLaunchArgument('camera_link_frame', default_value=''),
        DeclareLaunchArgument('app', default_value='false'),
        OpaqueFunction(function=launch_setup),
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
