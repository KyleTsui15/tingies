import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _safe_frame_prefix(namespace):
    cleaned = namespace.strip('/')
    return cleaned.replace('/', '_') if cleaned else 'camera'


def launch_setup(context, *args, **kwargs):
    compiled = os.environ.get('need_compile', 'False')
    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        peripherals_package_path = '/home/ubuntu/ros2_ws/src/peripherals'

    camera_namespace = LaunchConfiguration('camera_namespace').perform(context)
    video_device = LaunchConfiguration('video_device').perform(context)
    camera_name = LaunchConfiguration('camera_name').perform(context)
    camera_frame = LaunchConfiguration('camera_frame').perform(context)

    prefix = _safe_frame_prefix(camera_namespace)
    if not camera_name:
        camera_name = prefix + '_usb_cam'
    if not camera_frame:
        camera_frame = prefix + '_color_frame'

    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        output='screen',
        namespace=camera_namespace,
        name='usb_cam',
        parameters=[
            os.path.join(peripherals_package_path, 'config', 'usb_cam_param.yaml'),
            {'video_device': video_device},
            {'camera_name': camera_name},
            {'frame_id': camera_frame},
            {'pixel_format': 'yuyv2rgb'},
        ],
        remappings=[
            ('image_raw/compressed', 'rgb/image_compressed'),
            ('image_raw/compressedDepth', 'rgb/compressedDepth'),
            ('image_raw/theora', 'rgb/image_raw/theora'),
            ('camera_info', 'rgb/camera_info'),
        ]
    )

    image_proc_node = Node(
        package='image_proc',
        executable='image_proc',
        output='screen',
        namespace=camera_namespace,
        name='image_proc_node',
        remappings=[
            ('image', 'image_raw'),
            ('image_raw/compressed', 'rgb/image_compressed'),
            ('image_raw/compressedDepth', 'rgb/compressedDepth'),
            ('image_raw/theora', 'rgb/image_raw/theora'),
            ('image_rect', 'rgb/image_raw'),
            ('camera_info', 'rgb/camera_info'),
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )

    return [camera_node, image_proc_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_namespace', default_value='depth_cam'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_name', default_value=''),
        DeclareLaunchArgument('camera_frame', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])


if __name__ == '__main__':
    ld = generate_launch_description()
    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
