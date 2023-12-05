import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lio_sam')
    parameter_file = LaunchConfiguration('params_file')
    xacro_path = os.path.join(share_dir, 'config', 'robot.urdf.xacro')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    print("urdf_file_name : {}".format(xacro_path))

    return LaunchDescription([
        params_declare,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro', ' ', xacro_path])
            }]
        ),
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='ekf_filter_node',
        #     output='screen',
        #     remappings=[
        #         ('odometry/filtered', 'odometry/navsat')
        #     ]
        # ),
        # Node(
        #     package='robot_localization',
        #     executable='navsat_transform_node',
        #     name='navsat_transform_node',
        #     output='screen',
        #     remappings=[
        #         ('/imu/data', '/imu_correct'),
        #         ('gps/fix', 'gps/fix'),
        #         ('odometry/filtered', 'odometry/navsat')
        #     ]
        # ),
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher_cam0',
        #     arguments='0.08696546102648862 -0.017537580339192428 -0.06736622212957208 -1.56432762551363 -0.0050157139193218025 -1.5598397340512746 rs_helios_top camera0/camera_link'.split(' '),
        #     parameters=[parameter_file],
        #     output='screen'
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher_cam1',
        #     arguments='-0.0020708674709257625 0.050302764703668144 -0.060050850100904336 -1.5469580691450913 -0.030272431008802673 0.5415901834625709 rs_helios_top camera1/camera_link'.split(' '),
        #     parameters=[parameter_file],
        #     output='screen'
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher_cam2',
        #     arguments='-0.023537223118448396 -0.0436782178030819 -0.04235718660683201 -1.5604431961975036 -0.0008004950085338482 2.620501402213116 rs_helios_top camera2/camera_link'.split(' '),
        #     parameters=[parameter_file],
        #     output='screen'
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_transform_publisher_cam3',
        #     arguments='-0.042907827350912114 0.0023617584919831524 0.07342035097308623 -0.4916133 -0.4929164 -0.5071043 0.5081289 rs_helios_top camera3/camera_link'.split(' '),
        #     parameters=[parameter_file],
        #     output='screen'
        # )
    ])