import os

import ament_index_python.packages
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    pkg_prefix = get_package_share_directory('democar')
    pkg_prefix = '/home/democar/democar/src/democar'
    ssc_interface_param_file = os.path.join(pkg_prefix, 'param/ssc_interface.param.yaml')
    mpc_param_file = os.path.join(pkg_prefix, 'param/mpc.param.yaml')

    mpc_param = DeclareLaunchArgument(
        'mpc_param_file',
        default_value=mpc_param_file,
        description='Path to config file for MPC'
    )
    ssc_interface_param = DeclareLaunchArgument(
        'ssc_interface_param_file',
        default_value=ssc_interface_param_file,
        description='Path to config file for SSC interface'
    )

    def cam_tfs(prefix):
        frames = [
            "depth_frame_id",
            "infra1_frame_id",
            "infra2_frame_id",
            "color_frame_id",
            "fisheye_frame_id",
            "fisheye1_frame_id",
            "fisheye2_frame_id",
            "accel_frame_id",
            "gyro_frame_id",
            "pose_frame_id",
            "depth_optical_frame_id",
            "infra1_optical_frame_id",
            "infra2_optical_frame_id",
            "color_optical_frame_id",
            "fisheye_optical_frame_id",
            "fisheye1_optical_frame_id",
            "fisheye2_optical_frame_id",
            "accel_optical_frame_id",
            "gyro_optical_frame_id",
            "imu_optical_frame_id",
            "pose_optical_frame_id",
            "aligned_depth_to_color_frame_id",
            "aligned_depth_to_infra1_frame_id",
            "aligned_depth_to_infra2_frame_id",
            "aligned_depth_to_fisheye_frame_id",
            "aligned_depth_to_fisheye1_frame_id",
            "aligned_depth_to_fisheye2_frame_id",
        ]

        res = {}
        res[f'base_frame_id'] = f'{prefix}_link'
        for frame in frames:
            res[frame] = f'{prefix}_{frame}'
        return res


    def create_cam_node(serial, namespace):
        return Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace=namespace,
            output='screen',
            parameters=[
                {
                    'serial_no': serial,
                    'color_qos': 'SENSOR_DATA',
#                    'depth_qos': 'SYSTEM_DEFAULT',
                    'enable_infra1': False,
                    'enable_infra2': False,
                    'enable_depth': False,
#                    'filters': 'colorizer,pointcloud',
#                    'filters': 'colorizer',
#                    'tf_publish_rate': 10.0,
#                    'ordered_pc': True,
#                    'allow_no_texture_points': True,
#                    'align_depth': False,
                    'color_fps': 15.0,
#                    'tf_prefix': namespace.replace('/', ''),
#                    'base_frame_id': namespace.replace('/', '') + '_link',
#                    'depth_fps': 30.0,
#                    'color/image_raw': {
#                        'disable_pub_plugins': [
#                            'image_transport/compressedDepth',
#                        ],
#                    },
                    **cam_tfs(namespace.replace('/', '')),
                },
            ]
        )

    urdf = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        arguments=[os.path.join(pkg_prefix, "democar.urdf")]
    )

    tecko = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace='/cam2',
        output='screen',
        parameters=[
            {
                'serial_no': '909212111313',
                'enable_fisheye1': False,
                'enable_fisheye2': False,
                'odom_frame_id': 'odom',
                **cam_tfs('tcam'),
            },
        ]
    )

    detectnet_node = Node(
        package='ros_deep_learning',
        executable='detectnet',
        output='screen',
        remappings=[
            ('/detectnet/image_in', '/camera1/color/image_raw'),
        ],
        parameters=[
#            {"model_name": "xx"},
        ],
    )

    segnet_node = Node(
        package='ros_deep_learning',
        executable='segnet',
        output='screen',
        remappings=[
            ('/segnet/image_in', '/camera1/color/image_raw'),
        ],
    )

    gps_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        output='screen',
    )

    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='screen',
    )

    velodyne_laserscan = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        output='screen',
    )


    velodyne_convert_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        output='screen',
        parameters=[{
            'calibration': os.path.join(ament_index_python.packages.get_package_share_directory('velodyne_pointcloud'), 'params', 'VLP16db.yaml'),
            'min_range': 1.8,
            'max_range': 199.9,
            'view_direction': 0.0,
#            'organize_cloud': True, 
#            'target_frame': "base_link",
#            'target_id': 'odom',
        }]
    )

    zds_dir = '/home/democar/zenoh-plugin-dds-gstreamer/target/release/'
    zds_bridge = ExecuteProcess(cmd=[
            os.path.join(zds_dir, 'zenoh-bridge-dds'),
            '-m', 'peer', '-e', 'tcp/sumo2.cs.vsb.cz:7447',
            '--coders-config', os.path.join(zds_dir, '../../coders_config.yml'),
        ],
        output='screen'
    )

    container = ComposableNodeContainer(
        package='rclcpp_components',
        executable='component_container',
        name='container',
        namespace='',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='gpsd_client',
                plugin='gpsd_client::GPSDClientComponent',
                name='gps_client',
            ),
#            ComposableNode(
#                package='gps_tools',
#                plugin='gps_tools::UtmOdometryComponent',
#                name='gps',
#            )
        ]
    )

    mpc = Node(
        package='mpc_controller_nodes',
        executable='mpc_controller_node_exe',
        name='mpc_controller',
        namespace='control',
        output='screen',
#        arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[
            LaunchConfiguration('mpc_param_file'),
        ],
    )

    ssc_interface = Node(
        package='ssc_interface',
        executable='ssc_interface_node_exe',
        name='ssc_interface',
        namespace='vehicle',
        parameters=[LaunchConfiguration('ssc_interface_param_file')],
        output='screen',
        remappings=[
            ('gear_select', '/ssc/gear_select'),
            ('arbitrated_speed_commands', '/ssc/arbitrated_speed_commands'),
            ('arbitrated_steering_commands', '/ssc/arbitrated_steering_commands'),
            ('turn_signal_command', '/ssc/turn_signal_command'),
            ('dbw_enabled_feedback', '/ssc/dbw_enabled_feedback'),
            ('gear_feedback', '/ssc/gear_feedback'),
            ('velocity_accel_cov', '/ssc/velocity_accel_cov'),
            ('state_report_out', 'state_report'),
            ('steering_feedback', '/ssc/steering_feedback'),
            ('vehicle_kinematic_state_cog', 'vehicle_kinematic_state')
        ],
    )

    """
    return launch.LaunchDescription([
        urdf,
        tecko,

        mpc_param,
        mpc,

        ssc_interface_param,
        ssc_interface,
    ])
    """


    return launch.LaunchDescription([
#        urdf,
#        tecko,
#        mpc_param,
#        mpc,

#        ssc_interface_param,
#        ssc_interface,

#        zds_bridge,

        create_cam_node('825312073164', '/camera1'),
        create_cam_node('831612071848', '/camera2'),
        velodyne_driver_node,
#        gps_node,
#        container,

#        velodyne_laserscan,
        velodyne_convert_node,
        detectnet_node,
        segnet_node,
    ])
