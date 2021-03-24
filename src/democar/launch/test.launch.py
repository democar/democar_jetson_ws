import os

import ament_index_python.packages
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from launch_ros.actions import Node


def generate_launch_description():
    def create_cam_node(serial, namespace):
        return Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace=namespace,
            output='screen',
            parameters=[
                {
                    'serial_no': serial,
                    'color_qos': 'SYSTEM_DEFAULT',
                    'depth_qos': 'SYSTEM_DEFAULT',
                    'enable_infra1': False,
                    'enable_infra2': False,
                    'enable_depth': False,
#                    'filters': 'colorizer,pointcloud',
                    'filters': 'colorizer',
                    'tf_publish_rate': 10.0,
                    'ordered_pc': True,
                    'allow_no_texture_points': True,
                    'align_depth': False,
                    'color_fps': 15.0,
                    'depth_fps': 15.0,
#                    'initial_reset': True,
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

    velodyne_convert_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_convert_node',
        output='screen',
        parameters=[{
            'calibration': os.path.join(ament_index_python.packages.get_package_share_directory('velodyne_pointcloud'), 'params', 'VLP16db.yaml'),
            'min_range': 0.9,
            'max_range': 130.0,
            'view_direction': 0.0,
            'organize_cloud': True, 
        }]
    )


    return launch.LaunchDescription([
#        create_cam_node('825312073164', '/camera1'),
#        create_cam_node('831612071848', '/camera2'),
#        velodyne_driver_node,
#        gps_node,

        velodyne_convert_node,
        detectnet_node,
        segnet_node,
    ])
