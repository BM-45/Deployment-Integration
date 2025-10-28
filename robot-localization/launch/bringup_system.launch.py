from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():

    camera_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_pub',
        arguments=[
            '0.0', '0.0', '0.4', 
            '0', '0', '0' ,
            'chassis',
            'rover/rover_sensors/camera_link/front_camera'
        ]
    )

    imu_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_tf_pub',
        arguments=[
            '0.0', '0.0', '0.0', 
            '0', '0', '0' ,
            'chassis',
            'rover/rover_sensors/imu_link/imu'
        ]
    )

    lidar_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_tf_pub',
        arguments=[
            '0.0', '0.0', '0.0', 
            '0', '0', '0' ,
            'chassis',
            'rover/rover_sensors/lidar_link/lidar'
        ]
    )

    bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='sensor_bridges',
            arguments=[
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/model/rover/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            '/world/separate_plugins/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
            ],
            output='screen'
        )
    
    rgbd_odom = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        remappings=[
            ('rgb/image', '/camera/image'),
            ('depth/image', '/camera/depth_image'),
            ('rgb/camera_info', '/camera/camera_info'),
            ('imu', '/imu')
        ],
        parameters=[
            {
            'frame_id': 'chassis', 
            'odom_frame_id': 'visual_odom',
            'publish_tf': False,
            'approx_sync': True,
            'approx_sync_max_interval': 0.02,
            
            }],
        output='screen'
    )
    
    lidar_odom = Node(
        package='mola-lidar-odometry',
        executable='lidar_odometry_node',
        name='mola_lidar_odometry',
        output='screen',
        parameters=[{
            'base_frame_id': 'chassis',
            'odom_frame_id': 'lidar_odom',
            'publish_tf': False
        }]
    )
    
    return LaunchDescription(
        [ camera_static_transform, imu_static_transform, lidar_static_transform, bridge, rgbd_odom, lidar_odom]
        )
