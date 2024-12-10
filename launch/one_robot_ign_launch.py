import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

robot_model = 'd435i_robot'  # Updated model name for the RealSense Xacro file
robot_ns = 'D435'  # Robot namespace (aligned with Xacro default argument)
pose = ['1.0', '0.0', '0.0', '0.0']  # Initial robot pose: x, y, z, th
robot_base_color = '0.4 0.4 0.4 1.0'  # Default gray color for the base (rgba)
world_file = 'warehouse.sdf'  # Gazebo world file

def generate_launch_description():
    # Declare simulation time argument
    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Fetch the `use_sim_time` parameter value
    use_sim_time = LaunchConfiguration('use_sim_time')

    this_pkg_path = os.path.join(get_package_share_directory('depth_d435'))

    # Set Ignition Gazebo resource path
    ign_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(this_pkg_path, 'worlds'), ':' + str(Path(this_pkg_path).parent.resolve())
        ]
    )

    # Launch RViz2
    open_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(this_pkg_path + "/rviz/ns_robot.rviz")],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Launch Ignition Gazebo with the specified world
    open_ign = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f"{this_pkg_path}/worlds/{world_file} -v 4 -r",
        }.items()  # No `use_sim_time` required here
    )

    # Process the updated Xacro file
    xacro_file = os.path.join(this_pkg_path, 'urdf', robot_model + '.xacro')
    doc = xacro.process_file(xacro_file, mappings={'base_color': robot_base_color, 'ns': robot_ns})
    robot_desc = doc.toprettyxml(indent='  ')

    # Spawn the RealSense D435 in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', pose[0], '-y', pose[1], '-z', pose[2],
                   '-R', '0.0', '-P', '0.0', '-Y', pose[3],
                   '-name', robot_ns,
                   '-allow_renaming', 'false']
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_ns,
        output="screen",
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': use_sim_time}
        ]
    )

    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/' + robot_ns + '/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/' + robot_ns + '/camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/' + robot_ns + '/depth_camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            '/' + robot_ns + '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/' + robot_ns + '/depth_camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/' + robot_ns + '/depth_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/' + robot_ns + '/depth_camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'

        ],
        output='screen'
    )

    return LaunchDescription([
        simu_time,
        ign_resource_path,
        open_rviz,
        open_ign,
        gz_spawn_entity,
        robot_state_publisher,
        bridge
    ])
