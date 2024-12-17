
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

rviz = 0

def generate_launch_description():
    package_dir = get_package_share_directory('slam_rover')  # pkg dir after building in install/share
    urdf = os.path.join(package_dir,'urdf','rover_proto_A1','rover_proto_A1.urdf')   # urdf is saved here in install/share after build
    
    urdf_rviz = os.path.join(package_dir,'urdf','rover_proto_A1', 'rover_proto_A1_rviz.urdf')
    rviz_config_file=os.path.join(package_dir, 'urdf','rover_proto_A1', 'nav2_rviz.rviz') # config.rviz is saved here in install/share after build

    # print("pkg rover location:",urdf)


    # world_file = 'track_1.world' # for right wall following
    world_file = 'blocks12.sdf' 
    world_path = os.path.join(package_dir,'worlds', world_file) 

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')


    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    ld = LaunchDescription()
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_rviz',
            default_value= '1',
            description='Set to "1" to launch RViz'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),



        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='both',
            arguments=[urdf_rviz],
            parameters=[{'use_sim_time':True}, {'use_ros2_control': True}]
            ),

        ld,



        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher', #'joint_state_publisher_gui'
            name='joint_state_publisher',
            arguments=[urdf_rviz], 
            parameters=[{'use_sim_time':True}]),
            

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            arguments=["-topic", "robot_description", "-entity", "rover_proto_A1_rviz"],
            output='screen'
        ),





    ])

