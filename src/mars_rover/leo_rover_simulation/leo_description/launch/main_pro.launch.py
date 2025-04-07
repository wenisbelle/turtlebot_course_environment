
 
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_prefix
 
def generate_launch_description():
 
    # Constants for paths to different files and folders
    gazebo_models_path = 'models'
    package_name = 'leo_description'
    robot_name_in_model = 'leo'
    rviz_config_file_path = 'rviz/limo.rviz'
    urdf_file_path = 'urdf/leo_sim.urdf.xacro'
    # world_file = 'l4rralde_gym.world'
    world_file = 'space_terrain_env.world'

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.1'
    spawn_yaw_val = '0.00'

    ############ You do not need to change anything below this line #############

    # Set the path to different files and folders.  
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_urdf_model_path = os.path.join(pkg_share, urdf_file_path)
    default_rviz_config_path = os.path.join(pkg_share, rviz_config_file_path)
    world_path = os.path.join(pkg_share, 'worlds', world_file)
    gazebo_models_path = os.path.join(pkg_share, gazebo_models_path)
    os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    # Launch configuration variables specific to simulation
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui')
    headless = LaunchConfiguration('headless')
    namespace = LaunchConfiguration('namespace')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_namespace = LaunchConfiguration('use_namespace')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_simulator = LaunchConfiguration('use_simulator')
    world = LaunchConfiguration('world')

    # Declare the launch arguments  
    declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')

    declare_use_joint_state_publisher_cmd = DeclareLaunchArgument(
    name='gui',
    default_value='True',
    description='Flag to enable joint_state_publisher_gui')

    declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='False',
    description='Whether to apply a namespace to the navigation stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

    declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')

    declare_urdf_model_path_cmd = DeclareLaunchArgument(
    name='urdf_model', 
    default_value=default_urdf_model_path, 
    description='Absolute path to robot urdf file')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')

    declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
    start_robot_state_publisher_cmd = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': Command(['xacro ', urdf_model]),'use_sim_time': use_sim_time}]
    )

    # Launch RViz
    start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])
 
    # Start Gazebo server with verbose output
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world, 'verbose': 'true'}.items()) # Added 'verbose': 'true'

    # Start Gazebo client with verbose output   
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])),
        launch_arguments={'verbose': 'true'}.items()) # Added 'verbose': 'true'

 
    # Launch the robot
    spawn_entity_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', robot_name_in_model, 
                '-topic', 'robot_description',
                '-timeout', '120.0',
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val],
                    output='screen')
    
    # We add teh gazebo model path to be able to us ethe package: in XACROS and not the file:
    # Plugins
    gazebo_plugins_name = "gazebo_plugins"
    gazebo_plugins_name_path_install_dir = get_package_prefix(
        gazebo_plugins_name)

    
    description_package_name = "leo_description"
    install_dir = get_package_prefix(description_package_name)

    plugins_package_name = "leo_gazebo_plugins"
    plugins_install_dir = get_package_prefix(plugins_package_name)

    
    gazebo_ros_control_plugins_package_name = "gazebo_ros2_control"
    gazebo_ros_control_plugins_install_dir = get_package_prefix(gazebo_ros_control_plugins_package_name)


    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + \
            '/share' + ':' + gazebo_models_path 
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + \
            "/share" + ':' + gazebo_models_path

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib' + ':' + \
            gazebo_plugins_name_path_install_dir + '/lib' + ':' + plugins_install_dir + '/lib/leo_gazebo_plugins'+ ':' + gazebo_ros_control_plugins_install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib' + ':' + gazebo_plugins_name_path_install_dir + \
            '/lib' + ':' + plugins_install_dir + '/lib/leo_gazebo_plugins'+ ':' + gazebo_ros_control_plugins_install_dir + '/lib'

    print("GAZEBO MODELS PATH=="+str(os.environ["GAZEBO_MODEL_PATH"]))
    print("GAZEBO PLUGINS PATH=="+str(os.environ["GAZEBO_PLUGIN_PATH"]))


    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_joint_state_publisher_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)  
    ld.add_action(declare_use_rviz_cmd) 
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    # ld.add_action(start_dummy_sensors)
    #ld.add_action(start_rviz_cmd)

    return ld