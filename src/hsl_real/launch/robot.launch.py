"""Drivers and the existing autonomous stack on wall time; initially stopped."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from hsl_real.config import load_config, start_polygon


def nodes(context):
    cfg, mission = load_config(LaunchConfiguration('config_file').perform(context))
    drivers = LaunchConfiguration('drivers_enabled').perform(context).lower() == 'true'
    role = mission['robot']['role']
    motion = mission['motion']
    reverse = role == 'explorer' and motion['allow_reverse']
    speed, angular = motion['max_speed'], motion['max_angular_speed']
    processes = []
    def add(package, executable, parameters=None, **kwargs):
        node = Node(package=package, executable=executable, output='screen',
                    parameters=[{'use_sim_time': False}] + (parameters or []), **kwargs)
        processes.append(node)
    def static(parent, child, pose):
        x,y,z,roll,pitch,yaw = pose
        add('tf2_ros','static_transform_publisher', arguments=[
            '--x',str(x),'--y',str(y),'--z',str(z),'--roll',str(roll),
            '--pitch',str(pitch),'--yaw',str(yaw),'--frame-id',parent,'--child-frame-id',child])
    if drivers:
        add('kobuki_node','kobuki_ros_node', parameters=[{
            'device_port':cfg['kobuki_port'], 'odom_frame':'odom', 'base_frame':'base_footprint',
            'publish_tf':True, 'use_imu_heading':True, 'acceleration_limiter':False,
            'cmd_vel_timeout_sec':.6}], remappings=[('commands/velocity','/cmd_vel'),('odom',cfg['odom_topic'])])
        add('livox_ros_driver2','livox_ros_driver2_node', name='livox_lidar_publisher',
            parameters=[{'xfer_format':0, 'multi_topic':0, 'data_src':0, 'publish_freq':10.,
                         'output_data_type':0, 'frame_id':'livox', 'user_config_path':cfg['livox_config'],
                         'lvx_file_path':'', 'cmdline_input_bd_code':'livox0000000001'}],
            remappings=[('/livox/lidar',cfg['lidar_topic'])])
    description = Path(get_package_share_directory('kobuki_description'))/'urdf/kobuki_standalone.urdf.xacro'
    add('robot_state_publisher','robot_state_publisher', parameters=[{'robot_description':Command(['xacro ',str(description)])}],
        remappings=[('joint_states','/joint_states')])
    static('base_link','livox',cfg['lidar_mount'])
    if cfg['localization'] == 'odometry':
        x,y,yaw = mission['robot']['start']
        static('map','odom',[x,y,0.,0.,0.,yaw])
    add('hsl_real','real_observations', parameters=[{'odom_topic':cfg['odom_topic'],'lidar_topic':cfg['lidar_topic']}])
    add('hsl_perception','opponent_detector', parameters=[{'opponent_max_height':float(mission['perception']['opponent_max_height'])}])
    add('hsl_decision','decision_manager', parameters=[{'role':role,'own_max_speed':speed,
        'own_start':start_polygon(mission['robot']),'opponent_start':start_polygon(mission['opponent'])}])
    add('hsl_planning','trajectory_planner',parameters=[{'role':role,'max_speed':speed,
        'random_seed':mission['match']['seed'],'arena_bounds':cfg['arena_bounds'],'require_match_active':True}])
    native = str(Path(get_package_share_directory('hsl_nav2_control'))/'config/native_mppi.yaml')
    add('hsl_nav2_control','native_mppi',parameters=[native, {'use_sim_time':False,
        'role':role,'random_seed':mission['match']['seed'], 'MPPI.vx_max':speed,
        'MPPI.vx_min':-speed if reverse else 0., 'MPPI.wz_max':angular,
        'MPPI.PathAngleCritic.forward_preference':not reverse,
        'MPPI.PreferForwardCritic.enabled':False,'MPPI.GoalAngleCritic.enabled':False,
        'MPPI.GoalCritic.cost_weight':15. if role=='guardian' else 5.,
        'costmap.plugins':['static_layer','obstacle_layer','inflation_layer'] if cfg['map_file'] else ['obstacle_layer','inflation_layer']}])
    add('hsl_debug_control','motion_gate',parameters=[{'require_match_active':True}],name='hsl_motion_gate')
    add('hsl_real','real_match',parameters=[{'active_seconds':float(mission['match']['active_seconds'])}])
    if cfg['map_file']:
        add('nav2_map_server','map_server', name='map_server', parameters=[{'yaml_filename':cfg['map_file']}])
        add('nav2_lifecycle_manager','lifecycle_manager', name='map_lifecycle_manager',parameters=[{
            'autostart':True,'node_names':['map_server']}])
    if cfg['rviz']:
        rviz = str(Path(get_package_share_directory('hsl_real'))/'config/robot.rviz')
        # RViz is optional; closing it must not terminate the robot stack.
        viewer = Node(package='rviz2',executable='rviz2',arguments=['-d',rviz],parameters=[{'use_sim_time':False}])
    handlers = [RegisterEventHandler(OnProcessExit(target_action=p,
        on_exit=[EmitEvent(event=Shutdown(reason='Real robot process exited'))])) for p in processes]
    return processes + handlers + ([viewer] if cfg['rviz'] else [])


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('config_file',default_value='/config/real.yaml'),
        DeclareLaunchArgument('drivers_enabled',default_value='true'),OpaqueFunction(function=nodes)])
